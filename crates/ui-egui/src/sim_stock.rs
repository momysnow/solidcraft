//! The stock of the G-code simulation (`sim.setup`, `sim.run`) in the viewport: the uncut box, and the
//! surface of each chunk a cut has reached (`solidcraft_machining::mesh`), opaque with its crease edges
//! dark, as a STEP model's body edges. The cutting tool stands where the engine's playhead puts it (its
//! pose, interpolated within the move).
//!
//! Only the chunks the stock marks dirty are meshed, on a worker thread (or inline where there is none,
//! as on wasm), one batch at a time, like the toolpaths. A batch made for an older stock, revision or
//! display step is dropped. The display step coarsens as the cut grows (`mesh::display_step`); a step
//! change remeshes the whole cut once. The uncut box is rebuilt for the chunks that have no mesh.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::f64::consts::TAU;
use std::hash::{Hash, Hasher};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError, mpsc::Receiver, mpsc::TryRecvError};
use std::time::Duration;

use serde_json::json;
use solidcore_record::Vec3;
use solidcraft_engine::geom::{Mesh, prelude::*};
use solidcraft_machining::dexel::Stock;
use solidcraft_machining::mesh;

use crate::gpu::{BlockBuilder, BlockBytes, GpuScene, PaletteIndex, SceneBlock, TriClass};
use solidcraft_theme::Tokens;
use solidcraft_ui_text::tr;

use crate::{SolveApp, ui_kit, viewport::c4};

/// How often the window asks again while a batch is being meshed.
const POLL: Duration = Duration::from_millis(100);
/// How long one frame may play the simulation, in seconds: the window stays responsive meanwhile.
const PLAY_SLICE_S: f64 = 0.03;
/// How long one frame may run a scrub, in seconds: a backward seek restores a checkpoint, so it is cheap.
const SCRUB_SLICE_S: f64 = 0.05;
/// The playback speed a simulation starts at, in machine seconds per real second, and its range.
const DEFAULT_SPEED: f64 = 10.0;
const MIN_SPEED: f64 = 1.0;
const MAX_SPEED: f64 = 500.0;
/// The longest real time one frame moves the playhead for, so a stalled frame does not jump the cut.
const MAX_FRAME_S: f64 = 0.1;
/// The cosine of 30 degrees: faces that turn further than this from each other meet at a crease, so
/// their edge is drawn and each side keeps its own shading.
const CREASE_COS: f64 = 0.866;
/// The width of the crease edges: the body edges' (`viewport.rs`), so the stock reads as the model.
const EDGE_WIDTH: f32 = 1.3;
/// The sides of the cutting tool's cylinder: round at the size it is drawn.
const TOOL_SIDES: usize = 24;
/// The playback bar's width: this share of the view, within these limits. A view too narrow for the
/// minimum hides the bar.
const BAR_SHARE: f32 = 0.7;
const BAR_MIN_WIDTH: f32 = 520.0;
const BAR_MAX_WIDTH: f32 = 1100.0;
/// The speed slider's width, in points.

/// The meshes of the chunks in one batch, by display index: `None` for a chunk with no surface now.
type Batch = Vec<([usize; 3], Option<Surface>)>;

/// One chunk's surface, or the uncut box: its mesh and the edges where its faces meet at a crease.
#[derive(Default)]
struct Surface {
    mesh: Mesh,
    /// The crease edges in the work frame, drawn dark over the faces.
    edges: Vec<[[f32; 3]; 2]>,
    /// Names this surface among those the state holds, so its block is told apart from the one it replaces.
    serial: u64,
}

impl Surface {
    /// A chunk's surface, found on the worker thread that meshed it. The mesher shares vertices by index
    /// within a chunk, so its edges are matched by index.
    fn new(mesh: Mesh) -> Surface {
        let edges = crease_edges(&mesh, |i, j| (i.min(j), i.max(j)));
        Surface { mesh, edges, serial: 0 }
    }

    /// The uncut box, whose quads do not share vertices: its edges are matched by where their ends are.
    fn by_position(mesh: Mesh) -> Surface {
        let edges = crease_edges(&mesh, |i, j| position_pair(&mesh, i, j));
        Surface { mesh, edges, serial: 0 }
    }
}

/// What the viewport draws of the simulation's stock.
pub struct State {
    /// The stock's revision these meshes belong to; 0 without a simulation.
    revision: u64,
    /// The display step (1, 2, 4 or 8) the meshes are made at.
    step: u32,
    /// The surface of each display chunk a cut reached.
    meshes: BTreeMap<[usize; 3], Surface>,
    /// The last serial given to a surface (see `Surface::serial`).
    serials: u64,
    /// The block each chunk's surface was last built into, with the key it was built for. Built on the
    /// window thread, where the palette slots are known; kept while the key comes back.
    blocks: Mutex<BTreeMap<[usize; 3], (u64, Arc<BlockBytes>)>>,
    /// The display chunks to mesh next, each once.
    pending: BTreeSet<[usize; 3]>,
    /// The one batch being meshed.
    job: Option<Job>,
    /// Changes whenever the drawn geometry does.
    generation: u64,
    /// The uncut box, for the chunks without a mesh, built for `box_gen` (the box's own key: bumped only
    /// when a chunk gains or loses its mesh, so a cursor step or a tool move does not rebuild it).
    boxed: Surface,
    box_gen: u64,
    box_for: Option<u64>,
    /// The box's block, with the key it was built for (see `box_gen`); built in `draw`, kept while the key
    /// comes back.
    box_block: Mutex<Option<(u64, Arc<BlockBytes>)>>,
    /// Whether the simulation plays: each frame moves the playhead on by its share of real time times
    /// `speed`.
    playing: bool,
    /// The playback speed, in machine seconds per real second; kept across Simulate runs.
    pub(crate) speed: f64,
    /// The time a scrub (the timeline slider) is still on its way to: the engine may cut only part of
    /// the way per frame, so the seek goes on over the following frames.
    scrub: Option<f64>,
    /// The work frame the stock is in, as origin and X, Y, Z axes in the model; `None` is the model's.
    /// shortcut: kept from the last Simulate only, so a `sim.setup` sent another way draws in the frame
    /// of the last Simulate; take it from the simulation when it records one.
    frame: Option<[[f32; 3]; 4]>,
    /// The playback bar's rectangle as last drawn over the view; `None` while it is not drawn.
    bar: Option<egui::Rect>,
    /// What the Simulation window's View tab has set.
    pub(crate) view: crate::sim_view::ViewOptions,
    /// The toolpath's lines, one block per chunk of moves, kept while the key built from the stock's revision,
    /// the frame, the colours and the View flags comes back.
    path_blocks: Mutex<BTreeMap<usize, (u64, Arc<BlockBytes>)>>,
    /// Chunks meshed so far (tests only).
    #[cfg(test)]
    meshed: usize,
}

struct Job {
    revision: u64,
    step: u32,
    /// Set when the batch is let go of: the thread stops at its next chunk instead of meshing on.
    cancel: Arc<AtomicBool>,
    rx: Receiver<Batch>,
}

/// How a batch was meshed: on a thread that reports later, or at once.
enum Start {
    Thread(Job),
    Inline(Batch),
}

/// The work of a batch, taken by the thread that meshes it.
struct Work {
    stock: Stock,
    chunks: Vec<[usize; 3]>,
    cancel: Arc<AtomicBool>,
}

impl Default for State {
    fn default() -> State {
        State {
            revision: 0,
            step: 0,
            meshes: BTreeMap::new(),
            serials: 0,
            blocks: Mutex::default(),
            pending: BTreeSet::new(),
            job: None,
            generation: 0,
            boxed: Surface::default(),
            box_gen: 0,
            box_for: None,
            box_block: Mutex::default(),
            playing: false,
            speed: DEFAULT_SPEED,
            scrub: None,
            frame: None,
            bar: None,
            view: crate::sim_view::ViewOptions::default(),
            path_blocks: Mutex::default(),
            #[cfg(test)]
            meshed: 0,
        }
    }
}

impl State {
    fn changed(&mut self) {
        self.generation += 1;
    }
}

/// Starts the active setup's simulation (the Simulate button): posts it and sets its stock up, then
/// plays it from the start. A refusal goes to the status bar.
pub fn simulate_active(app: &mut SolveApp) {
    let setup = app.manufacture_workspace.active_setup.clone();
    match app.run("manufacture.post.simulate", json!({"setup": setup})) {
        Ok(reply) => {
            app.sim_stock.frame = frame(&reply["frame"]);
            app.sim_stock.scrub = None;
            app.sim_stock.playing = true;
            app.sim_stock.view.open = true;
            crate::manufacture_workspace::show_simulation_tab(app);
        }
        Err(e) => app.set_status(e, true),
    }
}

/// The work frame of a Simulate reply: origin and X, Y, Z axes in the model.
fn frame(v: &serde_json::Value) -> Option<[[f32; 3]; 4]> {
    let vec = |k: &str| -> Option<[f32; 3]> {
        let a = v[k].as_array()?;
        Some([a.first()?.as_f64()? as f32, a.get(1)?.as_f64()? as f32, a.get(2)?.as_f64()? as f32])
    };
    Some([vec("origin")?, vec("x")?, vec("y")?, vec("z")?])
}

/// Whether the model body is hidden: while a simulation's stock is shown in Manufacture, so the cut is
/// visible. Other workspaces keep their bodies.
pub fn hides_bodies(app: &SolveApp) -> bool {
    // The session's revision, not the state's: Simulate hides the body at once, before the next poll.
    crate::manufacture_workspace::active(app) && app.session.sim_revision() != 0 && app.sim_stock.view.model_opacity <= 0.0
}

/// The factor the model's opacity is multiplied by while a simulation runs: the View tab's Opacity, so a
/// model shown through the stock is see-through. 1 outside a simulation.
pub fn model_opacity(app: &SolveApp) -> f32 {
    if crate::manufacture_workspace::active(app) && app.session.sim_revision() != 0 { app.sim_stock.view.model_opacity.clamp(0.0, 1.0) } else { 1.0 }
}

/// The playhead one frame on: `dt` real seconds at `speed` machine seconds each, with `dt` capped at
/// `MAX_FRAME_S`, and never past `total`.
pub fn next_time(time: f64, total: f64, dt: f64, speed: f64) -> f64 {
    (time + dt.min(MAX_FRAME_S) * speed).min(total)
}

/// Whether the playhead is at the program's end, allowing for float error in the engine's time.
fn at_end(time: f64, total: f64) -> bool {
    time >= total - 1e-9
}

/// Moves the simulation to where the playhead is to go this frame: the pending scrub while there is
/// one, else one frame on while it plays. The engine's reply is the truth: it cuts only as far as the
/// budget allows, so the next frame goes on from the playhead it reports, never from the target.
fn advance(app: &mut SolveApp, ctx: &egui::Context) {
    // Not in Manufacture the stock is not drawn: playback pauses rather than cut what no one sees.
    if !crate::manufacture_workspace::active(app) {
        app.sim_stock.playing = false;
        return;
    }
    // Cleared elsewhere (a `sim.clear` command, a new setup) while it played: stop quietly, no error.
    let Some((time, total)) = app.session.sim_time() else {
        app.sim_stock.playing = false;
        app.sim_stock.scrub = None;
        return;
    };
    if let Some(target) = app.sim_stock.scrub {
        match app.session.execute("sim.run", &json!({"time": target, "budget_s": SCRUB_SLICE_S})) {
            Ok(reply) if reply["reached"].as_bool() == Some(true) => app.sim_stock.scrub = None,
            Ok(_) => ctx.request_repaint(),
            Err(e) => {
                app.sim_stock.scrub = None;
                app.set_status(e.to_string(), true);
            }
        }
        return;
    }
    if !app.sim_stock.playing {
        return;
    }
    let dt = f64::from(ctx.input(|i| i.stable_dt));
    let target = next_time(time, total, dt, app.sim_stock.speed.clamp(MIN_SPEED, MAX_SPEED));
    match app.session.execute("sim.run", &json!({"time": target, "budget_s": PLAY_SLICE_S})) {
        Ok(reply) if reply["done"].as_bool() == Some(true) || reply["time"].as_f64().is_some_and(|t| at_end(t, total)) => {
            app.sim_stock.playing = false;
        }
        Ok(_) => ctx.request_repaint(),
        Err(e) => {
            app.sim_stock.playing = false;
            app.set_status(e.to_string(), true);
        }
    }
}

/// Plays the simulation on from the playhead; a finished run plays again from the start.
pub fn play(app: &mut SolveApp) {
    if app.session.sim_time().is_some_and(|(time, total)| at_end(time, total)) {
        app.sim_stock.scrub = Some(0.0);
    }
    app.sim_stock.playing = true;
}

/// Moves the playhead to `time` (the timeline): playback pauses, and the frames that follow run the
/// simulation there, as far as each frame's budget allows.
pub fn seek(app: &mut SolveApp, time: f64) {
    app.sim_stock.playing = false;
    app.sim_stock.scrub = Some(time);
}

/// Clears the simulation from the playback bar's Clear button: stops playback and drops the stock.
/// Whether a simulation is on screen in the Manufacture workspace: the ribbon shows the Simulation tab,
/// the history bar is hidden and the playback bar is drawn.
pub fn simulating(app: &SolveApp) -> bool {
    crate::manufacture_workspace::active(app) && app.session.sim_progress().is_some()
}

/// Ends the simulation (Exit Simulation): drops the stock and closes the View window. The ribbon goes
/// back to the Manufacture tabs on the next frame.
pub fn exit_simulation(app: &mut SolveApp) {
    clear(app);
    app.sim_stock.view.open = false;
    crate::manufacture_workspace::sync_tab(app);
}

fn clear(app: &mut SolveApp) {
    app.sim_stock.playing = false;
    app.sim_stock.scrub = None;
    if let Err(e) = app.session.execute("sim.clear", &json!({})) {
        app.set_status(e.to_string(), true);
    }
}

/// The bar's place: centred on the view at `BAR_SHARE` of its width within the limits, and just above the
/// navigation bar when one sits on the bottom edge (`nav`), else at the view's bottom. `None` when the
/// view cannot hold the minimum bar.
fn bar_rect(view: egui::Rect, nav: Option<egui::Rect>) -> Option<egui::Rect> {
    let t = Tokens::get();
    if view.width() < BAR_MIN_WIDTH + 2.0 * t.space_16 {
        return None;
    }
    let width = (view.width() * BAR_SHARE).clamp(BAR_MIN_WIDTH, BAR_MAX_WIDTH).min(view.width() - 2.0 * t.space_16);
    let height = (1.0 + BUTTON_SCALE) * t.control_height + 3.0 * t.space_4;
    let bottom = nav.map_or(view.bottom() - t.space_8, |nav| nav.top() - t.space_8);
    Some(egui::Rect::from_center_size(egui::pos2(view.center().x, bottom - height / 2.0), egui::vec2(width, height)))
}

/// The slider's position for a speed: 0 at `MIN_SPEED`, 1 at `MAX_SPEED`, on a log scale so the slow
/// speeds keep room on the slider.
fn speed_level(speed: f64) -> f64 {
    (speed.clamp(MIN_SPEED, MAX_SPEED) / MIN_SPEED).ln() / (MAX_SPEED / MIN_SPEED).ln()
}

/// The speed at a slider position (the inverse of `speed_level`).
fn speed_at(level: f64) -> f64 {
    MIN_SPEED * (MAX_SPEED / MIN_SPEED).powf(level.clamp(0.0, 1.0))
}

/// The bar's rectangle as last drawn over the view, for the overlays that keep clear of it.
pub fn playback_rect(app: &SolveApp) -> Option<egui::Rect> {
    app.sim_stock.bar
}

/// What a transport button does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Control {
    Start,
    PreviousOperation,
    StepBack,
    Toggle,
    StepForward,
    NextOperation,
    End,
}

/// How much larger than the History bar's the simulation's transport glyphs are drawn.
const GLYPH_SCALE: f32 = 1.6;

/// How much larger than a control row the simulation's transport buttons are.
const BUTTON_SCALE: f32 = 1.25;

/// One transport button on the view: the History bar's artwork, larger, with a shadow so it reads over
/// the model, and a step and an operation glyph of its own.
fn transport_button(ui: &mut egui::Ui, size: f32, glyph: &str, label: &str) -> egui::Response {
    let t = Tokens::get();
    let (rect, response) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::click());
    if response.hovered() {
        ui.painter().rect_filled(rect, t.radius_sm, t.hover);
    }
    let ink = if response.hovered() { t.accent } else { t.icon };
    crate::icons::glyphs::transport_glyph_scaled(ui.painter(), rect.center() + egui::vec2(1.0, 1.0), glyph, t.overlay, GLYPH_SCALE);
    crate::icons::glyphs::transport_glyph_scaled(ui.painter(), rect.center(), glyph, ink, GLYPH_SCALE);
    response.widget_info(|| egui::WidgetInfo::labeled(egui::Role::Button, ui.is_enabled(), label));
    response.on_hover_cursor(egui::CursorIcon::PointingHand).on_hover_text(label)
}

/// Where the playhead goes to for the previous or next operation: forward, the start of the next one (the
/// end of the program after the last); back, the start of the one before the playhead, or of the current
/// one when the playhead is more than a second into it.
fn operation_jump(sections: &[(String, f64, f64, usize)], shown: f64, total: f64, forward: bool) -> f64 {
    if forward {
        sections.iter().map(|s| s.1).find(|&start| start > shown + 1.0e-6).unwrap_or(total)
    } else {
        sections.iter().map(|s| s.1).filter(|&start| start < shown - 1.0).last().unwrap_or(0.0)
    }
}

/// Moves the playhead one move on or back (the step buttons): to the start of the move the playhead is in
/// when stepping back from inside one.
fn step_move(app: &mut SolveApp, forward: bool) {
    let (Some((cursor, total)), Some((time, _))) = (app.session.sim_progress(), app.session.sim_time()) else { return };
    let inside = app.session.sim_move_start(cursor).is_some_and(|start| time > start + 1.0e-6);
    let target = match (forward, inside) {
        (true, _) => (cursor + 1).min(total),
        (false, true) => cursor,
        (false, false) => cursor.saturating_sub(1),
    };
    if let Some(at) = app.session.sim_move_start(target) {
        seek(app, at);
    }
}

/// The playback bar over the 3D viewport, in two rows: the transport buttons (start, previous operation,
/// step back, play or pause, step forward, next operation, end) and the speed slider under them, as wide as
/// the buttons. Where the playhead is shows on the bar along the bottom of the window, not here. Shown while Manufacture shows a simulation, and
/// only where it fits. Its widgets claim the pointer over the bar, so the camera and picking beneath get no
/// clicks from it.
pub fn playback(app: &mut SolveApp, ui: &mut egui::Ui, viewport: egui::Rect) {
    app.sim_stock.bar = None;
    if !crate::manufacture_workspace::active(app) {
        return;
    }
    let Some((time, total_time)) = app.session.sim_time() else { return };
    let view = crate::shell::bottom_overlay_rect(app, crate::shell::content_rect(app, viewport));
    let nav = app.viewport.layout.as_ref().map(|l| l.navigation).filter(|n| {
        n.is_positive() && n.is_finite() && app.ui.show_navigation_bar && app.preferences.navigation_bar.edge == crate::ui_kit::NavigationEdge::Bottom
    });
    let Some(rect) = bar_rect(view, nav).filter(|r| r.is_finite() && r.is_positive()) else { return };
    let t = Tokens::get();
    let h = t.control_height;
    let b = (BUTTON_SCALE * h).round();
    let gap = t.space_8;
    let cluster = 7.0 * b + 6.0 * gap;
    // Only as wide as the buttons, over the middle of the navigation bar (of the view without one), and
    // left of the simulation window while it is open.
    let width = (cluster + 2.0 * t.space_12).min(rect.width());
    let mut x = nav.map_or(rect.center().x, |n| n.center().x);
    if app.sim_stock.view.open {
        x = x.min(view.right() - crate::sim_view::window_width() - 32.0 - width / 2.0);
    }
    let rect = egui::Rect::from_center_size(egui::pos2(x.max(view.left() + width / 2.0), rect.center().y), egui::vec2(width, rect.height()));
    let inner = rect.shrink2(egui::vec2(t.space_12, t.space_4));
    let top = egui::Rect::from_center_size(egui::pos2(inner.center().x, inner.top() + b / 2.0), egui::vec2(cluster, b));
    let low = egui::Rect::from_min_size(egui::pos2(top.left(), top.bottom() + t.space_4), egui::vec2(cluster, h));
    let speed = format!("{}: {:.0}{}", tr("sim.stock.speed"), app.sim_stock.speed, tr("sim.stock.speed_suffix"));

    // No surface under the buttons: they sit on the view. Claimed before the row's widgets, so they stay on top of it and the viewport gets no press on the bar.
    ui.interact(rect, ui.scope_id().with("sim_playback"), egui::Sense::click_and_drag());
    app.sim_stock.bar = Some(rect);

    let playing = app.sim_stock.playing;
    let mut level = speed_level(app.sim_stock.speed);
    // While a scrub is on its way the buttons work from the target, not the playhead that lags it.
    let shown = app.sim_stock.scrub.unwrap_or(time);
    let mut pressed: Option<Control> = None;
    ui.scope_builder(egui::UiBuilder::new().max_rect(top).layout(egui::Layout::left_to_right(egui::Align::Center)), |ui| {
        ui.spacing_mut().item_spacing.x = gap;
        let buttons = [
            (Control::Start, "first", "sim.stock.start"),
            (Control::PreviousOperation, "prev_op", "sim.stock.previous_operation"),
            (Control::StepBack, "step_back", "sim.stock.step_back"),
            (Control::Toggle, if playing { "pause" } else { "play" }, if playing { "sim.stock.pause" } else { "sim.stock.play" }),
            (Control::StepForward, "step_forward", "sim.stock.step_forward"),
            (Control::NextOperation, "next_op", "sim.stock.next_operation"),
            (Control::End, "last", "sim.stock.end"),
        ];
        for (control, glyph, label) in buttons {
            if transport_button(ui, b, glyph, tr(label)).clicked() {
                pressed = Some(control);
            }
        }
    });
    ui.scope_builder(egui::UiBuilder::new().max_rect(low).layout(egui::Layout::left_to_right(egui::Align::Center)), |ui| {
        ui.set_clip_rect(rect);
        ui.spacing_mut().slider_width = low.width();
        let slider = ui_kit::slider(ui, &mut level, 0.0..=1.0, ui_kit::Slider { show_value: false, ..Default::default() });
        if slider.on_hover_text(speed).changed() {
            app.sim_stock.speed = speed_at(level);
        }
    });
    let sections = app.session.sim_sections();
    match pressed {
        Some(Control::Start) => seek(app, 0.0),
        Some(Control::End) => seek(app, total_time),
        Some(Control::PreviousOperation) => seek(app, operation_jump(&sections, shown, total_time, false)),
        Some(Control::NextOperation) => seek(app, operation_jump(&sections, shown, total_time, true)),
        Some(Control::StepBack) => step_move(app, false),
        Some(Control::StepForward) => step_move(app, true),
        Some(Control::Toggle) if playing => app.sim_stock.playing = false,
        Some(Control::Toggle) => play(app),
        None => {}
    }
}

/// The bar along the bottom of the window, in the history bar's place while a simulation shows: one
/// segment per operation, its name shown under the pointer only. In each segment green is what the playhead has
/// played, dark is what is computed ahead of it and light is what is still to compute. A line marks the
/// playhead, and a press or a drag sets it: the time it gives is returned.
fn section_bar(ui: &mut egui::Ui, size: egui::Vec2, sections: &[(String, f64, f64, usize)], computed: f64, shown: f64, total: f64) -> Option<f64> {
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click_and_drag());
    if !(total > 0.0) || !rect.is_positive() || !rect.is_finite() || sections.is_empty() {
        return None;
    }
    let t = Tokens::get();
    let pad = t.space_12;
    let height = (rect.height() * 0.36).clamp(6.0, 16.0);
    let bar_top = rect.center().y - height / 2.0;
    let track = egui::Rect::from_min_max(egui::pos2(rect.left() + pad, bar_top), egui::pos2(rect.right() - pad, bar_top + height));
    let x = |s: f64| track.left() + track.width() * (s / total).clamp(0.0, 1.0) as f32;
    let gap = 3.0;
    let round = (height / 4.0).max(2.0);
    // Dark blue is computed, light blue is still to compute, whatever the theme's background.
    let shade =
        |c: egui::Color32, k: f32| egui::Color32::from_rgb((f32::from(c.r()) * k) as u8, (f32::from(c.g()) * k) as u8, (f32::from(c.b()) * k) as u8);
    let (green, dark, light) = (t.form_success, shade(t.accent, 0.6), t.accent.lerp_to_gamma(egui::Color32::WHITE, 0.5));
    let p = ui.painter();
    let slice = |a: f64, b: f64| {
        egui::Rect::from_min_max(egui::pos2(x(a) + gap / 2.0, track.top()), egui::pos2((x(b) - gap / 2.0).max(x(a) + gap / 2.0), track.bottom()))
    };
    for (name, a, b, _) in sections {
        let whole = slice(*a, *b);
        if whole.width() < 1.0 {
            continue;
        }
        p.rect_filled(whole, round, light);
        let ahead = computed.clamp(*a, *b);
        if ahead > *a {
            p.rect_filled(slice(*a, ahead), round, dark);
        }
        let played = shown.clamp(*a, *b);
        if played > *a {
            p.rect_filled(slice(*a, played), round, green);
        }
        let _ = name;
    }
    let at = x(shown);
    p.line_segment([egui::pos2(at, track.top() - 3.0), egui::pos2(at, track.bottom() + 3.0)], egui::Stroke::new(2.0, t.text));
    let seconds = |pos: egui::Pos2| (f64::from((pos.x - track.left()) / track.width().max(1.0)) * total).clamp(0.0, total);
    let name = response.hover_pos().map(seconds).and_then(|s| sections.iter().find(|(_, a, b, _)| s >= *a && s < *b)).map(|(n, _, _, _)| n.as_str());
    let tip = match name {
        Some(n) => solidcraft_ui_text::format("sim.bar.operation", &[("name", n)]),
        None => tr("sim.stock.playhead").to_string(),
    };
    let response = response.on_hover_text(tip);
    if response.is_pointer_button_down_on() || response.clicked() {
        return response.interact_pointer_pos().map(seconds);
    }
    None
}

/// The simulation's bar, drawn in the slot the history bar has (see `timeline::timeline`): the whole
/// window's width, with a click or a drag setting the playhead.
pub fn bottom_bar(app: &mut SolveApp, ui: &mut egui::Ui) {
    let (Some((time, total)), Some(computed)) = (app.session.sim_time(), app.session.sim_computed_time()) else { return };
    let sections = app.session.sim_sections();
    let shown = app.sim_stock.scrub.unwrap_or(time);
    let size = egui::vec2(ui.available_width(), ui.available_height());
    if let Some(to) = section_bar(ui, size, &sections, computed, shown, total) {
        seek(app, to);
    }
}

/// Meshes the stock's dirty chunks and collects what is done. Called every frame.
pub fn poll(app: &mut SolveApp, ctx: &egui::Context) {
    advance(app, ctx);
    let revision = app.session.sim_revision();
    let dirty = app.session.sim_take_dirty();
    let stock = app.session.sim_stock();
    let state = &mut app.sim_stock;
    if revision != state.revision {
        state.revision = revision;
        state.step = 1;
        state.meshes.clear();
        forget_blocks(state);
        state.pending.clear();
        state.box_gen += 1;
        state.changed();
    }
    let Some(stock) = stock else { return };

    if !dirty.is_empty() {
        let step = mesh::display_step(stock.touched_chunks().len());
        let counts = mesh::display_counts(stock, step);
        if step == state.step {
            for chunk in dirty {
                state.pending.extend(mesh::display_chunks_for(chunk, step, counts));
            }
        } else {
            // A coarser display: every cut chunk is meshed again at the new step.
            state.step = step;
            state.meshes.clear();
            forget_blocks(state);
            let touched = stock.touched_chunks();
            state.pending = touched.into_iter().flat_map(|c| mesh::display_chunks_for(c, step, counts)).collect();
            state.box_gen += 1;
            state.changed();
        }
    }

    // A batch made for an older revision or step is never applied, so it is let go of here rather than
    // waited for: its worker stops at the next chunk, and the work for the stock as it is now starts at
    // once. Its chunks are dirty again (a new revision marks the cut's chunks, a new step all of them).
    if let Some(job) = state.job.take() {
        if job.revision == state.revision && job.step == state.step {
            match job.rx.try_recv() {
                Ok(batch) => apply(state, job.revision, job.step, batch),
                Err(TryRecvError::Empty) => state.job = Some(job),
                Err(TryRecvError::Disconnected) => {}
            }
        } else {
            job.cancel.store(true, Ordering::Relaxed);
        }
    }

    if state.job.is_none() && !state.pending.is_empty() {
        let cancel = Arc::new(AtomicBool::new(false));
        let work = Work { stock: stock.clone(), chunks: std::mem::take(&mut state.pending).into_iter().collect(), cancel };
        let (revision, step) = (state.revision, state.step);
        match start(work, revision, step) {
            Start::Thread(job) => state.job = Some(job),
            Start::Inline(batch) => apply(state, revision, step, batch),
        }
    }

    if state.box_for != Some(state.box_gen) {
        let meshes = &state.meshes;
        let mesh = mesh::box_faces(stock, state.step, |idx| meshes.contains_key(&idx));
        // The box is rebuilt on this thread, a few hundred quads: its crease edges are found here too.
        // Its serial is its generation, so its block's key changes with it and only with it.
        state.boxed = Surface::by_position(mesh);
        state.boxed.serial = state.box_gen;
        state.box_for = Some(state.box_gen);
    }

    if state.job.is_some() || !state.pending.is_empty() {
        ctx.request_repaint_after(POLL);
    }
}

/// Drops the cached blocks along with the meshes they were built from. A block kept past a Clear would
/// hold its bytes (and GPU buffers) for nothing, since `draw` returns early without a simulation.
fn forget_blocks(state: &mut State) {
    state.blocks.get_mut().unwrap_or_else(PoisonError::into_inner).clear();
    *state.box_block.get_mut().unwrap_or_else(PoisonError::into_inner) = None;
}

/// Takes a batch in, if it was made for the stock and step the state holds now.
fn apply(state: &mut State, revision: u64, step: u32, batch: Batch) {
    if revision != state.revision || step != state.step {
        return;
    }
    // The box covers the chunks that have a mesh, so only a chunk gaining or losing its mesh changes it.
    let mut covered_changed = false;
    for (idx, surface) in batch {
        #[cfg(test)]
        {
            state.meshed += 1;
        }
        match surface {
            Some(mut surface) => {
                state.serials += 1;
                surface.serial = state.serials;
                covered_changed |= state.meshes.insert(idx, surface).is_none();
            }
            None => {
                covered_changed |= state.meshes.remove(&idx).is_some();
            }
        }
    }
    if covered_changed {
        state.box_gen += 1;
    }
    state.changed();
}

/// Meshes `work` on a worker thread, or inline where there is none (wasm, or a thread that cannot be
/// started).
fn start(work: Work, revision: u64, step: u32) -> Start {
    #[cfg(target_arch = "wasm32")]
    return Start::Inline(compute(&work, step));
    #[cfg(not(target_arch = "wasm32"))]
    {
        let (tx, rx) = std::sync::mpsc::channel();
        let cancel = Arc::clone(&work.cancel);
        let slot = Arc::new(Mutex::new(Some(work)));
        let worker = Arc::clone(&slot);
        let spawned = std::thread::Builder::new().name("cam-sim-stock".into()).spawn(move || {
            let taken = worker.lock().ok().and_then(|mut w| w.take());
            if let Some(work) = taken {
                let _ = tx.send(compute(&work, step));
            }
        });
        if spawned.is_ok() {
            return Start::Thread(Job { revision, step, cancel, rx });
        }
        // No thread could be started: mesh here, from the work the thread never took.
        let taken = slot.lock().ok().and_then(|mut w| w.take());
        Start::Inline(taken.map(|work| compute(&work, step)).unwrap_or_default())
    }
}

fn compute(work: &Work, step: u32) -> Batch {
    // A chunk the cut took all of has an empty mesh, so the uncut box is not drawn there.
    let surface =
        |c| mesh::chunk_mesh_lod(&work.stock, c, step).or_else(|| mesh::display_reached(&work.stock, c, step).then(Mesh::default)).map(Surface::new);
    // Stops before the next chunk once the batch is let go of: its result would only be dropped.
    work.chunks.iter().take_while(|_| !work.cancel.load(Ordering::Relaxed)).map(|&c| (c, surface(c))).collect()
}

/// The edges where two faces meet at more than 30 degrees, as end points in the mesh's frame. `key`
/// names an edge by its two vertices, so the same edge of both faces gets the same key. An edge with
/// one face is a boundary and one with three or more is not a plain crease: neither is drawn.
fn crease_edges<K: Eq + Hash>(mesh: &Mesh, key: impl Fn(u32, u32) -> K) -> Vec<[[f32; 3]; 2]> {
    /// An edge's end points and the normals of the faces on it, up to two.
    #[derive(Default)]
    struct Side {
        ends: [[f32; 3]; 2],
        normals: [Option<Vec3>; 2],
        more: bool,
    }
    let mut sides: HashMap<K, Side> = HashMap::new();
    for &[i0, i1, i2] in &mesh.triangles {
        let [Some(a), Some(b), Some(c)] = [i0, i1, i2].map(|i| mesh.positions.get(i as usize).copied()) else { continue };
        // A degenerate triangle has no face to meet another along an edge.
        let Some(normal) = (b - a).cross(c - a).normalized() else { continue };
        for (i, j, p, q) in [(i0, i1, a, b), (i1, i2, b, c), (i2, i0, c, a)] {
            let side = sides.entry(key(i, j)).or_insert_with(|| Side { ends: [p, q].map(Vec3::to_f32), ..Side::default() });
            match side.normals {
                [None, _] => side.normals = [Some(normal), None],
                [first, None] => side.normals = [first, Some(normal)],
                _ => side.more = true,
            }
        }
    }
    // shortcut: creases whose two faces are owned by neighbouring chunks are not found (a one-cell gap where a crease crosses a seam); join seam edges across chunks if it shows.
    sides.into_values().filter(|s| !s.more && matches!(s.normals, [Some(n), Some(m)] if n.dot(m) < CREASE_COS)).map(|s| s.ends).collect()
}

/// An edge of a mesh whose vertices are not shared by index: its two ends' coordinates, in order, so
/// the same edge gets the same key from either of its faces.
fn position_pair(mesh: &Mesh, i: u32, j: u32) -> ([u64; 3], [u64; 3]) {
    // `+ 0.0` makes -0.0 and 0.0 one point.
    let bits = |i: u32| mesh.positions.get(i as usize).map_or([0; 3], |p| [p.x, p.y, p.z].map(|c| (c + 0.0).to_bits()));
    let (p, q) = (bits(i), bits(j));
    if p <= q { (p, q) } else { (q, p) }
}

/// The work frame the stock is drawn in: its origin and X, Y, Z axes in the model.
#[derive(Clone, Copy)]
struct WorkFrame {
    o: [f32; 3],
    x: [f32; 3],
    y: [f32; 3],
    z: [f32; 3],
}

impl WorkFrame {
    /// A point of the work frame, in the model.
    fn place(self, w: [f32; 3]) -> [f32; 3] {
        let d = self.turn(w);
        std::array::from_fn(|k| self.o[k] + d[k])
    }

    /// A direction of the work frame, in the model.
    fn turn(self, w: [f32; 3]) -> [f32; 3] {
        std::array::from_fn(|k| self.x[k] * w[0] + self.y[k] * w[1] + self.z[k] * w[2])
    }

    fn bits(self) -> [[u32; 3]; 4] {
        [self.o, self.x, self.y, self.z].map(|v| v.map(f32::to_bits))
    }
}

fn work_frame(state: &State) -> WorkFrame {
    let [o, x, y, z] = state.frame.unwrap_or([[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
    WorkFrame { o, x, y, z }
}

/// A triangle's corners in the mesh's frame, each with the normal it shades with: the stored vertex
/// normal while it is within 30 degrees of the face's own, else the face's, so a crease shades flat on
/// each side instead of being smoothed across.
fn shaded_corners(mesh: &Mesh, t: [u32; 3]) -> Option<[(Vec3, Vec3); 3]> {
    let [i0, i1, i2] = t;
    let [Some(pa), Some(pb), Some(pc)] = [i0, i1, i2].map(|i| mesh.positions.get(i as usize).copied()) else { return None };
    let face = (pb - pa).cross(pc - pa).normalized();
    let corner = |i: u32, p: Vec3| -> Option<(Vec3, Vec3)> {
        let n = match (mesh.normals.get(i as usize).copied(), face) {
            (Some(s), Some(f)) if s.dot(f) >= CREASE_COS => s,
            (_, Some(f)) => f,
            (s, None) => s?,
        };
        Some((p, n))
    };
    Some([corner(i0, pa)?, corner(i1, pb)?, corner(i2, pc)?])
}

/// The triangles of a cylinder of `diameter` and `length` along `axis` from `tip` (its base disc is at
/// `tip`), as corners of (position, outward normal). Every triangle's winding agrees with its normals.
fn cylinder(tip: Vec3, axis: Vec3, diameter: f64, length: f64) -> Vec<[(Vec3, Vec3); 3]> {
    // The axis, and two unit vectors across it for the circles. (u, v, w) is right-handed.
    let w = axis.normalized().unwrap_or(Vec3::Z);
    let across = if w.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let u = w.cross(across).normalized().unwrap_or(Vec3::X);
    let v = w.cross(u);
    let top = tip + w * length;
    let r = diameter / 2.0;
    let radial = |k: usize| {
        let a = TAU * (k % TOOL_SIDES) as f64 / TOOL_SIDES as f64;
        u * a.cos() + v * a.sin()
    };
    let mut tris: Vec<[(Vec3, Vec3); 3]> = Vec::with_capacity(TOOL_SIDES * 4);
    for k in 0..TOOL_SIDES {
        let (d0, d1) = (radial(k), radial(k + 1));
        let (p0, p1) = (tip + d0 * r, tip + d1 * r);
        let (q0, q1) = (top + d0 * r, top + d1 * r);
        tris.push([(p0, d0), (p1, d1), (q1, d1)]);
        tris.push([(p0, d0), (q1, d1), (q0, d0)]);
        tris.push([(tip, -w), (p1, -w), (p0, -w)]);
        tris.push([(top, w), (q0, w), (q1, w)]);
    }
    tris
}

/// The cutting tool at the playhead, in the work frame: the engine's pose there (its tip, and its axis
/// turned within the move). Its flute is a cylinder of the tool's diameter and flute length along the
/// axis, tip to spindle; ball and bull tools are drawn flat-bottomed too. A tool the program does not
/// list draws nothing.
// shortcut: the shank and the holder are generic cylinders (the shank as wide as the cutter, the holder
// about twice as wide), since a tool carries neither yet. Give them the tool's own dimensions once it does.
fn tool_triangles(app: &SolveApp) -> Vec<[(Vec3, Vec3); 3]> {
    use crate::sim_view::ToolPart;
    let view = &app.sim_stock.view;
    let (Some(program), Some((id, tip, axis))) = (app.session.sim_program(), app.session.sim_tool_pose()) else { return Vec::new() };
    let Some(tool) = program.tools.iter().find(|t| t.id == id) else { return Vec::new() };
    if !view.tool {
        return Vec::new();
    }
    let (tip, axis) = (Vec3::from(tip), Vec3::from(axis));
    let along = |from: Vec3, d: f64| Vec3::new(from.x + axis.x * d, from.y + axis.y * d, from.z + axis.z * d);
    let mut out = cylinder(tip, axis, tool.diameter, tool.flute_length);
    if view.tool_part != ToolPart::Cutting {
        let shank = (2.0 * tool.flute_length).max(25.0);
        let base = along(tip, tool.flute_length);
        out.extend(cylinder(base, axis, tool.diameter, shank));
        if view.tool_part == ToolPart::Holder {
            out.extend(cylinder(along(base, shank), axis, (2.2 * tool.diameter).max(22.0), 40.0));
        }
    }
    out
}

/// What the model scene keys on for the simulation: the stock's meshes and box (`generation`) and the
/// cutting tool's pose, bit for bit (it moves within a move, so a cursor step alone would miss it). Only
/// the model scene draws these, so the sketch and highlight scenes keep their keys across a re-mesh or a
/// playhead step.
pub fn model_key(app: &SolveApp) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    app.sim_stock.generation.hash(&mut h);
    app.session.sim_tool_pose().map(|(id, tip, axis)| (id, tip.map(f64::to_bits), axis.map(f64::to_bits))).hash(&mut h);
    let v = &app.sim_stock.view;
    (v.tool, v.tool_part as u8, v.tool_transparent, v.programmed_point, v.path, v.links, v.retracts, v.cuts, v.connections, v.points).hash(&mut h);
    (v.path_mode as u8, v.stock, v.material as u8, v.colouring as u8, v.stock_transparent, v.spindle_direction).hash(&mut h);
    app.session.sim_progress().hash(&mut h);
    h.finish()
}

/// The key of a surface's block: the surface, the frame it is placed in, and the palette slots and
/// colours its faces and edges take. A block whose key comes back keeps its bytes, and so its buffers.
fn block_key(idx: [usize; 3], surface: &Surface, frame: WorkFrame, colours: [(PaletteIndex, [u8; 4]); 2], class: TriClass, paint: &Paint) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    ("sim-stock", idx, surface.serial, frame.bits(), colours.map(|(i, rgba)| (i.0, rgba)), class as u8, paint.key).hash(&mut h);
    h.finish()
}

/// The colour of each piece of the cut surface, by the move that cut it (a triangle's `tri_face` is that
/// move plus one, 0 for the stock's own faces): one colour per group of moves, the operation or the tool,
/// from the components' identification colours.
struct Paint {
    /// Per move, its group; empty when the stock is one colour.
    group: Vec<u32>,
    /// Per group, its palette slot.
    colours: Vec<PaletteIndex>,
    /// Tells the paints apart in a block's key.
    key: u64,
}

impl Paint {
    fn plain() -> Paint {
        Paint { group: Vec::new(), colours: Vec::new(), key: 0 }
    }

    /// The paint for the View tab's colouring of the program being simulated.
    fn new(sc: &mut GpuScene, app: &SolveApp) -> Paint {
        use crate::sim_view::Colouring;
        let Some(program) = app.session.sim_program() else { return Paint::plain() };
        let group: Vec<u32> = match app.sim_stock.view.colouring {
            Colouring::Material => return Paint::plain(),
            Colouring::Operation => {
                let firsts: Vec<usize> = app.session.sim_sections().iter().map(|s| s.3).collect();
                (0..program.moves.len()).map(|m| firsts.partition_point(|&f| f <= m).saturating_sub(1) as u32).collect()
            }
            Colouring::Tool => program.moves.iter().map(|m| m.tool).collect(),
        };
        let groups = group.iter().max().map_or(0, |g| *g as usize + 1).min(MAX_PAINT_GROUPS);
        let colours = (0..groups)
            .map(|g| {
                let [r, g, b] = solidcraft_engine::doc::component_color::automatic_color(g as u64);
                sc.color([r, g, b, 255])
            })
            .collect();
        let mut h = std::collections::hash_map::DefaultHasher::new();
        (app.sim_stock.view.colouring as u8, &group).hash(&mut h);
        Paint { group, colours, key: h.finish() }
    }

    /// The slot for a triangle with tag `tag`, `base` for the stock's own faces.
    fn colour(&self, tag: u32, base: PaletteIndex) -> PaletteIndex {
        let Some(m) = (tag as usize).checked_sub(1) else { return base };
        self.group.get(m).and_then(|&g| self.colours.get(g as usize)).copied().unwrap_or(base)
    }
}

/// The most groups a colouring tells apart; further operations or tools take the stock's colour.
const MAX_PAINT_GROUPS: usize = 256;

/// A chunk's surface as a block of the model scene: its shaded triangles and crease edges, placed in
/// the model.
fn surface_block(surface: &Surface, frame: WorkFrame, stock: PaletteIndex, edge: PaletteIndex, class: TriClass, paint: &Paint) -> Arc<BlockBytes> {
    let mut block = BlockBuilder::default();
    block.reserve_triangles(surface.mesh.triangles.len());
    for (k, &t) in surface.mesh.triangles.iter().enumerate() {
        let Some(corners) = shaded_corners(&surface.mesh, t) else { continue };
        let p = corners.map(|(p, _)| frame.place(p.to_f32()));
        let n = corners.map(|(_, n)| frame.turn(n.to_f32()));
        let tag = surface.mesh.tri_face.get(k).copied().unwrap_or(0);
        block.triangle(p, n, paint.colour(tag, stock), class);
    }
    for e in &surface.edges {
        block.line(frame.place(e[0]), frame.place(e[1]), edge, EDGE_WIDTH);
    }
    block.freeze()
}

/// The stock colour for a material: the theme's stock colour, scaled per channel.
fn tinted(rgba: [u8; 4], tint: [f32; 3]) -> [u8; 4] {
    let scale = |c: u8, k: f32| (f32::from(c) * k).round().clamp(0.0, 255.0) as u8;
    [scale(rgba[0], tint[0]), scale(rgba[1], tint[1]), scale(rgba[2], tint[2]), rgba[3]]
}

/// Adds the simulation to a model scene: the stock (the cut chunks, each one block that is built once and
/// kept while its key comes back, then the uncut box), the toolpath's lines, the cutting tool and its
/// markers. What shows is what the View tab sets. The stock is opaque so the cut reads, in the material's
/// tint of the token's colour with the body edges' dark edges, unless the View tab makes it see-through.
pub fn draw(sc: &mut GpuScene, app: &SolveApp) {
    let state = &app.sim_stock;
    if state.revision == 0 {
        return;
    }
    let view = &state.view;
    let tk = Tokens::get();
    // The tokens' alpha is for translucent views; the stock is drawn at full alpha.
    let opaque = |c| {
        let [r, g, b, _] = c4(c);
        [r, g, b, 255]
    };
    let frame = work_frame(state);
    if view.stock {
        let class = if view.stock_transparent { TriClass::Glass } else { TriClass::Opaque };
        let paint = Paint::new(sc, app);
        draw_stock(sc, state, frame, tinted(opaque(tk.cam_stock), view.tint()), opaque(tk.body_edge), class, &paint);
    }
    if view.path {
        draw_path(sc, app, frame);
    }
    let tool = sc.color(opaque(tk.body));
    let triangles = tool_triangles(app);
    if view.tool_transparent {
        if !triangles.is_empty() {
            let mut block = BlockBuilder::default();
            block.reserve_triangles(triangles.len());
            for corners in &triangles {
                let p = corners.map(|(p, _)| frame.place(p.to_f32()));
                let n = corners.map(|(_, n)| frame.turn(n.to_f32()));
                block.triangle(p, n, tool, TriClass::Glass);
            }
            let mut h = std::collections::hash_map::DefaultHasher::new();
            let pose = app.session.sim_tool_pose().map(|(i, t, a)| (i, t.map(f64::to_bits), a.map(f64::to_bits)));
            ("sim-tool", triangles.len(), frame.bits(), pose).hash(&mut h);
            sc.blocks.push(SceneBlock { key: h.finish(), bytes: block.freeze() });
        }
    } else {
        for corners in triangles {
            for (p, n) in corners {
                sc.tri(frame.place(p.to_f32()), frame.turn(n.to_f32()), tool);
            }
        }
    }
    draw_markers(sc, app, frame, c4(tk.cam_selected));
}

/// The stock's blocks: the cut chunks, then the uncut box.
fn draw_stock(sc: &mut GpuScene, state: &State, frame: WorkFrame, stock_rgba: [u8; 4], edge_rgba: [u8; 4], class: TriClass, paint: &Paint) {
    let stock = sc.color(stock_rgba);
    let edge = sc.color(edge_rgba);
    let mut cache = state.blocks.lock().unwrap_or_else(|e| e.into_inner());
    for (&idx, surface) in &state.meshes {
        let key = block_key(idx, surface, frame, [(stock, stock_rgba), (edge, edge_rgba)], class, paint);
        let cached = match cache.get(&idx) {
            Some((k, bytes)) if *k == key => Some(bytes.clone()),
            _ => None,
        };
        let bytes = match cached {
            Some(bytes) => bytes,
            None => {
                let bytes = surface_block(surface, frame, stock, edge, class, paint);
                cache.insert(idx, (key, bytes.clone()));
                bytes
            }
        };
        sc.blocks.push(SceneBlock { key, bytes });
    }
    // A chunk no longer drawn (a new revision or display step) leaves the cache.
    cache.retain(|idx, _| state.meshes.contains_key(idx));
    drop(cache);

    // The uncut box is one block too, kept while its generation, the frame and the colours come back: a
    // tool move rebuilds the model scene, and must not tessellate the box again.
    // The uncut box is the stock's own faces only: one colour whatever the paint.
    let plain = Paint::plain();
    let box_key = block_key([usize::MAX; 3], &state.boxed, frame, [(stock, stock_rgba), (edge, edge_rgba)], class, &plain);
    let mut box_cache = state.box_block.lock().unwrap_or_else(|e| e.into_inner());
    let bytes = match &*box_cache {
        Some((k, bytes)) if *k == box_key => bytes.clone(),
        _ => {
            let bytes = surface_block(&state.boxed, frame, stock, edge, class, &plain);
            *box_cache = Some((box_key, bytes.clone()));
            bytes
        }
    };
    drop(box_cache);
    sc.blocks.push(SceneBlock { key: box_key, bytes });
}

/// The toolpath's lines for the display mode: a whole chunk of moves comes from the cache, the chunks
/// the range ends in are built for this frame.
fn draw_path(sc: &mut GpuScene, app: &SolveApp, frame: WorkFrame) {
    use crate::sim_path::{CHUNK, Flags, build, move_range};
    let (Some(program), Some((cursor, total))) = (app.session.sim_program(), app.session.sim_progress()) else { return };
    let view = &app.sim_stock.view;
    let tk = Tokens::get();
    let flags = Flags { links: view.links, retracts: view.retracts, cuts: view.cuts, jumps: view.connections, points: view.points };
    let rgba = [c4(tk.cam_feed), c4(tk.cam_rapid), c4(tk.cam_dimmed)];
    let colours = rgba.map(|c| sc.color(c));
    let time = app.session.sim_time().map_or(0.0, |(t, _)| t);
    let sections = app.session.sim_sections();
    let operation = sections.iter().enumerate().find(|(_, s)| time >= s.1 && time < s.2).map(|(k, s)| {
        let end = sections.get(k + 1).map_or(total, |next| next.3);
        (s.3, end)
    });
    let (from, to) = move_range(view.path_mode, cursor, total, operation);
    if from >= to {
        return;
    }
    let place = |p| frame.place(p);
    let mut cache = app.sim_stock.path_blocks.lock().unwrap_or_else(|e| e.into_inner());
    let mut live = BTreeSet::new();
    for chunk in from / CHUNK..=(to - 1) / CHUNK {
        let range = (chunk * CHUNK, ((chunk + 1) * CHUNK).min(total));
        let part = (range.0.max(from), range.1.min(to));
        let mut h = std::collections::hash_map::DefaultHasher::new();
        ("sim-path", app.session.sim_revision(), chunk, part, flags.bits(), frame.bits(), rgba, colours.map(|c| c.0)).hash(&mut h);
        let key = h.finish();
        let bytes = match cache.get(&chunk) {
            Some((k, bytes)) if *k == key => bytes.clone(),
            _ => {
                let bytes = build(program, part, flags, colours, place);
                // Only a whole chunk is worth keeping: a partial one changes with the playhead.
                if part == range {
                    cache.insert(chunk, (key, bytes.clone()));
                }
                bytes
            }
        };
        if part == range {
            live.insert(chunk);
        }
        sc.blocks.push(SceneBlock { key, bytes });
    }
    cache.retain(|chunk, _| live.contains(chunk));
}

/// The programmed point and the spindle's direction, as lines on the tool.
fn draw_markers(sc: &mut GpuScene, app: &SolveApp, frame: WorkFrame, colour: [u8; 4]) {
    let view = &app.sim_stock.view;
    if !(view.programmed_point || view.spindle_direction) {
        return;
    }
    let (Some(program), Some((id, tip, axis))) = (app.session.sim_program(), app.session.sim_tool_pose()) else { return };
    let Some(tool) = program.tools.iter().find(|t| t.id == id) else { return };
    let spindle = program.moves.iter().rev().find(|m| m.spindle != 0.0).map_or(0.0, |m| m.spindle);
    let c = sc.color(colour);
    let at = |p: [f64; 3]| frame.place([p[0] as f32, p[1] as f32, p[2] as f32]);
    if view.programmed_point {
        let r = (tool.diameter * 0.35).max(0.5);
        for d in 0..3 {
            let (mut a, mut b) = (tip, tip);
            a[d] -= r;
            b[d] += r;
            sc.line(at(a), at(b), c, 2.5, true);
        }
    }
    if view.spindle_direction && spindle != 0.0 {
        // A ring round the tool above its flutes, open at one end where an arrowhead points the way the
        // spindle turns: clockwise seen from above the tool for a positive speed.
        let n = Vec3::from(axis);
        let helper = if n.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
        let Some(u) = helper.cross(n).normalized() else { return };
        let v = n.cross(u);
        let (u, v) = ([u.x, u.y, u.z], [v.x, v.y, v.z]);
        let r = tool.diameter * 0.9;
        let top: [f64; 3] = std::array::from_fn(|k| tip[k] + axis[k] * tool.flute_length * 1.2);
        let sign = if spindle > 0.0 { -1.0 } else { 1.0 };
        let point = |a: f64| -> [f64; 3] { std::array::from_fn(|k| top[k] + r * (a.cos() * u[k] + sign * a.sin() * v[k])) };
        let sweep = TAU * 0.8;
        let steps = 24;
        for s in 0..steps {
            let (a0, a1) = (sweep * s as f64 / steps as f64, sweep * (s + 1) as f64 / steps as f64);
            sc.line(at(point(a0)), at(point(a1)), c, 2.0, true);
        }
        let (head, back) = (point(sweep), point(sweep - 0.35));
        let outward: [f64; 3] = std::array::from_fn(|k| back[k] + (head[k] - top[k]) * 0.25);
        sc.line(at(head), at(back), c, 2.0, true);
        sc.line(at(head), at(outward), c, 2.0, true);
    }
}

/// Waits for the batch being meshed and takes it in (tests only: the window never waits).
#[cfg(test)]
pub fn wait(app: &mut SolveApp) {
    let state = &mut app.sim_stock;
    if let Some(job) = state.job.take()
        && let Ok(batch) = job.rx.recv()
    {
        apply(state, job.revision, job.step, batch);
    }
}

/// Polls and waits until nothing is pending (tests only).
#[cfg(test)]
pub fn settle(app: &mut SolveApp) {
    let ctx = egui::Context::default();
    loop {
        poll(app, &ctx);
        if app.sim_stock.job.is_none() && app.sim_stock.pending.is_empty() {
            break;
        }
        wait(app);
    }
}

/// The chunks with a mesh now, by display index (tests only).
#[cfg(test)]
pub fn meshed_chunks(app: &SolveApp) -> Vec<[usize; 3]> {
    app.sim_stock.meshes.keys().copied().collect()
}

/// The triangles of each chunk's mesh now, by display index (tests only).
#[cfg(test)]
pub fn meshed_triangles(app: &SolveApp) -> BTreeMap<[usize; 3], usize> {
    app.sim_stock.meshes.iter().map(|(idx, s)| (*idx, s.mesh.triangles.len())).collect()
}

/// Chunks meshed so far, over every batch (tests only).
#[cfg(test)]
pub fn meshed_count(app: &SolveApp) -> usize {
    app.sim_stock.meshed
}

/// The cutting tool's triangles as drawn, each corner with its normal (tests only).
#[cfg(test)]
pub fn drawn_tool(app: &SolveApp) -> Vec<[(Vec3, Vec3); 3]> {
    tool_triangles(app)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    /// Two triangles sharing the edge from the origin to X: the first in the plane z = 0, the second
    /// with its apex at `apex`.
    fn pair(apex: Vec3) -> Mesh {
        Mesh { positions: vec![Vec3::ZERO, Vec3::X, Vec3::Y, apex], triangles: vec![[0, 1, 2], [0, 1, 3]], ..Mesh::default() }
    }

    /// The second face bent `deg` degrees away from the first about the shared edge: its normal is
    /// `deg` from the first's.
    fn bent(deg: f64) -> Mesh {
        pair(Vec3::new(0.0, deg.to_radians().cos(), deg.to_radians().sin()))
    }

    #[test]
    fn a_bend_of_20_degrees_is_no_edge() {
        assert!(Surface::new(bent(20.0)).edges.is_empty(), "under 30 degrees is one smooth surface");
    }

    #[test]
    fn a_bend_of_45_degrees_is_one_edge_with_its_ends() {
        let edges = Surface::new(bent(45.0)).edges;
        assert_eq!(edges.len(), 1, "the shared edge; the outer sides are boundaries");
        assert!(edges[0].contains(&[0.0; 3]) && edges[0].contains(&[1.0, 0.0, 0.0]), "its ends are the shared edge's: {:?}", edges[0]);
    }

    #[test]
    fn a_fold_of_90_degrees_is_one_edge() {
        assert_eq!(Surface::new(bent(90.0)).edges.len(), 1);
    }

    #[test]
    fn coplanar_faces_are_no_edge() {
        let flat = Mesh {
            positions: vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::new(1.0, 1.0, 0.0)],
            triangles: vec![[0, 1, 2], [1, 3, 2]],
            ..Mesh::default()
        };
        assert!(Surface::new(flat).edges.is_empty());
    }

    #[test]
    fn three_faces_on_one_edge_are_skipped() {
        // Three triangles share the edge from the origin to X, so it is not a plain crease.
        let mesh = Mesh {
            positions: vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z, Vec3::new(0.0, -1.0, 0.0)],
            triangles: vec![[0, 1, 2], [0, 1, 3], [0, 1, 4]],
            ..Mesh::default()
        };
        assert!(Surface::new(mesh).edges.is_empty());
    }

    #[test]
    fn a_box_with_no_cut_has_its_edges_on_its_outline() {
        let (min, max) = ([0.0f32; 3], [20.0f32, 10.0, 5.0]);
        let stock = Stock::new_box(Vec3::ZERO, Vec3::new(20.0, 10.0, 5.0), 1.0, u64::MAX).expect("a box stock");
        let surface = Surface::by_position(mesh::box_faces(&stock, 1, |_| false));
        let length = |e: &[[f32; 3]; 2]| (0..3).map(|k| f64::from(e[1][k] - e[0][k]).powi(2)).sum::<f64>().sqrt();
        let total: f64 = surface.edges.iter().map(length).sum();
        // The box's twelve edges: four of each length.
        assert!((total - 4.0 * (20.0 + 10.0 + 5.0)).abs() < 1e-3, "the twelve edges' total length, got {total}");
        for e in &surface.edges {
            for p in e {
                let on_outline = (0..3).filter(|&k| p[k] == min[k] || p[k] == max[k]).count() >= 2;
                assert!(on_outline, "an edge end {p:?} is off the box's outline");
            }
        }
    }

    #[test]
    fn the_playback_bar_is_a_share_of_the_view_within_its_limits() {
        let view = |w: f32| egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(w, 600.0));
        let width = |w: f32| bar_rect(view(w), None).map(|r| r.width());
        assert_eq!(width(1000.0), Some(700.0), "70 per cent of the view");
        assert_eq!(width(2000.0), Some(BAR_MAX_WIDTH), "at most 1100");
        assert_eq!(width(600.0), Some(BAR_MIN_WIDTH), "at least 520, and within the view");
    }

    #[test]
    fn the_playback_bar_is_hidden_when_the_view_cannot_hold_the_minimum() {
        let t = Tokens::get();
        let fits = BAR_MIN_WIDTH + 2.0 * t.space_16;
        let view = |w: f32| egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(w, 600.0));
        assert!(bar_rect(view(fits), None).is_some(), "the minimum fits exactly");
        assert!(bar_rect(view(fits - 1.0), None).is_none(), "one point less hides it");
        assert!(bar_rect(view(500.0), None).is_none());
    }

    #[test]
    fn the_playback_bar_sits_centred_just_above_the_navigation_bar() {
        let t = Tokens::get();
        let nav = egui::Rect::from_min_size(egui::pos2(400.0, 560.0), egui::vec2(200.0, 32.0));
        let bar = bar_rect(egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(1000.0, 600.0)), Some(nav)).unwrap_or(nav);
        assert!((bar.bottom() - (560.0 - t.space_8)).abs() < 1e-3, "a gap above the navigation bar");
        assert!((bar.center().x - 500.0).abs() < 1e-3, "centred on the view");
    }

    #[test]
    fn the_operation_buttons_jump_to_the_starts_of_the_operations() {
        let sections = |v: &[(f64, f64)]| v.iter().enumerate().map(|(k, (a, b))| (format!("Op{k}"), *a, *b, k * 10)).collect::<Vec<_>>();
        let s = sections(&[(0.0, 10.0), (10.0, 30.0), (30.0, 60.0)]);
        assert_eq!(operation_jump(&s, 15.0, 60.0, true), 30.0, "next: the start of the next operation");
        assert_eq!(operation_jump(&s, 45.0, 60.0, true), 60.0, "next after the last: the end of the program");
        assert_eq!(operation_jump(&s, 15.0, 60.0, false), 10.0, "previous from inside an operation: its own start");
        assert_eq!(operation_jump(&s, 10.5, 60.0, false), 0.0, "previous from the first second of one: the one before");
        assert_eq!(operation_jump(&s, 0.5, 60.0, false), 0.0, "previous at the very start stays at the start");
        assert_eq!(operation_jump(&[], 5.0, 60.0, true), 60.0, "no operations: next is the end");
    }

    #[test]
    fn the_speed_slider_is_a_log_scale_from_one_to_500() {
        assert!(speed_level(MIN_SPEED).abs() < 1e-12);
        assert!((speed_level(MAX_SPEED) - 1.0).abs() < 1e-12);
        for speed in [1.0, 10.0, 37.5, 500.0] {
            assert!((speed_at(speed_level(speed)) - speed).abs() < 1e-9, "round trip at {speed}");
        }
        assert!((speed_at(0.5) - (MIN_SPEED * MAX_SPEED).sqrt()).abs() < 1e-9, "the middle is the geometric mean");
    }

    #[test]
    fn every_face_of_the_tool_points_outward_on_any_axis() {
        for axis in [Vec3::Z, -Vec3::Z, Vec3::X, Vec3::ZERO] {
            let tris = cylinder(Vec3::new(1.0, 2.0, 3.0), axis, 6.0, 10.0);
            assert_eq!(tris.len(), TOOL_SIDES * 4);
            for [a, b, c] in tris {
                let geometric = (b.0 - a.0).cross(c.0 - a.0);
                for (_, n) in [a, b, c] {
                    assert!(geometric.dot(n) > 0.0, "axis {axis:?}: a triangle faces against its normal");
                }
            }
        }
    }
}
