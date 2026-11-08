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
use solidcraft_engine::geom::{Mesh, Vec3};
use solidcraft_machining::dexel::Stock;
use solidcraft_machining::mesh;

use crate::gpu::{BlockBuilder, BlockBytes, GpuScene, PaletteIndex, SceneBlock, TriClass};
use crate::{SolveApp, i18n::tr, theme::Tokens, ui_kit, viewport::c4};

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
const SPEED_WIDTH: f32 = 90.0;

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
    crate::manufacture_workspace::active(app) && app.session.sim_revision() != 0
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

/// Minutes and seconds, as `01:05`, for the playhead and the program's length.
fn clock(seconds: f64) -> String {
    let s = seconds.max(0.0) as u64;
    format!("{:02}:{:02}", s / 60, s % 60)
}

/// Clears the simulation from the playback bar's Clear button: stops playback and drops the stock.
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
    let height = t.control_height + 2.0 * t.space_4;
    let bottom = nav.map_or(view.bottom() - t.space_8, |nav| nav.top() - t.space_8);
    Some(egui::Rect::from_center_size(egui::pos2(view.center().x, bottom - height / 2.0), egui::vec2(width, height)))
}

/// Which optional controls the row shows, for `inner` points: the move count, then the speed control,
/// go first when the timeline would keep less than half the row. `core` is what the row always shows,
/// the timeline excepted. `None` when even the core leaves the timeline under half: the bar is hidden.
fn row_items(inner: f32, core: f32, moves: f32, speed: f32) -> Option<(bool, bool)> {
    [(true, true), (false, true), (false, false)].into_iter().find(|&(moves_on, speed_on)| {
        let used = core + if moves_on { moves } else { 0.0 } + if speed_on { speed } else { 0.0 };
        inner - used >= inner * 0.5
    })
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

/// The play or pause control: History's transport glyph, with its hover fill and a hand cursor.
fn transport(ui: &mut egui::Ui, size: f32, playing: bool) -> egui::Response {
    let t = Tokens::get();
    let label = tr(if playing { "sim.stock.pause" } else { "sim.stock.play" });
    let (rect, response) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::click());
    if response.hovered() {
        ui.painter().rect_filled(rect, t.radius_sm, t.hover);
    }
    crate::icons::glyphs::transport_glyph(ui.painter(), rect.center(), if playing { "pause" } else { "play" }, t.icon);
    response.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, ui.is_enabled(), label));
    response.on_hover_cursor(egui::CursorIcon::PointingHand).on_hover_text(label)
}

/// The width of `text` in `font`, as the painter lays it out.
fn text_width(ui: &egui::Ui, text: &str, font: egui::FontId) -> f32 {
    ui.painter().layout_no_wrap(text.to_owned(), font, Tokens::get().text).size().x
}

/// The bar's rectangle as last drawn over the view, for the overlays that keep clear of it.
pub fn playback_rect(app: &SolveApp) -> Option<egui::Rect> {
    app.sim_stock.bar
}

/// The playback bar over the 3D viewport, one row: play or pause, the elapsed time, the timeline, the
/// program's length, the move count, the speed and Clear. Shown while Manufacture shows a simulation,
/// and only where it fits (`row_items`). Its widgets claim the pointer over the bar, so the camera and
/// picking beneath get no clicks from it.
pub fn playback(app: &mut SolveApp, ui: &mut egui::Ui, viewport: egui::Rect) {
    app.sim_stock.bar = None;
    if !crate::manufacture_workspace::active(app) {
        return;
    }
    let (Some((cursor, total)), Some((time, total_time))) = (app.session.sim_progress(), app.session.sim_time()) else { return };
    let view = crate::shell::bottom_overlay_rect(app, crate::shell::content_rect(app, viewport));
    let nav = app.viewport.layout.as_ref().map(|l| l.navigation).filter(|n| {
        n.is_positive() && n.is_finite() && app.ui.show_navigation_bar && app.preferences.navigation_bar.edge == crate::navigation_bar::Edge::Bottom
    });
    let Some(rect) = bar_rect(view, nav).filter(|r| r.is_finite() && r.is_positive()) else { return };
    let t = Tokens::get();
    let h = t.control_height;
    let inner = rect.width() - 2.0 * t.space_8;
    let gap = ui.spacing().item_spacing.x;
    let (cursor, total) = (cursor.to_string(), total.to_string());
    let moves = crate::i18n::format("sim.stock.progress", &[("cursor", &cursor), ("total", &total)]);
    let (elapsed, length) = (clock(time), clock(total_time));
    let speed = format!("{:.0}{}", app.sim_stock.speed, tr("sim.stock.speed_suffix"));
    let elapsed_w = text_width(ui, &elapsed, egui::FontId::monospace(t.font_label));
    let clear_w = text_width(ui, tr("sim.stock.clear"), egui::TextStyle::Small.resolve(ui.style())) + 2.0 * ui.spacing().button_padding.x;
    let core = h + elapsed_w + text_width(ui, &length, egui::FontId::monospace(t.font_small)) + clear_w + 4.0 * gap;
    let moves_w = text_width(ui, &moves, egui::FontId::proportional(t.font_small)) + 2.0 * gap;
    let speed_w = text_width(ui, &speed, egui::FontId::proportional(t.font_small)) + SPEED_WIDTH + 2.0 * gap;
    let Some((moves_on, speed_on)) = row_items(inner, core, moves_w, speed_w) else { return };

    ui_kit::playback_surface(ui, rect, app.preferences.navigation_bar.size);
    // Claimed before the row's widgets, so they stay on top of it and the viewport gets no press on the bar.
    ui.interact(rect, ui.id().with("sim_playback"), egui::Sense::click_and_drag());
    app.sim_stock.bar = Some(rect);

    let playing = app.sim_stock.playing;
    let mut level = speed_level(app.sim_stock.speed);
    // While a scrub is on its way the timeline holds the target, not the playhead that lags it.
    let mut shown = app.sim_stock.scrub.unwrap_or(time);
    let (mut toggle, mut wipe) = (false, false);
    let row = egui::Rect::from_center_size(rect.center(), egui::vec2(inner, h));
    // The row is laid out from the right, so the timeline takes the width the controls on its left leave.
    ui.scope_builder(egui::UiBuilder::new().max_rect(row).layout(egui::Layout::right_to_left(egui::Align::Center)), |ui| {
        ui.set_clip_rect(rect);
        wipe = ui_kit::button(ui, tr("sim.stock.clear"), ui_kit::Button::Small).clicked();
        if speed_on {
            ui.label(egui::RichText::new(speed).size(t.font_small).color(t.text_dim));
            ui.allocate_ui_with_layout(egui::vec2(SPEED_WIDTH, h), egui::Layout::left_to_right(egui::Align::Center), |ui| {
                let slider = ui_kit::slider(ui, &mut level, 0.0..=1.0, ui_kit::Slider { show_value: false, ..Default::default() });
                if slider.on_hover_text(tr("sim.stock.speed")).changed() {
                    app.sim_stock.speed = speed_at(level);
                }
            });
        }
        if moves_on {
            ui.label(egui::RichText::new(moves).size(t.font_small).color(t.text_dim));
        }
        ui.label(egui::RichText::new(length).monospace().size(t.font_small).color(t.text_dim));
        let width = (ui.available_width() - h - elapsed_w - 2.0 * gap).max(0.0);
        if total_time > 0.0 {
            ui.allocate_ui_with_layout(egui::vec2(width, h), egui::Layout::left_to_right(egui::Align::Center), |ui| {
                let timeline = ui_kit::slider(ui, &mut shown, 0.0..=total_time, ui_kit::Slider { show_value: false, ..Default::default() });
                if timeline.on_hover_text(tr("sim.stock.playhead")).changed() {
                    seek(app, shown);
                }
            });
        } else {
            ui.allocate_exact_size(egui::vec2(width, h), egui::Sense::hover());
        }
        ui.label(egui::RichText::new(elapsed).monospace().size(t.font_label).color(t.text));
        toggle = transport(ui, h, playing).clicked();
    });
    if toggle {
        if playing {
            app.sim_stock.playing = false;
        } else {
            play(app);
        }
    }
    if wipe {
        clear(app);
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
// shortcut: no shank drawn, since the engine sends an empty shank and a zero stickout; draw the shank
// above the flute once it sends them.
fn tool_triangles(app: &SolveApp) -> Vec<[(Vec3, Vec3); 3]> {
    let (Some(program), Some((id, tip, axis))) = (app.session.sim_program(), app.session.sim_tool_pose()) else { return Vec::new() };
    let Some(tool) = program.tools.iter().find(|t| t.id == id) else { return Vec::new() };
    cylinder(Vec3::from(tip), Vec3::from(axis), tool.diameter, tool.flute_length)
}

/// What the model scene keys on for the simulation: the stock's meshes and box (`generation`) and the
/// cutting tool's pose, bit for bit (it moves within a move, so a cursor step alone would miss it). Only
/// the model scene draws these, so the sketch and highlight scenes keep their keys across a re-mesh or a
/// playhead step.
pub fn model_key(app: &SolveApp) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    app.sim_stock.generation.hash(&mut h);
    app.session.sim_tool_pose().map(|(id, tip, axis)| (id, tip.map(f64::to_bits), axis.map(f64::to_bits))).hash(&mut h);
    h.finish()
}

/// The key of a surface's block: the surface, the frame it is placed in, and the palette slots and
/// colours its faces and edges take. A block whose key comes back keeps its bytes, and so its buffers.
fn block_key(idx: [usize; 3], surface: &Surface, frame: WorkFrame, colours: [(PaletteIndex, [u8; 4]); 2]) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    ("sim-stock", idx, surface.serial, frame.bits(), colours.map(|(i, rgba)| (i.0, rgba))).hash(&mut h);
    h.finish()
}

/// A chunk's surface as a block of the model scene: its shaded triangles and crease edges, placed in
/// the model.
fn surface_block(surface: &Surface, frame: WorkFrame, stock: PaletteIndex, edge: PaletteIndex) -> Arc<BlockBytes> {
    let mut block = BlockBuilder::default();
    block.reserve_triangles(surface.mesh.triangles.len());
    for &t in &surface.mesh.triangles {
        let Some(corners) = shaded_corners(&surface.mesh, t) else { continue };
        let p = corners.map(|(p, _)| frame.place(p.to_f32()));
        let n = corners.map(|(_, n)| frame.turn(n.to_f32()));
        block.triangle(p, n, stock, TriClass::Opaque);
    }
    for e in &surface.edges {
        block.line(frame.place(e[0]), frame.place(e[1]), edge, EDGE_WIDTH);
    }
    block.freeze()
}

/// Adds the simulation's stock to a model scene: the cut chunks, each one block that is built once and
/// kept while its key comes back (a cursor step uploads nothing); then the uncut box and the cutting
/// tool, loose and rebuilt each frame as they are small. Opaque so the cut reads, in the token's colour
/// with the body edges' dark edges, and the tool in the body's colour.
pub fn draw(sc: &mut GpuScene, app: &SolveApp) {
    let state = &app.sim_stock;
    if state.revision == 0 {
        return;
    }
    let tk = Tokens::get();
    // The tokens' alpha is for translucent views; the stock is drawn at full alpha.
    let opaque = |c| {
        let [r, g, b, _] = c4(c);
        [r, g, b, 255]
    };
    let stock_rgba = opaque(tk.cam_stock);
    let edge_rgba = opaque(tk.body_edge);
    let stock = sc.color(stock_rgba);
    let edge = sc.color(edge_rgba);
    let frame = work_frame(state);
    let mut cache = state.blocks.lock().unwrap_or_else(|e| e.into_inner());
    for (&idx, surface) in &state.meshes {
        let key = block_key(idx, surface, frame, [(stock, stock_rgba), (edge, edge_rgba)]);
        let cached = match cache.get(&idx) {
            Some((k, bytes)) if *k == key => Some(bytes.clone()),
            _ => None,
        };
        let bytes = match cached {
            Some(bytes) => bytes,
            None => {
                let bytes = surface_block(surface, frame, stock, edge);
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
    let box_key = block_key([usize::MAX; 3], &state.boxed, frame, [(stock, stock_rgba), (edge, edge_rgba)]);
    let mut box_cache = state.box_block.lock().unwrap_or_else(|e| e.into_inner());
    let bytes = match &*box_cache {
        Some((k, bytes)) if *k == box_key => bytes.clone(),
        _ => {
            let bytes = surface_block(&state.boxed, frame, stock, edge);
            *box_cache = Some((box_key, bytes.clone()));
            bytes
        }
    };
    drop(box_cache);
    sc.blocks.push(SceneBlock { key: box_key, bytes });
    let tool = sc.color(opaque(tk.body));
    for corners in tool_triangles(app) {
        for (p, n) in corners {
            sc.tri(frame.place(p.to_f32()), frame.turn(n.to_f32()), tool);
        }
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
    fn the_timeline_keeps_half_the_row_and_the_optional_controls_go_first() {
        // (inner, core, moves, speed) and the controls shown: all of them while the timeline keeps half.
        assert_eq!(row_items(1000.0, 200.0, 100.0, 150.0), Some((true, true)), "room for all");
        assert_eq!(row_items(1000.0, 300.0, 100.0, 150.0), Some((false, true)), "the move count goes first");
        assert_eq!(row_items(1000.0, 400.0, 100.0, 150.0), Some((false, false)), "then the speed control");
        assert_eq!(row_items(1000.0, 500.0, 100.0, 150.0), Some((false, false)), "exactly half still fits");
        assert_eq!(row_items(1000.0, 600.0, 100.0, 150.0), None, "the core alone leaves less than half: hidden");
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
