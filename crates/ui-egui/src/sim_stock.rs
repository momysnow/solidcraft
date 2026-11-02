//! The stock of the G-code simulation (`sim.setup`, `sim.run`) in the viewport: the uncut box, and the
//! surface of each chunk a cut has reached (`solidcraft_machining::mesh`), opaque with its crease edges
//! dark, as a STEP model's body edges. The cutting tool stands at the tip of the move the cursor has
//! reached.
//!
//! Only the chunks the stock marks dirty are meshed, on a worker thread (or inline where there is none,
//! as on wasm), one batch at a time, like the toolpaths. A batch made for an older stock, revision or
//! display step is dropped. The display step coarsens as the cut grows (`mesh::display_step`); a step
//! change remeshes the whole cut once. The uncut box is rebuilt for the chunks that have no mesh.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::f64::consts::TAU;
use std::hash::{Hash, Hasher};
use std::sync::{Arc, Mutex, PoisonError, mpsc::Receiver, mpsc::TryRecvError};
use std::time::Duration;

use serde_json::json;
use solidcraft_engine::geom::{Mesh, Vec3};
use solidcraft_machining::dexel::Stock;
use solidcraft_machining::mesh;

use crate::gpu::{BlockBuilder, BlockBytes, GpuScene, PaletteIndex, SceneBlock, TriClass};
use crate::{SolveApp, i18n::tr, theme::Tokens, viewport::c4};

/// How often the window asks again while a batch is being meshed.
const POLL: Duration = Duration::from_millis(100);
/// How long one frame may play the simulation, in seconds: the window stays responsive meanwhile.
const PLAY_SLICE_S: f64 = 0.03;
/// The cosine of 30 degrees: faces that turn further than this from each other meet at a crease, so
/// their edge is drawn and each side keeps its own shading.
const CREASE_COS: f64 = 0.866;
/// The width of the crease edges: the body edges' (`viewport.rs`), so the stock reads as the model.
const EDGE_WIDTH: f32 = 1.3;
/// The sides of the cutting tool's cylinder: round at the size it is drawn.
const TOOL_SIDES: usize = 24;

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
#[derive(Default)]
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
    /// The uncut box, for the chunks without a mesh, built for `box_for`.
    boxed: Surface,
    box_for: Option<u64>,
    /// Whether the simulation plays: each frame runs it on for a short slice.
    playing: bool,
    /// The work frame the stock is in, as origin and X, Y, Z axes in the model; `None` is the model's.
    /// shortcut: kept from the last Simulate only, so a `sim.setup` sent another way draws in the frame
    /// of the last Simulate; take it from the simulation when it records one.
    frame: Option<[[f32; 3]; 4]>,
    /// Chunks meshed so far (tests only).
    #[cfg(test)]
    meshed: usize,
}

struct Job {
    revision: u64,
    step: u32,
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

/// Runs the simulation on for one slice while it plays, so a long program takes many frames and the
/// window never waits for it.
fn advance(app: &mut SolveApp, ctx: &egui::Context) {
    if !app.sim_stock.playing {
        return;
    }
    // Cleared elsewhere (a `sim.clear` command, a new setup) while it played: stop quietly, no error.
    if app.session.sim_progress().is_none() {
        app.sim_stock.playing = false;
        return;
    }
    match app.session.execute("sim.run", &json!({"to": "end", "budget_s": PLAY_SLICE_S})) {
        Ok(reply) if reply["done"].as_bool() == Some(true) => app.sim_stock.playing = false,
        Ok(_) => ctx.request_repaint(),
        Err(e) => {
            app.sim_stock.playing = false;
            app.set_status(e.to_string(), true);
        }
    }
}

/// The play, pause and clear controls and the progress of the simulation, in the Manufacture panel.
pub fn panel(app: &mut SolveApp, ui: &mut egui::Ui) {
    let Some((cursor, total)) = app.session.sim_progress() else { return };
    ui.horizontal(|ui| {
        let playing = app.sim_stock.playing;
        if ui.button(if playing { tr("sim.stock.pause") } else { tr("sim.stock.play") }).clicked() {
            if playing {
                app.sim_stock.playing = false;
            } else {
                // A finished run plays again from the start.
                let restart = if cursor >= total { app.session.execute("sim.run", &json!({"to": 0})).map(|_| ()) } else { Ok(()) };
                match restart {
                    Ok(()) => app.sim_stock.playing = true,
                    Err(e) => app.set_status(e.to_string(), true),
                }
            }
        }
        let (cursor, total) = (cursor.to_string(), total.to_string());
        ui.label(crate::i18n::format("sim.stock.progress", &[("cursor", &cursor), ("total", &total)]));
        if ui.button(tr("sim.stock.clear")).clicked() {
            app.sim_stock.playing = false;
            if let Err(e) = app.session.execute("sim.clear", &json!({})) {
                app.set_status(e.to_string(), true);
            }
        }
    });
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
            state.changed();
        }
    }

    if let Some(job) = state.job.take() {
        match job.rx.try_recv() {
            Ok(batch) => apply(state, job.revision, job.step, batch),
            Err(TryRecvError::Empty) => state.job = Some(job),
            Err(TryRecvError::Disconnected) => {}
        }
    }

    if state.job.is_none() && !state.pending.is_empty() {
        let work = Work { stock: stock.clone(), chunks: std::mem::take(&mut state.pending).into_iter().collect() };
        let (revision, step) = (state.revision, state.step);
        match start(work, revision, step) {
            Start::Thread(job) => state.job = Some(job),
            Start::Inline(batch) => apply(state, revision, step, batch),
        }
    }

    if state.box_for != Some(state.generation) {
        let meshes = &state.meshes;
        let mesh = mesh::box_faces(stock, state.step, |idx| meshes.contains_key(&idx));
        // The box is rebuilt on this thread, a few hundred quads: its crease edges are found here too.
        state.boxed = Surface::by_position(mesh);
        state.box_for = Some(state.generation);
    }

    if state.job.is_some() || !state.pending.is_empty() {
        ctx.request_repaint_after(POLL);
    }
}

/// Drops the cached blocks along with the meshes they were built from. A block kept past a Clear would
/// hold its bytes (and GPU buffers) for nothing, since `draw` returns early without a simulation.
fn forget_blocks(state: &mut State) {
    state.blocks.get_mut().unwrap_or_else(PoisonError::into_inner).clear();
}

/// Takes a batch in, if it was made for the stock and step the state holds now.
fn apply(state: &mut State, revision: u64, step: u32, batch: Batch) {
    if revision != state.revision || step != state.step {
        return;
    }
    for (idx, surface) in batch {
        #[cfg(test)]
        {
            state.meshed += 1;
        }
        match surface {
            Some(mut surface) => {
                state.serials += 1;
                surface.serial = state.serials;
                state.meshes.insert(idx, surface);
            }
            None => {
                state.meshes.remove(&idx);
            }
        }
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
        let slot = Arc::new(Mutex::new(Some(work)));
        let worker = Arc::clone(&slot);
        let spawned = std::thread::Builder::new().name("cam-sim-stock".into()).spawn(move || {
            let taken = worker.lock().ok().and_then(|mut w| w.take());
            if let Some(work) = taken {
                let _ = tx.send(compute(&work, step));
            }
        });
        if spawned.is_ok() {
            return Start::Thread(Job { revision, step, rx });
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
    work.chunks.iter().map(|&c| (c, surface(c))).collect()
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

/// The cutting tool at the tip of the move the cursor has reached (the move before it, or the first
/// move before any cut), in the work frame. Its flute is a cylinder of the tool's diameter and flute
/// length along the move's axis, tip to spindle; ball and bull tools are drawn flat-bottomed too. A
/// tool the program does not list draws nothing.
// shortcut: no shank drawn, since the engine sends an empty shank and a zero stickout; draw the shank
// above the flute once it sends them.
fn tool_triangles(app: &SolveApp) -> Vec<[(Vec3, Vec3); 3]> {
    let (Some(program), Some((cursor, _))) = (app.session.sim_program(), app.session.sim_progress()) else { return Vec::new() };
    let Some(mv) = program.moves.get(cursor.saturating_sub(1)) else { return Vec::new() };
    let Some(tool) = program.tools.iter().find(|t| t.id == mv.tool) else { return Vec::new() };
    cylinder(Vec3::from(mv.tip), Vec3::from(mv.axis), tool.diameter, tool.flute_length)
}

/// What the model scene keys on for the simulation: the stock's meshes and box (`generation`) and the
/// cursor the cutting tool stands at. Only the model scene draws these, so the sketch and highlight
/// scenes keep their keys across a re-mesh or a cursor step.
pub fn model_key(app: &SolveApp) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    app.sim_stock.generation.hash(&mut h);
    app.session.sim_progress().hash(&mut h);
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

    for &t in &state.boxed.mesh.triangles {
        if let Some(corners) = shaded_corners(&state.boxed.mesh, t) {
            for (p, n) in corners {
                sc.tri(frame.place(p.to_f32()), frame.turn(n.to_f32()), stock);
            }
        }
    }
    for e in &state.boxed.edges {
        sc.line(frame.place(e[0]), frame.place(e[1]), edge, EDGE_WIDTH, false);
    }
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

/// Chunks meshed so far, over every batch (tests only).
#[cfg(test)]
pub fn meshed_count(app: &SolveApp) -> usize {
    app.sim_stock.meshed
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
