//! The toolpaths of the Manufacture workspace's active setup, drawn in the viewport: cutting moves
//! as solid lines, rapids fainter and dashed, and the operation picked in the Browser on its own.
//!
//! `cam.toolpath` runs on a copy of the design, once for each design and setup, after the design has
//! been still for [`DEBOUNCE_MS`]. At most one computation runs at a time: a design changed meanwhile
//! waits for it and then computes its own. It runs on a worker thread, or inline where there is none
//! (wasm, or a thread that cannot be started). The result is kept until the setup changes; the strokes
//! to draw are rebuilt only when the result or the pick does. Until a new result is in, the last one for
//! the same setup stays drawn, and nothing is drawn for another setup.

use std::sync::{Arc, Mutex, Weak, mpsc::Receiver, mpsc::TryRecvError};
use std::time::Duration;

use serde_json::{Value, json};
use solidcraft_engine::Session;
use solidcraft_engine::doc::Document;
use solidcraft_engine::geom::Vec3;

use crate::{
    SolveApp,
    gpu::GpuScene,
    theme::Tokens,
    viewport::{c4, dashes},
};

/// How often the window asks again while a computation runs or waits.
const POLL: Duration = Duration::from_millis(100);
/// The design must stay unchanged this long (ms) before its toolpaths are computed.
const DEBOUNCE_MS: f64 = 150.0;
/// The arc chord tolerance the window asks for (mm): coarser than the command's default, for drawing.
const CHORD_MM: f64 = 0.05;
/// The dash pattern of a rapid, in mm: on, off.
const RAPID_DASH: [f64; 2] = [1.5, 1.0];

/// One operation's path. `points[i]` is where a move ends; the move is a rapid when its flag says so,
/// and a segment runs from `points[i - 1]` to `points[i]`. An operation that did not build has no
/// points and says why in `error`.
#[derive(Clone, Debug, PartialEq)]
pub struct OpPath {
    pub name: String,
    pub points: Vec<(Vec3, bool)>,
    pub error: Option<String>,
}

/// A design and a setup. The design is held weakly, so holding it does not make an edit copy it.
type Want = (Weak<Document>, String);

/// The toolpaths of the active setup, as the Manufacture workspace keeps them.
#[derive(Default)]
pub struct Toolpaths {
    /// Wanted now: the design and the active setup, when Manufacture is on and the setup exists.
    want: Option<Want>,
    /// When `want` last changed (ms): a computation starts once it has been still for `DEBOUNCE_MS`.
    want_since: f64,
    /// The one computation running.
    job: Option<Job>,
    /// What the last computation was started for.
    computed: Option<Want>,
    /// The setup the paths belong to, and the paths.
    shown: Option<(String, Arc<Vec<OpPath>>)>,
    /// Counts the changes to `shown`.
    generation: u64,
    /// The strokes to draw, built for `built_for`: (generation, pick, active setup).
    strokes: Arc<Vec<Stroke>>,
    built_for: Option<(u64, Option<String>, String)>,
    /// Computations started, for the throttle tests.
    #[cfg(test)]
    started: usize,
}

struct Job {
    setup: String,
    rx: Receiver<Vec<OpPath>>,
}

/// How a computation was run: on a thread that reports later, or at once.
enum Start {
    Thread(Job),
    Inline(Vec<OpPath>),
}

/// Which colour a stroke is drawn in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    /// A cutting move, with nothing picked.
    Feed,
    /// A rapid, with nothing picked.
    Rapid,
    /// A move of the operation picked in the Browser.
    Selected,
    /// A move of an operation other than the picked one.
    Dimmed,
}

/// One straight piece of a toolpath to draw (a rapid is already cut into dashes).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stroke {
    pub a: Vec3,
    pub b: Vec3,
    pub rapid: bool,
    pub role: Role,
}

/// Starts or collects the active setup's toolpaths. Called every frame.
pub fn poll(app: &mut SolveApp, ctx: &egui::Context) {
    poll_at(app, ctx, crate::now_ms());
}

/// [`poll`] at time `now` (ms).
pub(crate) fn poll_at(app: &mut SolveApp, ctx: &egui::Context, now: f64) {
    // A pick the setup no longer has (the operation was deleted, or its setup replaced) is let go.
    if let Some(name) = app.manufacture_workspace.selected_op.clone() {
        let has = app.session.doc.cam.setup(&app.manufacture_workspace.active_setup).is_some_and(|s| s.operations.iter().any(|o| o.name() == name));
        if !has {
            app.manufacture_workspace.selected_op = None;
        }
    }
    let want = wanted(app);
    let pick = app.manufacture_workspace.selected_op.clone();
    let active_setup = app.manufacture_workspace.active_setup.clone();
    let state = &mut app.manufacture_workspace.toolpaths;

    if !same(&want, &state.want) {
        state.want = want.clone();
        state.want_since = now;
    }
    let wanted_setup = want.as_ref().map(|(_, setup)| setup.as_str());
    if state.want.is_none() {
        state.computed = None;
    }
    if state.shown.as_ref().is_some_and(|(setup, _)| Some(setup.as_str()) != wanted_setup) {
        state.shown = None;
        state.generation += 1;
    }

    // The computation running takes its result in, when it has one.
    let finished = state.job.as_ref().map(|job| (job.setup.clone(), job.rx.try_recv()));
    match finished {
        None => {}
        Some((setup, Ok(paths))) => {
            state.job = None;
            if Some(setup.as_str()) == wanted_setup {
                state.shown = Some((setup, Arc::new(paths)));
                state.generation += 1;
            }
        }
        Some((_, Err(TryRecvError::Empty))) => {}
        Some((_, Err(TryRecvError::Disconnected))) => state.job = None,
    }

    // The wanted setup is computed once its design has been still for a moment, and only one at a time.
    let due = state.job.is_none() && state.want.is_some() && !same(&state.want, &state.computed) && now - state.want_since >= DEBOUNCE_MS;
    if due && let Some((doc, setup)) = state.want.clone() {
        let copy = app.session.scratch();
        state.computed = Some((doc, setup.clone()));
        #[cfg(test)]
        {
            state.started += 1;
        }
        match start(copy, setup.clone()) {
            Start::Thread(job) => state.job = Some(job),
            Start::Inline(paths) => {
                state.shown = Some((setup, Arc::new(paths)));
                state.generation += 1;
            }
        }
    }

    // The strokes are rebuilt when the paths or the pick change.
    let key = (state.generation, pick.clone(), active_setup.clone());
    if state.built_for.as_ref() != Some(&key) {
        state.strokes = Arc::new(shown_strokes(state.shown.as_ref(), &active_setup, pick.as_deref()));
        state.built_for = Some(key);
    }

    if state.job.is_some() || (state.want.is_some() && !same(&state.want, &state.computed)) {
        ctx.request_repaint_after(POLL);
    }
}

/// The active setup, with the design it is in, when Manufacture is active and the setup exists.
fn wanted(app: &SolveApp) -> Option<Want> {
    if !crate::manufacture_workspace::active(app) {
        return None;
    }
    let name = app.manufacture_workspace.active_setup.as_str();
    app.session.doc.cam.setup(name).map(|_| (Arc::downgrade(&app.session.doc), name.to_string()))
}

/// Whether two wants are the same design and setup.
fn same(a: &Option<Want>, b: &Option<Want>) -> bool {
    match (a, b) {
        (Some((da, sa)), Some((db, sb))) => Weak::ptr_eq(da, db) && sa == sb,
        (None, None) => true,
        _ => false,
    }
}

/// Computes `setup`'s toolpaths on a worker thread, or inline where there is none.
fn start(copy: Session, setup: String) -> Start {
    #[cfg(target_arch = "wasm32")]
    return Start::Inline(compute(copy, &setup));
    #[cfg(not(target_arch = "wasm32"))]
    {
        let (tx, rx) = std::sync::mpsc::channel();
        let slot = Arc::new(Mutex::new(Some(copy)));
        let worker = Arc::clone(&slot);
        let (job_setup, thread_setup) = (setup.clone(), setup.clone());
        let spawned = std::thread::Builder::new().name("cam-toolpaths".into()).spawn(move || {
            let taken = worker.lock().ok().and_then(|mut copy| copy.take());
            if let Some(copy) = taken {
                let _ = tx.send(compute(copy, &thread_setup));
            }
        });
        if spawned.is_ok() {
            return Start::Thread(Job { setup: job_setup, rx });
        }
        // No thread could be started: compute here, from the copy the thread never took.
        let copy = slot.lock().ok().and_then(|mut copy| copy.take());
        Start::Inline(copy.map(|copy| compute(copy, &setup)).unwrap_or_default())
    }
}

/// The paths of `setup`, from a `cam.toolpath` reply at the window's chord tolerance. A failed command
/// gives no paths.
fn compute(mut copy: Session, setup: &str) -> Vec<OpPath> {
    copy.execute("cam.toolpath", &json!({"setup": setup, "tolerance": CHORD_MM})).map(|reply| parse(&reply)).unwrap_or_default()
}

/// The operations of a `cam.toolpath` reply. An operation that did not build is kept with no points.
fn parse(reply: &Value) -> Vec<OpPath> {
    let Some(ops) = reply.get("operations").and_then(Value::as_array) else {
        return Vec::new();
    };
    ops.iter()
        .filter_map(|op| {
            let name = op.get("name")?.as_str()?.to_string();
            let points = op.get("points")?.as_array()?;
            let kinds = op.get("kinds")?.as_array()?;
            let points = points
                .iter()
                .zip(kinds)
                .map(|(p, kind)| {
                    let c = p.as_array()?;
                    let at = Vec3::new(c.first()?.as_f64()?, c.get(1)?.as_f64()?, c.get(2)?.as_f64()?);
                    Some((at, kind.as_str() == Some("rapid")))
                })
                .collect::<Option<Vec<_>>>()?;
            let error = op.get("error").and_then(Value::as_str).map(str::to_string);
            Some(OpPath { name, points, error })
        })
        .collect()
}

/// The strokes for `setup`'s shown paths. A pick the setup does not have picks nothing.
fn shown_strokes(shown: Option<&(String, Arc<Vec<OpPath>>)>, setup: &str, pick: Option<&str>) -> Vec<Stroke> {
    let Some((shown_setup, paths)) = shown else {
        return Vec::new();
    };
    if shown_setup != setup {
        return Vec::new();
    }
    let pick = pick.filter(|name| paths.iter().any(|op| op.name == *name));
    let mut out = Vec::new();
    for op in paths.iter() {
        for pair in op.points.windows(2) {
            let ((a, _), (b, rapid)) = (pair[0], pair[1]);
            let role = match pick {
                None if rapid => Role::Rapid,
                None => Role::Feed,
                Some(name) if name == op.name => Role::Selected,
                Some(_) => Role::Dimmed,
            };
            if rapid {
                out.extend(dashes(&[a, b], &RAPID_DASH).into_iter().map(|(a, b)| Stroke { a, b, rapid, role }));
            } else {
                out.push(Stroke { a, b, rapid, role });
            }
        }
    }
    out
}

/// What the viewport's model scene depends on here: the setup, its paths and the pick.
pub fn key(app: &SolveApp) -> u64 {
    use std::hash::{Hash, Hasher};
    let state = &app.manufacture_workspace;
    let mut h = std::collections::hash_map::DefaultHasher::new();
    crate::manufacture_workspace::active(app).hash(&mut h);
    state.active_setup.hash(&mut h);
    state.toolpaths.generation.hash(&mut h);
    state.selected_op.hash(&mut h);
    h.finish()
}

/// The strokes drawn now (empty when Manufacture is off).
pub fn strokes(app: &SolveApp) -> Vec<Stroke> {
    if !crate::manufacture_workspace::active(app) {
        return Vec::new();
    }
    app.manufacture_workspace.toolpaths.strokes.to_vec()
}

/// The error a `cam.toolpath` gave for `op` of `setup`, when its paths are the ones shown.
pub fn error_of(app: &SolveApp, setup: &str, op: &str) -> Option<String> {
    let (shown_setup, paths) = app.manufacture_workspace.toolpaths.shown.as_ref()?;
    if shown_setup != setup {
        return None;
    }
    paths.iter().find(|p| p.name == op).and_then(|p| p.error.clone())
}

/// Adds the drawn strokes to a model scene: cutting moves solid, rapids dashed.
pub fn draw(sc: &mut GpuScene, app: &SolveApp) {
    if !crate::manufacture_workspace::active(app) {
        return;
    }
    let strokes = &app.manufacture_workspace.toolpaths.strokes;
    if strokes.is_empty() {
        return;
    }
    let t = Tokens::get();
    let feed = sc.color(c4(t.cam_feed));
    let rapid = sc.color(c4(t.cam_rapid));
    let selected = sc.color(c4(t.cam_selected));
    let dimmed = sc.color(c4(t.cam_dimmed));
    for s in strokes.iter() {
        let colour = match s.role {
            Role::Feed => feed,
            Role::Rapid => rapid,
            Role::Selected => selected,
            Role::Dimmed => dimmed,
        };
        let width = if s.rapid { 1.0 } else { 1.6 };
        sc.line(s.a.to_f32(), s.b.to_f32(), colour, width, true);
    }
}

/// Whether a computation is running (tests only).
#[cfg(test)]
pub fn computing(app: &SolveApp) -> bool {
    app.manufacture_workspace.toolpaths.job.is_some()
}

/// Computations started so far (tests only).
#[cfg(test)]
pub fn started(app: &SolveApp) -> usize {
    app.manufacture_workspace.toolpaths.started
}

/// Waits for the computation running and takes it in, then starts any wanted one that waited
/// (tests only: the window never waits, and does not wait out the debounce).
#[cfg(test)]
pub fn wait(app: &mut SolveApp) {
    let ctx = egui::Context::default();
    app.manufacture_workspace.toolpaths.want_since = f64::MIN;
    poll_at(app, &ctx, 0.0);
    let state = &mut app.manufacture_workspace.toolpaths;
    if let Some(job) = state.job.take()
        && let Ok(paths) = job.rx.recv()
        && Some(job.setup.as_str()) == state.want.as_ref().map(|(_, s)| s.as_str())
    {
        state.shown = Some((job.setup, Arc::new(paths)));
        state.generation += 1;
    }
    state.built_for = None;
    poll_at(app, &ctx, 0.0);
}
