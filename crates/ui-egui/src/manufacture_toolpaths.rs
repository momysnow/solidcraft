//! The toolpaths of the Manufacture workspace's active setup, drawn in the viewport: cutting moves
//! as solid lines, rapids fainter and dashed, and the operation picked in the Browser on its own.
//!
//! `cam.toolpath` runs on a copy of the design on a worker thread, once for each design and setup
//! (a path can take seconds for an adaptive clear, so the window does not wait for it). The result
//! is kept until the design or the setup changes, and the viewport builds its lines again only
//! when [`key`] does. Until a new result is in, the last one for the same setup stays drawn;
//! nothing is drawn for another setup.

use std::sync::{Arc, mpsc::Receiver, mpsc::TryRecvError};
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

/// How often the window asks again while a setup's toolpaths are being computed.
const POLL: Duration = Duration::from_millis(100);
/// The dash pattern of a rapid, in mm: on, off.
const RAPID_DASH: [f64; 2] = [1.5, 1.0];

/// One operation's path. `points[i]` is where the move ends, and the move is a rapid when the
/// flag says so; a segment runs from `points[i - 1]` to `points[i]`.
#[derive(Clone, Debug, PartialEq)]
pub struct OpPath {
    pub name: String,
    pub points: Vec<(Vec3, bool)>,
}

/// The toolpaths of the active setup, as the Manufacture workspace keeps them.
#[derive(Default)]
pub struct Toolpaths {
    /// What the paths are wanted for: the design (by identity) and the setup.
    want: Option<(Arc<Document>, String)>,
    /// The computation in progress for `want`.
    job: Option<Job>,
    /// The setup the paths belong to, and the paths.
    shown: Option<(String, Arc<Vec<OpPath>>)>,
    /// Counts the changes to what is drawn.
    generation: u64,
}

struct Job {
    setup: String,
    rx: Receiver<Vec<OpPath>>,
}

/// Which strokes take which colour.
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

/// One straight piece of a toolpath to draw.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stroke {
    pub a: Vec3,
    pub b: Vec3,
    pub rapid: bool,
    pub role: Role,
}

/// Starts the active setup's toolpaths when they are wanted and not computed, and takes in a
/// finished computation. Called every frame; it does nothing while Manufacture is not active.
pub fn poll(app: &mut SolveApp, ctx: &egui::Context) {
    let want = wanted(app);
    let changed = match (&want, &app.manufacture_workspace.toolpaths.want) {
        (Some((doc, setup)), Some((held, held_setup))) => !Arc::ptr_eq(doc, held) || setup != held_setup,
        (None, None) => false,
        _ => true,
    };
    if changed {
        let job = want.as_ref().and_then(|(_, setup)| start(app.session.scratch(), setup.clone()));
        let state = &mut app.manufacture_workspace.toolpaths;
        state.job = job;
        state.want = want;
        // Paths of another setup, or of a design that is gone, are not drawn.
        let same_setup = matches!((&state.shown, &state.want), (Some((shown, _)), Some((_, setup))) if shown == setup);
        if !same_setup && state.shown.take().is_some() {
            state.generation += 1;
        }
    }
    let state = &mut app.manufacture_workspace.toolpaths;
    let finished = state.job.as_ref().map(|job| (job.setup.clone(), job.rx.try_recv()));
    match finished {
        None => {}
        Some((setup, Ok(paths))) => {
            state.job = None;
            state.shown = Some((setup, Arc::new(paths)));
            state.generation += 1;
        }
        Some((_, Err(TryRecvError::Empty))) => ctx.request_repaint_after(POLL),
        Some((_, Err(TryRecvError::Disconnected))) => state.job = None,
    }
}

/// The active setup, with the design it is in, when Manufacture is active and the setup exists.
fn wanted(app: &SolveApp) -> Option<(Arc<Document>, String)> {
    if !crate::manufacture_workspace::active(app) {
        return None;
    }
    let name = app.manufacture_workspace.active_setup.as_str();
    app.session.doc.cam.setup(name).map(|_| (app.session.doc.clone(), name.to_string()))
}

/// Computes `setup`'s toolpaths on a worker thread. A failed command gives no paths.
fn start(mut copy: Session, setup: String) -> Option<Job> {
    let (tx, rx) = std::sync::mpsc::channel();
    let job_setup = setup.clone();
    let spawned = std::thread::Builder::new().name("cam-toolpaths".into()).spawn(move || {
        let paths = copy.execute("cam.toolpath", &json!({"setup": setup})).map(|reply| parse(&reply)).unwrap_or_default();
        let _ = tx.send(paths);
    });
    spawned.ok().map(|_| Job { setup: job_setup, rx })
}

/// The operations of a `cam.toolpath` reply. An operation without points (it did not build) is
/// kept with none.
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
            Some(OpPath { name, points })
        })
        .collect()
}

/// What the viewport's sketch scene depends on here: the design's setup, its paths and the pick.
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

/// The strokes of the shown toolpaths of the active setup, each with the role it is drawn in.
pub fn strokes(app: &SolveApp) -> Vec<Stroke> {
    let state = &app.manufacture_workspace;
    if !crate::manufacture_workspace::active(app) {
        return Vec::new();
    }
    let Some((setup, paths)) = &state.toolpaths.shown else {
        return Vec::new();
    };
    if *setup != state.active_setup {
        return Vec::new();
    }
    let picked = state.selected_op.as_deref();
    let mut out = Vec::new();
    for op in paths.iter() {
        for pair in op.points.windows(2) {
            let ((a, _), (b, rapid)) = (pair[0], pair[1]);
            let role = match picked {
                None if rapid => Role::Rapid,
                None => Role::Feed,
                Some(name) if name == op.name => Role::Selected,
                Some(_) => Role::Dimmed,
            };
            out.push(Stroke { a, b, rapid, role });
        }
    }
    out
}

/// Adds the shown toolpaths to a sketch scene: cutting moves solid, rapids dashed.
pub fn draw(sc: &mut GpuScene, app: &SolveApp) {
    let strokes = strokes(app);
    if strokes.is_empty() {
        return;
    }
    let t = Tokens::get();
    let feed = sc.color(c4(t.cam_feed));
    let rapid = sc.color(c4(t.cam_rapid));
    let selected = sc.color(c4(t.cam_selected));
    let dimmed = sc.color(c4(t.cam_dimmed));
    for s in strokes {
        let colour = match s.role {
            Role::Feed => feed,
            Role::Rapid => rapid,
            Role::Selected => selected,
            Role::Dimmed => dimmed,
        };
        if s.rapid {
            for (a, b) in dashes(&[s.a, s.b], &RAPID_DASH) {
                sc.line(a.to_f32(), b.to_f32(), colour, 1.0, true);
            }
        } else {
            sc.line(s.a.to_f32(), s.b.to_f32(), colour, 1.6, true);
        }
    }
}

/// Whether a computation is in progress (tests only).
#[cfg(test)]
pub fn computing(app: &SolveApp) -> bool {
    app.manufacture_workspace.toolpaths.job.is_some()
}

/// Waits for the computation in progress and takes it in (tests only: the window never waits).
#[cfg(test)]
pub fn wait(app: &mut SolveApp) {
    let state = &mut app.manufacture_workspace.toolpaths;
    if let Some(job) = state.job.take()
        && let Ok(paths) = job.rx.recv()
    {
        state.shown = Some((job.setup, Arc::new(paths)));
        state.generation += 1;
    }
}
