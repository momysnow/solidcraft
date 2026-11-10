//! Stock-removal simulation of G-code on a box stock (S1). `sim.setup` loads a program, `sim.run`
//! cuts it forward or back, `sim.status` reports it and `sim.clear` drops it. The state is the
//! session's runtime state (`sim_jobs.rs`), not the design: none of these is an undo step, and none
//! reads the evaluated model.

use std::time::Duration;

use serde::Deserialize;
use serde_json::Value;
use solidcore_record::Vec3;
use solidcraft_io::MeshQuality;
use solidcraft_machining::SimError;
use solidcraft_machining::accuracy::{self, Accuracy};
use solidcraft_machining::dexel::Stock;
use solidcraft_machining::gcode::{self, CodeTable};
use solidcraft_machining::sweep::FlatTool;
use solidcraft_machining::toolpath::{Diag, MoveKind, NO_TOOL, Severity, Shape, Tool};
use web_time::Instant;

use super::CommandSpec;
use crate::params::bad;
use crate::schema::sim::{
    AccuracyParams, AccuracyReply, BoundsReply, ClearParams, ClearReply, DiagReply, GridReply, MemoryReply, MoveTarget, PartialReply, RunParams,
    RunReply, SetupParams, SetupReply, ShapeName, SpotReply, StatusParams, StatusReply, ToolParams, ToolPoseReply,
};
use crate::schema::{TypedContract, output_schema_for};
use crate::sim_jobs::{Job, Setup};
use crate::{EngineError, Result, Session};

const SETUP: TypedContract<SetupParams> = TypedContract::new();
const RUN: TypedContract<RunParams> = TypedContract::new();
const STATUS: TypedContract<StatusParams> = TypedContract::new();
const CLEAR: TypedContract<ClearParams> = TypedContract::new();
const ACCURACY: TypedContract<AccuracyParams> = TypedContract::new();

/// The memory cap a setup gets when it names none, in MB.
const DEFAULT_MEMORY_MB: u64 = 512;
/// The rapid rate a setup gets when it names none, mm/min.
const DEFAULT_RAPID_MM_MIN: f64 = 5000.0;
/// The largest cap a setup may name, in MB: the web build has about 4 GB of address space.
const MAX_MEMORY_MB: u64 = 1024;
const MAX_TOOLS: usize = 64;
/// Ray spacings a default resolution is picked from, finest first (mm).
const RESOLUTIONS: [f64; 10] = [0.05, 0.1, 0.2, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0];
/// Problems `sim.setup` lists; the reply says how many there are in all.
const SETUP_DIAGS: usize = 200;
const DEFAULT_LIMIT: u64 = 100;
const MAX_LIMIT: u64 = 1000;
/// A run's budget is capped at an hour, so its deadline cannot overflow.
const MAX_BUDGET_S: f64 = 3600.0;
const NO_SIM: &str = "no simulation: run sim.setup first";

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("sim.setup", "Set Up Simulation", crate::checked!("sim.setup", setup))
        .contract(&SETUP)
        .output(output_schema_for::<SetupReply>)
        .noundo()
        .doc_only()
        .params("stock: {box: {min: [x,y,z], max: [x,y,z]}}, mm in the workpiece frame; tools: [{id, diameter, flute_length, shape?: flat|ball|bull (cut) or cone|taper|profile (reported, not cut), plus the fields that shape needs: bull takes corner_radius, above 0 and at most the tool radius (equal to it is a ball)}], mm; gcode: text for a generic 3-axis machine; initial_tool?: the tool the moves before the first M6 cut with; resolution?: ray spacing h in mm (default: the finest of 0.05 to 20 mm whose stock fits half of memory_mb); memory_mb?: cap from 1 to 1024 (default 512); rapid_mm_min?: rapid feed in mm/min, above 0 (default 5000), used for rapids and for moves with no feed. Parses the G-code, builds the stock uncut and resets the cursor and the playhead to 0, replacing any earlier simulation. Each move takes its path length over its feed; an arc takes its own arc length. A move with no tool, or a tool not listed, is a diagnostic. Replies moves, bounds, h, memory_estimate {bytes, budget}, diag_count and diagnostics [{line, col, severity, msg}] (the first 200). Bad or too large input is refused and nothing changes. Nothing is cut yet: see sim.run."),
    CommandSpec::new("sim.run", "Run Simulation", crate::checked!("sim.run", run))
        .contract(&RUN)
        .output(output_schema_for::<RunReply>)
        .noundo()
        .doc_only()
        .params("to?: move index, or \"end\" (default); past the last move is an error. time?: playhead in seconds of machine time, from 0, clamped to the program's total_time; cuts the whole moves that end by then and the steps of the move in progress that are done by then. to and time together are an error. budget_s?: wall-clock seconds; the run stops after the first step past it, with reached false, and always cuts at least one step. Cuts forward, or goes back from the nearest checkpoint and cuts forward again. Replies at (moves completed), total, reached (the to or time was reached), done (at is total), removed_volume (mm^3), revision (changes when the stock is rebuilt from the setup), partial ({step, steps} of the move in progress), time (the playhead: the time asked for when reached, else where the cut got to) and total_time (s). Errors without sim.setup, and forward once a step has failed (see sim.status)."),
    CommandSpec::new("sim.status", "Simulation Status", crate::checked!("sim.status", status))
        .contract(&STATUS)
        .output(output_schema_for::<StatusReply>)
        .noundo()
        .doc_only()
        .params("offset?: first diagnostic to list (default 0); limit?: most to list (default 100, at most 1000). Read only. Replies cursor, total, stock_volume, remaining_volume, removed_volume (mm^3), memory {bytes, budget}, diag_count, error (why the run stopped, or null), revision, partial, time and total_time (the playhead and the program's machine time, s), tool ({id, tip: [x,y,z], axis: [x,y,z]}: the tool at the playhead, its tip along the move it is on, or null without moves or when that move has no tool) and the page of diagnostics. Errors without sim.setup."),
    CommandSpec::new("sim.clear", "Clear Simulation", crate::checked!("sim.clear", clear))
        .contract(&CLEAR)
        .output(output_schema_for::<ClearReply>)
        .noundo()
        .doc_only()
        .params("No parameters. Drops the simulation and its stock. Replies cleared: whether there was one."),
    CommandSpec::new("sim.accuracy", "Simulation Accuracy", crate::checked!("sim.accuracy", accuracy))
        .contract(&ACCURACY)
        .output(output_schema_for::<AccuracyReply>)
        .noundo()
        .params("body: the name of a body in the design. Compares the stock as the simulation has cut it so far with that body's placed tessellation, in the program's workpiece frame (no work-offset mapping): the export's Fine quality, with a chord no more than 0.05 of the grid spacing h (replied as chord_tolerance, mm). The stock's surface is sampled where its three grids' rays enter and leave material; each sample is a deviation of its distance to the part: a sample inside the part is a gouge (cut too deep), one outside a left-over (cut too thin). Points between the rays are not sampled, so features narrower than the resolution may be missed. Replies rays and odd_rays (rays skipped because the mesh crosses them an odd number of times), resolution (the grid's largest cell, mm), max_left_over and max_gouge (mm, the largest of the grids' figures, with worst_left_over and worst_gouge as {at: [x,y,z], depth}: the stock surface point of the deepest sample), mean_abs_dev and rms (mm, area-weighted over the part's surface), cursor and total (moves cut so far, and in the program), chord_tolerance (mm) and grids: each grid's own figures in x, y, z order. Read only. Errors without sim.setup, with no such body, or when the part is open, degenerate, non-finite or too large for the stock's memory."),
];

/// Reads the parameters into their contract's type. Unknown keys are the contract's business
/// (`checked!`), so a missing or badly typed key is the only refusal here.
fn parse<'de, T: Deserialize<'de>>(command: &str, p: &'de Value) -> Result<T> {
    T::deserialize(p).map_err(|e| bad(command, e.to_string()))
}

fn sim_error(command: &str, e: SimError) -> EngineError {
    bad(command, e.to_string())
}

fn count(n: usize) -> u64 {
    u64::try_from(n).unwrap_or(u64::MAX)
}

fn diag_reply(d: &Diag) -> DiagReply {
    let severity = match d.severity {
        Severity::Error => "error",
        Severity::Warning => "warning",
    };
    DiagReply { line: d.line, col: d.col, severity: severity.into(), msg: d.msg.clone() }
}

fn memory_reply(job: &Job) -> MemoryReply {
    let (bytes, budget) = job.memory();
    MemoryReply { bytes, budget }
}

fn partial_reply(job: &Job) -> Option<PartialReply> {
    job.partial().map(|(step, steps)| PartialReply { step: count(step), steps: count(steps) })
}

/// The finest spacing of [`RESOLUTIONS`] whose stock fits half of `budget`, so the cuts have room to
/// split rays. A spacing too fine for this box is skipped; when none fits, the error says how much
/// the coarsest one needs.
fn default_resolution(min: Vec3, max: Vec3, budget: u64) -> std::result::Result<f64, SimError> {
    let mut coarsest_need = None;
    for h in RESOLUTIONS {
        if let Ok(need) = Stock::box_bytes(min, max, h) {
            if need <= budget / 2 {
                return Ok(h);
            }
            coarsest_need = Some(need);
        }
    }
    match coarsest_need {
        Some(need) => Err(SimError::Budget { need, budget }),
        None => Err(Stock::box_bytes(min, max, RESOLUTIONS[RESOLUTIONS.len() - 1])
            .err()
            .unwrap_or_else(|| SimError::BadInput("no ray spacing fits this box".into()))),
    }
}

/// The cutting shape a tool names, with the fields that shape needs. A field that belongs to another
/// shape is refused, so a misspelt or misplaced field does not pass silently.
fn shape_of(t: &ToolParams) -> std::result::Result<Shape, String> {
    let id = t.id;
    let name = t.shape.unwrap_or_default();
    let given = [
        ("corner_radius", t.corner_radius.is_some_and(|r| r != 0.0)),
        ("tip_diameter", t.tip_diameter.is_some()),
        ("angle", t.angle.is_some()),
        ("a0", t.a0.is_some()),
        ("r0", t.r0.is_some()),
        ("profile", t.profile.is_some()),
    ];
    let wanted: &[&str] = match name {
        ShapeName::Flat | ShapeName::Ball => &[],
        ShapeName::Bull => &["corner_radius"],
        ShapeName::Cone => &["tip_diameter", "angle"],
        ShapeName::Taper => &["a0", "r0", "angle"],
        ShapeName::Profile => &["profile"],
    };
    if let Some((key, _)) = given.iter().find(|(key, set)| *set && !wanted.contains(key)) {
        return Err(format!("tool {id}: {key} does not apply to its shape"));
    }
    let length = |v: Option<f64>, key: &str| {
        v.filter(|x| x.is_finite() && *x >= 0.0).ok_or_else(|| format!("tool {id} needs {key}, a length in mm (zero or more)"))
    };
    let half_angle = || {
        t.angle
            .filter(|a| a.is_finite() && *a > 0.0 && *a < std::f64::consts::FRAC_PI_2)
            .ok_or_else(|| format!("tool {id} needs angle, in radians from 0 to pi/2"))
    };
    match name {
        ShapeName::Flat => Ok(Shape::Flat),
        ShapeName::Ball => Ok(Shape::Ball),
        ShapeName::Bull => {
            // A corner radius above 0 and at most the tool radius: at the radius the corner is a ball, which is
            // what the bull-nose becomes (a corner of 0 is a flat end, which takes no corner radius).
            let radius = t.diameter / 2.0;
            let rc = t
                .corner_radius
                .filter(|r| r.is_finite() && *r > 0.0 && *r <= radius)
                .ok_or_else(|| format!("tool {id}: bull corner_radius must be above 0 and at most the tool radius {radius} mm"))?;
            Ok(if rc == radius { Shape::Ball } else { Shape::Bull { rc } })
        }
        ShapeName::Cone => Ok(Shape::Cone { tip_d: length(t.tip_diameter, "tip_diameter")?, angle: half_angle()? }),
        ShapeName::Taper => Ok(Shape::Taper { a0: length(t.a0, "a0")?, r0: length(t.r0, "r0")?, angle: half_angle()? }),
        ShapeName::Profile => {
            let points = t.profile.clone().unwrap_or_default();
            if points.len() < 2 || !points.iter().all(|p| p.iter().all(|x| x.is_finite() && *x >= 0.0)) {
                return Err(format!("tool {id} needs profile: at least two [a, r] pairs, zero or more mm"));
            }
            Ok(Shape::Profile(points))
        }
    }
}

fn setup(s: &mut Session, p: &Value) -> Result<Value> {
    const C: &str = "sim.setup";
    let input: SetupParams = parse(C, p)?;
    let memory_mb = input.memory_mb.unwrap_or(DEFAULT_MEMORY_MB);
    if !(1..=MAX_MEMORY_MB).contains(&memory_mb) {
        return Err(bad(C, format!("memory_mb must be from 1 to {MAX_MEMORY_MB}")));
    }
    let budget_bytes = memory_mb << 20;
    if input.gcode.len() > gcode::MAX_INPUT_BYTES {
        return Err(bad(C, format!("gcode is over {} bytes", gcode::MAX_INPUT_BYTES)));
    }
    if input.tools.len() > MAX_TOOLS {
        return Err(bad(C, format!("at most {MAX_TOOLS} tools")));
    }
    let mut tools: Vec<Tool> = Vec::with_capacity(input.tools.len());
    for t in &input.tools {
        if tools.iter().any(|o| o.id == t.id) {
            return Err(bad(C, format!("tool {} is listed twice", t.id)));
        }
        let shape = shape_of(t).map_err(|e| bad(C, e))?;
        if FlatTool::new(t.diameter / 2.0, t.flute_length, Vec3::Z).is_none() {
            return Err(bad(C, format!("tool {} needs a positive diameter and flute length in mm", t.id)));
        }
        tools.push(Tool {
            id: t.id,
            shape,
            diameter: t.diameter,
            flute_length: t.flute_length,
            shank: Vec::new(),
            holder: Vec::new(),
            stickout: 0.0,
        });
    }
    let mut program = gcode::parse(&input.gcode, &CodeTable::default());
    // Moves before the first tool change take the initial tool, when the caller names one. Otherwise
    // the simulator reports them. A tool a move cuts with and the list lacks is reported per move too.
    if let Some(initial) = input.initial_tool {
        for m in &mut program.moves {
            if m.tool == NO_TOOL {
                m.tool = initial;
            }
        }
    }
    // Cutting moves with no tool are not simulated: an error, with the count, so the caller sees it.
    let cutting = |m: &solidcraft_machining::toolpath::Move| m.tool == NO_TOOL && matches!(m.kind, MoveKind::Feed | MoveKind::Arc { .. });
    let uncut = program.moves.iter().filter(|m| cutting(m)).count();
    if let Some(first) = program.moves.iter().find(|m| cutting(m)) {
        let msg = if uncut == 1 {
            "1 cutting move before the first M6 is not simulated: give initial_tool or a tool change".to_string()
        } else {
            format!("{uncut} cutting moves before the first M6 are not simulated: give initial_tool or a tool change")
        };
        program.diags.push(Diag::error(first.source, 0, msg));
    }
    program.tools = tools;
    let (min, max) = (Vec3::from(input.stock.boxed.min), Vec3::from(input.stock.boxed.max));
    let h = match input.resolution {
        Some(h) if h.is_finite() && h > 0.0 => h,
        Some(_) => return Err(bad(C, "resolution must be a positive length in mm")),
        None => default_resolution(min, max, budget_bytes).map_err(|e| sim_error(C, e))?,
    };
    let rapid_mm_min = match input.rapid_mm_min {
        Some(r) if r.is_finite() && r > 0.0 => r,
        Some(_) => return Err(bad(C, "rapid_mm_min must be a positive feed in mm/min")),
        None => DEFAULT_RAPID_MM_MIN,
    };
    let setup = Setup { min, max, h, budget_bytes, rapid_mm_min, program };
    let revision = crate::sim_jobs::next_revision();
    let job = Job::new(setup, revision).map_err(|e| sim_error(C, e))?;
    let (min, max) = job.bounds();
    let reply = SetupReply {
        moves: count(job.total()),
        bounds: BoundsReply { min: min.into(), max: max.into() },
        h: job.h(),
        memory_estimate: memory_reply(&job),
        diag_count: count(job.diagnostics().count()),
        diagnostics: job.diagnostics().take(SETUP_DIAGS).map(diag_reply).collect(),
    };
    s.sim.set(job);
    serde_json::to_value(reply).map_err(|e| bad(C, e.to_string()))
}

fn run(s: &mut Session, p: &Value) -> Result<Value> {
    const C: &str = "sim.run";
    let input: RunParams = parse(C, p)?;
    let job = s.sim.job_mut().ok_or_else(|| bad(C, NO_SIM))?;
    let total = job.total();
    if input.to.is_some() && input.time.is_some() {
        return Err(bad(C, "give to or time, not both"));
    }
    if let Some(t) = input.time
        && !(t.is_finite() && t >= 0.0)
    {
        return Err(bad(C, "time must be zero or more seconds"));
    }
    let to = match input.to {
        None | Some(MoveTarget::End(_)) => total,
        Some(MoveTarget::Index(i)) => {
            let i = usize::try_from(i).unwrap_or(usize::MAX);
            if i > total {
                return Err(bad(C, format!("to {i} is past the last move ({total})")));
            }
            i
        }
    };
    let until = match input.budget_s {
        None => None,
        Some(b) if b.is_finite() && b >= 0.0 => Some(Instant::now() + Duration::from_secs_f64(b.min(MAX_BUDGET_S))),
        Some(_) => return Err(bad(C, "budget_s must be zero or more seconds")),
    };
    let reached = match input.time {
        Some(t) => job.run_at(t, until),
        None => job.run_to(to, until),
    }
    .map_err(|e| sim_error(C, e))?;
    let at = job.cursor();
    let reply = RunReply {
        at: count(at),
        total: count(total),
        reached,
        done: at == total,
        removed_volume: job.removed_volume(),
        revision: job.revision(),
        partial: partial_reply(job),
        time: job.playhead(),
        total_time: job.total_time(),
    };
    serde_json::to_value(reply).map_err(|e| bad(C, e.to_string()))
}

fn status(s: &mut Session, p: &Value) -> Result<Value> {
    const C: &str = "sim.status";
    let input: StatusParams = parse(C, p)?;
    let job = s.sim.job().ok_or_else(|| bad(C, NO_SIM))?;
    let offset = usize::try_from(input.offset.unwrap_or(0)).unwrap_or(usize::MAX);
    let limit = usize::try_from(input.limit.unwrap_or(DEFAULT_LIMIT).min(MAX_LIMIT)).unwrap_or(usize::MAX);
    let reply = StatusReply {
        cursor: count(job.cursor()),
        total: count(job.total()),
        stock_volume: job.start_volume(),
        remaining_volume: job.remaining_volume(),
        removed_volume: job.removed_volume(),
        memory: memory_reply(job),
        diag_count: count(job.diagnostics().count()),
        error: job.error().map(|e| e.to_string()),
        revision: job.revision(),
        partial: partial_reply(job),
        time: job.playhead(),
        total_time: job.total_time(),
        tool: job.tool_pose().map(|(id, tip, axis)| ToolPoseReply { id, tip, axis }),
        diagnostics: job.diagnostics().skip(offset).take(limit).map(diag_reply).collect(),
    };
    serde_json::to_value(reply).map_err(|e| bad(C, e.to_string()))
}

fn clear(s: &mut Session, p: &Value) -> Result<Value> {
    const C: &str = "sim.clear";
    parse::<ClearParams>(C, p)?;
    serde_json::to_value(ClearReply { cleared: s.sim.clear() }).map_err(|e| bad(C, e.to_string()))
}

/// Compares the stock with a body's tessellation (see the command's text). Read only.
fn accuracy(s: &mut Session, p: &Value) -> Result<Value> {
    const C: &str = "sim.accuracy";
    // serde reads a one-field struct from a one-element array too: the parameters must be an object.
    if !p.is_object() {
        return Err(bad(C, "parameters must be an object with a body name"));
    }
    let input: AccuracyParams = parse(C, p)?;
    let (cursor, total) = s.sim_progress().ok_or_else(|| bad(C, NO_SIM))?;
    let h = s.sim.job().map(|j| j.h()).ok_or_else(|| bad(C, NO_SIM))?;
    let st = s.world_state()?;
    let body = st.body(&input.body).ok_or_else(|| bad(C, format!("no body `{}`", input.body)))?;
    // The Fine export's chord, but no coarser than the grid resolves: a facet may stand off by 0.05 h at most.
    let chord = solidcraft_io::quality_chord(&body.body, MeshQuality::Fine).min(0.05 * h);
    let mesh = solidcraft_io::export_mesh_chord(&body.body, chord).map_err(|e| bad(C, e.to_string()))?;
    let stock = s.sim_stock().ok_or_else(|| bad(C, NO_SIM))?;
    let a = accuracy::compare(stock, &mesh.positions, &mesh.triangles).map_err(|e| sim_error(C, e))?;
    serde_json::to_value(accuracy_reply(&input.body, cursor, total, chord, &a)).map_err(|e| bad(C, e.to_string()))
}

fn accuracy_reply(body: &str, cursor: usize, total: usize, chord: f64, a: &Accuracy) -> AccuracyReply {
    let spot = |s: &accuracy::Spot| SpotReply { at: s.at.into(), depth: s.depth };
    let grid = |g: &accuracy::GridStats| GridReply {
        axis: ["x", "y", "z"].get(g.axis).copied().unwrap_or("?").to_string(),
        max_left_over: g.max_left_over,
        max_gouge: g.max_gouge,
        mean_abs_dev: g.mean_abs_dev,
        rms: g.rms,
    };
    AccuracyReply {
        body: body.to_string(),
        cursor: count(cursor),
        total: count(total),
        rays: a.rays,
        odd_rays: a.odd_rays,
        chord_tolerance: chord,
        resolution: a.resolution,
        max_left_over: a.max_left_over,
        worst_left_over: a.worst_left_over.as_ref().map(spot),
        max_gouge: a.max_gouge,
        worst_gouge: a.worst_gouge.as_ref().map(spot),
        mean_abs_dev: a.mean_abs_dev,
        rms: a.rms,
        grids: a.grids.iter().map(grid).collect(),
    }
}
