//! Stock-removal simulation of G-code on a box stock (S1). `sim.setup` loads a program, `sim.run`
//! cuts it forward or back, `sim.status` reports it and `sim.clear` drops it. The state is the
//! session's runtime state (`sim_jobs.rs`), not the design: none of these is an undo step.

use std::collections::BTreeSet;
use std::time::Duration;

use serde::Deserialize;
use serde_json::Value;
use solidcraft_geom::Vec3;
use solidcraft_machining::SimError;
use solidcraft_machining::dexel::Stock;
use solidcraft_machining::gcode::{self, CodeTable};
use solidcraft_machining::sweep::FlatTool;
use solidcraft_machining::toolpath::{Diag, Severity, Shape, Tool};
use web_time::Instant;

use super::CommandSpec;
use crate::params::bad;
use crate::schema::sim::{
    BoundsReply, ClearParams, ClearReply, DiagReply, MemoryReply, MoveTarget, RunParams, RunReply, SetupParams, SetupReply, StatusParams, StatusReply,
};
use crate::schema::{TypedContract, output_schema_for};
use crate::sim_jobs::{Job, Setup};
use crate::{EngineError, Result, Session};

const SETUP: TypedContract<SetupParams> = TypedContract::new();
const RUN: TypedContract<RunParams> = TypedContract::new();
const STATUS: TypedContract<StatusParams> = TypedContract::new();
const CLEAR: TypedContract<ClearParams> = TypedContract::new();

/// The memory cap a setup gets when it names none, in MB.
const DEFAULT_MEMORY_MB: u64 = 1024;
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
        .params("stock: {box: {min: [x,y,z], max: [x,y,z]}}, mm in the workpiece frame; tools: [{id, diameter, flute_length, shape?: \"flat\"}], flat end mills in mm, one for each T the G-code cuts with and no others; gcode: text for a generic 3-axis machine; resolution?: ray spacing h in mm (default: the finest of 0.05 to 20 mm that fits memory_mb); memory_mb?: cap from 1 to 1024 (default 1024). Parses the G-code, builds the stock uncut and resets the cursor to 0, replacing any earlier simulation. Replies moves, bounds, h, memory_estimate {bytes, budget}, diag_count and diagnostics [{line, col, severity, msg}] (the first 200). Bad or too large input is refused and nothing changes. Nothing is cut yet: see sim.run."),
    CommandSpec::new("sim.run", "Run Simulation", crate::checked!("sim.run", run))
        .contract(&RUN)
        .output(output_schema_for::<RunReply>)
        .noundo()
        .params("to?: move index, or \"end\" (default); past the last move is an error. budget_s?: wall-clock seconds; the run stops after the move that passes it, with done false. Cuts forward, or rebuilds the stock from the start and cuts back to `to`. Replies at, total, done and removed_volume (mm^3). Errors without sim.setup."),
    CommandSpec::new("sim.status", "Simulation Status", crate::checked!("sim.status", status))
        .contract(&STATUS)
        .output(output_schema_for::<StatusReply>)
        .noundo()
        .doc_only()
        .params("offset?: first diagnostic to list (default 0); limit?: most to list (default 100, at most 1000). Read only. Replies cursor, total, stock_volume, remaining_volume, removed_volume (mm^3), memory {bytes, budget}, diag_count and the page of diagnostics. Errors without sim.setup."),
    CommandSpec::new("sim.clear", "Clear Simulation", crate::checked!("sim.clear", clear))
        .contract(&CLEAR)
        .output(output_schema_for::<ClearReply>)
        .noundo()
        .params("No parameters. Drops the simulation and its stock. Replies cleared: whether there was one."),
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

/// The finest spacing of [`RESOLUTIONS`] whose stock fits `budget`. A spacing too fine for this box
/// is skipped; when none fits, the error says how much the coarsest one needs.
fn default_resolution(min: Vec3, max: Vec3, budget: u64) -> std::result::Result<f64, SimError> {
    let mut coarsest_need = None;
    for h in RESOLUTIONS {
        if let Ok(need) = Stock::box_bytes(min, max, h) {
            if need <= budget {
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
        if FlatTool::new(t.diameter / 2.0, t.flute_length, Vec3::Z).is_none() {
            return Err(bad(C, format!("tool {} needs a positive diameter and flute length in mm", t.id)));
        }
        tools.push(Tool {
            id: t.id,
            shape: Shape::Flat,
            diameter: t.diameter,
            flute_length: t.flute_length,
            shank: Vec::new(),
            holder: Vec::new(),
            stickout: 0.0,
        });
    }
    let mut program = gcode::parse(&input.gcode, &CodeTable::default());
    // Every tool the moves cut with must be listed, and every listed tool must be used: a mismatch is
    // a setup error, not a move that is silently skipped.
    let used: BTreeSet<u32> = program.moves.iter().map(|m| m.tool).collect();
    if let Some(&t) = used.iter().find(|t| !tools.iter().any(|d| d.id == **t)) {
        let line = program.moves.iter().find(|m| m.tool == t).map_or(0, |m| m.source);
        return Err(bad(C, format!("the G-code cuts with tool {t} (line {line}), which tools does not list")));
    }
    if let Some(d) = tools.iter().find(|d| !used.contains(&d.id)) {
        return Err(bad(C, format!("tool {} is listed, but the G-code never selects it", d.id)));
    }
    program.tools = tools;
    let (min, max) = (Vec3::from(input.stock.boxed.min), Vec3::from(input.stock.boxed.max));
    let h = match input.resolution {
        Some(h) if h.is_finite() && h > 0.0 => h,
        Some(_) => return Err(bad(C, "resolution must be a positive length in mm")),
        None => default_resolution(min, max, budget_bytes).map_err(|e| sim_error(C, e))?,
    };
    let setup = Setup { min, max, h, budget_bytes, program };
    let job = Job::new(setup).map_err(|e| sim_error(C, e))?;
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
    job.seek(to, until).map_err(|e| sim_error(C, e))?;
    let at = job.cursor();
    let reply = RunReply { at: count(at), total: count(total), done: at == total, removed_volume: job.removed_volume() };
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
        diagnostics: job.diagnostics().skip(offset).take(limit).map(diag_reply).collect(),
    };
    serde_json::to_value(reply).map_err(|e| bad(C, e.to_string()))
}

fn clear(s: &mut Session, p: &Value) -> Result<Value> {
    const C: &str = "sim.clear";
    parse::<ClearParams>(C, p)?;
    serde_json::to_value(ClearReply { cleared: s.sim.clear() }).map_err(|e| bad(C, e.to_string()))
}
