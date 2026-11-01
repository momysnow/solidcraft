//! Contracts and replies for the commands in `cmd/sim.rs`: stock-removal simulation of G-code.
//! Lengths are mm in the workpiece frame; volumes are mm^3; memory is bytes.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// The stock, as a box. Only `box` is accepted for now.
#[derive(Debug, Deserialize, JsonSchema)]
pub struct StockParams {
    #[serde(rename = "box")]
    pub boxed: BoxParams,
}

/// The box `min..max` in the workpiece frame. Each axis needs `min < max`.
#[derive(Debug, Deserialize, JsonSchema)]
pub struct BoxParams {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

/// The cutting shape. Only flat end mills are simulated for now.
#[derive(Debug, Default, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum ToolShape {
    #[default]
    Flat,
}

/// A tool the G-code selects with `T` and `M6`. `diameter` and `flute_length` are mm and must be
/// positive. `shape` defaults to `flat`.
#[derive(Debug, Deserialize, JsonSchema)]
pub struct ToolParams {
    pub id: u32,
    pub diameter: f64,
    pub flute_length: f64,
    #[serde(default)]
    pub shape: Option<ToolShape>,
}

#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct SetupParams {
    pub stock: StockParams,
    /// The tools the G-code selects: every `T` the program uses must be listed, and every listed tool
    /// must be used.
    pub tools: Vec<ToolParams>,
    /// G-code for a generic 3-axis machine (G0 to G3, G17 to G19, G20 and G21, G90 and G91).
    pub gcode: String,
    /// Ray spacing in mm. Smaller is finer and needs more memory. Default: the finest of 0.05, 0.1,
    /// 0.2, 0.25, 0.5, 1, 2, 5, 10 and 20 mm that fits `memory_mb`.
    #[serde(default)]
    pub resolution: Option<f64>,
    /// Memory cap in MB, 1 to 1024 (default 1024).
    #[serde(default)]
    pub memory_mb: Option<u64>,
}

/// A problem in the G-code or the simulation: `severity` is `error` or `warning`.
#[derive(Debug, Serialize, JsonSchema)]
pub struct DiagReply {
    pub line: u32,
    pub col: u32,
    pub severity: String,
    pub msg: String,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct BoundsReply {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

/// Bytes a stock takes, and the cap they are checked against.
#[derive(Debug, Serialize, JsonSchema)]
pub struct MemoryReply {
    pub bytes: u64,
    pub budget: u64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SetupReply {
    pub moves: u64,
    /// The stock box, mm.
    pub bounds: BoundsReply,
    /// Ray spacing in mm.
    pub h: f64,
    /// Memory the uncut stock takes, bytes, against its cap.
    pub memory_estimate: MemoryReply,
    /// How many problems the G-code has, in all.
    pub diag_count: u64,
    /// The first 200 of them.
    pub diagnostics: Vec<DiagReply>,
}

/// A move index, or `"end"` for the last move.
#[derive(Debug, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum MoveTarget {
    Index(u64),
    End(EndWord),
}

#[derive(Debug, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum EndWord {
    End,
}

#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct RunParams {
    /// Move count to cut up to (default `"end"`). Past the last move is an error.
    #[serde(default)]
    pub to: Option<MoveTarget>,
    /// Wall-clock budget in seconds. The run stops at the first move after it, with `done` false.
    #[serde(default)]
    pub budget_s: Option<f64>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct RunReply {
    /// Moves cut so far.
    pub at: u64,
    pub total: u64,
    pub done: bool,
    /// Material taken away so far, mm^3.
    pub removed_volume: f64,
}

#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct StatusParams {
    /// First diagnostic to list (default 0).
    #[serde(default)]
    pub offset: Option<u64>,
    /// Most diagnostics to list (default 100, at most 1000).
    #[serde(default)]
    pub limit: Option<u64>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct StatusReply {
    /// Moves cut so far.
    pub cursor: u64,
    pub total: u64,
    /// The box before any cut, mm^3.
    pub stock_volume: f64,
    /// Material left, mm^3.
    pub remaining_volume: f64,
    /// Material taken away, mm^3.
    pub removed_volume: f64,
    pub memory: MemoryReply,
    /// How many problems there are, in all.
    pub diag_count: u64,
    /// The page of problems asked for.
    pub diagnostics: Vec<DiagReply>,
}

#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct ClearParams {}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ClearReply {
    /// Whether there was a simulation to drop.
    pub cleared: bool,
}

#[cfg(test)]
#[path = "sim_tests.rs"]
mod tests;
