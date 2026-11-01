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

/// The cutting shape a tool has. Only `flat` is cut; the others are listed and reported as not
/// simulated yet.
#[derive(Debug, Default, Clone, Copy, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum ShapeName {
    #[default]
    Flat,
    Ball,
    /// Needs `corner_radius`.
    Bull,
    /// Needs `tip_diameter` and `angle`.
    Cone,
    /// Needs `a0`, `r0` and `angle`.
    Taper,
    /// Needs `profile`.
    Profile,
}

/// A tool the G-code selects with `T` and `M6`. `diameter` and `flute_length` are mm and must be
/// positive. The other fields belong to particular shapes and are refused for any other.
#[derive(Debug, Deserialize, JsonSchema)]
pub struct ToolParams {
    pub id: u32,
    pub diameter: f64,
    pub flute_length: f64,
    /// The shape (default `flat`).
    #[serde(default)]
    pub shape: Option<ShapeName>,
    /// `bull`: the corner radius, mm.
    #[serde(default)]
    pub corner_radius: Option<f64>,
    /// `cone`: the tip diameter, mm.
    #[serde(default)]
    pub tip_diameter: Option<f64>,
    /// `cone` and `taper`: the half angle, radians, from 0 to pi/2 (not exclusive of the ends).
    #[serde(default)]
    pub angle: Option<f64>,
    /// `taper`: the radius a0 mm from the tip where the taper starts.
    #[serde(default)]
    pub a0: Option<f64>,
    /// `taper`: the radius r0 mm at a0.
    #[serde(default)]
    pub r0: Option<f64>,
    /// `profile`: at least two `[a, r]` pairs, a mm from the tip and r mm in radius.
    #[serde(default)]
    pub profile: Option<Vec<[f64; 2]>>,
}

#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct SetupParams {
    pub stock: StockParams,
    /// The tools the G-code can select with `T` and `M6`. A move that cuts with a tool not listed
    /// is a diagnostic; a listed tool the G-code never selects is fine.
    pub tools: Vec<ToolParams>,
    /// G-code for a generic 3-axis machine (G0 to G3, G17 to G19, G20 and G21, G90 and G91).
    pub gcode: String,
    /// The tool the moves before the first `M6` cut with, for a program that never makes a first tool
    /// change. Without it those moves are reported and not cut.
    #[serde(default)]
    pub initial_tool: Option<u32>,
    /// Ray spacing in mm. Smaller is finer and needs more memory. Default: the finest of 0.05, 0.1,
    /// 0.2, 0.25, 0.5, 1, 2, 5, 10 and 20 mm whose stock fits half of `memory_mb`.
    #[serde(default)]
    pub resolution: Option<f64>,
    /// Memory cap in MB, 1 to 1024 (default 512). It covers the stock's ray grids as `memory_estimate`
    /// reports them. A move with several arc revolutions is cut into a copy of the stock that shares
    /// its unchanged tiles; the copy's extra tiles are not counted.
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

/// The move after the `at` moves of a run that is part-cut: `step` of its `steps` revolutions or
/// pieces are cut. They are not in the stock until the last one, so `at` does not move meanwhile.
#[derive(Debug, Serialize, JsonSchema)]
pub struct PartialReply {
    pub step: u64,
    pub steps: u64,
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
    /// Wall-clock budget in seconds. The run stops after the first step past it (a move, or one arc
    /// revolution), with `reached` false. At least one step is always cut.
    #[serde(default)]
    pub budget_s: Option<f64>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct RunReply {
    /// Moves cut so far.
    pub at: u64,
    pub total: u64,
    /// Whether the requested `to` was reached (`at` is `to`).
    pub reached: bool,
    /// Whether the whole program is cut (`at` is `total`).
    pub done: bool,
    /// Material taken away so far, mm^3.
    pub removed_volume: f64,
    /// Moves when the stock was last built again: setup, a backward seek or clear. Changes when the
    /// surface is redrawn in full.
    pub revision: u64,
    /// The move in progress, when a run stopped part-way through one.
    pub partial: Option<PartialReply>,
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
    /// Why the run stopped, when a step failed. A forward run then refuses until a backward seek.
    pub error: Option<String>,
    pub revision: u64,
    pub partial: Option<PartialReply>,
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
