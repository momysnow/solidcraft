//! Contracts and result types for the commands in `cmd/cam.rs`.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// Read the toolpaths of a setup's operations, in model coordinates. The design does not change.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct ToolpathParams {
    /// The setup, by name.
    pub setup: String,
    /// One operation of the setup, by name. Default: every operation of the setup, in order.
    #[serde(default)]
    pub operation: Option<String>,
    /// The chord tolerance in mm for cutting arcs as straight points (default 0.01; 0 < t ≤ 1).
    #[serde(default)]
    pub tolerance: Option<f64>,
}

/// How a move reaches its point: `rapid` at the machine's rapid rate, `feed` along a cut (an arc
/// is cut as short straight segments), or `plunge` straight down at the plunge rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum PathKind {
    Rapid,
    Feed,
    Plunge,
}

/// One operation's toolpath.
#[derive(Debug, Serialize, JsonSchema)]
pub struct ToolpathOperation {
    pub name: String,
    /// The tool's number.
    pub tool: u32,
    /// Why the operation has no path, when it does not build. The other fields are then empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// The length travelled, rapid and feed together (mm).
    pub length_mm: f64,
    /// The path's points in model coordinates (mm), in order. The first is where the first move ends.
    pub points: Vec<[f64; 3]>,
    /// `kinds[i]` is the kind of the move that ends at `points[i]`: one kind per point.
    pub kinds: Vec<PathKind>,
}

/// What `cam.toolpath` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct ToolpathResult {
    /// The setup, by name.
    pub setup: String,
    /// The operations asked for, in the setup's order.
    pub operations: Vec<ToolpathOperation>,
}

#[cfg(test)]
mod tests {
    use super::ToolpathParams;
    use crate::schema::input_schema_for;
    use crate::schema::tests::assert_keys;

    #[test]
    fn schemas_and_the_key_check_agree() {
        assert_keys(input_schema_for::<ToolpathParams>(), &["operation", "setup", "tolerance"]);
    }
}
