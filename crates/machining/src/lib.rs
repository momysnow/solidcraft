//! Stock-removal simulation for SolidCraft: a neutral move list, the G-code that fills it, and the
//! swept volumes of a flat end mill. Units are millimetres, in the workpiece frame. Depends on the
//! geometry foundation only.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]
#![forbid(unsafe_code)]

mod det_math;
pub mod gcode;
pub mod sweep;
pub mod toolpath;

/// Errors from the simulation core. Never a panic: hostile sizes and counts come back as these.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SimError {
    #[error("bad input: {0}")]
    BadInput(String),
    #[error("stock needs about {need} bytes, over the {budget} byte budget")]
    Budget { need: u64, budget: u64 },
    #[error("more than {max} segments on one ray")]
    SegmentCap { max: usize },
    #[error("more than {max} moves")]
    TooManyMoves { max: usize },
}
