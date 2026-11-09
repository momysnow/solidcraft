//! The planner boundary: the only module that talks to raygeo.
//!
//! raygeo is the motion authority — it plans and generates every cut. This
//! module turns SolidCraft's regions into the planner's inputs, runs one
//! plan per region and depth level, and translates the planner's motion
//! commands back into this crate's checked [`Move`]s. The rest of the crate
//! never sees a raygeo type: operations are validate → prepare → run →
//! verify frontends. Offsetting (the stock to leave, and the offset API) goes
//! through raygeo's exact offsetting here; the translator clamps against the
//! regions' walls.

pub(crate) mod geometry;
pub(crate) mod plan;
pub(crate) mod translate;

pub(crate) use plan::{clearing_region, contour_region};
