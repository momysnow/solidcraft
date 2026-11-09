//! Turning SolidCraft's planar regions into the engine's inputs: curve-true
//! geometries for the exact profile passes, flattened polygons for the
//! clearing planners. Splines and other non-line-and-arc curves are
//! followed as line chains within the operation's tolerance, as everywhere
//! else in the crate.

use raygeo::geo::algo::offset_exact::offset_exact;
use raygeo::geo::geometry::Geometry;
use raygeo::geo::shape::arc::get_arc_sweep;
use raygeo::geo::types::{Command, Point, Polygon};
use solidcore_record::{Loop2, Region2, Seg2, Vec2};
use solidcraft_geom::prelude::*;

use crate::piece::region_pieces;
use crate::{Result, invalid};

/// Appends one closed loop to `g` at Z 0 (the engine lifts every move to
/// the pass height itself). Lines and arcs stay exact; other curves become
/// line chains within `tolerance`.
fn append_loop(g: &mut raygeo::geo::geometry::Geometry, lp: &Loop2, tolerance: f64) {
    let mut current = Vec2::ZERO;
    let mut first = true;
    for seg in &lp.segs {
        match *seg {
            solidcore_record::Seg2::Line { a, b } => {
                if first {
                    g.move_to(a.x, a.y, 0.0);
                    first = false;
                }
                g.line_to(b.x, b.y, 0.0);
                current = b;
            }
            solidcore_record::Seg2::Arc { center, radius, start, sweep } => {
                let a = center + Vec2::from_angle(start) * radius;
                let b = center + Vec2::from_angle(start + sweep) * radius;
                if first {
                    g.move_to(a.x, a.y, 0.0);
                    first = false;
                } else if a.dist(current) > 1e-9 {
                    g.line_to(a.x, a.y, 0.0);
                }
                // A full-circle arc starts and ends at the same point, and the
                // planner's exact offsetting degenerates on the zero chord: the
                // wall silently comes out as a tolerance chain instead of arcs.
                // No arc segment of the geometry spans more than half a turn —
                // the same rule the translator holds output arcs to.
                if sweep.abs() > std::f64::consts::PI {
                    let mid = center + Vec2::from_angle(start + sweep * 0.5) * radius;
                    let mid_offset = center - mid;
                    g.arc_to(mid.x, mid.y, center.x - a.x, center.y - a.y, sweep < 0.0, 0.0);
                    g.arc_to(b.x, b.y, mid_offset.x, mid_offset.y, sweep < 0.0, 0.0);
                } else {
                    g.arc_to(b.x, b.y, center.x - a.x, center.y - a.y, sweep < 0.0, 0.0);
                }
                current = b;
            }
            ref other => {
                let pts = other.polyline(tolerance.max(1e-4));
                for p in pts {
                    if first {
                        g.move_to(p.x, p.y, 0.0);
                        first = false;
                    } else if p.dist(current) > 1e-9 {
                        g.line_to(p.x, p.y, 0.0);
                    }
                    current = p;
                }
            }
        }
    }
    g.close_path();
}

/// A region as one curve-true geometry: the outer loop and its holes as
/// separate closed subpaths. The engine reads the nesting for islands
/// itself.
pub(crate) fn loop_group(region: &Region2, tolerance: f64) -> raygeo::geo::geometry::Geometry {
    let mut g = raygeo::geo::geometry::Geometry::new();
    append_loop(&mut g, &region.outer, tolerance);
    for hole in &region.holes {
        append_loop(&mut g, hole, tolerance);
    }
    g
}

/// A single loop as a curve-true geometry.
pub(crate) fn loop_geometry(lp: &Loop2, tolerance: f64) -> raygeo::geo::geometry::Geometry {
    let mut g = raygeo::geo::geometry::Geometry::new();
    append_loop(&mut g, lp, tolerance);
    g
}

/// A loop as a flattened polygon for the clearing planners, wound
/// counter-clockwise whatever the region's own convention is.
pub(crate) fn loop_polygon(lp: &Loop2, tolerance: f64) -> Polygon {
    lp.ccw().polyline(tolerance.max(1e-4)).into_iter().map(|p| Point::new(p.x, p.y)).collect()
}

/// The boundary polygon and island polygons of one region, for the clearing
/// planners.
pub(crate) fn region_polygons(region: &Region2, tolerance: f64) -> (Polygon, Vec<Polygon>) {
    let boundary = loop_polygon(&region.outer, tolerance);
    let islands = region.holes.iter().map(|h| loop_polygon(h, tolerance)).collect();
    (boundary, islands)
}

/// The boundaries of `regions` moved by `distance` (positive: into the regions, so they shrink;
/// negative: outwards), as closed loops with what is left on their left. Lines and arcs stay
/// exact: raygeo's exact offsetting does the work, and it grows for a positive distance, so the
/// sign is flipped here. Other curves are followed within `tolerance`.
///
/// Limit: raygeo returns nothing both for a region that collapses and for a failed offset, so
/// an empty result cannot tell them apart. Inputs are checked first (see [`region_pieces`]),
/// so bad input is still an error.
pub fn offset_regions(regions: &[Region2], distance: f64, tolerance: f64) -> Result<Vec<Loop2>> {
    if !distance.is_finite() {
        return invalid("the offset distance must be finite");
    }
    let loops = region_pieces(regions, tolerance)?;
    // Nothing moves: the loops come back as given (raygeo returns nothing for a zero offset).
    if distance.abs() <= 1e-9 {
        return Ok(loops.iter().map(|pieces| Loop2 { segs: pieces.iter().map(|p| p.seg()).collect() }).collect());
    }
    let mut g = Geometry::new();
    for region in regions {
        for lp in region.loops() {
            append_loop(&mut g, lp, tolerance);
        }
    }
    Ok(offset_exact(&g, -distance, tolerance).iter().map(loop_segs).collect())
}

/// A loop raygeo traced, as lines and arcs: one move, then its lines and arcs.
fn loop_segs(g: &Geometry) -> Loop2 {
    let mut segs = Vec::new();
    let mut at = Vec2::ZERO;
    for cmd in g.data() {
        match *cmd {
            Command::Move { end } => at = Vec2::new(end.x, end.y),
            Command::Line { end } => {
                let b = Vec2::new(end.x, end.y);
                segs.push(Seg2::Line { a: at, b });
                at = b;
            }
            Command::Arc { end, center_offset, normal } => {
                let b = Vec2::new(end.x, end.y);
                let off = Vec2::new(center_offset.x, center_offset.y);
                let c = at + off;
                let start = (at - c).angle();
                // A full circle comes back closed on its start, which the sweep reads as a whole turn.
                let sweep = get_arc_sweep(start, (b - c).angle(), normal.z < 0.0);
                segs.push(Seg2::Arc { center: c, radius: off.len(), start, sweep });
                at = b;
            }
            // The offsetting emits lines and arcs only.
            Command::Bezier { end, .. } => at = Vec2::new(end.x, end.y),
        }
    }
    Loop2 { segs }
}
