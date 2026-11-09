//! Planar boundary pieces: straight lines and circular arcs, with the distance query the
//! clearing clamp needs, and the checks a profile must pass before any operation uses it.

use std::f64::consts::TAU;

use solidcore_record::{Loop2, Region2, Seg2, Vec2};
use solidcraft_geom::prelude::*;

use crate::{CamError, Result, invalid};

/// Most boundary pieces of one operation's profiles (curves count by their line approximation).
pub(crate) const MAX_PIECES: usize = 4000;
/// How far apart consecutive profile segments may start and end (mm).
const CLOSED_TOL: f64 = 1e-6;
/// A parameter within this of 0 or 1 counts as on the piece.
const PARAM_SLACK: f64 = 1e-9;

/// Angle wrapped into `[0, 2π)`.
pub(crate) fn wrap(a: f64) -> f64 {
    let r = a.rem_euclid(TAU);
    if r.is_finite() { r } else { 0.0 }
}

/// Signed angle from `a` to `b` in `(-π, π]`.
pub(crate) fn turn(a: Vec2, b: Vec2) -> f64 {
    a.cross(b).atan2(a.dot(b))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Piece {
    Line {
        a: Vec2,
        b: Vec2,
    },
    /// Arc about `c` from angle `a0` sweeping `sweep` radians (positive = counter-clockwise).
    Arc {
        c: Vec2,
        r: f64,
        a0: f64,
        sweep: f64,
    },
}

impl Piece {
    pub fn start(&self) -> Vec2 {
        self.point_at(0.0)
    }
    pub fn end(&self) -> Vec2 {
        self.point_at(1.0)
    }
    pub fn point_at(&self, t: f64) -> Vec2 {
        match *self {
            Piece::Line { a, b } => a.lerp(b, t),
            Piece::Arc { c, r, a0, sweep } => c + Vec2::from_angle(a0 + sweep * t) * r,
        }
    }
    /// Distance from `p` to the nearest point of the piece.
    pub fn dist(&self, p: Vec2) -> f64 {
        match *self {
            Piece::Line { a, b } => {
                let d = b - a;
                let l2 = d.len2();
                let t = if l2 > 0.0 { ((p - a).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
                p.dist(a + d * t)
            }
            Piece::Arc { c, r, a0, sweep } => match arc_param(a0, sweep, (p - c).angle()) {
                Some(_) => (p.dist(c) - r).abs(),
                None => p.dist(self.start()).min(p.dist(self.end())),
            },
        }
    }
    /// Half of ∮(x dy − y dx) along the piece: the pieces of a closed loop sum to its signed area.
    pub fn area_term(&self) -> f64 {
        match *self {
            Piece::Line { a, b } => 0.5 * a.cross(b),
            Piece::Arc { c, r, sweep, .. } => 0.5 * (r * r * sweep + c.cross(self.end() - self.start())),
        }
    }
    pub fn is_finite(&self) -> bool {
        match *self {
            Piece::Line { a, b } => a.is_finite() && b.is_finite(),
            Piece::Arc { c, r, a0, sweep } => c.is_finite() && r.is_finite() && a0.is_finite() && sweep.is_finite(),
        }
    }
    pub fn seg(self) -> Seg2 {
        match self {
            Piece::Line { a, b } => Seg2::Line { a, b },
            Piece::Arc { c, r, a0, sweep } => Seg2::Arc { center: c, radius: r, start: a0, sweep },
        }
    }
}

/// The parameter at which an arc from `a0` sweeping `sweep` passes angle `theta`.
fn arc_param(a0: f64, sweep: f64, theta: f64) -> Option<f64> {
    let span = sweep.abs();
    if !(span > 0.0) {
        return None;
    }
    let delta = if sweep > 0.0 { wrap(theta - a0) } else { wrap(a0 - theta) };
    if delta <= span + PARAM_SLACK {
        Some((delta / span).min(1.0))
    } else if delta >= TAU - PARAM_SLACK {
        // Just before the start: the angle wrapped to nearly a full turn.
        Some(0.0)
    } else {
        None
    }
}

/// A profile loop as pieces (its other curves as lines within `tol`). Refuses a loop that is
/// empty, not finite, open or encloses no area.
fn loop_pieces(lp: &Loop2, tol: f64) -> Result<Vec<Piece>> {
    let mut out = Vec::new();
    for seg in &lp.segs {
        match *seg {
            Seg2::Line { a, b } => {
                if a.dist(b) > 1e-9 {
                    out.push(Piece::Line { a, b });
                }
            }
            Seg2::Arc { center, radius, start, sweep } => {
                if radius > 1e-9 && sweep.abs() > 1e-12 {
                    out.push(Piece::Arc { c: center, r: radius, a0: start, sweep });
                }
            }
            _ => {
                for w in seg.polyline(tol).windows(2) {
                    if let [a, b] = *w
                        && a.dist(b) > 1e-9
                    {
                        out.push(Piece::Line { a, b });
                    }
                }
            }
        }
        if out.len() > MAX_PIECES {
            return Err(CamError::TooLarge(format!("a profile has more than {MAX_PIECES} segments")));
        }
    }
    if out.is_empty() {
        return invalid("a profile is empty");
    }
    if !out.iter().all(Piece::is_finite) {
        return invalid("a profile has a coordinate that is not a finite number");
    }
    if out.iter().zip(out.iter().cycle().skip(1)).any(|(a, b)| a.end().dist(b.start()) > CLOSED_TOL) {
        return invalid("a profile is not closed");
    }
    let area: f64 = out.iter().map(Piece::area_term).sum();
    if !(area.abs() > 1e-9) {
        return invalid("a profile encloses no area");
    }
    Ok(out)
}

/// The loops of `regions` as pieces, outer loops counter-clockwise and holes clockwise, every
/// loop checked (see [`loop_pieces`]) and the profiles kept within [`MAX_PIECES`] segments.
/// `tol`: chord error allowed when a spline boundary is replaced by lines.
pub(crate) fn region_pieces(regions: &[Region2], tol: f64) -> Result<Vec<Vec<Piece>>> {
    if !(tol > 0.0) || !tol.is_finite() {
        return invalid("the tolerance must be above 0");
    }
    let mut loops = Vec::new();
    let mut count = 0usize;
    for region in regions {
        for (lp, ccw) in std::iter::once((&region.outer, true)).chain(region.holes.iter().map(|h| (h, false))) {
            let lp = if lp.is_ccw() == ccw { lp.clone() } else { lp.reversed() };
            let pieces = loop_pieces(&lp, tol)?;
            count += pieces.len();
            if count > MAX_PIECES {
                return Err(CamError::TooLarge(format!("the profiles have more than {MAX_PIECES} segments")));
            }
            loops.push(pieces);
        }
    }
    if loops.is_empty() {
        return invalid("there is no profile to machine");
    }
    Ok(loops)
}
