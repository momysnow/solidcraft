//! Swept volumes of a flat end mill, intersected with rays. A sweep answers, for one ray, which
//! parts of it lie inside the tool at some moment of the move; the dexel stock subtracts those.
//!
//! The tool is a flat cylinder: radius R, cutting from the tip up the axis to the flute length L.
//! Linear moves use closed forms for rays parallel and perpendicular to the axis, and an envelope
//! over the move for other rays. Arcs use an envelope over pieces that split at 90 degrees and at
//! the angles where the distance to the ray is extreme.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use solidcraft_geom::{Aabb3, Vec3};

use crate::det_math;

/// The ray-intersection interface the dexel stock is cut with.
pub trait Sweep {
    /// A box that contains the whole swept volume.
    fn bounds(&self) -> Aabb3;
    /// Appends to `out` the parts of the ray `o + t d` (`t` in `t_range`) inside the swept volume.
    /// The parts may overlap or come unsorted; `out` is not cleared.
    fn ray(&self, o: Vec3, d: Vec3, t_range: (f64, f64), out: &mut Vec<(f64, f64)>);
}

/// A flat end mill: radius, flute length (cutting from the tip up the axis) and the tool axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlatTool {
    pub radius: f64,
    pub length: f64,
    pub axis: Vec3,
}

impl FlatTool {
    /// `None` unless the radius and length are positive and finite and the axis is not zero.
    pub fn new(radius: f64, length: f64, axis: Vec3) -> Option<FlatTool> {
        let axis = axis.normalized()?;
        (radius.is_finite() && radius > 0.0 && length.is_finite() && length > 0.0).then_some(FlatTool { radius, length, axis })
    }
}

/// Parallel test thresholds: `|d . axis|` near 1 is parallel, near 0 is perpendicular.
const AXIS_EPS: f64 = 1e-12;
/// Brent evaluation budget per minimisation.
const BRENT_EVALS: usize = 25;
/// Penalty for "no tool here" in the minimisations. Far outside any real coordinate.
const PENALTY: f64 = 1e15;

fn perp(v: Vec3, u: Vec3) -> Vec3 {
    v - u * v.dot(u)
}

/// The part of the ray inside the flat cylinder with tip at `p`, as an interval of `t`, clipped to
/// `lim`. Closed form: a quadratic (radius) intersected with a linear pair (cutting length).
fn fiber(o: Vec3, d: Vec3, p: Vec3, tool: &FlatTool, lim: (f64, f64)) -> Option<(f64, f64)> {
    let u = tool.axis;
    let w = o - p;
    let alpha = w.dot(u);
    let beta = d.dot(u);
    let wp = perp(w, u);
    let dp = perp(d, u);
    let (mut lo, mut hi) = lim;
    if beta.abs() < AXIS_EPS {
        if alpha < 0.0 || alpha > tool.length {
            return None;
        }
    } else {
        let ta = -alpha / beta;
        let tb = (tool.length - alpha) / beta;
        lo = lo.max(ta.min(tb));
        hi = hi.min(ta.max(tb));
    }
    let qa = dp.len2();
    let qb = 2.0 * wp.dot(dp);
    let qc = wp.len2() - tool.radius * tool.radius;
    if qa < 1e-20 {
        if qc > 0.0 {
            return None;
        }
    } else {
        let c = -qb / (2.0 * qa);
        let disc = c * c - qc / qa;
        if disc < 0.0 {
            return None;
        }
        let h = disc.sqrt();
        lo = lo.max(c - h);
        hi = hi.min(c + h);
    }
    (lo <= hi).then_some((lo, hi))
}

/// Interval of `t` where `a0 + t a1` lies in `[lo, hi]`; unbounded when `a1` is zero and `a0` is in.
fn linear_window(a0: f64, a1: f64, lo: f64, hi: f64) -> Option<(f64, f64)> {
    if a1.abs() < AXIS_EPS {
        return (a0 >= lo && a0 <= hi).then_some((f64::NEG_INFINITY, f64::INFINITY));
    }
    let t1 = (lo - a0) / a1;
    let t2 = (hi - a0) / a1;
    Some((t1.min(t2), t1.max(t2)))
}

/// The chord of a stadium (segment `q0 q1` swept by a disc of radius `r`, all in the plane
/// perpendicular to `u`) along the line `o + t d`. The stadium is convex, so the union of its
/// three pieces (two discs and the strip between them) is one interval.
fn stadium_chord(o: Vec3, d: Vec3, q0: Vec3, q1: Vec3, r: f64, u: Vec3) -> Option<(f64, f64)> {
    let mut pieces: Vec<(f64, f64)> = Vec::with_capacity(3);
    for q in [q0, q1] {
        let w = o - q;
        let b = w.dot(d);
        let c = w.len2() - r * r;
        let disc = b * b - c;
        if disc >= 0.0 {
            let h = disc.sqrt();
            pieces.push((-b - h, -b + h));
        }
    }
    let seg = q1 - q0;
    let len = seg.len();
    if len > AXIS_EPS {
        let f = seg * (1.0 / len);
        let n = u.cross(f);
        let w0 = o - q0;
        if let (Some(a), Some(b)) = (linear_window(w0.dot(f), d.dot(f), 0.0, len), linear_window(w0.dot(n), d.dot(n), -r, r)) {
            let lo = a.0.max(b.0);
            let hi = a.1.min(b.1);
            if lo <= hi {
                pieces.push((lo, hi));
            }
        }
    }
    let lo = pieces.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let hi = pieces.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    (lo <= hi).then_some((lo, hi))
}

/// Brent's minimisation of `f` on `[ax, bx]`: returns the abscissa and value of the best point
/// found within `max_eval` evaluations, to tolerance `tol`.
fn brent_min(f: &dyn Fn(f64) -> f64, ax: f64, bx: f64, tol: f64, max_eval: usize) -> (f64, f64) {
    const CGOLD: f64 = 0.381_966_011_250_105;
    const ZEPS: f64 = 1e-12;
    let (mut a, mut b) = if ax <= bx { (ax, bx) } else { (bx, ax) };
    let mut x = a + CGOLD * (b - a);
    let mut w = x;
    let mut v = x;
    let mut fx = f(x);
    let mut fw = fx;
    let mut fv = fx;
    let mut d = 0.0f64;
    let mut e = 0.0f64;
    let mut evals = 1usize;
    while evals < max_eval {
        let xm = 0.5 * (a + b);
        let tol1 = tol + ZEPS;
        let tol2 = 2.0 * tol1;
        if (x - xm).abs() <= tol2 - 0.5 * (b - a) {
            break;
        }
        let mut golden = true;
        if e.abs() > tol1 {
            let r = (x - w) * (fx - fv);
            let mut q = (x - v) * (fx - fw);
            let mut p = (x - v) * q - (x - w) * r;
            q = 2.0 * (q - r);
            if q > 0.0 {
                p = -p;
            }
            q = q.abs();
            let etemp = e;
            e = d;
            if !(p.abs() >= (0.5 * q * etemp).abs() || p <= q * (a - x) || p >= q * (b - x)) {
                d = p / q;
                let u = x + d;
                if u - a < tol2 || b - u < tol2 {
                    d = if xm - x >= 0.0 { tol1 } else { -tol1 };
                }
                golden = false;
            }
        }
        if golden {
            e = if x >= xm { a - x } else { b - x };
            d = CGOLD * e;
        }
        let u = if d.abs() >= tol1 {
            x + d
        } else if d >= 0.0 {
            x + tol1
        } else {
            x - tol1
        };
        let fu = f(u);
        evals += 1;
        if fu <= fx {
            if u >= x {
                a = x;
            } else {
                b = x;
            }
            v = w;
            fv = fw;
            w = x;
            fw = fx;
            x = u;
            fx = fu;
        } else {
            if u < x {
                a = u;
            } else {
                b = u;
            }
            if fu <= fw || w == x {
                v = w;
                fv = fw;
                w = u;
                fw = fu;
            } else if fu <= fv || v == x || v == w {
                v = u;
                fv = fu;
            }
        }
    }
    (x, fx)
}

/// Extremes over a parameter range of the removed interval `(lo, hi)` at each parameter value:
/// the least `lo` and the greatest `hi`. Samples first, then Brent around the best sample on each
/// side. A `None` fiber means the tool is not on the ray at that parameter.
/// shortcut: a feasible window narrower than one sample step can be missed, which loses only a
/// sliver of volume at a grazing contact.
fn envelope(fib: &dyn Fn(f64) -> Option<(f64, f64)>, a: f64, b: f64, samples: usize, tol: f64) -> Option<(f64, f64)> {
    let x_of = |k: usize| a + (b - a) * (k as f64) / (samples as f64);
    let mut best_lo: Option<(usize, f64)> = None;
    let mut best_hi: Option<(usize, f64)> = None;
    for k in 0..=samples {
        if let Some((l, h)) = fib(x_of(k)) {
            if best_lo.is_none_or(|(_, v)| l < v) {
                best_lo = Some((k, l));
            }
            if best_hi.is_none_or(|(_, v)| h > v) {
                best_hi = Some((k, h));
            }
        }
    }
    let (kl, vl) = best_lo?;
    let (kh, vh) = best_hi?;
    let bracket = |k: usize| (x_of(k.saturating_sub(1)), x_of((k + 1).min(samples)));
    let (pl, ql) = bracket(kl);
    let (_, lo_ref) = brent_min(&|x| fib(x).map_or(PENALTY, |(l, _)| l), pl, ql, tol, BRENT_EVALS);
    let (ph, qh) = bracket(kh);
    let (_, neg_hi_ref) = brent_min(&|x| fib(x).map_or(PENALTY, |(_, h)| -h), ph, qh, tol, BRENT_EVALS);
    let lo = if lo_ref < vl { lo_ref } else { vl };
    let hi = if -neg_hi_ref > vh { -neg_hi_ref } else { vh };
    (lo <= hi).then_some((lo, hi))
}

fn swept_box(base: Aabb3, tool: &FlatTool) -> Aabb3 {
    let shifted = Aabb3 { min: base.min + tool.axis * tool.length, max: base.max + tool.axis * tool.length };
    base.union(&shifted).inflated(tool.radius)
}

/// A flat end mill moving in a straight line from `from` to `to`, its axis fixed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearFixedAxis {
    pub from: Vec3,
    pub to: Vec3,
    pub tool: FlatTool,
}

impl LinearFixedAxis {
    fn parallel(&self, o: Vec3, d: Vec3, tr: (f64, f64)) -> Option<(f64, f64)> {
        let u = self.tool.axis;
        let r = self.tool.radius;
        // Which s put the tool's footprint over the ray: a quadratic in s, convex, so an interval.
        let g = perp(o - self.from, u);
        let dp = perp(self.to - self.from, u);
        let qa = dp.len2();
        let qb = -2.0 * g.dot(dp);
        let qc = g.len2() - r * r;
        let (s0, s1) = if qa < 1e-20 {
            if qc > 0.0 {
                return None;
            }
            (0.0, 1.0)
        } else {
            let c = -qb / (2.0 * qa);
            let disc = c * c - qc / qa;
            if disc < 0.0 {
                return None;
            }
            let h = disc.sqrt();
            ((c - h).max(0.0), (c + h).min(1.0))
        };
        if s0 > s1 {
            return None;
        }
        // Along the ray the cut is the tool's height window, linear in s, so its extremes are at
        // the ends of the s interval.
        // The ray runs along the axis, so the height along it changes by +-1 per unit t.
        let beta = d.dot(u);
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for s in [s0, s1] {
            let p = self.from + (self.to - self.from) * s;
            let alpha = (o - p).dot(u);
            let ta = -alpha / beta;
            let tb = (self.tool.length - alpha) / beta;
            lo = lo.min(ta.min(tb));
            hi = hi.max(ta.max(tb));
        }
        let (lo, hi) = (lo.max(tr.0), hi.min(tr.1));
        (lo <= hi).then_some((lo, hi))
    }

    fn perpendicular(&self, o: Vec3, d: Vec3, tr: (f64, f64)) -> Option<(f64, f64)> {
        let u = self.tool.axis;
        let len = self.tool.length;
        let alpha0 = (o - self.from).dot(u);
        let delta = (self.to - self.from).dot(u);
        // The ray sits at one height; the s values that put the tip window over it.
        let (s0, s1) = if delta.abs() < AXIS_EPS {
            if alpha0 < 0.0 || alpha0 > len {
                return None;
            }
            (0.0, 1.0)
        } else {
            let sa = alpha0 / delta;
            let sb = (alpha0 - len) / delta;
            (sa.min(sb).max(0.0), sa.max(sb).min(1.0))
        };
        if s0 > s1 {
            return None;
        }
        let q0 = perp(self.from + (self.to - self.from) * s0, u);
        let q1 = perp(self.from + (self.to - self.from) * s1, u);
        let (lo, hi) = stadium_chord(perp(o, u), d, q0, q1, self.tool.radius, u)?;
        let (lo, hi) = (lo.max(tr.0), hi.min(tr.1));
        (lo <= hi).then_some((lo, hi))
    }

    fn general(&self, o: Vec3, d: Vec3, tr: (f64, f64)) -> Option<(f64, f64)> {
        let path = (self.to - self.from).len().max(1e-9);
        let fib = |s: f64| fiber(o, d, self.from + (self.to - self.from) * s, &self.tool, tr);
        envelope(&fib, 0.0, 1.0, 16, 1e-6 / path)
    }
}

impl Sweep for LinearFixedAxis {
    fn bounds(&self) -> Aabb3 {
        let base: Aabb3 = [self.from, self.to].into_iter().collect();
        swept_box(base, &self.tool)
    }

    fn ray(&self, o: Vec3, d: Vec3, t_range: (f64, f64), out: &mut Vec<(f64, f64)>) {
        let beta = d.dot(self.tool.axis).abs();
        let found = if beta > 1.0 - AXIS_EPS {
            self.parallel(o, d, t_range)
        } else if beta < AXIS_EPS {
            self.perpendicular(o, d, t_range)
        } else {
            self.general(o, d, t_range)
        };
        out.extend(found);
    }
}

/// A flat end mill on a circular or helical arc: centre, plane normal, sweep and lift. The tool
/// axis is fixed; the arc's plane may be any plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArcFixedAxis {
    /// Arc centre at the start height.
    centre: Vec3,
    e1: Vec3,
    e2: Vec3,
    r: f64,
    /// Signed sweep, radians: positive is counter-clockwise about the normal.
    sweep: f64,
    /// Total change along the normal from start to end.
    lift: Vec3,
    tool: FlatTool,
    /// The arc plane is perpendicular to the tool axis (the common G17 case).
    axis_normal: bool,
    normal: Vec3,
}

impl ArcFixedAxis {
    /// The arc from `start` to `end` about `centre` in the plane with `normal`, counter-clockwise
    /// when `ccw`, plus `turns` extra full revolutions. `None` for a degenerate arc.
    pub fn new(start: Vec3, end: Vec3, centre: Vec3, normal: Vec3, ccw: bool, turns: u32, tool: FlatTool) -> Option<ArcFixedAxis> {
        let n = normal.normalized()?;
        let to_start = start - centre;
        let height = to_start.dot(n);
        let centre = centre + n * height;
        let s_in = to_start - n * height;
        let r = s_in.len();
        if !(r.is_finite() && r > 1e-9) {
            return None;
        }
        let e1 = s_in * (1.0 / r);
        let e2 = n.cross(e1);
        let ev = end - centre;
        let a1 = det_math::atan2(ev.dot(e2), ev.dot(e1));
        let base = if ccw {
            if a1 <= 1e-12 { a1 + TAU } else { a1 }
        } else if a1 >= -1e-12 {
            a1 - TAU
        } else {
            a1
        };
        let extra = TAU * f64::from(turns);
        let sweep = if ccw { base + extra } else { base - extra };
        let lift = n * (end - start).dot(n);
        Some(ArcFixedAxis { centre, e1, e2, r, sweep, lift, tool, axis_normal: n.dot(tool.axis).abs() > 1.0 - AXIS_EPS, normal: n })
    }

    /// The tool tip at arc parameter `u` (radians swept so far, `0..=|sweep|`).
    fn tip(&self, u: f64) -> Vec3 {
        let total = self.sweep.abs();
        let s = if total > 0.0 { u / total } else { 0.0 };
        let (sn, cs) = det_math::sin_cos(self.sweep.signum() * u);
        self.centre + self.lift * s + (self.e1 * cs + self.e2 * sn) * self.r
    }

    /// Angles (as arc parameters, `0..TAU`) of the extremes of the distance to `w`, where the arc's
    /// in-plane offset along `w` peaks and troughs.
    fn feet(&self, w: Vec3, total: f64, pts: &mut Vec<f64>) {
        let theta = det_math::atan2(w.dot(self.e2), w.dot(self.e1));
        for th in [theta, theta + PI] {
            let u = (self.sweep.signum() * th).rem_euclid(TAU);
            if u > 0.0 && u < total {
                pts.push(u);
            }
        }
    }
}

impl Sweep for ArcFixedAxis {
    fn bounds(&self) -> Aabb3 {
        let ext = Vec3::new(
            self.r * (self.e1.x * self.e1.x + self.e2.x * self.e2.x).sqrt(),
            self.r * (self.e1.y * self.e1.y + self.e2.y * self.e2.y).sqrt(),
            self.r * (self.e1.z * self.e1.z + self.e2.z * self.e2.z).sqrt(),
        );
        let low: Aabb3 = [self.centre - ext, self.centre + ext].into_iter().collect();
        let high: Aabb3 = [self.centre + self.lift - ext, self.centre + self.lift + ext].into_iter().collect();
        swept_box(low.union(&high), &self.tool)
    }

    fn ray(&self, o: Vec3, d: Vec3, t_range: (f64, f64), out: &mut Vec<(f64, f64)>) {
        let total = self.sweep.abs();
        if !(total > 1e-12) {
            return;
        }
        // Pieces of at most a quarter turn, cut where the distance to the ray is extreme, so that
        // each piece's removed part is one interval.
        let mut pts = vec![0.0, total];
        let mut k = 1.0f64;
        while k * FRAC_PI_2 < total {
            pts.push(k * FRAC_PI_2);
            k += 1.0;
        }
        if self.axis_normal {
            let along = d.dot(self.tool.axis).abs();
            if along > 1.0 - AXIS_EPS {
                // Parallel ray: the distance to its foot peaks where the arc's offset from the foot points.
                self.feet(perp(self.centre - o, self.normal), total, &mut pts);
            } else if along < AXIS_EPS {
                // Perpendicular ray: the distance to its line peaks along the in-plane normal of the line.
                self.feet(self.normal.cross(d), total, &mut pts);
            }
        }
        pts.sort_by(|a, b| a.total_cmp(b));
        pts.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
        let tol = 1e-6 / self.r.max(1e-3);
        let fib = |u: f64| fiber(o, d, self.tip(u), &self.tool, t_range);
        for pair in pts.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if let Some(iv) = envelope(&fib, a, b, 8, tol) {
                out.push(iv);
            }
        }
    }
}

#[cfg(test)]
#[path = "sweep_tests.rs"]
mod tests;
