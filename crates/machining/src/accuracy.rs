//! Accuracy of the simulated stock against the designed part. Every ray of the stock's three grids is
//! intersected with the part, a closed triangle mesh, and the material intervals the stock keeps on the
//! ray are compared with the part's. Stock material the part does not have is a left-over (the cut is too
//! thin); part material the stock lacks is a gouge (the cut went too deep).
//!
//! A ray crosses a band of deviation of thickness `d` over at least `d`, and over exactly `d` only when it
//! is normal to the band. A grid's deepest run therefore overstates a band that is tangent to its rays (a
//! slot floor seen by the X rays, for instance), so each headline figure is the smallest of the three grids'.
//! That is an upper bound on the true depth, exact when some grid's rays are normal to the deviation. A
//! deviation under the grid resolution ([`Accuracy::resolution`]) is not meaningful.
//!
//! Each reference ray is nudged a tiny fixed amount across its plane, so it never passes exactly through a
//! mesh edge or vertex and every crossing is counted once. The parity of the crossings is then exact for a
//! closed mesh, which [`compare`] checks first.

use std::collections::HashMap;

use solidcore_record::Vec3;

use crate::SimError;
use crate::dexel::Stock;

/// The reference rays' nudge, as a share of a cell, across the ray's plane (see the module note).
const NUDGE: f64 = 1.37e-6;
/// Vertices closer than this share of the mesh's size are one vertex when the mesh is checked closed.
const WELD_SHARE: f64 = 1e-9;

/// A deviation: where it is and how deep. `at` is the middle of its run along its ray, in the stock's frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spot {
    pub at: Vec3,
    pub depth: f64,
}

/// One grid's figures. `axis` is the grid's ray direction: 0 for X, 1 for Y, 2 for Z.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GridStats {
    pub axis: usize,
    pub max_left_over: f64,
    pub worst_left_over: Option<Spot>,
    pub max_gouge: f64,
    pub worst_gouge: Option<Spot>,
    pub mean_abs_dev: f64,
    pub rms: f64,
}

/// How far the stock is from the part. The headline figures are the smallest of the three grids' (see the
/// module note); `grids` gives each grid's own.
#[derive(Clone, Debug, PartialEq)]
pub struct Accuracy {
    /// Rays compared, over the three grids.
    pub rays: u64,
    /// The largest cell of the grid, mm: deviations under it are not meaningful.
    pub resolution: f64,
    pub max_left_over: f64,
    pub worst_left_over: Option<Spot>,
    pub max_gouge: f64,
    pub worst_gouge: Option<Spot>,
    /// Mean and root mean square over the rays of each ray's deepest deviation (0 where it agrees), the
    /// smallest of the grids' like the depths.
    pub mean_abs_dev: f64,
    pub rms: f64,
    pub grids: [GridStats; 3],
}

/// Compares the stock with the part `positions` and `triangles`, a closed triangle mesh in the stock's frame.
/// Refused, never a panic: an empty, open or degenerate mesh, a missing vertex, a non-finite position, or a
/// stock without the memory the comparison needs.
pub fn compare(stock: &Stock, positions: &[Vec3], triangles: &[[u32; 3]]) -> Result<Accuracy, SimError> {
    if triangles.is_empty() {
        return Err(SimError::BadInput("reference mesh has no triangles".into()));
    }
    if positions.iter().any(|p| !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())) {
        return Err(SimError::BadInput("reference mesh has a non-finite position".into()));
    }
    if triangles.iter().flatten().any(|&i| i as usize >= positions.len()) {
        return Err(SimError::BadInput("reference mesh refers to a vertex it does not have".into()));
    }
    let size = diagonal(positions);
    if !(size > 0.0) || enclosed_volume(positions, triangles).abs() <= 1e-12 * size * size * size {
        return Err(SimError::BadInput("reference mesh encloses no volume".into()));
    }
    let (min, max) = stock.bounds();
    let n = stock.cell_counts();
    let cell: [f64; 3] = std::array::from_fn(|k| (max[k] - min[k]) / n[k] as f64);
    let (used, budget) = stock.memory();
    let ctx = Ctx { stock, positions, triangles, min, cell, n, used, budget, avail: budget.saturating_sub(used) };
    ctx.check_budget(60 * positions.len() as u64 + 24 * triangles.len() as u64)?;
    check_closed(positions, triangles, (size * WELD_SHARE).max(1e-9))?;
    let runs = [grid(&ctx, 0)?, grid(&ctx, 1)?, grid(&ctx, 2)?];
    let rays: u64 = runs.iter().map(|r| r.rays).sum();
    let grids = runs.map(|r| r.stats);
    let (max_left_over, worst_left_over) = smallest(&grids, |g| g.max_left_over, |g| g.worst_left_over);
    let (max_gouge, worst_gouge) = smallest(&grids, |g| g.max_gouge, |g| g.worst_gouge);
    Ok(Accuracy {
        rays,
        resolution: cell.iter().copied().fold(0.0, f64::max),
        max_left_over,
        worst_left_over,
        max_gouge,
        worst_gouge,
        mean_abs_dev: grids.iter().map(|g| g.mean_abs_dev).fold(f64::INFINITY, f64::min),
        rms: grids.iter().map(|g| g.rms).fold(f64::INFINITY, f64::min),
        grids,
    })
}

/// The figure of the grid whose `value` is smallest, with its spot.
fn smallest(grids: &[GridStats], value: fn(&GridStats) -> f64, spot: fn(&GridStats) -> Option<Spot>) -> (f64, Option<Spot>) {
    let mut best: Option<&GridStats> = None;
    for g in grids {
        if best.is_none_or(|b| value(g) < value(b)) {
            best = Some(g);
        }
    }
    best.map_or((0.0, None), |g| (value(g), spot(g)))
}

/// What the comparison shares: the stock, the mesh, and the grid's sizes and memory.
struct Ctx<'a> {
    stock: &'a Stock,
    positions: &'a [Vec3],
    triangles: &'a [[u32; 3]],
    min: [f64; 3],
    cell: [f64; 3],
    n: [usize; 3],
    used: u64,
    budget: u64,
    avail: u64,
}

impl Ctx<'_> {
    /// Refuses a step that needs more transient bytes than the stock has left under its budget.
    fn check_budget(&self, need: u64) -> Result<(), SimError> {
        if need > self.avail {
            return Err(SimError::Budget { need: self.used.saturating_add(need), budget: self.budget });
        }
        Ok(())
    }
}

/// One grid's figures and the number of rays they cover.
struct GridRun {
    stats: GridStats,
    rays: u64,
}

/// The deviations of one grid, summed up as its rays go by.
#[derive(Default)]
struct Tally {
    rays: u64,
    sum: f64,
    sum_sq: f64,
    left: Option<Spot>,
    gouge: Option<Spot>,
}

/// Keeps `spot` if it is deeper than the one kept.
fn keep(best: &mut Option<Spot>, spot: Spot) {
    if best.is_none_or(|b| spot.depth > b.depth) {
        *best = Some(spot);
    }
}

/// The reference intervals of ray `(iu, iv)` of the grid along `axis` are compared with the stock's.
fn grid(ctx: &Ctx, axis: usize) -> Result<GridRun, SimError> {
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let (nu, nv) = (ctx.n[u], ctx.n[v]);
    let (start, items) = buckets(ctx, axis)?;
    let mut tally = Tally::default();
    let (mut crossings, mut ref_iv, mut stock_iv, mut left, mut gouge) = (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for iv in 0..nv {
        for iu in 0..nu {
            let r = iv * nu + iu;
            let (pu, pv) = (ctx.stock.pos(u, iu as i64), ctx.stock.pos(v, iv as i64));
            let (ou, ov) = (pu + NUDGE * ctx.cell[u], pv + 0.7 * NUDGE * ctx.cell[v]);
            crossings.clear();
            let (lo, hi) = (start.get(r).copied().unwrap_or(0) as usize, start.get(r + 1).copied().unwrap_or(0) as usize);
            for &ti in items.get(lo..hi).unwrap_or(&[]) {
                let Some(c) = ctx.triangles.get(ti as usize).and_then(|t| corners(ctx.positions, *t)) else { continue };
                if let Some(t) = hit(c, axis, u, v, ou, ov) {
                    crossings.push(t);
                }
            }
            crossings.sort_by(f64::total_cmp);
            if crossings.len() % 2 == 1 {
                return Err(SimError::BadInput("reference mesh is not closed: a ray crosses it an odd number of times".into()));
            }
            pair_up(&crossings, &mut ref_iv);
            stock_intervals(ctx, axis, iu, iv, &mut stock_iv);
            subtract(&stock_iv, &ref_iv, &mut left);
            subtract(&ref_iv, &stock_iv, &mut gouge);
            let mut ray_dev = 0.0f64;
            for &(a, b) in &left {
                ray_dev = ray_dev.max(b - a);
                keep(&mut tally.left, spot(axis, u, v, a, b, pu, pv));
            }
            for &(a, b) in &gouge {
                ray_dev = ray_dev.max(b - a);
                keep(&mut tally.gouge, spot(axis, u, v, a, b, pu, pv));
            }
            tally.rays += 1;
            tally.sum += ray_dev;
            tally.sum_sq += ray_dev * ray_dev;
        }
    }
    let n = tally.rays.max(1) as f64;
    let stats = GridStats {
        axis,
        max_left_over: tally.left.map_or(0.0, |s| s.depth),
        worst_left_over: tally.left,
        max_gouge: tally.gouge.map_or(0.0, |s| s.depth),
        worst_gouge: tally.gouge,
        mean_abs_dev: tally.sum / n,
        rms: (tally.sum_sq / n).sqrt(),
    };
    Ok(GridRun { stats, rays: tally.rays })
}

/// The spot of a run `[a, b]` along the ray of grid `axis` through `(pu, pv)`, at its middle.
fn spot(axis: usize, u: usize, v: usize, a: f64, b: f64, pu: f64, pv: f64) -> Spot {
    let mut p = [0.0f64; 3];
    p[axis] = 0.5 * (a + b);
    p[u] = pu;
    p[v] = pv;
    Spot { at: Vec3::from(p), depth: b - a }
}

/// The triangles each ray of the grid along `axis` can cross, as a table: ray `r` owns `items[start[r]..start[r + 1]]`.
fn buckets(ctx: &Ctx, axis: usize) -> Result<(Vec<u32>, Vec<u32>), SimError> {
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let rays = ctx.n[u] * ctx.n[v];
    let mut entries: u64 = 0;
    for t in ctx.triangles {
        if let Some(c) = cover(ctx, axis, *t) {
            entries += c.count();
        }
    }
    ctx.check_budget((rays as u64).saturating_mul(8).saturating_add(entries.saturating_mul(4)))?;
    let mut start = vec![0u32; rays + 1];
    for t in ctx.triangles {
        if let Some(c) = cover(ctx, axis, *t) {
            for r in c.rays() {
                if let Some(s) = start.get_mut(r + 1) {
                    *s += 1;
                }
            }
        }
    }
    let mut acc = 0u32;
    for s in start.iter_mut() {
        acc = acc.checked_add(*s).ok_or_else(|| SimError::BadInput("reference mesh is too large for the comparison".into()))?;
        *s = acc;
    }
    let mut fill: Vec<u32> = start.get(..rays).unwrap_or(&[]).to_vec();
    let mut items = vec![0u32; acc as usize];
    for (ti, t) in ctx.triangles.iter().enumerate() {
        if let Some(c) = cover(ctx, axis, *t) {
            let ti = u32::try_from(ti).map_err(|_| SimError::BadInput("reference mesh has too many triangles".into()))?;
            for r in c.rays() {
                let Some(f) = fill.get_mut(r) else { continue };
                if let Some(slot) = items.get_mut(*f as usize) {
                    *slot = ti;
                    *f += 1;
                }
            }
        }
    }
    Ok((start, items))
}

/// The block of rays a triangle's projection can reach: inclusive index ranges along the two perpendicular axes.
#[derive(Clone, Copy)]
struct Cover {
    u0: usize,
    u1: usize,
    v0: usize,
    v1: usize,
    nu: usize,
}

impl Cover {
    fn count(&self) -> u64 {
        ((self.u1 - self.u0 + 1) * (self.v1 - self.v0 + 1)) as u64
    }

    fn rays(self) -> impl Iterator<Item = usize> {
        (self.v0..=self.v1).flat_map(move |iv| (self.u0..=self.u1).map(move |iu| iv * self.nu + iu))
    }
}

/// The rays a triangle can cross, with a slack of a ray each side. A triangle that projects to zero area
/// along the axis crosses no ray, so it covers none.
fn cover(ctx: &Ctx, axis: usize, t: [u32; 3]) -> Option<Cover> {
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let c = corners(ctx.positions, t)?;
    let (p0, p1, p2) = ((c[0][u], c[0][v]), (c[1][u], c[1][v]), (c[2][u], c[2][v]));
    if cross((p1.0 - p0.0, p1.1 - p0.1), (p2.0 - p0.0, p2.1 - p0.1)) == 0.0 {
        return None;
    }
    let (umin, umax) = (p0.0.min(p1.0).min(p2.0), p0.0.max(p1.0).max(p2.0));
    let (vmin, vmax) = (p0.1.min(p1.1).min(p2.1), p0.1.max(p1.1).max(p2.1));
    let (u0, u1) = ray_range(umin, umax, ctx.min[u], ctx.cell[u], ctx.n[u])?;
    let (v0, v1) = ray_range(vmin, vmax, ctx.min[v], ctx.cell[v], ctx.n[v])?;
    Some(Cover { u0, u1, v0, v1, nu: ctx.n[u] })
}

/// The inclusive ray indices along one axis whose lattice positions lie within `lo..=hi`, with a slack of one.
fn ray_range(lo: f64, hi: f64, min: f64, cell: f64, n: usize) -> Option<(usize, usize)> {
    let a = (((lo - min) / cell - 0.5).floor() - 1.0).max(0.0);
    let b = (((hi - min) / cell - 0.5).ceil() + 1.0).min(n as f64 - 1.0);
    (a <= b).then_some((a as usize, b as usize))
}

/// The three corners of triangle `t`, if its indices are in range.
fn corners(positions: &[Vec3], t: [u32; 3]) -> Option<[[f64; 3]; 3]> {
    let a = positions.get(t[0] as usize)?;
    let b = positions.get(t[1] as usize)?;
    let c = positions.get(t[2] as usize)?;
    Some([(*a).into(), (*b).into(), (*c).into()])
}

fn cross(p: (f64, f64), q: (f64, f64)) -> f64 {
    p.0 * q.1 - p.1 * q.0
}

/// Where the nudged ray `(ou, ov)` along `axis` crosses triangle `c`, if it does: the barycentric weights of
/// the ray's point in the triangle's projection all share a sign, and the crossing is their weighted height.
fn hit(c: [[f64; 3]; 3], axis: usize, u: usize, v: usize, ou: f64, ov: f64) -> Option<f64> {
    let q = |p: [f64; 3]| (p[u] - ou, p[v] - ov);
    let (a, b, d) = (q(c[0]), q(c[1]), q(c[2]));
    let (w0, w1, w2) = (cross(b, d), cross(d, a), cross(a, b));
    let inside = (w0 > 0.0 && w1 > 0.0 && w2 > 0.0) || (w0 < 0.0 && w1 < 0.0 && w2 < 0.0);
    if !inside {
        return None;
    }
    Some((w0 * c[0][axis] + w1 * c[1][axis] + w2 * c[2][axis]) / (w0 + w1 + w2))
}

/// The stock's material intervals on ray `(iu, iv)` of the grid along `axis`, in the stock's own frame.
fn stock_intervals(ctx: &Ctx, axis: usize, iu: usize, iv: usize, out: &mut Vec<(f64, f64)>) {
    out.clear();
    for s in ctx.stock.ray_segs(axis, iu, iv) {
        out.push((f64::from(s.lo) + ctx.min[axis], f64::from(s.hi) + ctx.min[axis]));
    }
}

/// The sorted crossings, taken in pairs as the intervals the ray is inside the part.
fn pair_up(crossings: &[f64], out: &mut Vec<(f64, f64)>) {
    out.clear();
    let mut it = crossings.iter().copied();
    while let (Some(a), Some(b)) = (it.next(), it.next()) {
        if b > a {
            out.push((a, b));
        }
    }
}

/// The parts of the sorted, disjoint intervals `a` that `b` does not cover, sorted and disjoint.
fn subtract(a: &[(f64, f64)], b: &[(f64, f64)], out: &mut Vec<(f64, f64)>) {
    out.clear();
    let mut j = 0;
    for &(a_lo, a_hi) in a {
        let mut lo = a_lo;
        while j < b.len() && b.get(j).is_some_and(|x| x.1 <= lo) {
            j += 1;
        }
        let mut k = j;
        while let Some(&(b_lo, b_hi)) = b.get(k)
            && b_lo < a_hi
        {
            if b_lo > lo {
                out.push((lo, b_lo));
            }
            lo = lo.max(b_hi);
            if lo >= a_hi {
                break;
            }
            k += 1;
        }
        if lo < a_hi {
            out.push((lo, a_hi));
        }
    }
}

/// Every edge of the mesh is shared by an even number of triangles, after its vertices are welded: the
/// surface is closed, so the parity of the crossings is exact.
fn check_closed(positions: &[Vec3], triangles: &[[u32; 3]], tol: f64) -> Result<(), SimError> {
    let ids = weld(positions, tol)?;
    let mut edges: Vec<u64> = Vec::with_capacity(triangles.len() * 3);
    for t in triangles {
        let [a, b, c] = t.map(|i| ids.get(i as usize).copied().unwrap_or(u32::MAX));
        for (x, y) in [(a, b), (b, c), (c, a)] {
            if x != y {
                let (lo, hi) = (u64::from(x.min(y)), u64::from(x.max(y)));
                edges.push((lo << 32) | hi);
            }
        }
    }
    edges.sort_unstable();
    for run in edges.chunk_by(|x, y| x == y) {
        if run.len() % 2 == 1 {
            return Err(SimError::BadInput("reference mesh is not closed: an edge is shared by an odd number of triangles".into()));
        }
    }
    Ok(())
}

/// The id of each vertex, where vertices within `tol` of each other share one id.
fn weld(positions: &[Vec3], tol: f64) -> Result<Vec<u32>, SimError> {
    let mut reps: Vec<[f64; 3]> = Vec::new();
    let mut cells: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
    let mut ids = Vec::with_capacity(positions.len());
    let inv = 1.0 / tol;
    for p in positions {
        let a: [f64; 3] = (*p).into();
        let key = a.map(|x| (x * inv).floor() as i64);
        let mut found = None;
        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let k = [key[0].saturating_add(dx), key[1].saturating_add(dy), key[2].saturating_add(dz)];
                    for &r in cells.get(&k).map_or(&[][..], Vec::as_slice) {
                        if let Some(q) = reps.get(r as usize)
                            && (0..3).map(|i| (q[i] - a[i]).powi(2)).sum::<f64>() <= tol * tol
                        {
                            found = Some(r);
                            break 'search;
                        }
                    }
                }
            }
        }
        let id = match found {
            Some(r) => r,
            None => {
                let r = u32::try_from(reps.len()).map_err(|_| SimError::BadInput("reference mesh has too many vertices".into()))?;
                reps.push(a);
                cells.entry(key).or_default().push(r);
                r
            }
        };
        ids.push(id);
    }
    Ok(ids)
}

/// The diagonal of the box around the positions, mm.
fn diagonal(positions: &[Vec3]) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in positions {
        let a: [f64; 3] = (*p).into();
        for k in 0..3 {
            lo[k] = lo[k].min(a[k]);
            hi[k] = hi[k].max(a[k]);
        }
    }
    (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt()
}

/// The signed volume the mesh encloses (positive when it faces outward), mm^3.
fn enclosed_volume(positions: &[Vec3], triangles: &[[u32; 3]]) -> f64 {
    let mut six = 0.0;
    for t in triangles {
        let Some([a, b, c]) = corners(positions, *t) else { continue };
        six += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    six / 6.0
}

#[cfg(test)]
#[path = "accuracy_tests.rs"]
mod tests;
