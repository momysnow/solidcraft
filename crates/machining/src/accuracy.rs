//! Accuracy of the simulated stock against the designed part. Every ray of the stock's three grids is
//! intersected with the part, a closed triangle mesh, and the material intervals the stock keeps on the
//! ray are compared with the part's. Stock material the part does not have is a left-over (the cut is too
//! thin); part material the stock lacks is a gouge (the cut went too deep).
//!
//! A deviation run is a stretch of a ray where the stock and the part disagree. One end must be a reference
//! crossing and the other a stock surface: a run bounded by the stock on both ends is another grid's to see,
//! and one bounded by the part on both ends (a slot's walls, seen along its floor) is no depth at all. The
//! run's depth is `length * |n_axis|`, where `n` is the unit normal of the triangle its reference crossing lies
//! on: exact along the ray for a band normal to that face, an overstatement for one lying along it. Only a
//! crossing with `|n_axis| >= 1/sqrt(3)` counts, so a grazing face is left to the grid it is normal to.
//! Features narrower than the grid's resolution may be missed.
//!
//! Parity cannot tell overlapping shells or a self-intersecting mesh from a void: a region covered twice
//! counts as empty and shows as a false gouge. A ray whose crossings are odd in number is skipped and
//! counted in [`Accuracy::odd_rays`].
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
/// Vertices closer than this share of the mesh's size are one vertex when the mesh is checked closed: the
/// export's weld distance (`solidcraft_io`).
const WELD_SHARE: f64 = 1e-6;
/// Runs shorter than this, mm, are f32 round-off on a face the two sides share, not deviations.
const MIN_RUN: f64 = 1e-6;
/// The least `|n_axis|` a grid counts: 1/sqrt(3).
const NORMAL_MIN: f64 = 0.577_350_269_189_625_8;

/// A deviation: where it is and how deep. `at` is the middle of its run along its ray, in the stock's frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spot {
    pub at: Vec3,
    pub depth: f64,
}

/// One grid's figures. `axis` is the grid's ray direction: 0 for X, 1 for Y, 2 for Z. Its mean and RMS are
/// over the part's surface samples the grid counts (see [`compare`]).
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

/// How far the stock is from the part. Each headline figure is the largest of the three grids' figures, with
/// the spot where it is deepest; `grids` gives each grid's own.
#[derive(Clone, Debug, PartialEq)]
pub struct Accuracy {
    /// Rays compared, over the three grids.
    pub rays: u64,
    /// Rays skipped because the mesh crosses them an odd number of times, over the three grids.
    pub odd_rays: u64,
    /// The largest cell of the grid, mm: deviations under it may be missed.
    pub resolution: f64,
    pub max_left_over: f64,
    pub worst_left_over: Option<Spot>,
    pub max_gouge: f64,
    pub worst_gouge: Option<Spot>,
    /// Mean and root mean square deviation over the part's surface, mm, area-weighted and independent of the
    /// stock's size.
    pub mean_abs_dev: f64,
    pub rms: f64,
    pub grids: [GridStats; 3],
}

/// Compares the stock with the part `positions` and `triangles`, a closed triangle mesh in the stock's frame.
/// Refused, never a panic: an empty, open or degenerate mesh, a missing vertex, a non-finite position or size,
/// or a stock without the memory the comparison needs.
///
/// The mean and RMS are over the part's surface. Each reference crossing whose triangle a grid counts is a
/// sample: its deviation is its run's depth (0 where stock and part agree), its weight `cell_u * cell_v /
/// |n_axis|`, the surface area its ray stands for.
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
    if !size.is_finite() {
        return Err(SimError::BadInput("reference mesh has a non-finite size".into()));
    }
    if !(size > 0.0) || enclosed_volume(positions, triangles).abs() <= 1e-12 * size * size * size {
        return Err(SimError::BadInput("reference mesh encloses no volume".into()));
    }
    let (min, max) = stock.bounds();
    let n = stock.cell_counts();
    let cell: [f64; 3] = std::array::from_fn(|k| (max[k] - min[k]) / n[k] as f64);
    let (used, budget) = stock.memory();
    let ctx = Ctx { stock, positions, triangles, min, cell, n, used, budget, avail: budget.saturating_sub(used) };
    ctx.check_budget(60 * positions.len() as u64 + 24 * triangles.len() as u64)?;
    check_closed(positions, triangles, (size * WELD_SHARE).max(1e-7))?;
    let runs = [grid(&ctx, 0)?, grid(&ctx, 1)?, grid(&ctx, 2)?];
    let grids = runs.each_ref().map(|r| r.stats);
    let rays: u64 = runs.iter().map(|r| r.rays).sum();
    let odd_rays: u64 = runs.iter().map(|r| r.odd).sum();
    let (w, wd, wd2) = runs.iter().fold((0.0, 0.0, 0.0), |(w, wd, wd2), r| (w + r.w, wd + r.wd, wd2 + r.wd2));
    let (max_left_over, worst_left_over) = largest(&grids, |g| g.max_left_over, |g| g.worst_left_over);
    let (max_gouge, worst_gouge) = largest(&grids, |g| g.max_gouge, |g| g.worst_gouge);
    Ok(Accuracy {
        rays,
        odd_rays,
        resolution: cell.iter().copied().fold(0.0, f64::max),
        max_left_over,
        worst_left_over,
        max_gouge,
        worst_gouge,
        mean_abs_dev: if w > 0.0 { wd / w } else { 0.0 },
        rms: if w > 0.0 { (wd2 / w).sqrt() } else { 0.0 },
        grids,
    })
}

/// The figure of the grid whose `value` is largest, with its spot.
fn largest(grids: &[GridStats], value: fn(&GridStats) -> f64, spot: fn(&GridStats) -> Option<Spot>) -> (f64, Option<Spot>) {
    let mut best: Option<&GridStats> = None;
    for g in grids {
        if best.is_none_or(|b| value(g) > value(b)) {
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

/// One grid's figures, and the sums its samples add to the headline mean and RMS.
struct GridRun {
    stats: GridStats,
    rays: u64,
    odd: u64,
    w: f64,
    wd: f64,
    wd2: f64,
}

/// Keeps `spot` if it is deeper than the one kept.
fn keep(best: &mut Option<Spot>, spot: Spot) {
    if best.is_none_or(|b| spot.depth > b.depth) {
        *best = Some(spot);
    }
}

/// The deviation runs of the rays of the grid along `axis`, and its surface samples.
fn grid(ctx: &Ctx, axis: usize) -> Result<GridRun, SimError> {
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let (nu, nv) = (ctx.n[u], ctx.n[v]);
    let area = ctx.cell[u] * ctx.cell[v];
    let (start, mut hits) = crossings(ctx, axis)?;
    let (mut run_left, mut run_gouge) = (None, None);
    let (mut rays, mut odd, mut w, mut wd, mut wd2) = (0u64, 0u64, 0.0f64, 0.0f64, 0.0f64);
    let (mut vals, mut ref_iv, mut stock_iv, mut left, mut gouge, mut dev) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::<f64>::new());
    for iv in 0..nv {
        for iu in 0..nu {
            let r = iv * nu + iu;
            let ray = &mut hits[start[r] as usize..start[r + 1] as usize];
            ray.sort_by(|a, b| a.0.total_cmp(&b.0));
            if ray.len() % 2 == 1 {
                odd += 1;
                continue;
            }
            rays += 1;
            vals.clear();
            vals.extend(ray.iter().map(|h| h.0));
            pair_up(&vals, &mut ref_iv);
            stock_intervals(ctx, axis, iu, iv, &mut stock_iv);
            subtract(&stock_iv, &ref_iv, &mut left);
            subtract(&ref_iv, &stock_iv, &mut gouge);
            dev.clear();
            dev.resize(ray.len(), 0.0);
            let (pu, pv) = (ctx.stock.pos(u, iu as i64), ctx.stock.pos(v, iv as i64));
            for (runs, is_left) in [(&left, true), (&gouge, false)] {
                for &(a, b) in runs.iter() {
                    let Some(j) = single_ref(ray, a, b) else { continue };
                    let len = b - a;
                    let share = axis_share(ctx, ray[j].1, axis);
                    if len < MIN_RUN || share < NORMAL_MIN {
                        continue;
                    }
                    let depth = len * share;
                    dev[j] = dev[j].max(depth);
                    let s = spot(axis, (u, v), (a, b), (pu, pv), depth);
                    keep(if is_left { &mut run_left } else { &mut run_gouge }, s);
                }
            }
            for (j, h) in ray.iter().enumerate() {
                let share = axis_share(ctx, h.1, axis);
                if share < NORMAL_MIN {
                    continue;
                }
                let weight = area / share;
                w += weight;
                wd += weight * dev[j];
                wd2 += weight * dev[j] * dev[j];
            }
        }
    }
    let stats = GridStats {
        axis,
        max_left_over: run_left.map_or(0.0, |s| s.depth),
        worst_left_over: run_left,
        max_gouge: run_gouge.map_or(0.0, |s| s.depth),
        worst_gouge: run_gouge,
        mean_abs_dev: if w > 0.0 { wd / w } else { 0.0 },
        rms: if w > 0.0 { (wd2 / w).sqrt() } else { 0.0 },
    };
    Ok(GridRun { stats, rays, odd, w, wd, wd2 })
}

/// The spot of a run `(a, b)` along the ray of grid `axis` through `(pu, pv)`, at its middle.
fn spot(axis: usize, (u, v): (usize, usize), (a, b): (f64, f64), (pu, pv): (f64, f64), depth: f64) -> Spot {
    let mut p = [0.0f64; 3];
    p[axis] = 0.5 * (a + b);
    p[u] = pu;
    p[v] = pv;
    Spot { at: Vec3::from(p), depth }
}

/// The index of the reference crossing at `x`, when a run's end `x` is one. `subtract` copies its endpoints
/// unchanged, so the match is bitwise.
fn ref_end(ray: &[(f64, u32)], x: f64) -> Option<usize> {
    let i = ray.partition_point(|h| h.0 < x);
    ray.get(i).filter(|h| h.0.to_bits() == x.to_bits()).map(|_| i)
}

/// The crossing a run is bounded by, when exactly one of its ends is a reference crossing. A run bounded by
/// the stock on both ends, or by the reference on both, is skipped.
fn single_ref(ray: &[(f64, u32)], a: f64, b: f64) -> Option<usize> {
    match (ref_end(ray, a), ref_end(ray, b)) {
        (Some(j), None) | (None, Some(j)) => Some(j),
        _ => None,
    }
}

/// `|n_axis|` of triangle `tri`: the unit normal's component along the grid's rays (0 when degenerate).
fn axis_share(ctx: &Ctx, tri: u32, axis: usize) -> f64 {
    let Some(c) = ctx.triangles.get(tri as usize).and_then(|t| corners(ctx.positions, *t)) else { return 0.0 };
    let e1: [f64; 3] = std::array::from_fn(|k| c[1][k] - c[0][k]);
    let e2: [f64; 3] = std::array::from_fn(|k| c[2][k] - c[0][k]);
    let n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 0.0 { n[axis].abs() / len } else { 0.0 }
}

/// The crossings of the grid's rays with the reference, as a table: ray `r` owns `hits[start[r]..start[r + 1]]`,
/// each crossing its `(t, triangle)`. Counted first, then filled, so the table is only as big as the real hits.
fn crossings(ctx: &Ctx, axis: usize) -> Result<(Vec<u32>, Vec<(f64, u32)>), SimError> {
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let rays = ctx.n[u] * ctx.n[v];
    ctx.check_budget((rays as u64 + 1).saturating_mul(4))?;
    let mut start = vec![0u32; rays + 1];
    scan(ctx, axis, |r, _, _| start[r + 1] += 1)?;
    let mut total: u32 = 0;
    for s in start.iter_mut() {
        total = total.checked_add(*s).ok_or_else(|| SimError::BadInput("reference mesh crosses the rays too often for the comparison".into()))?;
        *s = total;
    }
    ctx.check_budget((total as u64).saturating_mul(16))?;
    let mut fill: Vec<u32> = start.get(..rays).unwrap_or(&[]).to_vec();
    let mut hits = vec![(0.0f64, 0u32); total as usize];
    scan(ctx, axis, |r, t, ti| {
        if let Some(f) = fill.get_mut(r)
            && let Some(slot) = hits.get_mut(*f as usize)
        {
            *slot = (t, ti);
            *f += 1;
        }
    })?;
    Ok((start, hits))
}

/// Calls `emit(ray, t, triangle)` for every real crossing of the grid's rays with the mesh. Each triangle is
/// scan-converted: row by row over its projection, only the rays inside the row's span are tested, so a long
/// sliver costs its own area and not its bounding box.
fn scan(ctx: &Ctx, axis: usize, mut emit: impl FnMut(usize, f64, u32)) -> Result<(), SimError> {
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    for (ti, t) in ctx.triangles.iter().enumerate() {
        let Some(c) = corners(ctx.positions, *t) else { continue };
        let p: [(f64, f64); 3] = c.map(|q| (q[u], q[v]));
        if cross((p[1].0 - p[0].0, p[1].1 - p[0].1), (p[2].0 - p[0].0, p[2].1 - p[0].1)) == 0.0 {
            continue;
        }
        let vmin = p.iter().fold(f64::INFINITY, |m, q| m.min(q.1));
        let vmax = p.iter().fold(f64::NEG_INFINITY, |m, q| m.max(q.1));
        let Some((iv0, iv1)) = ray_range(vmin, vmax, ctx.min[v], ctx.cell[v], ctx.n[v]) else { continue };
        let ti = u32::try_from(ti).map_err(|_| SimError::BadInput("reference mesh has too many triangles".into()))?;
        for iv in iv0..=iv1 {
            let y = ctx.stock.pos(v, iv as i64);
            let Some((lo, hi)) = span(&p, y) else { continue };
            let Some((iu0, iu1)) = ray_range(lo, hi, ctx.min[u], ctx.cell[u], ctx.n[u]) else { continue };
            for iu in iu0..=iu1 {
                let (ou, ov) = (ctx.stock.pos(u, iu as i64) + NUDGE * ctx.cell[u], y + 0.7 * NUDGE * ctx.cell[v]);
                if let Some(t) = hit(c, axis, u, v, ou, ov) {
                    emit(iv * ctx.n[u] + iu, t, ti);
                }
            }
        }
    }
    Ok(())
}

/// The `u` span where the projected triangle `p` meets the line `v = y`, if it does.
fn span(p: &[(f64, f64); 3], y: f64) -> Option<(f64, f64)> {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for k in 0..3 {
        let (a, b) = (p[k], p[(k + 1) % 3]);
        if (a.1 - y) * (b.1 - y) <= 0.0 && a.1 != b.1 {
            let x = a.0 + (y - a.1) * (b.0 - a.0) / (b.1 - a.1);
            lo = lo.min(x);
            hi = hi.max(x);
        }
    }
    (lo <= hi).then_some((lo, hi))
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

/// The parts of the sorted, disjoint intervals `a` that `b` does not cover, sorted and disjoint. Endpoints are
/// copied from `a` or `b` unchanged.
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
