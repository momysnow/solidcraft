//! Face segmentation of a stock surface: the faces, and the lines between them, that a CAD viewer
//! draws for a solid.
//!
//! The surface is welded into one indexed mesh (chunk meshes share their boundary vertices bit for
//! bit). Each triangle's normal is first smoothed over about `SMOOTH_STEPS` rings of triangles, leaving
//! out neighbours that turn by more than 35 degrees. That is enough to take out the facets of a curved
//! surface and the 45 degree chamfers that dual contouring puts at a 90 degree corner, so a crease stays
//! sharp and a cylinder reads as one smooth curve.
//!
//! Planar faces grow first, from seeds whose neighbours' smoothed normals agree within a quarter of a
//! degree. A neighbour joins when its corners lie within `plane_gap` of the least-squares plane through
//! the members so far, refitted as the face grows, and its smoothed normal lies within 1 degree of that
//! plane. Members that fail after the last refit are trimmed back to the curved pass. A planar face must
//! be at least `min_plane_extent` (3.75 cells) wide across its two principal in-plane axes; otherwise its
//! triangles go back to the curved pass. Curved faces then grow over what is left, across neighbours whose
//! smoothed normals turn by less than `crease_angle`. A curved face under 64 triangles, or narrower than
//! `min_plane_extent` by its second principal extent, or narrower than `min_mean_width` (2 cells) by its
//! mean width (2 area over perimeter), merges into the neighbour across the smallest normal angle, and
//! never into a sliver. Degenerate triangles (no normal) join a neighbour's face.
//!
//! A boundary between faces is an edge polyline. Its angle at each segment is measured between the two
//! sides' normals, where a planar side contributes its fitted plane normal and a curved side its smoothed
//! normal, so a rim of chamfers does not turn a 90 degree crease into a tangent. The edge is tangent when
//! the median of those angles is at most `tangent_angle`.
//!
//! The output is one [`Mesh`]: vertices split per face, `tri_face` set, and `edges` and `edge_faces`
//! filled, with `display_only_edges` set and `smooth_edges` all false (those flag display-only G2 joins,
//! which a viewer hides; tangency is in `FaceSegmentation::tangent`). A planar face shades with its
//! plane's normal, a curved face with its smoothed normals, so a crease keeps its sharpness.
//!
//! Limits, as measured (cells of 0.25 mm unless noted): a round pocket of radius 5 and 12 mm gives 8
//! faces, and of radius 25 mm gives 8 at 0.5 mm. A 25 mm pocket at 0.25 mm gives 136 faces: its wall turns
//! by about 0.6 degrees per cell, which the smoothing flattens below the seed test, so the wall splits into
//! planar strips about a millimetre wide. The limit is a per-cell turn under about 1 degree, a radius above
//! about cell / 0.0175 (14 mm at 0.25 mm, 29 mm at 0.5 mm). A tangent edge sits one or two cells off the
//! true tangent line: the planar gap admits the first rows of a fillet, and the 1 degree test stops them.
//! Features narrower than about `SMOOTH_STEPS` triangle rings are smoothed over.
//!
//! shortcut: the whole surface is segmented on each call (every mesh welded, every triangle grown), so a
//! viewer that remeshes one chunk segments the whole set again. Segment only the dirty region once the
//! viewport needs it at interactive rates.

use std::collections::VecDeque;
use std::f64::consts::PI;

use solidcore_record::Vec3;
use solidcraft_geom::Mesh;
use solidcraft_geom::prelude::*;

use crate::SimError;
use crate::det_math;

/// The face of a triangle that no face has claimed yet. Faces are numbered from 0.
const NONE: u32 = u32::MAX;
/// Normals are averaged over about this many rings of triangles (see [`smooth_normals`]).
const SMOOTH_STEPS: usize = 24;
/// Smoothing leaves out neighbours that turn by more than 35 degrees (cosine 0.819): a crease, and the 45
/// degree chamfers that dual contouring puts at 90 degree corners, are not averaged into a face.
const FEATURE_COS: f64 = 0.819_152_044;
/// A planar member's smoothed normal must lie within 1 degree of the plane's normal (cosine of 1 degree). A
/// strip cut from a cylinder is about 2 * this angle * radius wide, narrower than the minimum width up to a
/// radius of about 25 mm at 0.25 mm cells.
const PLANE_NORMAL_COS: f64 = 0.999_847_695;
/// A curved face with fewer triangles than this is a sliver and joins a neighbour.
const MIN_FACE_TRIS: usize = 64;
/// Refit the planar face's plane after this many accepted triangles, as well as when a candidate fails.
const REFIT_EVERY: u32 = 64;
/// Estimated bytes per triangle and per vertex while segmenting, checked against the budget.
const BYTES_PER_TRI: u64 = 320;
const BYTES_PER_VERTEX: u64 = 64;

/// What decides where faces end. Angles are in radians and lengths in millimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaceTolerances {
    /// Largest angle between a seed's smoothed normal and each neighbour's: a seed is flat only when
    /// its surroundings are.
    pub plane_angle: f64,
    /// Largest distance from a planar face's plane to any of its corners.
    pub plane_gap: f64,
    /// A planar face must be at least this wide across its two principal in-plane axes, and a curved
    /// face at least this large across its second principal axis. A narrower flat region is a facet of
    /// a curved surface, not a face.
    pub min_plane_extent: f64,
    /// A curved face whose mean width (2 area over perimeter) is under this is a sliver, like a chamfer
    /// ring at a corner. A real fillet of 2 mm radius is wider than two cells.
    pub min_mean_width: f64,
    /// Smoothed normals that turn by more than this across an edge split the faces there.
    pub crease_angle: f64,
    /// A boundary whose two sides turn by at most this (median along the edge) is tangent.
    pub tangent_angle: f64,
}

impl FaceTolerances {
    /// The tolerances for a stock whose rays are `cell` mm apart. The planar gap is 0.35 of a cell: the
    /// corners of a true plane, and of the rim where a plane meets a curve, sit within about a third of a
    /// cell of it. The seed angle is a quarter of a degree. A planar face is at least 3.75 cells across, and
    /// a curved one has a mean width of at least 2 cells. A crease is 30 degrees, and a boundary turning by 15
    /// or less is tangent.
    pub fn for_cell(cell: f64) -> Result<FaceTolerances, SimError> {
        FaceTolerances {
            plane_angle: 0.25f64.to_radians(),
            plane_gap: 0.35 * cell,
            min_plane_extent: 3.75 * cell,
            min_mean_width: 2.0 * cell,
            crease_angle: 30f64.to_radians(),
            tangent_angle: 15f64.to_radians(),
        }
        .checked()
    }

    fn checked(self) -> Result<FaceTolerances, SimError> {
        let angle_ok = |a: f64| a > 0.0 && a < PI;
        if !(angle_ok(self.plane_angle) && angle_ok(self.crease_angle) && angle_ok(self.tangent_angle)) {
            return Err(SimError::BadInput("face angles must lie strictly between 0 and 180 degrees".into()));
        }
        if !(self.plane_gap > 0.0 && self.plane_gap.is_finite()) {
            return Err(SimError::BadInput("the planar gap must be a positive length".into()));
        }
        if !(self.min_mean_width > 0.0 && self.min_mean_width.is_finite()) {
            return Err(SimError::BadInput("the minimum mean width must be a positive length".into()));
        }
        if !(self.min_plane_extent > 0.0 && self.min_plane_extent.is_finite()) {
            return Err(SimError::BadInput("the minimum face extent must be a positive length".into()));
        }
        Ok(self)
    }
}

/// The kind of a face: a plane, or a smooth curved patch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceKind {
    Planar,
    Curved,
}

/// The faces of a surface and the lines between them.
#[derive(Clone, Debug)]
pub struct FaceSegmentation {
    /// One mesh of the whole surface: `tri_face` is each triangle's face, `edges` the boundary
    /// polylines in world coordinates (closed loops repeat their first point), and `edge_faces` the faces
    /// each one separates. Vertices are split per face. `smooth_edges` is all false: it flags display-only
    /// G2 joins, which the viewer must not hide; tangency is in `tangent`.
    pub mesh: Mesh,
    /// The kind of each face, by face id.
    pub kinds: Vec<FaceKind>,
    /// Per edge of `mesh.edges`: true where the two faces meet tangentially.
    pub tangent: Vec<bool>,
}

/// Segments the surface of `meshes`, taken together as one surface, into faces and boundary lines.
/// Chunk meshes of one stock weld along their shared vertices, so a face that crosses a chunk boundary
/// stays one face. Errors, never panics: bad tolerances, non-finite vertices, out-of-range indices and a
/// surface over `budget_bytes` (by estimate, from the input size) are refused.
pub fn segment_faces(meshes: &[Mesh], tol: &FaceTolerances, budget_bytes: u64) -> Result<FaceSegmentation, SimError> {
    let tol = tol.checked()?;
    let input_tris: u64 = meshes.iter().map(|m| m.triangles.len() as u64).sum();
    let input_verts: u64 = meshes.iter().map(|m| m.positions.len() as u64).sum();
    let need = input_tris * BYTES_PER_TRI + input_verts * BYTES_PER_VERTEX;
    if need > budget_bytes {
        return Err(SimError::Budget { need, budget: budget_bytes });
    }
    let surface = Surface::weld(meshes)?;
    let raw: Vec<Option<Vec3>> = surface.tris.iter().map(|t| surface.normal(*t)).collect();
    let adj = Adjacency::build(&surface);
    let smooth = smooth_normals(&raw, &adj);
    let mut grown = grow(&surface, &smooth, &adj, tol);
    merge_slivers(&surface, &smooth, &adj, &mut grown, tol);
    compact(&mut grown);
    let segments = boundary_segments(&adj, &smooth, &grown);
    let lines = chain(&segments);
    let tangent: Vec<bool> = lines.iter().map(|l| median(l.angles.clone()).is_some_and(|m| m <= tol.tangent_angle)).collect();
    let mesh = build_mesh(&surface, &smooth, &grown, &lines);
    Ok(FaceSegmentation { mesh, kinds: grown.kinds, tangent })
}

/// The meshes welded into one indexed surface: each position once, each triangle by global indices.
struct Surface {
    pos: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
}

impl Surface {
    fn weld(meshes: &[Mesh]) -> Result<Surface, SimError> {
        let too_big = || SimError::BadInput("too many vertices or triangles to segment".to_string());
        let mut pos: Vec<Vec3> = Vec::new();
        let mut index: std::collections::HashMap<[u64; 3], u32> = std::collections::HashMap::new();
        let mut tris: Vec<[u32; 3]> = Vec::new();
        for m in meshes {
            if m.positions.iter().any(|p| !p.is_finite()) {
                return Err(SimError::BadInput("a mesh vertex is not finite".to_string()));
            }
            let mut map: Vec<u32> = Vec::with_capacity(m.positions.len());
            for p in &m.positions {
                let key = [bits(p.x), bits(p.y), bits(p.z)];
                let id = match index.get(&key) {
                    Some(id) => *id,
                    None => {
                        let id = u32::try_from(pos.len()).map_err(|_| too_big())?;
                        pos.push(*p);
                        index.insert(key, id);
                        id
                    }
                };
                map.push(id);
            }
            for t in &m.triangles {
                let mut g = [0u32; 3];
                for (slot, i) in g.iter_mut().zip(t) {
                    *slot = *map.get(*i as usize).ok_or_else(|| SimError::BadInput("a triangle index is out of range".to_string()))?;
                }
                tris.push(g);
            }
        }
        // Each output vertex is one (position, face) pair of a triangle corner: 3 per triangle must fit.
        if tris.len().checked_mul(3).is_none_or(|n| n >= NONE as usize) {
            return Err(too_big());
        }
        Ok(Surface { pos, tris })
    }

    fn point(&self, v: u32) -> Vec3 {
        self.pos.get(v as usize).copied().unwrap_or(Vec3::ZERO)
    }

    fn corners(&self, t: [u32; 3]) -> [Vec3; 3] {
        t.map(|v| self.point(v))
    }

    /// The unit normal of a triangle, or `None` when it is degenerate.
    fn normal(&self, t: [u32; 3]) -> Option<Vec3> {
        let [a, b, c] = self.corners(t);
        (b - a).cross(c - a).normalized()
    }
}

/// Bit pattern of a coordinate, with -0.0 taken as 0.0: they are one point.
fn bits(v: f64) -> u64 {
    if v == 0.0 { 0 } else { v.to_bits() }
}

/// The edges of the surface, sorted by their two vertices, with the triangles on each, and the
/// triangles that share an edge with each triangle. Both are flat arrays.
struct Adjacency {
    edge_key: Vec<(u32, u32)>,
    /// Edge `e` has the triangles `edge_tris[edge_start[e]..edge_start[e + 1]]`.
    edge_start: Vec<u32>,
    edge_tris: Vec<u32>,
    /// Triangle `t` has the neighbours `nbr[nbr_start[t]..nbr_start[t + 1]]`.
    nbr_start: Vec<u32>,
    nbr: Vec<u32>,
}

impl Adjacency {
    fn build(surface: &Surface) -> Adjacency {
        let n = surface.tris.len();
        let mut pairs: Vec<((u32, u32), u32)> = Vec::with_capacity(3 * n);
        for (t, tri) in surface.tris.iter().enumerate() {
            let [a, b, c] = *tri;
            for (p, q) in [(a, b), (b, c), (c, a)] {
                if p != q {
                    pairs.push(((p.min(q), p.max(q)), t as u32));
                }
            }
        }
        pairs.sort_unstable();
        let mut edge_key: Vec<(u32, u32)> = Vec::new();
        let mut edge_start: Vec<u32> = Vec::new();
        let mut edge_tris: Vec<u32> = Vec::with_capacity(pairs.len());
        for (key, t) in pairs {
            if edge_key.last() != Some(&key) {
                edge_key.push(key);
                edge_start.push(edge_tris.len() as u32);
            }
            edge_tris.push(t);
        }
        edge_start.push(edge_tris.len() as u32);

        let mut nbr_start = vec![0u32; n + 1];
        for e in 0..edge_key.len() {
            let ts = edge_slice(&edge_start, &edge_tris, e);
            for &t in ts {
                if let Some(slot) = nbr_start.get_mut(t as usize + 1) {
                    *slot += ts.len().saturating_sub(1) as u32;
                }
            }
        }
        let mut acc = 0u32;
        for x in nbr_start.iter_mut() {
            acc += *x;
            *x = acc;
        }
        let mut cursor = nbr_start.clone();
        let mut nbr = vec![0u32; nbr_start[n] as usize];
        for e in 0..edge_key.len() {
            let ts = edge_slice(&edge_start, &edge_tris, e);
            for &t in ts {
                for &u in ts {
                    if u != t
                        && let Some(c) = cursor.get_mut(t as usize)
                    {
                        if let Some(dst) = nbr.get_mut(*c as usize) {
                            *dst = u;
                        }
                        *c += 1;
                    }
                }
            }
        }
        Adjacency { edge_key, edge_start, edge_tris, nbr_start, nbr }
    }

    fn nbrs(&self, t: usize) -> &[u32] {
        let (a, b) = (self.nbr_start.get(t).copied().unwrap_or(0), self.nbr_start.get(t + 1).copied().unwrap_or(0));
        self.nbr.get(a as usize..b as usize).unwrap_or(&[])
    }

    fn edge_tris(&self, e: usize) -> &[u32] {
        edge_slice(&self.edge_start, &self.edge_tris, e)
    }
}

fn edge_slice<'a>(start: &[u32], tris: &'a [u32], e: usize) -> &'a [u32] {
    let (a, b) = (start.get(e).copied().unwrap_or(0), start.get(e + 1).copied().unwrap_or(0));
    tris.get(a as usize..b as usize).unwrap_or(&[])
}

/// Whether a normal lies within the angle whose cosine is `cos_limit` of `reference`. A degenerate
/// triangle (no normal) never does.
fn aligned(n: Option<Vec3>, reference: Vec3, cos_limit: f64) -> bool {
    n.is_some_and(|n| n.dot(reference) >= cos_limit)
}

/// Each triangle's normal, averaged over about `SMOOTH_STEPS` rings of triangles. A neighbour counts
/// only when its raw normal is within 60 degrees of the triangle's own, so a crease is never averaged
/// across. Each step averages the current smoothed normals, so the reach grows with the steps.
fn smooth_normals(raw: &[Option<Vec3>], adj: &Adjacency) -> Vec<Option<Vec3>> {
    let mut cur = raw.to_vec();
    let mut next = raw.to_vec();
    for _ in 0..SMOOTH_STEPS {
        for (t, r) in raw.iter().enumerate() {
            let Some(r) = *r else {
                next[t] = None;
                continue;
            };
            let mut sum = Vec3::ZERO;
            let mut add = |u: usize| {
                if let (Some(ru), Some(su)) = (raw[u], cur[u])
                    && ru.dot(r) >= FEATURE_COS
                {
                    sum += su;
                }
            };
            add(t);
            for &u in adj.nbrs(t) {
                add(u as usize);
            }
            next[t] = Some(sum.normalized().unwrap_or(r));
        }
        std::mem::swap(&mut cur, &mut next);
    }
    cur
}

/// Running moments of a set of points: the count, the sum and the sum of outer products, all relative
/// to an origin near the points so the covariance keeps its precision.
#[derive(Clone, Copy)]
struct Moments {
    origin: Vec3,
    n: f64,
    sum: Vec3,
    sq: [[f64; 3]; 3],
}

impl Moments {
    fn new(origin: Vec3) -> Moments {
        Moments { origin, n: 0.0, sum: Vec3::ZERO, sq: [[0.0; 3]; 3] }
    }

    /// Adds (`sign` 1) or removes (`sign` -1) one point.
    fn update(&mut self, p: Vec3, sign: f64) {
        let d = p - self.origin;
        let a: [f64; 3] = d.into();
        self.n += sign;
        self.sum += d * sign;
        for (r, row) in self.sq.iter_mut().enumerate() {
            for (c, cell) in row.iter_mut().enumerate() {
                *cell += sign * a[r] * a[c];
            }
        }
    }

    fn update_all(&mut self, ps: [Vec3; 3], sign: f64) {
        for p in ps {
            self.update(p, sign);
        }
    }
}

/// The principal axes of a point set: its centroid, the eigenvalues of its covariance (largest first),
/// and the unit axes in that order. `None` with fewer than three points.
fn principal_axes(m: &Moments) -> Option<(Vec3, [f64; 3], [Vec3; 3])> {
    if m.n < 3.0 {
        return None;
    }
    let mean = m.sum / m.n;
    let mu: [f64; 3] = mean.into();
    let mut cov = [[0.0f64; 3]; 3];
    for (r, row) in cov.iter_mut().enumerate() {
        for (c, cell) in row.iter_mut().enumerate() {
            *cell = m.sq[r][c] / m.n - mu[r] * mu[c];
        }
    }
    let (vals, vecs) = eigen3(cov);
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| vals[b].total_cmp(&vals[a]));
    Some((m.origin + mean, order.map(|i| vals[i]), order.map(|i| vecs[i])))
}

/// The least-squares plane of a point set: its centroid and the unit normal, the axis of least variance.
fn plane_of(m: &Moments) -> Option<(Vec3, Vec3)> {
    let (c, _, axes) = principal_axes(m)?;
    Some((c, axes[2]))
}

/// The eigenvalues and unit eigenvectors of a symmetric 3 by 3 matrix, by cyclic Jacobi rotations. The
/// eigenvectors are unit and orthogonal, or the identity when the matrix is zero.
fn eigen3(mut m: [[f64; 3]; 3]) -> ([f64; 3], [Vec3; 3]) {
    let mut v = [[1.0f64, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    const PAIRS: [(usize, usize); 3] = [(0, 1), (0, 2), (1, 2)];
    for _ in 0..16 {
        let off: f64 = PAIRS.iter().map(|&(p, q)| m[p][q].abs()).sum();
        let scale: f64 = m[0][0].abs() + m[1][1].abs() + m[2][2].abs();
        if off <= 1e-14 * scale || !off.is_finite() {
            break;
        }
        for (p, q) in PAIRS {
            if m[p][q] == 0.0 {
                continue;
            }
            let theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for row in m.iter_mut() {
                let (mkp, mkq) = (row[p], row[q]);
                row[p] = c * mkp - s * mkq;
                row[q] = s * mkp + c * mkq;
            }
            let (rp, rq) = (m[p], m[q]);
            m[p] = std::array::from_fn(|k| c * rp[k] - s * rq[k]);
            m[q] = std::array::from_fn(|k| s * rp[k] + c * rq[k]);
            for row in v.iter_mut() {
                let (vkp, vkq) = (row[p], row[q]);
                row[p] = c * vkp - s * vkq;
                row[q] = s * vkp + c * vkq;
            }
        }
    }
    let vals = [m[0][0], m[1][1], m[2][2]];
    let vecs = [0usize, 1, 2].map(|i| Vec3::new(v[0][i], v[1][i], v[2][i]).normalized().unwrap_or(Vec3::Z));
    (vals, vecs)
}

/// The extent of the corners of `members` along each of `axes`, measured from `c`.
fn extents(surface: &Surface, members: &[usize], c: Vec3, axes: &[Vec3]) -> Vec<f64> {
    let mut lo = vec![f64::INFINITY; axes.len()];
    let mut hi = vec![f64::NEG_INFINITY; axes.len()];
    for &t in members {
        for q in surface.corners(surface.tris[t]) {
            for (k, e) in axes.iter().enumerate() {
                let d = (q - c).dot(*e);
                lo[k] = lo[k].min(d);
                hi[k] = hi[k].max(d);
            }
        }
    }
    lo.iter().zip(&hi).map(|(l, h)| h - l).collect()
}

/// Whether a member's three corners lie within `gap` of the plane through `c` with normal `nrm`, and its
/// smoothed normal `s` lies within 2 degrees of that normal.
fn member_fits(corners: [Vec3; 3], s: Option<Vec3>, c: Vec3, nrm: Vec3, gap: f64) -> bool {
    corners.iter().all(|q| (*q - c).dot(nrm).abs() <= gap) && s.is_some_and(|s| s.dot(nrm).abs() >= PLANE_NORMAL_COS)
}

/// The face of every triangle, and each face's kind and plane (planar faces only).
struct Grown {
    face: Vec<u32>,
    kinds: Vec<FaceKind>,
    planes: Vec<Option<Vec3>>,
}

/// Grows the faces: planar ones first, then curved ones over what is left, then degenerate triangles.
fn grow(surface: &Surface, smooth: &[Option<Vec3>], adj: &Adjacency, tol: FaceTolerances) -> Grown {
    let n = surface.tris.len();
    let cos_plane = tol.plane_angle.cos();
    let cos_crease = tol.crease_angle.cos();
    let mut g = Grown { face: vec![NONE; n], kinds: Vec::new(), planes: Vec::new() };
    // Triangles of a planar flood that came out too narrow: never a seed or a member again.
    let mut rejected = vec![false; n];
    let mut queue: VecDeque<usize> = VecDeque::new();

    for seed in 0..n {
        let Some(sn) = smooth[seed] else { continue };
        if g.face[seed] != NONE || rejected[seed] || !adj.nbrs(seed).iter().all(|&u| aligned(smooth[u as usize], sn, cos_plane)) {
            continue;
        }
        let id = g.kinds.len() as u32;
        let seed_corners = surface.corners(surface.tris[seed]);
        let mut m = Moments::new(seed_corners[0]);
        m.update_all(seed_corners, 1.0);
        let mut members = vec![seed];
        let mut plane = plane_of(&m);
        let mut since_refit = 0u32;
        g.face[seed] = id;
        queue.push_back(seed);
        while let Some(x) = queue.pop_front() {
            for &u in adj.nbrs(x) {
                let u = u as usize;
                if g.face[u] != NONE || rejected[u] {
                    continue;
                }
                let Some(su) = smooth[u] else { continue };
                let cs = surface.corners(surface.tris[u]);
                m.update_all(cs, 1.0);
                let fits = |p: Option<(Vec3, Vec3)>| p.is_some_and(|(c, nrm)| member_fits(cs, Some(su), c, nrm, tol.plane_gap));
                // The cached plane serves until a candidate fails it, or it is REFIT_EVERY accepts old.
                let ok = if fits(plane) && since_refit < REFIT_EVERY {
                    true
                } else {
                    plane = plane_of(&m);
                    since_refit = 0;
                    fits(plane)
                };
                if ok {
                    since_refit += 1;
                    g.face[u] = id;
                    members.push(u);
                    queue.push_back(u);
                } else {
                    m.update_all(cs, -1.0);
                }
            }
        }
        // Trim: a member that strays beyond the gap, or whose normal strays from the refitted plane's,
        // goes back to the curved pass, and the plane is refitted, for a few rounds.
        let orient = |c: Vec3, nrm: Vec3| (c, if nrm.dot(sn) < 0.0 { -nrm } else { nrm });
        let mut oriented = plane.map(|(c, nrm)| orient(c, nrm));
        for _ in 0..3 {
            let Some((c, nrm)) = oriented else { break };
            let mut bad: Vec<usize> = members
                .iter()
                .copied()
                .filter(|&t| !member_fits(surface.corners(surface.tris[t]), smooth.get(t).copied().flatten(), c, nrm, tol.plane_gap))
                .collect();
            if bad.is_empty() {
                break;
            }
            bad.sort_unstable();
            for &t in &bad {
                g.face[t] = NONE;
                m.update_all(surface.corners(surface.tris[t]), -1.0);
            }
            members.retain(|t| bad.binary_search(t).is_err());
            oriented = plane_of(&m).map(|(c, nrm)| orient(c, nrm));
        }
        let verified = oriented.is_some_and(|(c, nrm)| {
            members.iter().all(|&t| member_fits(surface.corners(surface.tris[t]), smooth.get(t).copied().flatten(), c, nrm, tol.plane_gap))
        });
        let width = match (oriented, principal_axes(&m)) {
            (Some(_), Some((c, _, axes))) => extents(surface, &members, c, &axes[..2]).into_iter().fold(f64::INFINITY, f64::min),
            _ => 0.0,
        };
        match oriented {
            Some((_, nrm)) if verified && width >= tol.min_plane_extent => {
                g.kinds.push(FaceKind::Planar);
                g.planes.push(Some(nrm));
            }
            _ => {
                for &t in &members {
                    g.face[t] = NONE;
                    rejected[t] = true;
                }
            }
        }
    }

    for seed in 0..n {
        if g.face[seed] != NONE || smooth[seed].is_none() {
            continue;
        }
        let id = g.kinds.len() as u32;
        g.kinds.push(FaceKind::Curved);
        g.planes.push(None);
        g.face[seed] = id;
        queue.push_back(seed);
        while let Some(x) = queue.pop_front() {
            let Some(nx) = smooth[x] else { continue };
            for &u in adj.nbrs(x) {
                let u = u as usize;
                if g.face[u] == NONE && aligned(smooth[u], nx, cos_crease) {
                    g.face[u] = id;
                    queue.push_back(u);
                }
            }
        }
    }

    // Degenerate triangles take a neighbour's face; a group with none next to it is a face of its own.
    let mut queue: VecDeque<usize> = (0..n).filter(|&t| g.face[t] != NONE).collect();
    spread(&mut g.face, adj, &mut queue);
    for t in 0..n {
        if g.face[t] == NONE {
            let id = g.kinds.len() as u32;
            g.kinds.push(FaceKind::Curved);
            g.planes.push(None);
            g.face[t] = id;
            queue.push_back(t);
            spread(&mut g.face, adj, &mut queue);
        }
    }
    g
}

/// Gives every unclaimed neighbour of the queued triangles the face of the triangle that reaches it.
fn spread(face: &mut [u32], adj: &Adjacency, queue: &mut VecDeque<usize>) {
    while let Some(x) = queue.pop_front() {
        let f = face[x];
        for &u in adj.nbrs(x) {
            let u = u as usize;
            if face[u] == NONE {
                face[u] = f;
                queue.push_back(u);
            }
        }
    }
}

/// Merges each curved face that is a sliver (under `MIN_FACE_TRIS` triangles, or whose second principal
/// extent is under `min_plane_extent`) into the neighbour across its smallest normal angle. A few passes
/// let a chain of slivers settle.
fn merge_slivers(surface: &Surface, smooth: &[Option<Vec3>], adj: &Adjacency, g: &mut Grown, tol: FaceTolerances) {
    let n = surface.tris.len();
    for _ in 0..4 {
        let k = g.kinds.len();
        let mut count = vec![0usize; k];
        let mut moments = vec![Moments::new(Vec3::ZERO); k];
        let mut area = vec![0.0f64; k];
        for t in 0..n {
            let f = g.face[t] as usize;
            count[f] += 1;
            let c = surface.corners(surface.tris[t]);
            moments[f].update_all(c, 1.0);
            let [a, b, d] = c;
            area[f] += 0.5 * (b - a).cross(d - a).len();
        }
        // The perimeter of each face: the edges where its triangles meet another face, or the outline.
        let mut perimeter = vec![0.0f64; k];
        for (e, &(p, q)) in adj.edge_key.iter().enumerate() {
            let ts = adj.edge_tris(e);
            let first = ts.first().map(|&t| g.face[t as usize]);
            if ts.len() > 1 && ts.iter().all(|&t| Some(g.face[t as usize]) == first) {
                continue;
            }
            let len = (surface.point(q) - surface.point(p)).len();
            let mut seen: Vec<u32> = ts.iter().map(|&t| g.face[t as usize]).collect();
            seen.sort_unstable();
            seen.dedup();
            for f in seen {
                perimeter[f as usize] += len;
            }
        }
        let axes: Vec<Option<(Vec3, [f64; 3], [Vec3; 3])>> = moments.iter().map(principal_axes).collect();
        let mut lo = vec![[f64::INFINITY; 3]; k];
        let mut hi = vec![[f64::NEG_INFINITY; 3]; k];
        for t in 0..n {
            let f = g.face[t] as usize;
            let Some((c, _, ax)) = axes[f] else { continue };
            for q in surface.corners(surface.tris[t]) {
                for a in 0..3 {
                    let d = (q - c).dot(ax[a]);
                    lo[f][a] = lo[f][a].min(d);
                    hi[f][a] = hi[f][a].max(d);
                }
            }
        }
        // The second largest of a face's three principal extents: the breadth of a strip or a patch.
        let breadth: Vec<f64> = (0..k)
            .map(|f| {
                let mut e = [0.0f64; 3];
                for a in 0..3 {
                    e[a] = hi[f][a] - lo[f][a];
                }
                e.sort_by(|x, y| y.total_cmp(x));
                e[1]
            })
            .collect();
        // A sliver is thin by its second principal extent, or by its mean width (2 area over perimeter).
        let small: Vec<bool> = (0..k)
            .map(|f| {
                let mean_width = if perimeter[f] > 0.0 { 2.0 * area[f] / perimeter[f] } else { f64::INFINITY };
                g.kinds[f] == FaceKind::Curved
                    && count[f] > 0
                    && (count[f] < MIN_FACE_TRIS || breadth[f] < tol.min_plane_extent || mean_width < tol.min_mean_width)
            })
            .collect();
        if !small.iter().any(|&s| s) {
            break;
        }
        // Best neighbour of each sliver: the face across the edge with the largest cosine.
        let mut best: Vec<(f64, u32)> = vec![(-2.0, NONE); k];
        for t in 0..n {
            let f = g.face[t] as usize;
            if !small[f] {
                continue;
            }
            let Some(st) = smooth[t] else { continue };
            for &u in adj.nbrs(t) {
                let other = g.face[u as usize];
                // A sliver joins a face that stays: two slivers picking each other would just swap labels.
                if other as usize == f || small[other as usize] {
                    continue;
                }
                if let Some(su) = smooth[u as usize] {
                    let c = st.dot(su);
                    if c > best[f].0 {
                        best[f] = (c, other);
                    }
                }
            }
        }
        let mut changed = false;
        for t in 0..n {
            let f = g.face[t] as usize;
            if small[f] && best[f].1 != NONE {
                g.face[t] = best[f].1;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
}

/// Renumbers the faces that still have triangles, in order of first use, and drops the rest.
fn compact(g: &mut Grown) {
    let mut map = vec![NONE; g.kinds.len()];
    let mut kinds = Vec::new();
    let mut planes = Vec::new();
    for f in g.face.iter_mut() {
        let old = *f as usize;
        if map.get(old).copied() == Some(NONE) {
            if let Some(slot) = map.get_mut(old) {
                *slot = kinds.len() as u32;
            }
            kinds.push(g.kinds.get(old).copied().unwrap_or(FaceKind::Curved));
            planes.push(g.planes.get(old).copied().flatten());
        }
        *f = map.get(old).copied().unwrap_or(NONE);
    }
    g.kinds = kinds;
    g.planes = planes;
}

/// The normal a triangle presents to a boundary: a planar face's fitted plane normal, or else the
/// triangle's smoothed normal (curved faces, and planar faces without a plane).
fn side_normal(g: &Grown, smooth: &[Option<Vec3>], t: usize) -> Option<Vec3> {
    let f = g.face.get(t).copied().unwrap_or(NONE) as usize;
    match g.kinds.get(f) {
        Some(FaceKind::Planar) => g.planes.get(f).copied().flatten().or_else(|| smooth.get(t).copied().flatten()),
        _ => smooth.get(t).copied().flatten(),
    }
}

/// One edge between two different faces: its end vertices, the faces on it, and the angle between the
/// two sides' normals (`None` when neither side has one, as for degenerate triangles).
struct Segment {
    a: u32,
    b: u32,
    faces: Vec<u32>,
    angle: Option<f64>,
}

/// The edges whose triangles belong to more than one face.
fn boundary_segments(adj: &Adjacency, smooth: &[Option<Vec3>], g: &Grown) -> Vec<Segment> {
    let mut out = Vec::new();
    for (e, &(a, b)) in adj.edge_key.iter().enumerate() {
        let ts = adj.edge_tris(e);
        let mut faces: Vec<u32> = ts.iter().map(|&t| g.face[t as usize]).collect();
        faces.sort_unstable();
        faces.dedup();
        if faces.len() < 2 {
            continue;
        }
        let mut angle = None;
        'pairs: for &t in ts {
            for &u in ts {
                if g.face[t as usize] != g.face[u as usize]
                    && let (Some(nt), Some(nu)) = (side_normal(g, smooth, t as usize), side_normal(g, smooth, u as usize))
                {
                    angle = Some(det_math::acos(nt.dot(nu).clamp(-1.0, 1.0)));
                    break 'pairs;
                }
            }
        }
        out.push(Segment { a, b, faces, angle });
    }
    out
}

/// A polyline of boundary: its vertices, the faces on it, and the angle of each segment that has one.
struct Line {
    points: Vec<u32>,
    faces: Vec<u32>,
    angles: Vec<f64>,
}

/// Chains the segments into polylines, each segment once. Lines run between junctions and free ends
/// first; closed loops (all junctions of two) come after.
fn chain(segs: &[Segment]) -> Vec<Line> {
    let mut adj: std::collections::HashMap<u32, Vec<usize>> = std::collections::HashMap::new();
    for (i, s) in segs.iter().enumerate() {
        adj.entry(s.a).or_default().push(i);
        adj.entry(s.b).or_default().push(i);
    }
    let mut keys: Vec<u32> = adj.keys().copied().collect();
    keys.sort_unstable();
    let mut used = vec![false; segs.len()];
    let mut lines = Vec::new();
    for loops in [false, true] {
        for &v in &keys {
            let Some(list) = adj.get(&v) else { continue };
            if (list.len() == 2) != loops {
                continue;
            }
            for &s in list {
                if !used[s] {
                    lines.push(walk(v, s, segs, &adj, &mut used));
                }
            }
        }
    }
    lines
}

/// Follows the segments from `start` along `first` until a junction or a free end, or back to `start`.
fn walk(start: u32, first: usize, segs: &[Segment], adj: &std::collections::HashMap<u32, Vec<usize>>, used: &mut [bool]) -> Line {
    let mut line = Line { points: vec![start], faces: Vec::new(), angles: Vec::new() };
    let (mut cur, mut s) = (start, first);
    loop {
        used[s] = true;
        let seg = &segs[s];
        let next = if seg.a == cur { seg.b } else { seg.a };
        line.faces.extend_from_slice(&seg.faces);
        line.angles.extend(seg.angle);
        line.points.push(next);
        cur = next;
        let Some(list) = adj.get(&cur) else { break };
        if list.len() != 2 {
            break;
        }
        match list.iter().find(|&&x| !used[x]) {
            Some(&x) => s = x,
            None => break,
        }
    }
    line.faces.sort_unstable();
    line.faces.dedup();
    line
}

/// The median of a list of angles, or `None` when it is empty.
fn median(mut v: Vec<f64>) -> Option<f64> {
    v.sort_by(f64::total_cmp);
    v.get(v.len() / 2).copied()
}

/// The one output mesh: vertices split per face, normals per face, and the boundary lines. `smooth_edges`
/// is left all false (see [`FaceSegmentation::mesh`]).
fn build_mesh(surface: &Surface, smooth: &[Option<Vec3>], g: &Grown, lines: &[Line]) -> Mesh {
    let n = surface.tris.len();
    // One entry per triangle corner, grouped by (vertex, face) once sorted.
    let mut corners: Vec<(u32, u32, u32, u8)> = Vec::with_capacity(3 * n);
    for (t, tri) in surface.tris.iter().enumerate() {
        let f = g.face[t];
        for (slot, &v) in tri.iter().enumerate() {
            corners.push((v, f, t as u32, slot as u8));
        }
    }
    corners.sort_unstable();
    let mut triangles = vec![[0u32; 3]; n];
    let mut positions: Vec<Vec3> = Vec::new();
    let mut normals: Vec<Vec3> = Vec::new();
    let mut i = 0;
    while i < corners.len() {
        let (v, f, _, _) = corners[i];
        let mut j = i;
        let mut sum = Vec3::ZERO;
        while let Some(&(cv, cf, t, _)) = corners.get(j) {
            if cv != v || cf != f {
                break;
            }
            if let Some(s) = smooth.get(t as usize).copied().flatten() {
                sum += s;
            }
            j += 1;
        }
        let index = positions.len() as u32;
        positions.push(surface.point(v));
        let normal = match g.kinds.get(f as usize) {
            Some(FaceKind::Planar) => g.planes.get(f as usize).copied().flatten().unwrap_or(Vec3::ZERO),
            _ => sum.normalized().unwrap_or(Vec3::ZERO),
        };
        normals.push(normal);
        for &(_, _, t, slot) in corners.get(i..j).unwrap_or(&[]) {
            if let Some(tri) = triangles.get_mut(t as usize) {
                tri[slot as usize] = index;
            }
        }
        i = j;
    }
    let edges: Vec<Vec<Vec3>> = lines.iter().map(|l| l.points.iter().map(|&v| surface.point(v)).collect()).collect();
    let edge_faces: Vec<Vec<u32>> = lines.iter().map(|l| l.faces.clone()).collect();
    let smooth_edges = vec![false; lines.len()];
    Mesh { positions, normals, triangles, tri_face: g.face.clone(), edges, edge_faces, smooth_edges, display_only_edges: true, ..Mesh::default() }
}

#[cfg(test)]
#[path = "faces_tests.rs"]
mod tests;
