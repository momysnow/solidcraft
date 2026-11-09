//! Face segmentation of a stock surface: the faces, and the lines between them, that a CAD viewer
//! draws for a solid.
//!
//! The surface is welded into one indexed mesh (chunk meshes share their boundary vertices bit for
//! bit). Each triangle's normal is first smoothed over about `SMOOTH_STEPS` rings of triangles, leaving
//! out neighbours that turn by more than 60 degrees. A faceted cylinder then reads as one smooth
//! curve, while a 90 degree crease stays sharp.
//!
//! Planar faces grow first, from flat seeds (every neighbour's smoothed normal within `plane_angle`).
//! A neighbour joins when its corners lie within `plane_gap` of the least-squares plane through the
//! members so far, refitted as the face grows, so a tilted wall is one face. A planar face must be at
//! least `min_plane_extent` wide across two in-plane directions, and a flood that fails the fit or the
//! width gives its triangles back to the curved pass. Curved faces then grow over what is left, across
//! neighbours whose smoothed normals turn by less than `crease_angle`. Curved faces under
//! `MIN_FACE_TRIS` triangles, or narrower than `min_plane_extent`, merge into the neighbour across the
//! smallest normal angle. Degenerate triangles (no normal) join a neighbour's face.
//!
//! The boundary between two faces is an edge polyline. It is tangent when the median angle between the
//! smoothed normals on its two sides is at most `tangent_angle`.
//!
//! The output is one [`Mesh`]: vertices split per face, `tri_face` set, and `edges`, `edge_faces` and
//! `smooth_edges` filled, with `display_only_edges` set. A planar face shades with its plane's normal,
//! a curved face with its smoothed normals, so a crease keeps its sharpness.
//!
//! Limits: a curved surface turns by about cell / radius per cell. Where that is below `plane_angle`
//! (radius above about cell / plane_angle, 29 mm at a 0.25 mm cell) its rows pass as seeds, and it can
//! split into planar patches a few millimetres across. A tangent edge sits within about one row of the
//! true tangent line, since the planar gap admits the first row. Features narrower than about
//! `SMOOTH_STEPS` triangle rings are smoothed over, so they can merge into a neighbour.
//!
//! shortcut: the whole surface is segmented on each call (every mesh welded, every triangle grown), so
//! a viewer that remeshes one chunk segments the whole set again. Segment only the dirty region once
//! the viewport needs it at interactive rates.

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
/// Smoothing leaves out neighbours that turn by more than 60 degrees (cosine 0.5): a crease is not averaged.
const FEATURE_COS: f64 = 0.5;
/// A curved face with fewer triangles than this is a sliver and joins a neighbour.
const MIN_FACE_TRIS: usize = 32;
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
    /// A planar face must be at least this wide across two in-plane directions, and a curved face at
    /// least this large. A narrower flat region is a facet of a curved surface, not a face.
    pub min_plane_extent: f64,
    /// Smoothed normals that turn by more than this across an edge split the faces there.
    pub crease_angle: f64,
    /// A boundary whose two sides turn by at most this (median along the edge) is tangent.
    pub tangent_angle: f64,
}

impl FaceTolerances {
    /// The tolerances for a stock whose rays are `cell` mm apart. The planar gap is 0.35 of a cell:
    /// the corners of a true plane, and of the rim where a plane meets a curve, sit within about a third
    /// of a cell of it. The seed angle is half a
    /// degree. A planar face and a curved face are at least four cells across. A crease is 30 degrees,
    /// and a boundary turning by 15 or less is tangent, which still holds across one row of a fillet.
    pub fn for_cell(cell: f64) -> Result<FaceTolerances, SimError> {
        FaceTolerances {
            plane_angle: 0.5f64.to_radians(),
            plane_gap: 0.35 * cell,
            min_plane_extent: 4.0 * cell,
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
    /// polylines in world coordinates (closed loops repeat their first point), `edge_faces` the faces
    /// each one separates, and `smooth_edges` the tangent ones. Vertices are split per face.
    pub mesh: Mesh,
    /// The kind of each face, by face id.
    pub kinds: Vec<FaceKind>,
}

/// Segments the surface of `meshes`, taken together as one surface, into faces and boundary lines.
/// Chunk meshes of one stock weld along their shared vertices, so a face that crosses a chunk boundary
/// stays one face. Errors, never panics: bad tolerances, non-finite vertices, out-of-range indices and a
/// surface over `budget_bytes` (by estimate) are refused.
pub fn segment_faces(meshes: &[Mesh], tol: &FaceTolerances, budget_bytes: u64) -> Result<FaceSegmentation, SimError> {
    let tol = tol.checked()?;
    let surface = Surface::weld(meshes)?;
    let need = surface.tris.len() as u64 * BYTES_PER_TRI + surface.pos.len() as u64 * BYTES_PER_VERTEX;
    if need > budget_bytes {
        return Err(SimError::Budget { need, budget: budget_bytes });
    }
    let raw: Vec<Option<Vec3>> = surface.tris.iter().map(|t| surface.normal(*t)).collect();
    let adj = Adjacency::build(&surface);
    let smooth = smooth_normals(&raw, &adj);
    let mut grown = grow(&surface, &smooth, &adj, tol);
    merge_slivers(&surface, &smooth, &adj, &mut grown, tol);
    compact(&mut grown);
    let segments = boundary_segments(&adj, &smooth, &grown.face);
    let lines = chain(&segments);
    let mesh = build_mesh(&surface, &smooth, &grown, &lines, tol);
    Ok(FaceSegmentation { mesh, kinds: grown.kinds })
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

/// Running moments of a set of points, for their least-squares plane: the count, the sum and the sum of
/// outer products, all relative to an origin near the points so the covariance keeps its precision.
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

    /// The least-squares plane: its point (the centroid) and unit normal (the direction of least
    /// variance). `None` with fewer than three points or a singular solve.
    fn plane(&self) -> Option<(Vec3, Vec3)> {
        if self.n < 3.0 {
            return None;
        }
        let mean = self.sum / self.n;
        let mu: [f64; 3] = mean.into();
        let mut cov = [[0.0f64; 3]; 3];
        for (r, row) in cov.iter_mut().enumerate() {
            for (c, cell) in row.iter_mut().enumerate() {
                *cell = self.sq[r][c] / self.n - mu[r] * mu[c];
            }
        }
        Some((self.origin + mean, smallest_eigenvector(cov)?))
    }
}

/// The unit eigenvector of the smallest eigenvalue of a symmetric 3 by 3 matrix, by cyclic Jacobi
/// rotations. `None` when the result is not finite.
fn smallest_eigenvector(mut m: [[f64; 3]; 3]) -> Option<Vec3> {
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
    let i = (0..3).min_by(|&a, &b| m[a][a].total_cmp(&m[b][b]))?;
    Vec3::new(v[0][i], v[1][i], v[2][i]).normalized()
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
        let mut plane: Option<(Vec3, Vec3)> = m.plane();
        let mut since_refit = 0u32;
        g.face[seed] = id;
        queue.push_back(seed);
        while let Some(x) = queue.pop_front() {
            for &u in adj.nbrs(x) {
                let u = u as usize;
                if g.face[u] != NONE || rejected[u] {
                    continue;
                }
                let cs = surface.corners(surface.tris[u]);
                m.update_all(cs, 1.0);
                let within = |p: Option<(Vec3, Vec3)>| p.is_some_and(|(c, nrm)| cs.iter().all(|q| (*q - c).dot(nrm).abs() <= tol.plane_gap));
                let fits = if within(plane) && since_refit < REFIT_EVERY {
                    true
                } else {
                    plane = m.plane();
                    since_refit = 0;
                    within(plane)
                };
                if fits {
                    since_refit += 1;
                    g.face[u] = id;
                    members.push(u);
                    queue.push_back(u);
                } else {
                    m.update_all(cs, -1.0);
                }
            }
        }
        // Trim: a member whose corners stray beyond the gap from the refitted plane goes back to the
        // curved pass, and the plane is refitted, for a few rounds.
        let mut oriented = plane.map(|(c, nrm)| (c, if nrm.dot(sn) < 0.0 { -nrm } else { nrm }));
        for _ in 0..3 {
            let Some((c, nrm)) = oriented else { break };
            let mut bad: Vec<usize> = members
                .iter()
                .copied()
                .filter(|&t| surface.corners(surface.tris[t]).iter().any(|q| (*q - c).dot(nrm).abs() > tol.plane_gap))
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
            oriented = m.plane().map(|(c, nrm)| (c, if nrm.dot(sn) < 0.0 { -nrm } else { nrm }));
        }
        let verified = oriented.is_some_and(|(c, nrm)| {
            members.iter().all(|&t| surface.corners(surface.tris[t]).iter().all(|q| (*q - c).dot(nrm).abs() <= tol.plane_gap))
        });
        match oriented {
            Some((c, nrm)) if verified && width(surface, &members, c, nrm) >= tol.min_plane_extent => {
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

/// The width of a planar face across two in-plane directions: the smaller of its two extents.
fn width(surface: &Surface, members: &[usize], c: Vec3, nrm: Vec3) -> f64 {
    let e1 = nrm.any_perp();
    let e2 = nrm.cross(e1);
    [e1, e2]
        .iter()
        .map(|e| {
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for &t in members {
                for q in surface.corners(surface.tris[t]) {
                    let d = (q - c).dot(*e);
                    lo = lo.min(d);
                    hi = hi.max(d);
                }
            }
            hi - lo
        })
        .fold(f64::INFINITY, f64::min)
}

/// Merges each curved face that is a sliver (under `MIN_FACE_TRIS` triangles, or narrower than
/// `min_plane_extent`) into the neighbour across its smallest normal angle. A few passes let a chain of
/// slivers settle.
fn merge_slivers(surface: &Surface, smooth: &[Option<Vec3>], adj: &Adjacency, g: &mut Grown, tol: FaceTolerances) {
    let n = surface.tris.len();
    for _ in 0..4 {
        let k = g.kinds.len();
        let mut count = vec![0usize; k];
        let mut lo = vec![Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY); k];
        let mut hi = vec![Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY); k];
        for t in 0..n {
            let f = g.face[t] as usize;
            count[f] += 1;
            for q in surface.corners(surface.tris[t]) {
                lo[f] = lo[f].min(q);
                hi[f] = hi[f].max(q);
            }
        }
        let small: Vec<bool> = (0..k)
            .map(|f| {
                let d = hi[f] - lo[f];
                let widest = d.x.max(d.y).max(d.z);
                g.kinds[f] == FaceKind::Curved && count[f] > 0 && (count[f] < MIN_FACE_TRIS || widest < tol.min_plane_extent)
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
                if other as usize == f {
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

/// One edge between two different faces: its end vertices, the faces on it, and the angle between the
/// two sides' smoothed normals (`None` when neither side has a normal, as for degenerate triangles).
struct Segment {
    a: u32,
    b: u32,
    faces: Vec<u32>,
    angle: Option<f64>,
}

/// The edges whose triangles belong to more than one face.
fn boundary_segments(adj: &Adjacency, smooth: &[Option<Vec3>], face: &[u32]) -> Vec<Segment> {
    let mut out = Vec::new();
    for (e, &(a, b)) in adj.edge_key.iter().enumerate() {
        let ts = adj.edge_tris(e);
        let mut faces: Vec<u32> = ts.iter().map(|&t| face[t as usize]).collect();
        faces.sort_unstable();
        faces.dedup();
        if faces.len() < 2 {
            continue;
        }
        let mut angle = None;
        'pairs: for &t in ts {
            for &u in ts {
                if face[t as usize] != face[u as usize]
                    && let (Some(nt), Some(nu)) = (smooth[t as usize], smooth[u as usize])
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

/// The one output mesh: vertices split per face, normals per face, and the boundary lines.
fn build_mesh(surface: &Surface, smooth: &[Option<Vec3>], g: &Grown, lines: &[Line], tol: FaceTolerances) -> Mesh {
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
    let smooth_edges: Vec<bool> = lines.iter().map(|l| median(l.angles.clone()).is_some_and(|m| m <= tol.tangent_angle)).collect();
    Mesh { positions, normals, triangles, tri_face: g.face.clone(), edges, edge_faces, smooth_edges, display_only_edges: true, ..Mesh::default() }
}

#[cfg(test)]
#[path = "faces_tests.rs"]
mod tests;
