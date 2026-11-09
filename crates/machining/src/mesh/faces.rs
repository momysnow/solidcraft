//! Face segmentation of a stock surface: the faces, and the lines between them, that a CAD viewer
//! draws for a solid.
//!
//! The surface is welded into one indexed mesh (chunk meshes share their boundary vertices bit for
//! bit), then grown into faces in three passes. Planar faces go first, from flat seeds: a triangle
//! whose neighbours all turn by at most `plane_angle` from its normal. A planar face takes every
//! neighbour whose corners lie within `plane_gap` of the seed's plane; the normal is not tested there,
//! because the corners of a plane carry noise of a few hundredths of a cell that tilts their triangles.
//! A planar face narrower than `min_plane_extent` is a facet of a curved surface and is given back.
//! Planar faces claim their triangles before any curved face grows, so a plane never grows into a
//! fillet: where a plane meets a fillet tangentially, the boundary falls at the fillet's first row. The
//! triangles left over grow curved faces across neighbours that turn by less than `crease_angle`.
//! Finally the triangles with no normal (degenerate) join a neighbour. A planar face is shaded with its
//! plane's normal.
//!
//! The boundary between two faces is an edge, drawn as a polyline. It is marked tangent when its two
//! sides turn by at most `tangent_angle`, so a fillet meeting a floor or a wall is a tangent edge and
//! a corner is not.
//!
//! Limits: a curved surface turns by about cell / radius per row. Where that is below `plane_angle`
//! (radius above about cell / plane_angle, 29 mm at a 0.25 mm cell) its rows pass as seeds, and the
//! surface splits into planar strips a few millimetres across. A tangent edge sits within about one row
//! of the true tangent line. Two curved faces that meet at a small angle without a change of type stay
//! one face, as do two curved faces of different radius that join smoothly.
//!
//! shortcut: the whole surface is segmented on each call (every mesh welded, every triangle grown), so
//! a viewer that remeshes one chunk segments the whole set again. Segment only the dirty region once
//! the viewport needs it at interactive rates.

use std::collections::{HashMap, VecDeque};
use std::f64::consts::PI;

use solidcore_record::Vec3;
use solidcraft_geom::Mesh;
use solidcraft_geom::prelude::*;

use crate::SimError;
use crate::det_math;

/// The face of a triangle that no face has claimed yet. Faces are numbered from 0, so this is never one.
const NONE: u32 = u32::MAX;

/// What decides where faces end. Angles are in radians and the gap in millimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaceTolerances {
    /// Largest angle between a seed's normal and each of its neighbours' normals: a seed is flat only
    /// when it is. Curved rows turn by about cell / radius, so this keeps them out of a plane's seed.
    pub plane_angle: f64,
    /// Largest distance from a planar face's plane to any of its vertices.
    pub plane_gap: f64,
    /// A planar face must be at least this wide across (its middle bounding-box extent), in mm. A
    /// narrower flat region is one facet of a curved surface, not a face.
    pub min_plane_extent: f64,
    /// Normals that turn by more than this across an edge split the faces there.
    pub crease_angle: f64,
    /// A boundary whose two sides turn by at most this is tangent.
    pub tangent_angle: f64,
}

impl FaceTolerances {
    /// The tolerances for a stock whose rays are `cell` mm apart. The planar gap is a quarter of a cell:
    /// the corners of a true plane sit within a few hundredths of a cell of it, and a fillet's first row
    /// is taken into the plane only when it is that close. The seed angle is half a degree, below the
    /// turn of a fillet's first row, so a fillet is not a seed. A planar face is at least four cells
    /// across. A crease is 30 degrees. A boundary turning by 15 or less is tangent, which still holds
    /// across one row of a fillet.
    pub fn for_cell(cell: f64) -> Result<FaceTolerances, SimError> {
        FaceTolerances {
            plane_angle: 0.5f64.to_radians(),
            plane_gap: 0.25 * cell,
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
            return Err(SimError::BadInput("the minimum planar extent must be a positive length".into()));
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
    /// The input meshes, in order, with their vertices split per face. `tri_face` holds each triangle's
    /// face. Normals are per face: a plane shades flat, a curved face shades smooth, and a crease keeps
    /// its sharpness, because a vertex on a crease has one copy per face with that face's normal.
    pub meshes: Vec<Mesh>,
    /// The kind of each face, by face id.
    pub kinds: Vec<FaceKind>,
    /// The boundaries between faces, each edge once, as polylines in world coordinates. A polyline runs
    /// from one junction to the next, where three or more faces meet or the input's outline ends. A
    /// closed loop has its first point again as its last.
    pub edges: Vec<Vec<Vec3>>,
    /// Per polyline: the faces it separates, ascending.
    pub edge_faces: Vec<Vec<u32>>,
    /// Per polyline: true where the two faces meet tangentially, every segment turning by at most
    /// `tangent_angle`.
    pub tangent: Vec<bool>,
}

/// Segments the surface of `meshes`, taken together as one surface, into faces and boundary lines. The
/// meshes of one stock weld along their shared vertices, so a face that crosses a chunk boundary stays
/// one face. Errors, never panics: bad tolerances, non-finite vertices and out-of-range indices are
/// refused.
pub fn segment_faces(meshes: &[Mesh], tol: &FaceTolerances) -> Result<FaceSegmentation, SimError> {
    let tol = tol.checked()?;
    let surface = Surface::weld(meshes)?;
    let normal: Vec<Option<Vec3>> = surface.tris.iter().map(|t| surface.normal(*t)).collect();
    let edges = surface.edges();
    let nbrs = neighbours(&edges, surface.tris.len());
    let (face, kinds, planes) = grow(&surface, &normal, &nbrs, tol);
    let segments = boundary_segments(&edges, &normal, &face);
    let lines = chain(&segments);
    Ok(FaceSegmentation {
        meshes: split(&surface, meshes, &face, &kinds, &planes),
        kinds,
        edges: lines.iter().map(|l| l.points.iter().map(|&v| surface.point(v)).collect()).collect(),
        edge_faces: lines.iter().map(|l| l.faces.clone()).collect(),
        tangent: lines.iter().map(|l| l.angle <= tol.tangent_angle).collect(),
    })
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
        let mut index: HashMap<[u64; 3], u32> = HashMap::new();
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
        // Each output vertex is one (position, face) pair of a triangle corner, so 3 per triangle must fit.
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

    /// The cross product of two edges of a triangle: twice its area, along its normal.
    fn area_normal(&self, t: [u32; 3]) -> Vec3 {
        let [a, b, c] = self.corners(t);
        (b - a).cross(c - a)
    }

    /// The unit normal of a triangle, or `None` when it is degenerate.
    fn normal(&self, t: [u32; 3]) -> Option<Vec3> {
        self.area_normal(t).normalized()
    }

    /// Every edge of the surface with the triangles on it, sorted by its two vertices. An edge with one
    /// triangle lies on the input's outline.
    fn edges(&self) -> Vec<((u32, u32), Vec<u32>)> {
        let mut map: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
        for (t, tri) in self.tris.iter().enumerate() {
            let [a, b, c] = *tri;
            for (p, q) in [(a, b), (b, c), (c, a)] {
                if p != q {
                    map.entry((p.min(q), p.max(q))).or_default().push(t as u32);
                }
            }
        }
        let mut edges: Vec<_> = map.into_iter().collect();
        edges.sort_unstable_by_key(|(k, _)| *k);
        edges
    }
}

/// Bit pattern of a coordinate, with -0.0 taken as 0.0: they are one point.
fn bits(v: f64) -> u64 {
    if v == 0.0 { 0 } else { v.to_bits() }
}

/// The triangles that share an edge with each triangle.
fn neighbours(edges: &[((u32, u32), Vec<u32>)], n: usize) -> Vec<Vec<u32>> {
    let mut out = vec![Vec::new(); n];
    for (_, ts) in edges {
        for &t in ts {
            for &u in ts {
                if u != t
                    && let Some(list) = out.get_mut(t as usize)
                {
                    list.push(u);
                }
            }
        }
    }
    out
}

/// Whether a triangle with normal `n` (`None` when degenerate, which never joins) lies within the angle
/// whose cosine is `cos_limit` of `reference`.
fn aligned(n: Option<Vec3>, reference: Vec3, cos_limit: f64) -> bool {
    n.is_some_and(|n| n.dot(reference) >= cos_limit)
}

/// Whether all three corners lie within `gap` of the plane through `p0` with normal `n0`.
fn within_gap(corners: [Vec3; 3], p0: Vec3, n0: Vec3, gap: f64) -> bool {
    corners.iter().all(|c| (*c - p0).dot(n0).abs() <= gap)
}

/// Grows the faces: planar ones from flat seeds, curved ones over what is left, and degenerate triangles
/// last. Returns each triangle's face and each face's kind.
fn grow(surface: &Surface, normal: &[Option<Vec3>], nbrs: &[Vec<u32>], tol: FaceTolerances) -> (Vec<u32>, Vec<FaceKind>, Vec<Option<Vec3>>) {
    let n = surface.tris.len();
    let mut face = vec![NONE; n];
    let mut kinds: Vec<FaceKind> = Vec::new();
    // The plane of each planar face, by face id; `None` for a curved face.
    let mut planes: Vec<Option<Vec3>> = Vec::new();
    let cos_plane = tol.plane_angle.cos();
    let cos_crease = tol.crease_angle.cos();
    let mut queue = VecDeque::new();
    // Seeds whose flood came out too narrow to be a face: they seed no flood of their own.
    let mut rejected = vec![false; n];

    // Planar faces. A seed is flat when every neighbour is within the planar angle of it.
    for seed in 0..n {
        let Some(n0) = normal[seed] else { continue };
        if face[seed] != NONE || rejected[seed] || !nbrs[seed].iter().all(|&u| aligned(normal[u as usize], n0, cos_plane)) {
            continue;
        }
        let id = kinds.len() as u32;
        let p0 = surface.corners(surface.tris[seed])[0];
        let mut members = vec![seed];
        face[seed] = id;
        queue.push_back(seed);
        while let Some(x) = queue.pop_front() {
            for &u in &nbrs[x] {
                let u = u as usize;
                // The gap alone decides a neighbour: a corner's noise can tilt its triangle by degrees.
                if face[u] == NONE && within_gap(surface.corners(surface.tris[u]), p0, n0, tol.plane_gap) {
                    face[u] = id;
                    members.push(u);
                    queue.push_back(u);
                }
            }
        }
        if extent(surface, &members) < tol.min_plane_extent {
            for &t in &members {
                face[t] = NONE;
            }
            rejected[seed] = true;
        } else {
            kinds.push(FaceKind::Planar);
            planes.push(Some(n0));
        }
    }

    // Curved faces: whatever no plane claimed, grown across neighbours that turn by less than the crease.
    for seed in 0..n {
        if face[seed] != NONE || normal[seed].is_none() {
            continue;
        }
        let id = kinds.len() as u32;
        kinds.push(FaceKind::Curved);
        planes.push(None);
        face[seed] = id;
        queue.push_back(seed);
        while let Some(x) = queue.pop_front() {
            let Some(nx) = normal[x] else { continue };
            for &u in &nbrs[x] {
                let u = u as usize;
                if face[u] == NONE && aligned(normal[u], nx, cos_crease) {
                    face[u] = id;
                    queue.push_back(u);
                }
            }
        }
    }

    // Degenerate triangles take a neighbour's face. A group with no face next to it is a face of its own.
    let mut queue: VecDeque<usize> = (0..n).filter(|&t| face[t] != NONE).collect();
    spread(&mut face, nbrs, &mut queue);
    for t in 0..n {
        if face[t] == NONE {
            let id = kinds.len() as u32;
            kinds.push(FaceKind::Curved);
            planes.push(None);
            face[t] = id;
            queue.push_back(t);
            spread(&mut face, nbrs, &mut queue);
        }
    }
    (face, kinds, planes)
}

/// The middle of the three bounding-box extents of the triangles' corners. For a flat patch the
/// smallest extent is its thickness, so this is its width across.
fn extent(surface: &Surface, members: &[usize]) -> f64 {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for &t in members {
        for c in surface.corners(surface.tris[t]) {
            lo = lo.min(c);
            hi = hi.max(c);
        }
    }
    let d = hi - lo;
    let mut e = [d.x, d.y, d.z];
    e.sort_by(f64::total_cmp);
    e[1]
}

/// Gives every unclaimed neighbour of the queued triangles the face of the triangle that reaches it.
fn spread(face: &mut [u32], nbrs: &[Vec<u32>], queue: &mut VecDeque<usize>) {
    while let Some(x) = queue.pop_front() {
        let f = face[x];
        for &u in &nbrs[x] {
            let u = u as usize;
            if face[u] == NONE {
                face[u] = f;
                queue.push_back(u);
            }
        }
    }
}

/// One edge between two different faces: its end vertices, the faces on it, and the angle between the
/// two sides' normals.
struct Segment {
    a: u32,
    b: u32,
    faces: Vec<u32>,
    angle: f64,
}

/// The edges whose triangles belong to more than one face.
fn boundary_segments(edges: &[((u32, u32), Vec<u32>)], normal: &[Option<Vec3>], face: &[u32]) -> Vec<Segment> {
    let mut out = Vec::new();
    for ((a, b), ts) in edges {
        let mut faces: Vec<u32> = ts.iter().map(|&t| face[t as usize]).collect();
        faces.sort_unstable();
        faces.dedup();
        if faces.len() < 2 {
            continue;
        }
        let mut angle = 0.0f64;
        for (i, &t) in ts.iter().enumerate() {
            for &u in &ts[i + 1..] {
                if face[t as usize] != face[u as usize]
                    && let (Some(nt), Some(nu)) = (normal[t as usize], normal[u as usize])
                {
                    angle = angle.max(det_math::acos(nt.dot(nu).clamp(-1.0, 1.0)));
                }
            }
        }
        out.push(Segment { a: *a, b: *b, faces, angle });
    }
    out
}

/// A polyline of boundary: its vertices, the faces on it, and the largest angle of its segments.
struct Line {
    points: Vec<u32>,
    faces: Vec<u32>,
    angle: f64,
}

/// Chains the segments into polylines, each segment once. Lines run between junctions and free ends
/// first; closed loops (all junctions of two) come after.
fn chain(segs: &[Segment]) -> Vec<Line> {
    let mut adj: HashMap<u32, Vec<usize>> = HashMap::new();
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
fn walk(start: u32, first: usize, segs: &[Segment], adj: &HashMap<u32, Vec<usize>>, used: &mut [bool]) -> Line {
    let mut line = Line { points: vec![start], faces: Vec::new(), angle: 0.0 };
    let (mut cur, mut s) = (start, first);
    loop {
        used[s] = true;
        let seg = &segs[s];
        let next = if seg.a == cur { seg.b } else { seg.a };
        line.faces.extend_from_slice(&seg.faces);
        line.angle = line.angle.max(seg.angle);
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

/// The input meshes, each with its vertices split per face. A curved face's vertex normal is the
/// area-weighted average of that face's triangles at the vertex, over the whole surface, so chunks that
/// share a vertex agree on it. A planar face's normal is its plane's.
fn split(surface: &Surface, meshes: &[Mesh], face: &[u32], kinds: &[FaceKind], planes: &[Option<Vec3>]) -> Vec<Mesh> {
    let mut sum: HashMap<(u32, u32), Vec3> = HashMap::new();
    for (t, tri) in surface.tris.iter().enumerate() {
        let raw = surface.area_normal(*tri);
        for &v in tri {
            *sum.entry((v, face[t])).or_insert(Vec3::ZERO) += raw;
        }
    }
    let mut out = Vec::with_capacity(meshes.len());
    let mut offset = 0usize;
    for m in meshes {
        let mut mesh = Mesh::default();
        let mut vertex: HashMap<(u32, u32), u32> = HashMap::new();
        for (t, corners) in surface.tris.iter().enumerate().skip(offset).take(m.triangles.len()) {
            let f = face[t];
            let tri = corners.map(|v| {
                *vertex.entry((v, f)).or_insert_with(|| {
                    // The vertex count is below NONE, checked in `weld`.
                    let id = mesh.positions.len() as u32;
                    mesh.positions.push(surface.point(v));
                    // A planar face shades with its plane, even where a tilted triangle sits in it.
                    let normal = match (kinds.get(f as usize), planes.get(f as usize).copied().flatten()) {
                        (Some(FaceKind::Planar), Some(n)) => n,
                        _ => sum.get(&(v, f)).and_then(|s| s.normalized()).unwrap_or(Vec3::ZERO),
                    };
                    mesh.normals.push(normal);
                    id
                })
            });
            mesh.triangles.push(tri);
            mesh.tri_face.push(f);
        }
        offset += m.triangles.len();
        out.push(mesh);
    }
    out
}

#[cfg(test)]
#[path = "faces_tests.rs"]
mod tests;
