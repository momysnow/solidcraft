use std::collections::HashMap;
use std::f64::consts::FRAC_PI_2;

use solidcore_record::Vec3;
use solidcraft_geom::Mesh;
use solidcraft_geom::prelude::*;

use super::{FaceKind, FaceSegmentation, FaceTolerances, segment_faces};
use crate::SimError;
use crate::dexel::Stock;
use crate::mesh::chunk_meshes;
use crate::sweep::{FlatTool, LinearFixedAxis};

const BUDGET: u64 = 1 << 33;

fn slot_stock(h: f64, r: f64) -> Stock {
    let mut s = Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(30.0, 30.0, 0.0), h, 1 << 32).expect("stock");
    let tool = FlatTool::new(r, 20.0, Vec3::Z).expect("tool");
    s.apply(&LinearFixedAxis { from: Vec3::new(8.0, 15.0, -4.0), to: Vec3::new(22.0, 15.0, -4.0), tool }, 1).expect("cut");
    s
}

fn plunged_pocket(h: f64) -> Stock {
    let mut s = Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(30.0, 30.0, 0.0), h, 1 << 32).expect("stock");
    let tool = FlatTool::new(5.0, 20.0, Vec3::Z).expect("tool");
    s.apply(&LinearFixedAxis { from: Vec3::new(15.0, 15.0, 5.0), to: Vec3::new(15.0, 15.0, -4.0), tool }, 1).expect("plunge");
    s
}

fn segment_stock(s: &Stock, h: f64) -> FaceSegmentation {
    let tol = FaceTolerances::for_cell(h).expect("tolerances");
    segment_faces(&chunk_meshes(s), &tol, BUDGET).expect("segment")
}

fn xyz(p: Vec3) -> [f64; 3] {
    p.into()
}

/// The vertices of each face.
fn points_by_face(seg: &FaceSegmentation) -> HashMap<u32, Vec<Vec3>> {
    let m = &seg.mesh;
    let mut out: HashMap<u32, Vec<Vec3>> = HashMap::new();
    for (tri, f) in m.triangles.iter().zip(&m.tri_face) {
        for &v in tri {
            out.entry(*f).or_default().push(m.positions[v as usize]);
        }
    }
    out
}

/// The faces of `kind` whose every vertex satisfies `pred`, ascending.
fn faces_where(seg: &FaceSegmentation, kind: FaceKind, pred: impl Fn(Vec3) -> bool) -> Vec<u32> {
    let mut faces: Vec<u32> =
        points_by_face(seg).into_iter().filter(|(f, ps)| seg.kinds[*f as usize] == kind && ps.iter().all(|p| pred(*p))).map(|(f, _)| f).collect();
    faces.sort_unstable();
    faces
}

/// The planar faces whose every vertex lies within `tol` of `value` along `axis`.
fn planar_faces_on(seg: &FaceSegmentation, axis: usize, value: f64, tol: f64) -> Vec<u32> {
    faces_where(seg, FaceKind::Planar, |p| (xyz(p)[axis] - value).abs() <= tol)
}

/// The index of the first polyline with a point that `pred` accepts.
fn line_with(seg: &FaceSegmentation, pred: impl Fn(Vec3) -> bool) -> Option<usize> {
    seg.mesh.edges.iter().position(|l| l.iter().any(|p| pred(*p)))
}

/// The number of triangles of each face.
fn triangles_by_face(seg: &FaceSegmentation) -> HashMap<u32, usize> {
    let mut out: HashMap<u32, usize> = HashMap::new();
    for f in &seg.mesh.tri_face {
        *out.entry(*f).or_default() += 1;
    }
    out
}

/// Asserts that the output is well formed: every triangle has a face, every edge has two or more
/// points, and every edge names faces that exist.
fn assert_well_formed(seg: &FaceSegmentation) {
    let m = &seg.mesh;
    assert_eq!(m.tri_face.len(), m.triangles.len());
    assert!(m.tri_face.iter().all(|f| (*f as usize) < seg.kinds.len()), "a face id is out of range");
    assert!(m.triangles.iter().all(|t| t.iter().all(|v| (*v as usize) < m.positions.len())));
    assert_eq!(m.normals.len(), m.positions.len());
    assert_eq!(m.edges.len(), m.edge_faces.len());
    assert_eq!(m.edges.len(), m.smooth_edges.len());
    assert!(m.edges.iter().all(|l| l.len() >= 2 && l.iter().all(|p| p.is_finite())));
    assert!(m.display_only_edges);
}

#[test]
fn flat_slot_at_0_25_is_eleven_faces_with_planar_floor_and_walls() {
    let h = 0.25;
    let seg = segment_stock(&slot_stock(h, 2.5), h);
    assert_well_formed(&seg);
    // Six box faces, the floor, two planar walls and two curved ends.
    assert_eq!(seg.kinds.len(), 11, "faces {:?}", seg.kinds);
    let tol = 0.25 * h;
    assert_eq!(planar_faces_on(&seg, 2, -4.0, tol).len(), 1, "floor");
    assert_eq!(planar_faces_on(&seg, 1, 12.5, tol).len(), 1, "wall at y 12.5");
    assert_eq!(planar_faces_on(&seg, 1, 17.5, tol).len(), 1, "wall at y 17.5");
    assert_eq!(seg.kinds.iter().filter(|k| **k == FaceKind::Curved).count(), 2, "two curved ends");

    // The floor and wall meet at a right angle along the slot: a crease, not tangent.
    let on_floor_wall = |p: Vec3| {
        let [x, y, z] = xyz(p);
        (y - 12.5).abs() <= tol && (z + 4.0).abs() <= tol && (7.5..=22.5).contains(&x)
    };
    let crease = line_with(&seg, on_floor_wall).expect("a floor-wall edge");
    assert!(!seg.mesh.smooth_edges[crease], "a floor meeting a wall at 90 degrees is not tangent");
    let mut xs: Vec<f64> = seg.mesh.edges.iter().flatten().filter(|p| on_floor_wall(**p)).map(|p| xyz(*p)[0]).collect();
    xs.sort_by(f64::total_cmp);
    assert!(xs[0] <= 8.5 && *xs.last().expect("x") >= 21.5, "floor-wall edge spans {:?}..{:?}", xs.first(), xs.last());

    // The wall meets the rounded end tangentially: that edge is tangent.
    let on_end = |p: Vec3| {
        let [x, y, z] = xyz(p);
        (x - 8.0).abs() <= 0.5 && (y - 12.5).abs() <= tol && z > -3.5 && z < -0.5
    };
    let tangent_line = line_with(&seg, on_end).expect("the wall meets the slot end");
    assert!(seg.mesh.smooth_edges[tangent_line], "a wall meeting the rounded end is a tangent edge");
    assert_eq!(seg.mesh.edge_faces[tangent_line].len(), 2);
}

#[test]
fn round_slot_at_0_35_with_radius_3_keeps_the_end_side_edge_tangent() {
    let h = 0.35;
    let seg = segment_stock(&slot_stock(h, 3.0), h);
    assert_well_formed(&seg);
    assert_eq!(seg.kinds.len(), 11, "faces {:?}", seg.kinds);
    let tol = 0.35 * h;
    assert_eq!(planar_faces_on(&seg, 2, -4.0, tol).len(), 1, "floor");
    assert_eq!(planar_faces_on(&seg, 1, 12.0, tol).len(), 1, "wall at y 12");
    assert_eq!(planar_faces_on(&seg, 1, 18.0, tol).len(), 1, "wall at y 18");
    let on_end = |p: Vec3| {
        let [x, y, z] = xyz(p);
        (x - 8.0).abs() <= 1.0 && (y - 12.0).abs() <= 0.6 * h && z > -3.5 && z < -0.5
    };
    let end_edge = line_with(&seg, on_end).expect("the wall meets the slot end");
    assert!(seg.mesh.smooth_edges[end_edge], "the end-to-side edge is tangent");
}

#[test]
fn circular_pocket_is_eight_faces_with_one_curved_wall_at_every_resolution() {
    for h in [0.25, 0.35, 0.5] {
        let seg = segment_stock(&plunged_pocket(h), h);
        assert_well_formed(&seg);
        // Six box faces, the floor and the wall.
        assert_eq!(seg.kinds.len(), 8, "h {h}: faces {:?}", seg.kinds);
        let r = 5.0;
        let rho = |p: Vec3| {
            let [x, y, _] = xyz(p);
            ((x - 15.0).powi(2) + (y - 15.0).powi(2)).sqrt()
        };
        let wall_tol = 0.1 * h;
        let wall = faces_where(&seg, FaceKind::Curved, |p| {
            let z = xyz(p)[2];
            (rho(p) - r).abs() <= wall_tol && z >= -4.0 - wall_tol && z <= wall_tol
        });
        assert_eq!(wall.len(), 1, "h {h}: curved wall faces {wall:?}");
        let floor = faces_where(&seg, FaceKind::Planar, |p| (xyz(p)[2] + 4.0).abs() <= wall_tol && rho(p) <= r + wall_tol);
        assert_eq!(floor.len(), 1, "h {h}: floor faces {floor:?}");
        assert_eq!(planar_faces_on(&seg, 2, 0.0, 0.35 * h).len(), 1, "h {h}: one top face");

        // The wall's floor and top rims are closed circles at right angles: creases, not tangent.
        for z in [-4.0, 0.0] {
            let i = seg
                .mesh
                .edges
                .iter()
                .position(|l| {
                    l.len() > 8 && l.first() == l.last() && l.iter().all(|p| (rho(*p) - r).abs() <= wall_tol && (xyz(*p)[2] - z).abs() <= wall_tol)
                })
                .unwrap_or_else(|| panic!("h {h}: no closed circle edge at z {z}"));
            let length: f64 = seg.mesh.edges[i].windows(2).map(|w| (w[1] - w[0]).len()).sum();
            let circle = 2.0 * std::f64::consts::PI * r;
            assert!((length - circle).abs() < 0.02 * circle, "h {h}: circle at z {z} length {length} vs {circle}");
            assert!(!seg.mesh.smooth_edges[i], "h {h}: a wall meets its floor and top at right angles");
        }
    }
}

#[test]
fn tilted_tool_slot_has_one_planar_face_per_wall_and_no_slivers() {
    // A 2.5 mm flat tool whose axis is tilted 7 degrees in the x-z plane, moved along y through a box.
    // Its floor is the tool's tip plane and its walls are planes at 7 degrees, so every wall is one
    // planar face, and no curved face or sliver should appear.
    let h = 0.25;
    let tilt = 7f64.to_radians();
    let axis = Vec3::new(tilt.sin(), 0.0, tilt.cos());
    let normal = Vec3::new(tilt.cos(), 0.0, -tilt.sin());
    let from = Vec3::new(15.0, -5.0, -4.0);
    let tool = FlatTool::new(2.5, 20.0, axis).expect("tool");
    let mut s = Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(40.0, 40.0, 0.0), h, 1 << 33).expect("stock");
    s.apply(&LinearFixedAxis { from, to: Vec3::new(15.0, 45.0, -4.0), tool }, 1).expect("cut");
    let seg = segment_stock(&s, h);
    assert_well_formed(&seg);
    let tol = 0.35 * h;
    let dist_normal = |p: Vec3| (p - from).dot(normal);
    let dist_axis = |p: Vec3| (p - from).dot(axis);
    let walls_pos = faces_where(&seg, FaceKind::Planar, |p| (dist_normal(p) - 2.5).abs() <= tol);
    let walls_neg = faces_where(&seg, FaceKind::Planar, |p| (dist_normal(p) + 2.5).abs() <= tol);
    let floor = faces_where(&seg, FaceKind::Planar, |p| dist_axis(p).abs() <= tol);
    assert_eq!(walls_pos.len(), 1, "one planar face on the +wall: {walls_pos:?}");
    assert_eq!(walls_neg.len(), 1, "one planar face on the -wall: {walls_neg:?}");
    assert_eq!(floor.len(), 1, "one planar floor: {floor:?}");
    assert!(seg.kinds.iter().all(|k| *k == FaceKind::Planar), "a through slot has no curved face: {:?}", seg.kinds);
    let counts = triangles_by_face(&seg);
    let smallest = counts.values().copied().min().unwrap_or(0);
    assert!(smallest >= 16, "a sliver of {smallest} triangles: {counts:?}");
}

/// A synthetic plane, quarter-cylinder fillet and plane: the floor (z = 0, x from -10 to 0), a fillet
/// of radius 2 about (0, 2) in the x-z plane, and the wall (x = 2, z from 2 to 6), extruded along y
/// from 0 to 4 in four steps. Each quad is oriented to face its air side.
fn fillet_mesh() -> Mesh {
    let (r, steps, ydiv) = (2.0f64, 8usize, 4usize);
    // The profile in (x, z): the floor in `steps` pieces from x -10 to 0, the arc of radius `r` about
    // (0, 2) in `steps` pieces, and the wall in `steps` pieces from z 2 to 6. The joints are exact, so
    // the pieces share their vertices bit for bit.
    let mut profile: Vec<(f64, f64)> = Vec::new();
    for k in 0..=steps {
        profile.push((-10.0 + 10.0 * k as f64 / steps as f64, 0.0));
    }
    for k in 1..steps {
        let th = -FRAC_PI_2 + FRAC_PI_2 * (k as f64) / (steps as f64);
        profile.push((r * th.cos(), 2.0 + r * th.sin()));
    }
    for k in 0..=steps {
        profile.push((2.0, 2.0 + 4.0 * k as f64 / steps as f64));
    }
    let mut m = Mesh::default();
    for w in profile.windows(2) {
        let ((px, pz), (qx, qz)) = (w[0], w[1]);
        // The air side: up for the floor, toward the arc's centre for the fillet, and -x for the wall.
        let want = if pz == 0.0 && qz == 0.0 {
            (0.0, 0.0, 1.0)
        } else if px == 2.0 && qx == 2.0 {
            (-1.0, 0.0, 0.0)
        } else {
            let (mx, mz) = ((px + qx) / 2.0, (pz + qz) / 2.0);
            let (dx, dz) = (0.0 - mx, 2.0 - mz);
            let len = (dx * dx + dz * dz).sqrt();
            (dx / len, 0.0, dz / len)
        };
        for j in 0..ydiv {
            let (y0, y1) = (4.0 * j as f64 / ydiv as f64, 4.0 * (j + 1) as f64 / ydiv as f64);
            let corners = [Vec3::new(px, y0, pz), Vec3::new(qx, y0, qz), Vec3::new(qx, y1, qz), Vec3::new(px, y1, pz)];
            let base = m.positions.len() as u32;
            m.positions.extend(corners);
            let flip = (corners[1] - corners[0]).cross(corners[2] - corners[0]).dot(Vec3::new(want.0, want.1, want.2)) < 0.0;
            let tris = if flip { [[0, 2, 1], [0, 3, 2]] } else { [[0, 1, 2], [0, 2, 3]] };
            m.triangles.extend(tris.map(|t| t.map(|k| base + k)));
        }
    }
    m.normals = vec![Vec3::ZERO; m.positions.len()];
    m.tri_face = vec![0; m.triangles.len()];
    m
}

#[test]
fn a_fillet_between_a_floor_and_a_wall_gives_three_faces_and_two_tangent_edges() {
    let seg = segment_faces(&[fillet_mesh()], &FaceTolerances::for_cell(0.5).expect("tol"), BUDGET).expect("segment");
    assert_well_formed(&seg);
    assert_eq!(seg.kinds.len(), 3, "faces {:?}", seg.kinds);
    assert_eq!(seg.kinds.iter().filter(|k| **k == FaceKind::Planar).count(), 2);
    assert_eq!(seg.kinds.iter().filter(|k| **k == FaceKind::Curved).count(), 1);
    assert_eq!(seg.mesh.edges.len(), 2, "edges {:?}", seg.mesh.edges.len());
    assert!(seg.mesh.smooth_edges.iter().all(|t| *t), "both fillet edges are tangent: {:?}", seg.mesh.smooth_edges);

    // The planar faces take the first fillet row on each side (the gap admits its corners), so each
    // tangent edge sits one row off the true tangent line: the floor-fillet edge near (x 0, z 0) and
    // the fillet-wall edge near (x 2, z 2). Each runs the full length of the extrusion.
    let near = |line: &[Vec3], x: (f64, f64), z: (f64, f64)| {
        line.iter().all(|p| {
            let [px, _, pz] = xyz(*p);
            (x.0..=x.1).contains(&px) && (z.0..=z.1).contains(&pz)
        })
    };
    let floor_line = seg.mesh.edges.iter().position(|l| near(l, (-1e-9, 1.0), (-1e-9, 0.25))).expect("floor-fillet edge");
    let wall_line = seg.mesh.edges.iter().position(|l| near(l, (1.5, 2.0 + 1e-9), (1.0, 2.0 + 1e-9))).expect("fillet-wall edge");
    for i in [floor_line, wall_line] {
        let ys: Vec<f64> = seg.mesh.edges[i].iter().map(|p| xyz(*p)[1]).collect();
        assert!(ys.iter().any(|y| y.abs() < 1e-9) && ys.iter().any(|y| (y - 4.0).abs() < 1e-9), "edge spans {ys:?}");
    }
    assert_ne!(floor_line, wall_line);
}

#[test]
fn split_vertices_shade_each_face_on_its_own() {
    let seg = segment_faces(&[fillet_mesh()], &FaceTolerances::for_cell(0.5).expect("tol"), BUDGET).expect("segment");
    let m = &seg.mesh;
    // The tangent-edge vertex on the floor side, at the fillet's first row (x about 0.39, z about 0.04),
    // is two copies: the floor's, which shades flat, and the fillet's, which turns away from it.
    let copies: Vec<Vec3> = m
        .positions
        .iter()
        .enumerate()
        .filter(|(_, p)| {
            let [x, y, z] = xyz(**p);
            (0.5..=1.0).contains(&x) && (y - 2.0).abs() < 1e-9 && (0.0..=0.3).contains(&z)
        })
        .map(|(i, _)| m.normals[i])
        .collect();
    assert_eq!(copies.len(), 2, "copies of the tangent-edge vertex: {copies:?}");
    assert!(copies.iter().any(|n| n.z > 0.999), "the floor's copy shades flat: {copies:?}");
    assert!(copies.iter().any(|n| n.z < 0.999), "the fillet's copy turns: {copies:?}");
    for (i, n) in m.normals.iter().enumerate() {
        assert!((n.len() - 1.0).abs() < 1e-9 || *n == Vec3::ZERO, "vertex {i} normal {n:?} is not unit length");
    }
}

#[test]
fn a_degenerate_triangle_joins_a_neighbour_and_makes_no_edge() {
    let mut m = fillet_mesh();
    let before = m.triangles.len();
    // A sliver on the edge between vertices 0 and 1: it has no normal and must join the floor.
    let mid = m.positions.len() as u32;
    m.positions.push((m.positions[0] + m.positions[1]) * 0.5);
    m.triangles.push([0, mid, 1]);
    let seg = segment_faces(&[m], &FaceTolerances::for_cell(0.5).expect("tol"), BUDGET).expect("segment");
    assert_well_formed(&seg);
    assert_eq!(seg.kinds.len(), 3, "faces {:?}", seg.kinds);
    assert_eq!(seg.mesh.edges.len(), 2, "a degenerate triangle must not add an edge");
    assert_eq!(seg.mesh.triangles.len(), before + 1, "every triangle is kept");
}

#[test]
fn a_four_triangle_edge_segments_without_panicking() {
    // Four triangles share one edge, each in its own direction: a non-manifold edge.
    let a = Vec3::new(0.0, 0.0, 0.0);
    let b = Vec3::new(1.0, 0.0, 0.0);
    let apexes = [Vec3::new(0.5, 1.0, 0.0), Vec3::new(0.5, -1.0, 0.0), Vec3::new(0.5, 0.0, 1.0), Vec3::new(0.5, 0.0, -1.0)];
    let m = Mesh {
        positions: vec![a, b, apexes[0], apexes[1], apexes[2], apexes[3]],
        triangles: vec![[0, 1, 2], [0, 1, 3], [0, 1, 4], [0, 1, 5]],
        ..Mesh::default()
    };
    let seg = segment_faces(&[m], &FaceTolerances::for_cell(0.1).expect("tol"), BUDGET).expect("segment");
    assert_well_formed(&seg);
    assert_eq!(seg.mesh.triangles.len(), 4);
    let shared = seg.mesh.edges.iter().any(|l| l.iter().any(|p| (*p - a).len() < 1e-9) && l.iter().any(|p| (*p - b).len() < 1e-9) && l.len() >= 2);
    assert!(shared, "the shared edge is a boundary between faces: {:?}", seg.mesh.edges);
}

#[test]
fn a_junction_of_three_faces_gives_three_creases_at_one_vertex() {
    // Three 1 mm square faces meeting at the origin like the inside corner of a pocket. Each is
    // subdivided 4 by 4 so that it has interior seeds.
    let k = 4usize;
    let mut m = Mesh::default();
    let mut grid = |o: Vec3, u: Vec3, v: Vec3, want: Vec3| {
        for i in 0..k {
            for j in 0..k {
                let p = |a: usize, b: usize| o + u * (a as f64 / k as f64) + v * (b as f64 / k as f64);
                let c = [p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1)];
                let base = m.positions.len() as u32;
                m.positions.extend(c);
                let flip = (c[1] - c[0]).cross(c[2] - c[0]).dot(want) < 0.0;
                let tris = if flip { [[0, 2, 1], [0, 3, 2]] } else { [[0, 1, 2], [0, 2, 3]] };
                m.triangles.extend(tris.map(|t| t.map(|q| base + q)));
            }
        }
    };
    let zero = Vec3::ZERO;
    grid(zero, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    grid(zero, Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0));
    grid(zero, Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    let seg = segment_faces(&[m], &FaceTolerances::for_cell(0.1).expect("tol"), BUDGET).expect("segment");
    assert_well_formed(&seg);
    assert_eq!(seg.kinds.len(), 3, "faces {:?}", seg.kinds);
    assert_eq!(seg.mesh.edges.len(), 3, "one edge per pair of faces: {:?}", seg.mesh.edges.len());
    assert!(seg.mesh.smooth_edges.iter().all(|t| !t), "right-angle creases are not tangent");
    assert!(seg.mesh.edge_faces.iter().all(|f| f.len() == 2));
    assert!(seg.mesh.edges.iter().all(|l| l.iter().any(|p| p.len() < 1e-9)), "every edge reaches the junction");
}

#[test]
fn refuses_bad_tolerances_hostile_meshes_and_over_budget_without_panicking() {
    assert!(FaceTolerances::for_cell(0.0).is_err());
    assert!(FaceTolerances::for_cell(f64::NAN).is_err());
    let good = FaceTolerances::for_cell(0.25).expect("tol");
    assert!(segment_faces(&[], &FaceTolerances { plane_angle: 0.0, ..good }, BUDGET).is_err());
    assert!(segment_faces(&[], &FaceTolerances { crease_angle: 4.0, ..good }, BUDGET).is_err());
    assert!(segment_faces(&[], &FaceTolerances { plane_gap: f64::INFINITY, ..good }, BUDGET).is_err());
    let mut bad_index = fillet_mesh();
    bad_index.triangles[0][0] = 1_000_000;
    assert!(segment_faces(&[bad_index], &good, BUDGET).is_err());
    let mut nan = fillet_mesh();
    nan.positions[0] = Vec3::new(f64::NAN, 0.0, 0.0);
    assert!(segment_faces(&[nan], &good, BUDGET).is_err());
    let over = segment_faces(&[fillet_mesh()], &good, 1);
    assert!(matches!(over, Err(SimError::Budget { .. })), "over budget: {:?}", over.map(|s| s.kinds.len()));
    let empty = segment_faces(&[], &good, BUDGET).expect("an empty surface segments");
    assert!(empty.kinds.is_empty() && empty.mesh.edges.is_empty() && empty.mesh.triangles.is_empty());
}

/// Timing on a large stock (`cargo test -- --ignored --nocapture large_stock_timing`): a 120 by 120 mm
/// top at 0.25 mm with a 20 mm round pocket and a slot, about a million triangles.
#[test]
#[ignore = "timing only: run with --ignored --nocapture"]
fn large_stock_timing() {
    let h = 0.25;
    let mut s = Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(120.0, 120.0, 0.0), h, 1 << 34).expect("stock");
    let tool = FlatTool::new(10.0, 20.0, Vec3::Z).expect("tool");
    s.apply(&LinearFixedAxis { from: Vec3::new(60.0, 60.0, 5.0), to: Vec3::new(60.0, 60.0, -4.0), tool }, 1).expect("plunge");
    let tool = FlatTool::new(2.5, 20.0, Vec3::Z).expect("tool");
    s.apply(&LinearFixedAxis { from: Vec3::new(10.0, 20.0, -4.0), to: Vec3::new(110.0, 20.0, -4.0), tool }, 2).expect("slot");
    let meshes = chunk_meshes(&s);
    let tris: usize = meshes.iter().map(|m| m.triangles.len()).sum();
    let start = std::time::Instant::now();
    let seg = segment_faces(&meshes, &FaceTolerances::for_cell(h).expect("tol"), 1 << 34).expect("segment");
    eprintln!("large stock: {tris} triangles, {} faces, {} edges, segmented in {:?}", seg.kinds.len(), seg.mesh.edges.len(), start.elapsed());
    assert_well_formed(&seg);
}
