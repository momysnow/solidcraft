use std::collections::HashMap;
use std::f64::consts::FRAC_PI_2;

use solidcore_record::Vec3;
use solidcraft_geom::Mesh;
use solidcraft_geom::prelude::*;

use super::{FaceKind, FaceSegmentation, FaceTolerances, segment_faces};
use crate::dexel::Stock;
use crate::mesh::chunk_meshes;
use crate::sweep::{FlatTool, LinearFixedAxis};

fn slot_stock(h: f64) -> Stock {
    let mut s = Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(30.0, 30.0, 0.0), h, 1 << 32).expect("stock");
    let tool = FlatTool::new(2.5, 20.0, Vec3::Z).expect("tool");
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
    segment_faces(&chunk_meshes(s), &tol).expect("segment")
}

/// The vertices of each face, over every mesh of the segmentation.
fn points_by_face(seg: &FaceSegmentation) -> HashMap<u32, Vec<Vec3>> {
    let mut out: HashMap<u32, Vec<Vec3>> = HashMap::new();
    for m in &seg.meshes {
        for (tri, f) in m.triangles.iter().zip(&m.tri_face) {
            for &v in tri {
                out.entry(*f).or_default().push(m.positions[v as usize]);
            }
        }
    }
    out
}

/// The planar faces whose every vertex lies within `tol` of `value` along `axis`.
fn planar_faces_on(seg: &FaceSegmentation, axis: usize, value: f64, tol: f64) -> Vec<u32> {
    let mut faces: Vec<u32> = points_by_face(seg)
        .into_iter()
        .filter(|(f, ps)| seg.kinds[*f as usize] == FaceKind::Planar && ps.iter().all(|p| (<[f64; 3]>::from(*p)[axis] - value).abs() <= tol))
        .map(|(f, _)| f)
        .collect();
    faces.sort_unstable();
    faces
}

/// The index of the first polyline with a point that `pred` accepts.
fn line_with(seg: &FaceSegmentation, pred: impl Fn(Vec3) -> bool) -> Option<usize> {
    seg.edges.iter().position(|l| l.iter().any(|p| pred(*p)))
}

fn xyz(p: Vec3) -> [f64; 3] {
    p.into()
}

#[test]
fn flat_slot_has_a_planar_floor_and_two_planar_walls_with_their_edges() {
    let h = 0.25;
    let seg = segment_stock(&slot_stock(h), h);
    // Planar faces hold their corners within the gap, a quarter of a cell.
    let tol = 0.25 * h;
    let floor = planar_faces_on(&seg, 2, -4.0, tol);
    let wall_a = planar_faces_on(&seg, 1, 12.5, tol);
    let wall_b = planar_faces_on(&seg, 1, 17.5, tol);
    let top = planar_faces_on(&seg, 2, 0.0, tol);
    assert_eq!(floor.len(), 1, "floor faces {floor:?}");
    assert_eq!(wall_a.len(), 1, "wall at y 12.5: {wall_a:?}");
    assert_eq!(wall_b.len(), 1, "wall at y 17.5: {wall_b:?}");
    assert_eq!(top.len(), 1, "top faces {top:?}");
    assert_ne!(floor, wall_a);

    // The floor's normals are vertical: a planar face shades flat.
    let floor_id = floor[0];
    for m in &seg.meshes {
        for (tri, f) in m.triangles.iter().zip(&m.tri_face) {
            if *f == floor_id {
                for &v in tri {
                    let n = m.normals[v as usize];
                    assert!(n.z.abs() > 0.999_999, "floor normal {n:?}");
                }
            }
        }
    }

    // The floor and wall meet along a line at y 12.5, z -4, across the slot: a crease, not tangent.
    let on_floor_wall = |p: Vec3| {
        let [x, y, z] = xyz(p);
        (y - 12.5).abs() <= tol && (z + 4.0).abs() <= tol && (8.0 - 0.5..=22.0 + 0.5).contains(&x)
    };
    let mut xs: Vec<f64> = seg.edges.iter().flatten().filter(|p| on_floor_wall(**p)).map(|p| xyz(*p)[0]).collect();
    xs.sort_by(f64::total_cmp);
    assert!(!xs.is_empty(), "no edge along the floor and wall");
    assert!(xs[0] <= 8.5 && *xs.last().expect("x") >= 21.5, "floor-wall edge spans {:?}..{:?}", xs.first(), xs.last());
    let crease = line_with(&seg, on_floor_wall).expect("line");
    assert!(!seg.tangent[crease], "a floor meeting a wall at 90 degrees is not tangent");

    // The wall meets the rounded slot end tangentially: that edge is tangent.
    let on_end = |p: Vec3| {
        let [x, y, z] = xyz(p);
        (x - 8.0).abs() <= 0.5 && (y - 12.5).abs() <= tol && z > -3.5 && z < -0.5
    };
    let tangent_line = line_with(&seg, on_end).expect("the wall meets the slot end");
    assert!(seg.tangent[tangent_line], "a wall meeting the rounded end is a tangent edge");
    assert_eq!(seg.edge_faces[tangent_line].len(), 2);
}

#[test]
fn circular_pocket_has_one_curved_wall_a_floor_a_top_and_circle_edges() {
    let h = 0.5;
    let seg = segment_stock(&plunged_pocket(h), h);
    let r = 5.0;
    let rho = |p: Vec3| {
        let [x, y, _] = xyz(p);
        ((x - 15.0).powi(2) + (y - 15.0).powi(2)).sqrt()
    };
    let by_face = points_by_face(&seg);
    let wall: Vec<u32> = {
        let mut v: Vec<u32> = by_face
            .iter()
            .filter(|(f, ps)| {
                seg.kinds[**f as usize] == FaceKind::Curved
                    && ps.iter().all(|p| (rho(*p) - r).abs() <= 0.1 * h && xyz(*p)[2] >= -4.0 - 0.1 * h && xyz(*p)[2] <= 0.1 * h)
            })
            .map(|(f, _)| *f)
            .collect();
        v.sort_unstable();
        v
    };
    assert_eq!(wall.len(), 1, "curved wall faces {wall:?}");
    // The wall's facets that happen to look flat (lattice-aligned chords) are not planar faces.
    let flat_on_wall: Vec<u32> = by_face
        .iter()
        .filter(|(f, ps)| {
            seg.kinds[**f as usize] == FaceKind::Planar
                && ps.iter().all(|p| (rho(*p) - r).abs() <= 0.1 * h && xyz(*p)[2] >= -4.0 - 0.1 * h && xyz(*p)[2] <= 0.1 * h)
        })
        .map(|(f, _)| *f)
        .collect();
    assert!(flat_on_wall.is_empty(), "planar facets on the wall: {flat_on_wall:?}");

    let floor = {
        let mut v: Vec<u32> = by_face
            .iter()
            .filter(|(f, ps)| {
                seg.kinds[**f as usize] == FaceKind::Planar && ps.iter().all(|p| (xyz(*p)[2] + 4.0).abs() <= 0.1 * h && rho(*p) <= r + 0.1 * h)
            })
            .map(|(f, _)| *f)
            .collect();
        v.sort_unstable();
        v
    };
    assert_eq!(floor.len(), 1, "floor faces {floor:?}");
    assert_eq!(planar_faces_on(&seg, 2, 0.0, 0.1 * h).len(), 1, "one top face");

    // The rim at the floor and at the top are closed loops of radius r.
    let loop_at = |z: f64| {
        seg.edges.iter().position(|l| {
            l.len() > 8 && l.first() == l.last() && l.iter().all(|p| (rho(*p) - r).abs() <= 0.1 * h && (xyz(*p)[2] - z).abs() <= 0.1 * h)
        })
    };
    for z in [-4.0, 0.0] {
        let i = loop_at(z).unwrap_or_else(|| panic!("no closed circle edge at z {z}"));
        let length: f64 = seg.edges[i].windows(2).map(|w| (w[1] - w[0]).len()).sum();
        let circle = 2.0 * std::f64::consts::PI * r;
        assert!((length - circle).abs() < 0.02 * circle, "circle at z {z}: length {length} vs {circle}");
        assert!(!seg.tangent[i], "a wall meets its floor and top at right angles");
    }
}

/// A synthetic plane, quarter-cylinder fillet and plane: the floor (z = 0, x from -10 to 0), a fillet
/// of radius 2 about (0, 2) in the x-z plane, and the wall (x = 2, z from 2 to 6), extruded along y
/// from 0 to 4 in four steps. Each quad is oriented to face its air side.
fn fillet_mesh() -> Mesh {
    let (r, steps, ydiv) = (2.0f64, 8usize, 4usize);
    // The profile in (x, z), from the floor end to the wall end. The arc's ends are exact, so the
    // floor and the wall share their vertices with the fillet bit for bit.
    let mut profile: Vec<(f64, f64)> = vec![(-10.0, 0.0), (0.0, 0.0)];
    for k in 1..steps {
        let th = -FRAC_PI_2 + FRAC_PI_2 * (k as f64) / (steps as f64);
        profile.push((r * th.cos(), 2.0 + r * th.sin()));
    }
    profile.push((2.0, 2.0));
    profile.push((2.0, 6.0));
    let mut m = Mesh::default();
    for (i, w) in profile.windows(2).enumerate() {
        let ((px, pz), (qx, qz)) = (w[0], w[1]);
        // The air side: up for the floor, toward the arc's centre for the fillet, and -x for the wall.
        let (mx, mz) = ((px + qx) / 2.0, (pz + qz) / 2.0);
        let want = if i == 0 {
            (0.0, 0.0, 1.0)
        } else if i == profile.len() - 2 {
            (-1.0, 0.0, 0.0)
        } else {
            let (cx, cz) = (0.0, 2.0);
            let (dx, dz) = (cx - mx, cz - mz);
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
    let seg = segment_faces(&[fillet_mesh()], &FaceTolerances::for_cell(0.5).expect("tol")).expect("segment");
    assert_eq!(seg.kinds.len(), 3, "faces {:?}", seg.kinds);
    assert_eq!(seg.kinds.iter().filter(|k| **k == FaceKind::Planar).count(), 2);
    assert_eq!(seg.kinds.iter().filter(|k| **k == FaceKind::Curved).count(), 1);
    assert_eq!(seg.edges.len(), 2, "edges {:?}", seg.edges.len());
    assert!(seg.tangent.iter().all(|t| *t), "both fillet edges are tangent: {:?}", seg.tangent);

    // The planar faces take the first fillet row on each side (the gap admits its corners), so each
    // tangent edge sits one row off the true tangent line: the floor-fillet edge near (x 0, z 0) and
    // the fillet-wall edge near (x 2, z 2). Each runs the full length of the extrusion.
    let near = |line: &[Vec3], x: (f64, f64), z: (f64, f64)| {
        line.iter().all(|p| {
            let [px, _, pz] = xyz(*p);
            (x.0..=x.1).contains(&px) && (z.0..=z.1).contains(&pz)
        })
    };
    let floor_line = seg.edges.iter().position(|l| near(l, (-1e-9, 0.45), (-1e-9, 0.06))).expect("floor-fillet edge");
    let wall_line = seg.edges.iter().position(|l| near(l, (1.9, 2.0 + 1e-9), (1.5, 2.0 + 1e-9))).expect("fillet-wall edge");
    for i in [floor_line, wall_line] {
        let ys: Vec<f64> = seg.edges[i].iter().map(|p| xyz(*p)[1]).collect();
        assert!(ys.iter().any(|y| y.abs() < 1e-9) && ys.iter().any(|y| (y - 4.0).abs() < 1e-9), "edge spans {ys:?}");
    }
    assert_ne!(floor_line, wall_line);
}

#[test]
fn split_vertices_shade_each_face_on_its_own() {
    let seg = segment_faces(&[fillet_mesh()], &FaceTolerances::for_cell(0.5).expect("tol")).expect("segment");
    let m = &seg.meshes[0];
    // The tangent-edge vertex on the floor side, at the fillet's first row (x about 0.39, z about 0.04),
    // is two copies: the floor's, which shades flat, and the fillet's, which turns away from it.
    let copies: Vec<Vec3> = m
        .positions
        .iter()
        .enumerate()
        .filter(|(_, p)| {
            let [x, y, z] = xyz(**p);
            (0.3..0.45).contains(&x) && (y - 2.0).abs() < 1e-9 && (0.0..0.06).contains(&z)
        })
        .map(|(i, _)| m.normals[i])
        .collect();
    assert_eq!(copies.len(), 2, "copies of the tangent-edge vertex: {copies:?}");
    assert!(copies.iter().any(|n| n.z > 0.999_999), "the floor's copy shades flat: {copies:?}");
    assert!(copies.iter().any(|n| n.z < 0.999_999), "the fillet's copy turns: {copies:?}");
    for (i, n) in m.normals.iter().enumerate() {
        assert!((n.len() - 1.0).abs() < 1e-9 || *n == Vec3::ZERO, "vertex {i} normal {n:?} is not unit length");
    }
}

#[test]
fn refuses_bad_tolerances_and_hostile_meshes_without_panicking() {
    assert!(FaceTolerances::for_cell(0.0).is_err());
    assert!(FaceTolerances::for_cell(f64::NAN).is_err());
    let good = FaceTolerances::for_cell(0.25).expect("tol");
    assert!(segment_faces(&[], &FaceTolerances { plane_angle: 0.0, ..good }).is_err());
    assert!(segment_faces(&[], &FaceTolerances { crease_angle: 4.0, ..good }).is_err());
    assert!(segment_faces(&[], &FaceTolerances { plane_gap: f64::INFINITY, ..good }).is_err());
    let mut bad_index = fillet_mesh();
    bad_index.triangles[0][0] = 1_000_000;
    assert!(segment_faces(&[bad_index], &good).is_err());
    let mut nan = fillet_mesh();
    nan.positions[0] = Vec3::new(f64::NAN, 0.0, 0.0);
    assert!(segment_faces(&[nan], &good).is_err());
    let empty = segment_faces(&[], &good).expect("an empty surface segments");
    assert!(empty.kinds.is_empty() && empty.edges.is_empty() && empty.meshes.is_empty());
}
