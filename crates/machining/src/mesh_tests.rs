use std::collections::HashMap;

use solidcraft_geom::{Mesh, Vec3};

use super::chunk_meshes;
use crate::dexel::Stock;
use crate::sweep::{FlatTool, LinearFixedAxis};

/// Chunk meshes welded by exact position (chunks share their boundary vertices bit for bit), so the
/// result is one mesh whose edges can be counted.
fn weld(meshes: &[Mesh]) -> Mesh {
    let mut index: HashMap<[u64; 3], u32> = HashMap::new();
    let mut out = Mesh::default();
    for m in meshes {
        let map: Vec<u32> = m
            .positions
            .iter()
            .map(|p| {
                let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
                *index.entry(key).or_insert_with(|| {
                    out.positions.push(*p);
                    (out.positions.len() - 1) as u32
                })
            })
            .collect();
        for t in &m.triangles {
            out.triangles.push([map[t[0] as usize], map[t[1] as usize], map[t[2] as usize]]);
        }
    }
    out.tri_face = vec![0; out.triangles.len()];
    out
}

/// Every undirected edge is shared by exactly two triangles, and each directed edge appears once.
fn assert_closed(m: &Mesh) {
    let mut count: HashMap<(u32, u32), u32> = HashMap::new();
    let mut directed: HashMap<(u32, u32), u32> = HashMap::new();
    for t in &m.triangles {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            *count.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            *directed.entry((a, b)).or_insert(0) += 1;
        }
    }
    let bad: Vec<_> = count.iter().filter(|(_, n)| **n != 2).take(5).collect();
    assert!(bad.is_empty(), "edges not shared by two triangles: {bad:?} of {} edges", count.len());
    assert!(directed.values().all(|n| *n == 1), "a directed edge repeats");
    assert!(!m.triangles.is_empty());
}

fn slot_stock(h: f64) -> Stock {
    let mut s = Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(30.0, 30.0, 0.0), h, 1 << 32).expect("stock");
    let tool = FlatTool::new(2.5, 20.0, Vec3::Z).expect("tool");
    let slot = LinearFixedAxis { from: Vec3::new(8.0, 15.0, -4.0), to: Vec3::new(22.0, 15.0, -4.0), tool };
    s.apply(&slot, 1).expect("cut");
    s
}

#[test]
fn uncut_box_mesh_is_closed_and_its_volume_is_near_the_box() {
    let s = Stock::new_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 2.0), 0.25, 1 << 32).expect("stock");
    let meshes = chunk_meshes(&s);
    let m = weld(&meshes);
    assert_closed(&m);
    let v = m.measure().volume;
    assert!(v > 0.0, "outward orientation: {v}");
    assert!((v - 8.0).abs() < 0.5, "box volume {v}");
}

#[test]
fn cut_stock_mesh_is_closed_across_chunk_seams() {
    let h = 0.25;
    let s = slot_stock(h);
    let meshes = chunk_meshes(&s);
    assert!(meshes.len() > 1, "more than one chunk");
    let m = weld(&meshes);
    assert_closed(&m);
}

#[test]
fn cut_stock_mesh_volume_is_within_h_of_the_dexel_volume() {
    let h = 0.25;
    let s = slot_stock(h);
    let m = weld(&chunk_meshes(&s));
    let mesh_v = m.measure().volume;
    let dexel_v = s.volume();
    let area = m.measure().area;
    eprintln!("mesh {mesh_v:.3} dexel {dexel_v:.3} area {area:.3}");
    assert!((mesh_v - dexel_v).abs() <= h * area, "mesh {mesh_v} vs dexel {dexel_v}, area {area}");
}

#[test]
fn cut_stock_volume_matches_the_analytic_block_minus_slot() {
    let h = 0.25;
    let s = slot_stock(h);
    let m = weld(&chunk_meshes(&s));
    let analytic = 30.0 * 30.0 * 10.0 - 4.0 * (2.0 * 2.5 * 14.0 + std::f64::consts::PI * 2.5 * 2.5);
    let mesh_v = m.measure().volume;
    assert!((mesh_v - analytic).abs() < 2.0 * h * 30.0 * 4.0, "mesh {mesh_v} vs analytic {analytic}");
}

#[test]
fn every_chunk_mesh_is_non_empty_and_positions_are_finite() {
    let s = slot_stock(0.25);
    for m in chunk_meshes(&s) {
        assert!(!m.triangles.is_empty());
        assert!(m.positions.iter().all(|p| p.is_finite()));
        assert!(m.triangles.iter().all(|t| t.iter().all(|i| (*i as usize) < m.positions.len())));
    }
}
