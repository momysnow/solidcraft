use std::collections::HashMap;

use solidcraft_geom::{Mesh, Vec3};

use super::{Lattice, chunk_mesh, chunk_meshes, dirty_chunks, extract, mesh_volume};
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
fn assert_two_per_edge(m: &Mesh) {
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

/// A synthetic lattice of `n` cells per axis, with material given by a predicate on nodes. The
/// crossing of each edge is its midpoint, so tests check the topology and not the placement.
struct Synthetic<F: Fn([i64; 3]) -> bool> {
    n: i64,
    material: F,
}

impl<F: Fn([i64; 3]) -> bool> Lattice for Synthetic<F> {
    fn cell_counts(&self) -> [i64; 3] {
        [self.n; 3]
    }

    fn inside(&self, node: [i64; 3]) -> bool {
        node.iter().all(|&x| x >= 0 && x < self.n) && (self.material)(node)
    }

    fn crossing(&self, axis: usize, lower: [i64; 3]) -> Option<Vec3> {
        let p: [f64; 3] = std::array::from_fn(|k| lower[k] as f64 + if k == axis { 0.5 } else { 0.0 });
        Some(Vec3::from(p))
    }
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
    let m = weld(&chunk_meshes(&s));
    assert_two_per_edge(&m);
    let v = m.measure().volume;
    assert!(v > 0.0, "outward orientation: {v}");
    assert!((v - 8.0).abs() < 0.5, "box volume {v}");
}

#[test]
fn cut_stock_mesh_is_closed_across_chunk_seams() {
    let s = slot_stock(0.25);
    let meshes = chunk_meshes(&s);
    assert!(meshes.len() > 1, "more than one chunk");
    assert_two_per_edge(&weld(&meshes));
}

#[test]
fn checkerboard_lattice_has_two_triangles_per_edge() {
    // Every face of every cell is a checkerboard: the hardest case for the pairing.
    for n in [3, 4, 40] {
        let lat = Synthetic { n, material: |node: [i64; 3]| (node[0] + node[1] + node[2]) % 2 == 0 };
        let m = weld(&extract(&lat));
        assert_two_per_edge(&m);
    }
}

#[test]
fn diagonal_wall_lattice_has_two_triangles_per_edge() {
    // Material on one side of a diagonal plane: faces are checkerboards along the plane.
    for n in [4, 40] {
        let lat = Synthetic { n, material: |node: [i64; 3]| node[0] >= node[1] };
        assert_two_per_edge(&weld(&extract(&lat)));
    }
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

/// A lattice given by its material nodes, for the review's example.
struct NodeSet {
    n: i64,
    set: std::collections::HashSet<[i64; 3]>,
}

impl Lattice for NodeSet {
    fn cell_counts(&self) -> [i64; 3] {
        [self.n; 3]
    }

    fn inside(&self, node: [i64; 3]) -> bool {
        self.set.contains(&node)
    }

    fn crossing(&self, axis: usize, lower: [i64; 3]) -> Option<Vec3> {
        let p: [f64; 3] = std::array::from_fn(|k| lower[k] as f64 + if k == axis { 0.5 } else { 0.0 });
        Some(Vec3::from(p))
    }
}

#[test]
fn example_lattice_has_even_edge_counts_including_across_a_seam() {
    // Four material nodes, all else air: one group holds both pairs of a checkerboard face, so an
    // edge can be shared by four triangles. Every edge must still be shared an even number of times.
    let example = [[0i64, 0, -1], [0, 0, 0], [0, 0, 1], [1, 1, 0]];
    for (n, offset) in [(2i64, 0i64), (64, 30)] {
        let set = example.iter().map(|p| [p[0] + offset, p[1] + offset, p[2] + offset]).collect();
        let lat = NodeSet { n, set };
        let m = weld(&extract(&lat));
        assert!(!m.triangles.is_empty());
        let mut count: HashMap<(u32, u32), u32> = HashMap::new();
        for t in &m.triangles {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *count.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        let odd: Vec<_> = count.iter().filter(|(_, c)| *c % 2 != 0).take(5).collect();
        assert!(odd.is_empty(), "offset {offset}: edges with an odd count {odd:?}");
    }
}

#[test]
fn chunk_meshes_are_the_same_as_chunk_by_chunk() {
    let s = slot_stock(0.25);
    let all = chunk_meshes(&s);
    let mut by_index = Vec::new();
    for cz in 0..4usize {
        for cy in 0..4usize {
            for cx in 0..4usize {
                if let Some(m) = chunk_mesh(&s, [cx, cy, cz]) {
                    by_index.push(m);
                }
            }
        }
    }
    let tris = |ms: &[Mesh]| ms.iter().map(|m| m.triangles.len()).sum::<usize>();
    assert_eq!(tris(&all), tris(&by_index));
    assert_eq!(all.len(), by_index.len());
}

fn fingerprint(m: Option<Mesh>) -> (usize, u64) {
    match m {
        None => (0, 0),
        Some(m) => {
            let h = m
                .positions
                .iter()
                .fold(0u64, |acc, p| acc.wrapping_mul(31).wrapping_add(p.x.to_bits() ^ p.y.to_bits().rotate_left(7) ^ p.z.to_bits().rotate_left(13)));
            (m.triangles.len(), h)
        }
    }
}

#[test]
fn a_second_cut_changes_only_dirty_chunks() {
    let mut s = slot_stock(0.25);
    s.clear_dirty();
    assert!(dirty_chunks(&s).is_empty());
    let before: Vec<(usize, u64)> = (0..4usize)
        .flat_map(|cz| (0..4usize).flat_map(move |cy| (0..4usize).map(move |cx| (cx, cy, cz))))
        .map(|(cx, cy, cz)| fingerprint(chunk_mesh(&s, [cx, cy, cz])))
        .collect();
    // A small pocket far from the slot, near one corner.
    let tool = FlatTool::new(1.0, 20.0, Vec3::Z).expect("tool");
    let pocket = LinearFixedAxis { from: Vec3::new(26.0, 26.0, -6.0), to: Vec3::new(27.0, 26.0, -6.0), tool };
    s.apply(&pocket, 9).expect("cut");
    let dirty = dirty_chunks(&s);
    assert!(!dirty.is_empty());
    let after: Vec<(usize, u64)> = (0..4usize)
        .flat_map(|cz| (0..4usize).flat_map(move |cy| (0..4usize).map(move |cx| (cx, cy, cz))))
        .map(|(cx, cy, cz)| fingerprint(chunk_mesh(&s, [cx, cy, cz])))
        .collect();
    let mut idx = 0usize;
    for cz in 0..4usize {
        for cy in 0..4usize {
            for cx in 0..4usize {
                if before[idx] != after[idx] {
                    assert!(dirty.contains(&[cx, cy, cz]), "chunk {cx} {cy} {cz} changed but was not dirty");
                }
                idx += 1;
            }
        }
    }
    s.clear_dirty();
    assert!(dirty_chunks(&s).is_empty());
}

#[test]
fn mesh_volume_is_the_precise_number() {
    let s = slot_stock(0.25);
    let analytic = 30.0 * 30.0 * 10.0 - 4.0 * (2.0 * 2.5 * 14.0 + std::f64::consts::PI * 2.5 * 2.5);
    let err_mesh = (mesh_volume(&s) - analytic).abs();
    let err_dexel = (s.volume() - analytic).abs();
    eprintln!("analytic {analytic:.3}: mesh error {err_mesh:.4}, midpoint error {err_dexel:.4}");
    assert!(err_mesh < 6.0, "mesh error {err_mesh}");
}
