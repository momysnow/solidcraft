//! Surface mesh of the cut stock, one mesh per chunk of 32 by 32 by 32 cells.
//!
//! Dual contouring over the dexel lattice: lattice nodes are inside or outside by the majority of
//! their three rays; every lattice edge with a sign change lies on a ray, so its crossing is a ray
//! end; each cell with a sign change gets one vertex; each such edge emits a quad through the four
//! cells around it. Edges are owned by their lower node, so each quad is emitted once, by the chunk
//! that owns it, and the chunks share their cell vertices bit for bit.
//!
//! shortcut: the cell vertex is the mean of its edge crossings, so sharp edges are rounded by up to
//! one cell size; upgrade to a QEF with the tags' normals in S8.
//! shortcut: a lattice face with four sign changes (a checkerboard) makes its quads share an edge
//! four ways, so the mesh is not closed there; this needs a sign-resolve pass before S8.

use std::collections::HashMap;

use solidcraft_geom::{Mesh, Vec3};

use crate::dexel::Stock;

const CHUNK: i64 = 32;

/// The surface of the cut stock, one mesh per non-empty chunk, in chunk order.
pub fn chunk_meshes(stock: &Stock) -> Vec<Mesh> {
    let n = stock.cell_counts().map(|x| x as i64);
    let chunks = n.map(|x| (x + 2 + CHUNK - 1) / CHUNK);
    let mut out = Vec::new();
    for cz in 0..chunks[2] {
        for cy in 0..chunks[1] {
            for cx in 0..chunks[0] {
                let first = [cx * CHUNK - 1, cy * CHUNK - 1, cz * CHUNK - 1];
                let mut builder = Builder { stock, n, verts: HashMap::new(), positions: Vec::new(), triangles: Vec::new() };
                builder.chunk(first);
                if builder.triangles.is_empty() {
                    continue;
                }
                out.push(builder.finish());
            }
        }
    }
    out
}

struct Builder<'a> {
    stock: &'a Stock,
    n: [i64; 3],
    verts: HashMap<[i64; 3], Option<u32>>,
    positions: Vec<Vec3>,
    triangles: Vec<[u32; 3]>,
}

impl Builder<'_> {
    /// Emits the quads of the edges owned by the chunk whose first lower node is `first`.
    fn chunk(&mut self, first: [i64; 3]) {
        let hi: [i64; 3] = std::array::from_fn(|k| (first[k] + CHUNK).min(self.n[k] + 1));
        for a in 0..3 {
            for l0 in first[0].max(-1)..hi[0] {
                for l1 in first[1].max(-1)..hi[1] {
                    for l2 in first[2].max(-1)..hi[2] {
                        let lower = [l0, l1, l2];
                        if lower[a] > self.n[a] - 1 {
                            continue;
                        }
                        self.edge(a, lower);
                    }
                }
            }
        }
    }

    /// The quad for one lattice edge, if it has a sign change.
    fn edge(&mut self, a: usize, lower: [i64; 3]) {
        let mut upper = lower;
        upper[a] += 1;
        let inside = self.stock.node_inside(lower);
        if inside == self.stock.node_inside(upper) {
            return;
        }
        let (u, v) = ((a + 1) % 3, (a + 2) % 3);
        // The four cells around the edge, in counter-clockwise order seen from the +axis direction.
        let offsets = [(-1i64, -1i64), (0, -1), (0, 0), (-1, 0)];
        let mut quad = [0u32; 4];
        for (q, (du, dv)) in quad.iter_mut().zip(offsets) {
            let mut cell = lower;
            cell[u] += du;
            cell[v] += dv;
            let Some(id) = self.vertex(cell) else { return };
            *q = id;
        }
        let [v0, v1, v2, v3] = quad;
        let tris = if inside { [[v0, v1, v2], [v0, v2, v3]] } else { [[v0, v2, v1], [v0, v3, v2]] };
        self.triangles.extend(tris);
    }

    /// The vertex of a cell with a sign change: the mean of its crossings. Cached per chunk.
    fn vertex(&mut self, cell: [i64; 3]) -> Option<u32> {
        if let Some(v) = self.verts.get(&cell) {
            return *v;
        }
        let mut sum = Vec3::ZERO;
        let mut count = 0u32;
        for a in 0..3 {
            let (u, v) = ((a + 1) % 3, (a + 2) % 3);
            for (du, dv) in [(0i64, 0i64), (1, 0), (0, 1), (1, 1)] {
                let mut lower = cell;
                lower[u] += du;
                lower[v] += dv;
                let mut upper = lower;
                upper[a] += 1;
                if self.stock.node_inside(lower) == self.stock.node_inside(upper) {
                    continue;
                }
                if let Some(p) = self.stock.edge_crossing(a, lower) {
                    sum += p;
                    count += 1;
                }
            }
        }
        let id = if count == 0 {
            None
        } else {
            self.positions.push(sum / f64::from(count));
            u32::try_from(self.positions.len() - 1).ok()
        };
        self.verts.insert(cell, id);
        id
    }

    fn finish(self) -> Mesh {
        let mut normals = vec![Vec3::ZERO; self.positions.len()];
        for t in &self.triangles {
            let [a, b, c] = t.map(|i| self.positions.get(i as usize).copied().unwrap_or(Vec3::ZERO));
            let nrm = (b - a).cross(c - a);
            for i in t {
                if let Some(acc) = normals.get_mut(*i as usize) {
                    *acc += nrm;
                }
            }
        }
        for nrm in &mut normals {
            *nrm = nrm.normalized().unwrap_or(Vec3::ZERO);
        }
        let tri_face = vec![0u32; self.triangles.len()];
        Mesh { positions: self.positions, normals, triangles: self.triangles, tri_face, ..Mesh::default() }
    }
}

#[cfg(test)]
#[path = "mesh_tests.rs"]
mod tests;
