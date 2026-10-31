//! Surface mesh of the cut stock, one mesh per chunk of 32 by 32 by 32 cells.
//!
//! Dual marching cubes over the dexel lattice. Lattice nodes are inside or outside by the majority
//! of their three rays. Each lattice edge with a sign change lies on a ray, so its crossing is a
//! ray end. Within each cell the sign-changing edges are paired on every face (a face with two
//! sign changes pairs them; a checkerboard face with four pairs the edges around each outside
//! corner), and the pairs join into groups. Each group gets one vertex, at the mean of its
//! crossings. Each quad of the surface takes, from each of its four cells, the vertex of the group
//! holding its edge. The surface is manifold: every edge is shared by exactly two triangles, even
//! where two inside corners of a cell touch only at a vertex.
//!
//! Edges are owned by their lower node, so each quad is emitted once, by the chunk that owns it,
//! and the chunks share their cell vertices bit for bit.
//!
//! shortcut: a group's vertex is the mean of its crossings, so sharp edges are rounded by up to one
//! cell size; upgrade to a QEF with the tags' normals in S8.

use std::collections::HashMap;

use solidcraft_geom::{Mesh, Vec3};

use crate::dexel::Stock;

const CHUNK: i64 = 32;

/// What the mesher needs of a lattice: its cell counts, which nodes are material, and where each
/// sign-changing edge crosses the boundary. Node indices run from `-1` to `n` on each axis.
pub trait Lattice {
    fn cell_counts(&self) -> [i64; 3];
    fn inside(&self, node: [i64; 3]) -> bool;
    fn crossing(&self, axis: usize, lower: [i64; 3]) -> Option<Vec3>;
}

impl Lattice for Stock {
    fn cell_counts(&self) -> [i64; 3] {
        self.cell_counts().map(|x| x as i64)
    }

    fn inside(&self, node: [i64; 3]) -> bool {
        self.node_inside(node)
    }

    fn crossing(&self, axis: usize, lower: [i64; 3]) -> Option<Vec3> {
        self.edge_crossing(axis, lower)
    }
}

/// The surface of the cut stock, one mesh per non-empty chunk, in chunk order.
pub fn chunk_meshes(stock: &Stock) -> Vec<Mesh> {
    extract(stock)
}

/// The surface of any lattice, one mesh per non-empty chunk, in chunk order.
pub fn extract<L: Lattice>(lat: &L) -> Vec<Mesh> {
    let n = lat.cell_counts();
    let chunks = n.map(|x| (x + 2 + CHUNK - 1) / CHUNK);
    let mut out = Vec::new();
    for cz in 0..chunks[2] {
        for cy in 0..chunks[1] {
            for cx in 0..chunks[0] {
                let first = [cx * CHUNK - 1, cy * CHUNK - 1, cz * CHUNK - 1];
                let mut builder = Builder { lat, n, cells: HashMap::new(), positions: Vec::new(), triangles: Vec::new() };
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

/// Edge index within a cell: the axis, then the offsets (0 or 1) along the other two axes.
fn edge_id(axis: usize, off: [i64; 3]) -> usize {
    let p = off[(axis + 1) % 3] as usize;
    let q = off[(axis + 2) % 3] as usize;
    axis * 4 + p * 2 + q
}

/// The lower corner offset (0 or 1 on each axis, zero along the edge's own axis) of an edge index.
fn edge_offset(id: usize) -> (usize, [i64; 3]) {
    let axis = id / 4;
    let mut off = [0i64; 3];
    off[(axis + 1) % 3] = ((id / 2) % 2) as i64;
    off[(axis + 2) % 3] = (id % 2) as i64;
    (axis, off)
}

fn find(parent: &mut [usize; 12], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    x
}

fn join(parent: &mut [usize; 12], a: usize, b: usize) {
    let (ra, rb) = (find(parent, a), find(parent, b));
    if ra != rb {
        parent[ra.max(rb)] = ra.min(rb);
    }
}

/// The corners of a cell, with the signs of their nodes: index `x + 2 y + 4 z`.
fn corner_offset(c: usize) -> [i64; 3] {
    [(c & 1) as i64, ((c >> 1) & 1) as i64, ((c >> 2) & 1) as i64]
}

struct Builder<'a, L: Lattice> {
    lat: &'a L,
    n: [i64; 3],
    /// Per cell: the vertex of each sign-changing edge of the cell, by edge index.
    cells: HashMap<[i64; 3], [Option<u32>; 12]>,
    positions: Vec<Vec3>,
    triangles: Vec<[u32; 3]>,
}

impl<L: Lattice> Builder<'_, L> {
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
        let inside = self.lat.inside(lower);
        if inside == self.lat.inside(upper) {
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
            let mut local = [0i64; 3];
            local[u] = lower[u] - cell[u];
            local[v] = lower[v] - cell[v];
            let Some(id) = self.vertex(cell, edge_id(a, local)) else { return };
            *q = id;
        }
        let [v0, v1, v2, v3] = quad;
        let tris = if inside { [[v0, v1, v2], [v0, v2, v3]] } else { [[v0, v2, v1], [v0, v3, v2]] };
        self.triangles.extend(tris);
    }

    /// The vertex of edge `edge` of `cell`, computing the cell's vertices once.
    fn vertex(&mut self, cell: [i64; 3], edge: usize) -> Option<u32> {
        if !self.cells.contains_key(&cell) {
            let patch = self.patch(cell);
            self.cells.insert(cell, patch);
        }
        self.cells.get(&cell).and_then(|p| p.get(edge).copied().flatten())
    }

    /// The vertices of one cell: pair the sign-changing edges on each face, group the pairs, and
    /// give each group a vertex at the mean of its crossings.
    fn patch(&mut self, cell: [i64; 3]) -> [Option<u32>; 12] {
        let mut sign = [false; 8];
        for (c, s) in sign.iter_mut().enumerate() {
            let off = corner_offset(c);
            *s = self.lat.inside(std::array::from_fn(|k| cell[k] + off[k]));
        }
        let corner = |off: [i64; 3]| (off[0] + 2 * off[1] + 4 * off[2]) as usize;
        let mut change = [false; 12];
        for (id, ch) in change.iter_mut().enumerate() {
            let (axis, off0) = edge_offset(id);
            let mut off1 = off0;
            off1[axis] = 1;
            *ch = sign[corner(off0)] != sign[corner(off1)];
        }
        let mut parent: [usize; 12] = std::array::from_fn(|i| i);
        for a in 0..3 {
            let (u, v) = ((a + 1) % 3, (a + 2) % 3);
            for s in [0i64, 1] {
                let on_face = |axis: usize, other: usize, t: i64| {
                    let mut off = [0i64; 3];
                    off[a] = s;
                    off[other] = t;
                    edge_id(axis, off)
                };
                let face = [on_face(u, v, 0), on_face(u, v, 1), on_face(v, u, 0), on_face(v, u, 1)];
                let changed: Vec<usize> = face.iter().copied().filter(|&e| change[e]).collect();
                match changed.len() {
                    2 => join(&mut parent, changed[0], changed[1]),
                    4 => {
                        // A checkerboard: pair the edges around each outside corner of the face.
                        for pu in [0i64, 1] {
                            for pv in [0i64, 1] {
                                let mut off = [0i64; 3];
                                off[a] = s;
                                off[u] = pu;
                                off[v] = pv;
                                if sign[corner(off)] {
                                    continue;
                                }
                                join(&mut parent, on_face(u, v, pv), on_face(v, u, pu));
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
        // One vertex per group, at the mean of its crossings.
        let mut groups: HashMap<usize, (Vec3, u32, Vec<usize>)> = HashMap::new();
        for (id, ch) in change.iter().enumerate() {
            if !ch {
                continue;
            }
            let root = find(&mut parent, id);
            let (axis, off0) = edge_offset(id);
            let lower: [i64; 3] = std::array::from_fn(|k| cell[k] + off0[k]);
            let entry = groups.entry(root).or_insert((Vec3::ZERO, 0, Vec::new()));
            entry.2.push(id);
            if let Some(p) = self.lat.crossing(axis, lower) {
                entry.0 += p;
                entry.1 += 1;
            }
        }
        let mut patch = [None; 12];
        let mut roots: Vec<usize> = groups.keys().copied().collect();
        roots.sort_unstable();
        for root in roots {
            let Some((sum, count, edges)) = groups.get(&root) else { continue };
            if *count == 0 {
                continue;
            }
            self.positions.push(*sum / f64::from(*count));
            let Ok(id) = u32::try_from(self.positions.len() - 1) else { continue };
            for e in edges {
                if let Some(slot) = patch.get_mut(*e) {
                    *slot = Some(id);
                }
            }
        }
        patch
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
