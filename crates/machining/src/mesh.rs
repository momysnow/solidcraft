//! Surface mesh of the cut stock, one mesh per chunk of 32 by 32 by 32 cells.
//!
//! Dual marching cubes over the dexel lattice. Lattice nodes are inside or outside by the majority
//! of their three rays. Each lattice edge with a sign change lies on a ray, so its crossing is a
//! ray end. Within each cell the sign-changing edges are paired on every face (a face with two
//! sign changes pairs them; a checkerboard face with four pairs the edges around each outside
//! corner), and the pairs join into groups. Each group gets one vertex, at the mean of its
//! crossings. Each quad of the surface takes, from each of its four cells, the vertex of the group
//! holding its edge.
//!
//! The result is closed and oriented: every edge is shared by an even number of triangles, two in
//! the usual case. Where one group holds both pairs of a checkerboard face, the edge can be shared
//! by four. That is not a manifold surface there, and it is not claimed to be.
//!
//! Edges are owned by their lower node, so each quad is emitted once, by the chunk that owns it,
//! and the chunks share their cell vertices bit for bit. Each chunk caches the inside or outside
//! state of its nodes once, instead of evaluating each node about twenty times.
//!
//! shortcut: a group's vertex is the mean of its crossings, so sharp edges are rounded by up to one
//! cell size; upgrade to a QEF with the tags' normals in S8.

use std::collections::HashMap;

use solidcraft_geom::{Mesh, Vec3};

use crate::dexel::Stock;

pub const CHUNK: i64 = 32;
/// The lowest lower node the mesher walks. Nodes below it are taken as air: the stock never has
/// material there, and a general lattice must keep its material above it.
const LOWEST: i64 = -2;

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

/// Chunks along each axis: the nodes run from `-1` to `n`, so `n + 2` of them.
pub(crate) fn chunk_counts<L: Lattice>(lat: &L) -> [i64; 3] {
    lat.cell_counts().map(|x| (x - LOWEST + 1 + CHUNK - 1) / CHUNK)
}

/// The surface of the cut stock, one mesh per non-empty chunk, in chunk order.
pub fn chunk_meshes(stock: &Stock) -> Vec<Mesh> {
    extract(stock)
}

/// The surface of any lattice, one mesh per non-empty chunk, in chunk order.
pub fn extract<L: Lattice>(lat: &L) -> Vec<Mesh> {
    let counts = chunk_counts(lat);
    let mut out = Vec::new();
    for cz in 0..counts[2] {
        for cy in 0..counts[1] {
            for cx in 0..counts[0] {
                if let Some(m) = chunk_mesh_of(lat, [cx, cy, cz]) {
                    out.push(m);
                }
            }
        }
    }
    out
}

/// The mesh of one chunk of the cut stock, or `None` when the chunk has no surface. The index is
/// the chunk's position along each axis, as [`dirty_chunks`] reports it.
pub fn chunk_mesh(stock: &Stock, idx: [usize; 3]) -> Option<Mesh> {
    chunk_mesh_of(stock, idx.map(|x| x as i64))
}

/// The chunks whose surface may have changed since the last [`Stock::clear_dirty`]: each once. A cut
/// records them as it applies (see [`chunks_for_nodes`]), so a small cut in a large stock marks only
/// the chunks near it.
pub fn dirty_chunks(stock: &Stock) -> Vec<[usize; 3]> {
    stock.dirty_chunks()
}

/// The chunks whose mesh reads a node of the box `lo..=hi` (node indices per axis), with a margin of
/// two nodes either side. A chunk of first lower node `first` reads the nodes `first - 2` to
/// `first + CHUNK + 1`, so a node affects the chunks whose window holds it. `counts` are the chunk
/// counts of the lattice; the result is clipped to them.
pub(crate) fn chunks_for_nodes(lo: [i64; 3], hi: [i64; 3], counts: [i64; 3]) -> Vec<[usize; 3]> {
    let per_axis: [Vec<i64>; 3] = std::array::from_fn(|k| {
        // Chunk c reads nodes [CHUNK * c + LOWEST - 2, CHUNK * c + LOWEST + CHUNK + 1].
        let a = lo[k] - 2;
        let b = hi[k] + 2;
        let c_min = -((-(a - LOWEST - CHUNK - 1)).div_euclid(CHUNK));
        let c_max = (b - LOWEST + 2).div_euclid(CHUNK);
        let (from, to) = (c_min.max(0), c_max.min(counts[k] - 1));
        (from..=to).collect()
    });
    let mut out = Vec::new();
    for &x in &per_axis[0] {
        for &y in &per_axis[1] {
            for &z in &per_axis[2] {
                out.push([x as usize, y as usize, z as usize]);
            }
        }
    }
    out
}

/// The volume of the cut stock from its surface, by the divergence theorem. The chunk meshes share
/// their boundary vertices exactly, so their signed volumes add. It is not uniformly more precise than
/// [`Stock::volume`]: its vertices are cell means, which round sharp edges by up to a cell.
pub fn mesh_volume(stock: &Stock) -> f64 {
    chunk_meshes(stock).iter().map(|m| m.measure().volume).sum()
}

fn chunk_mesh_of<L: Lattice>(lat: &L, idx: [i64; 3]) -> Option<Mesh> {
    let counts = chunk_counts(lat);
    if idx.iter().zip(counts.iter()).any(|(i, c)| *i < 0 || i >= c) {
        return None;
    }
    let first = idx.map(|i| i * CHUNK + LOWEST);
    let mut builder = Builder::new(lat, lat.cell_counts(), first);
    builder.chunk(first);
    if builder.triangles.is_empty() {
        return None;
    }
    Some(builder.finish())
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

/// The offsets of corner `c` of a cell: index `x + 2 y + 4 z`.
fn corner_offset(c: usize) -> [i64; 3] {
    [(c & 1) as i64, ((c >> 1) & 1) as i64, ((c >> 2) & 1) as i64]
}

/// The inside or outside state of a box of nodes, evaluated once.
struct NodeCache {
    origin: [i64; 3],
    dims: [i64; 3],
    inside: Vec<bool>,
}

impl NodeCache {
    fn build<L: Lattice>(lat: &L, origin: [i64; 3], dims: [i64; 3]) -> NodeCache {
        let total = (dims[0] * dims[1] * dims[2]).max(0) as usize;
        let mut inside = Vec::with_capacity(total);
        for x in 0..dims[0] {
            for y in 0..dims[1] {
                for z in 0..dims[2] {
                    inside.push(lat.inside([origin[0] + x, origin[1] + y, origin[2] + z]));
                }
            }
        }
        NodeCache { origin, dims, inside }
    }

    fn get<L: Lattice>(&self, lat: &L, node: [i64; 3]) -> bool {
        let rel: [i64; 3] = std::array::from_fn(|k| node[k] - self.origin[k]);
        if rel.iter().zip(self.dims.iter()).all(|(r, d)| *r >= 0 && r < d) {
            let i = ((rel[0] * self.dims[1] + rel[1]) * self.dims[2] + rel[2]) as usize;
            if let Some(v) = self.inside.get(i) {
                return *v;
            }
        }
        lat.inside(node)
    }
}

struct Builder<'a, L: Lattice> {
    lat: &'a L,
    n: [i64; 3],
    nodes: NodeCache,
    /// Per cell: the vertex of each sign-changing edge of the cell, by edge index.
    cells: HashMap<[i64; 3], [Option<u32>; 12]>,
    positions: Vec<Vec3>,
    triangles: Vec<[u32; 3]>,
}

impl<'a, L: Lattice> Builder<'a, L> {
    /// A builder for the chunk whose first lower node is `first`. Quads and cell patches of the
    /// chunk read nodes from `first - 2` to `first + CHUNK + 1` on each axis, so the cache covers that.
    fn new(lat: &'a L, n: [i64; 3], first: [i64; 3]) -> Self {
        let origin = first.map(|f| f - 2);
        let nodes = NodeCache::build(lat, origin, [CHUNK + 4; 3]);
        Builder { lat, n, nodes, cells: HashMap::new(), positions: Vec::new(), triangles: Vec::new() }
    }

    fn inside(&self, node: [i64; 3]) -> bool {
        self.nodes.get(self.lat, node)
    }

    /// Emits the quads of the edges owned by the chunk whose first lower node is `first`.
    fn chunk(&mut self, first: [i64; 3]) {
        let hi: [i64; 3] = std::array::from_fn(|k| (first[k] + CHUNK).min(self.n[k] + 1));
        for a in 0..3 {
            for l0 in first[0].max(LOWEST)..hi[0] {
                for l1 in first[1].max(LOWEST)..hi[1] {
                    for l2 in first[2].max(LOWEST)..hi[2] {
                        // Lower nodes run up to n: an edge from material at n leaves the lattice to air,
                        // and its quad must still be emitted.
                        let lower = [l0, l1, l2];
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
        let inside = self.inside(lower);
        if inside == self.inside(upper) {
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
            *s = self.inside(std::array::from_fn(|k| cell[k] + off[k]));
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
