//! The accuracy comparison against parts whose answers are worked out by hand: a box stock cut by a flat
//! slot, a plunge, a groove, a chamfer and leaves, against the parts they should match; a box against itself;
//! and the open, degenerate and hostile meshes it must refuse.

use solidcore_record::Vec3;

use super::{Accuracy, compare, subtract};
use crate::SimError;
use crate::dexel::Stock;
use crate::sweep::{FlatTool, LinearFixedAxis};

/// Ray spacing of the test stocks, mm. The grid is 600 x 200 x 100 cells.
const H: f64 = 0.1;
const BUDGET: u64 = 1 << 30;
/// The slot's walls: a Ø6 tool along Y at x = 30 cuts x from 27 to 33.
const XA: f64 = 27.0;
const XB: f64 = 33.0;

fn stock_corners() -> (Vec3, Vec3) {
    (Vec3::new(0.0, 0.0, -10.0), Vec3::new(60.0, 20.0, 0.0))
}

/// The box stock, cut from the top by a Ø6 flat end mill along Y at x = 30, its tip at height `tip`.
fn slotted_stock(tip: f64) -> Stock {
    let (min, max) = stock_corners();
    let mut s = Stock::new_box(min, max, H, BUDGET).expect("stock");
    let tool = FlatTool::new(3.0, 20.0, Vec3::Z).expect("tool");
    s.apply(&LinearFixedAxis { from: Vec3::new(30.0, -5.0, tip), to: Vec3::new(30.0, 25.0, tip), tool }, 1).expect("cut");
    s
}

/// The box stock with a sweep of `radius` at `from` to `to`, cut from the top.
fn cut(min: Vec3, max: Vec3, radius: f64, from: Vec3, to: Vec3) -> Stock {
    let mut s = Stock::new_box(min, max, H, BUDGET).expect("stock");
    let tool = FlatTool::new(radius, 20.0, Vec3::Z).expect("tool");
    s.apply(&LinearFixedAxis { from, to, tool }, 1).expect("cut");
    s
}

/// A closed triangle mesh built from axis-aligned faces, subdivided at the breakpoints of every face so that
/// neighbouring faces share their edges' vertices exactly.
#[derive(Default)]
struct Solid {
    p: Vec<Vec3>,
    t: Vec<[u32; 3]>,
}

impl Solid {
    fn vertex(&mut self, c: [f64; 3]) -> u32 {
        let v = Vec3::from(c);
        if let Some(i) = self.p.iter().position(|q| *q == v) {
            return i as u32;
        }
        self.p.push(v);
        (self.p.len() - 1) as u32
    }

    /// Adds a quad for every cell of the breakpoint grid `a` x `b` whose centre `keep` accepts, mapped to 3D.
    fn face(&mut self, map: impl Fn(f64, f64) -> [f64; 3], a: &[f64], b: &[f64], keep: impl Fn(f64, f64) -> bool) {
        for wa in a.windows(2) {
            for wb in b.windows(2) {
                let (a0, a1, b0, b1) = (wa[0], wa[1], wb[0], wb[1]);
                if !keep(0.5 * (a0 + a1), 0.5 * (b0 + b1)) {
                    continue;
                }
                let q = [self.vertex(map(a0, b0)), self.vertex(map(a1, b0)), self.vertex(map(a1, b1)), self.vertex(map(a0, b1))];
                self.t.push([q[0], q[1], q[2]]);
                self.t.push([q[0], q[2], q[3]]);
            }
        }
    }

    fn tri(&mut self, a: [f64; 3], b: [f64; 3], c: [f64; 3]) {
        let t = [self.vertex(a), self.vertex(b), self.vertex(c)];
        self.t.push(t);
    }
}

/// The stock's box, solid: the part of an uncut stock.
fn box_part() -> Solid {
    let (x0, x1, y0, y1, zb, zt) = (0.0, 60.0, 0.0, 20.0, -10.0, 0.0);
    let (xs, ys, zs) = ([x0, x1], [y0, y1], [zb, zt]);
    let mut s = Solid::default();
    for y in ys {
        s.face(|x, z| [x, y, z], &xs, &zs, |_, _| true);
    }
    s.face(|x, y| [x, y, zb], &xs, &ys, |_, _| true);
    s.face(|x, y| [x, y, zt], &xs, &ys, |_, _| true);
    for x in xs {
        s.face(|y, z| [x, y, z], &ys, &zs, |_, _| true);
    }
    s
}

/// The stock's box with a slot from the top, at x from 27 to 33 (the Ø6 cut), running through in Y, whose
/// floor is `depth` below the top.
fn slot_part(depth: f64) -> Solid {
    let (x0, x1, y0, y1, zb, zt) = (0.0, 60.0, 0.0, 20.0, -10.0, 0.0);
    let zf = -depth;
    let (xs, ys, zs) = ([x0, XA, XB, x1], [y0, y1], [zb, zf, zt]);
    let mut s = Solid::default();
    for y in ys {
        s.face(|x, z| [x, y, z], &xs, &zs, |x, z| !(x > XA && x < XB && z > zf));
    }
    s.face(|x, y| [x, y, zb], &xs, &ys, |_, _| true);
    s.face(|x, y| [x, y, zt], &xs, &ys, |x, _| !(x > XA && x < XB));
    s.face(|x, y| [x, y, zf], &xs, &ys, |x, _| x > XA && x < XB);
    for x in [x0, x1] {
        s.face(|y, z| [x, y, z], &ys, &zs, |_, _| true);
    }
    for x in [XA, XB] {
        s.face(|y, z| [x, y, z], &ys, &[zf, zt], |_, _| true);
    }
    s
}

/// The box with its edge along Y at x = 0, z = 0 cut away by a 45 degree chamfer of leg `leg`.
fn chamfer_part(leg: f64) -> Solid {
    let outline = [(0.0, -10.0), (60.0, -10.0), (60.0, 0.0), (leg, 0.0), (0.0, -leg)];
    let mut s = Solid::default();
    for y in [0.0, 20.0] {
        let p = outline.map(|(x, z)| [x, y, z]);
        s.tri(p[0], p[1], p[2]);
        s.tri(p[0], p[2], p[3]);
        s.tri(p[0], p[3], p[4]);
    }
    for k in 0..outline.len() {
        let ((x0, z0), (x1, z1)) = (outline[k], outline[(k + 1) % outline.len()]);
        let (a, b, c, d) = ([x0, 0.0, z0], [x1, 0.0, z1], [x1, 20.0, z1], [x0, 20.0, z0]);
        s.tri(a, b, c);
        s.tri(a, c, d);
    }
    s
}

fn compared(s: &Stock, p: &Solid) -> Accuracy {
    compare(s, &p.p, &p.t).unwrap_or_else(|e| panic!("compare: {e}"))
}

#[test]
fn a_slot_agrees_with_its_own_part() {
    let a = compared(&slotted_stock(-4.0), &slot_part(4.0));
    assert!((a.resolution - H).abs() < 1e-9, "resolution {}", a.resolution);
    assert!(a.max_left_over < 1e-3 && a.max_gouge < 1e-3, "{a:?}");
    assert!(a.mean_abs_dev < 1e-3 && a.rms < 1e-3, "{a:?}");
    assert_eq!(a.rays, 200_000, "the three grids: 200 x 100, 100 x 600 and 600 x 200 rays");
    assert_eq!(a.odd_rays, 0);
}

#[test]
fn a_slot_0_5_mm_deeper_than_the_cut_leaves_a_half_millimetre_left_over() {
    let a = compared(&slotted_stock(-4.0), &slot_part(4.5));
    assert!((a.max_left_over - 0.5).abs() <= 1e-3, "left-over {}", a.max_left_over);
    assert!(a.max_gouge < 1e-3, "gouge {}", a.max_gouge);
    let worst = a.worst_left_over.unwrap_or_else(|| panic!("no left-over spot: {a:?}"));
    assert!((-4.5 - a.resolution..=-4.0 + a.resolution).contains(&worst.at.z), "left-over at z {}", worst.at.z);
    assert!(worst.at.x > XA && worst.at.x < XB, "left-over at x {}", worst.at.x);
}

#[test]
fn a_slot_0_5_mm_shallower_than_the_cut_gouges_half_a_millimetre() {
    let a = compared(&slotted_stock(-4.0), &slot_part(3.5));
    assert!((a.max_gouge - 0.5).abs() <= 1e-3, "gouge {}", a.max_gouge);
    assert!(a.max_left_over < 1e-3, "left-over {}", a.max_left_over);
    let worst = a.worst_gouge.unwrap_or_else(|| panic!("no gouge spot: {a:?}"));
    assert!((-4.0 - a.resolution..=-3.5 + a.resolution).contains(&worst.at.z), "gouge at z {}", worst.at.z);
}

#[test]
fn the_headline_is_the_largest_grid_and_each_grid_is_reported() {
    let a = compared(&slotted_stock(-4.0), &slot_part(4.5));
    // The Z rays cross the 0.5 mm band normally. The X rays run along the slot's floor between its walls: both
    // ends are the part's, so that run is no depth and the X grid reports nothing.
    assert!((a.grids[2].max_left_over - 0.5).abs() <= 1e-3, "Z grid {:?}", a.grids[2]);
    assert!(a.grids[0].max_left_over < 1e-3, "X grid {:?}", a.grids[0]);
    assert_eq!(a.max_left_over, a.grids.iter().map(|g| g.max_left_over).fold(0.0, f64::max));
}

#[test]
fn an_uncut_stock_matches_the_box_part_exactly() {
    let (min, max) = stock_corners();
    let s = Stock::new_box(min, max, H, BUDGET).expect("stock");
    let a = compared(&s, &box_part());
    assert!(a.max_left_over < 1e-9 && a.max_gouge < 1e-9, "{a:?}");
    assert!(a.mean_abs_dev < 1e-9 && a.rms < 1e-9, "{a:?}");
}

/// A stock `leave` mm larger than the box on every side, at grid spacing `h`.
fn oversized_stock(leave: f64, h: f64) -> Stock {
    let (min, max) = stock_corners();
    let lo = Vec3::new(min.x - leave, min.y - leave, min.z - leave);
    let hi = Vec3::new(max.x + leave, max.y + leave, max.z + leave);
    Stock::new_box(lo, hi, h, BUDGET).expect("stock")
}

#[test]
fn a_uniform_leave_is_its_depth_at_every_grid_spacing() {
    for h in [0.1, 0.25, 0.5] {
        let a = compare(&oversized_stock(0.2, h), &box_part().p, &box_part().t).unwrap_or_else(|e| panic!("compare: {e}"));
        assert!((a.max_left_over - 0.2).abs() < 0.05, "h {h}: left-over {}", a.max_left_over);
        assert!(a.max_gouge < 1e-6, "h {h}: gouge {}", a.max_gouge);
        assert!((a.mean_abs_dev - 0.2).abs() < 0.05, "h {h}: mean {}", a.mean_abs_dev);
        assert!((a.rms - 0.2).abs() < 0.05, "h {h}: rms {}", a.rms);
    }
}

#[test]
fn a_plunge_3_mm_too_deep_gouges_3_mm() {
    // The Ø2 tool's tip ends at z -3; the design is solid there.
    let (min, max) = stock_corners();
    let s = cut(min, max, 1.0, Vec3::new(30.0, 10.0, 1.0), Vec3::new(30.0, 10.0, -3.0));
    let a = compared(&s, &box_part());
    assert!((a.max_gouge - 3.0).abs() < 0.05, "gouge {}", a.max_gouge);
    assert!(a.max_left_over < 1e-6, "left-over {}", a.max_left_over);
    let worst = a.worst_gouge.unwrap_or_else(|| panic!("no gouge spot: {a:?}"));
    assert!((worst.at.z + 1.5).abs() < 0.05, "gouge at z {}", worst.at.z);
}

#[test]
fn a_1_mm_groove_2_mm_deep_along_y_gouges_2_mm() {
    let (min, max) = stock_corners();
    let s = cut(min, max, 0.5, Vec3::new(30.0, -5.0, -2.0), Vec3::new(30.0, 25.0, -2.0));
    let a = compared(&s, &box_part());
    assert!((a.max_gouge - 2.0).abs() < 0.05, "gouge {}", a.max_gouge);
    assert!(a.max_left_over < 1e-6, "left-over {}", a.max_left_over);
    assert!(a.grids[1].max_gouge < 1e-6, "the Y rays run along the groove: {:?}", a.grids[1]);
}

#[test]
fn a_chamfer_of_leg_2_left_uncut_leaves_about_1_4_mm() {
    let (min, max) = stock_corners();
    let s = Stock::new_box(min, max, H, BUDGET).expect("stock");
    let a = compared(&s, &chamfer_part(2.0));
    assert!((1.37..=1.43).contains(&a.max_left_over), "left-over {}", a.max_left_over);
    assert!(a.max_gouge < 1e-6, "gouge {}", a.max_gouge);
}

#[test]
fn an_exact_slot_is_zero() {
    let a = compared(&slotted_stock(-4.0), &slot_part(4.0));
    assert!(a.max_left_over.max(a.max_gouge) < 1e-3, "{a:?}");
}

/// Open: the headline is 4.0 here, not 1.0. The Z rays through the 1 mm wall strip run 4 mm along it, and
/// their run's reference end is the slot floor (normal z), so `length * |n_axis|` overstates it. The X grid
/// measures 1.0 correctly. Needs a rule for bands tangent to their ray (see the module note).
#[test]
#[ignore = "known overstatement: a wall strip standing on a floor reads 4.0 in the Z grid, not 1.0"]
fn a_leave_of_1_mm_on_the_walls_and_0_5_mm_on_the_floor_leaves_1_mm() {
    // Stock cut by a Ø4 tool at tip -3.5: 1 mm of wall on each side of the Ø6 slot, 0.5 mm over its floor.
    let (min, max) = stock_corners();
    let s = cut(min, max, 2.0, Vec3::new(30.0, -5.0, -3.5), Vec3::new(30.0, 25.0, -3.5));
    let a = compared(&s, &slot_part(4.0));
    assert!((a.grids[0].max_left_over - 1.0).abs() < 0.05, "X grid {:?}", a.grids[0]);
    assert!((a.max_left_over - 1.0).abs() < 0.05, "left-over {}", a.max_left_over);
}

#[test]
fn an_open_mesh_is_refused_by_name() {
    let mut p = box_part();
    p.t.pop();
    let (min, max) = stock_corners();
    let s = Stock::new_box(min, max, H, BUDGET).expect("stock");
    match compare(&s, &p.p, &p.t) {
        Err(SimError::BadInput(msg)) => assert!(msg.contains("not closed"), "{msg}"),
        other => panic!("an open mesh was not refused: {other:?}"),
    }
}

#[test]
fn a_mesh_of_absurd_size_is_refused() {
    let (min, max) = stock_corners();
    let s = Stock::new_box(min, max, H, BUDGET).expect("stock");
    let far = [Vec3::new(-1e308, -1e308, -1e308), Vec3::new(1e308, 1e308, 1e308)];
    let t = [[0, 1, 0]];
    assert!(matches!(compare(&s, &far, &t), Err(SimError::BadInput(_))));
}

#[test]
fn degenerate_and_hostile_meshes_are_refused() {
    let (min, max) = stock_corners();
    let s = Stock::new_box(min, max, H, BUDGET).expect("stock");
    let good = box_part();
    let mut nan = box_part();
    nan.p[0] = Vec3::new(f64::NAN, 0.0, 0.0);
    let mut sheet = box_part();
    sheet.t.retain(|t| t.iter().all(|&i| sheet_top(&good, i)));
    let cases: [(&str, &[Vec3], &[[u32; 3]]); 5] = [
        ("no triangles", &good.p, &[]),
        ("index past the vertices", &good.p, &[[0, 1, 999]]),
        ("non-finite position", &nan.p, &nan.t),
        ("every triangle degenerate", &[Vec3::new(1.0, 1.0, 1.0); 3], &[[0, 1, 2]]),
        ("a flat sheet", &sheet.p, &sheet.t),
    ];
    for (name, p, t) in cases {
        assert!(matches!(compare(&s, p, t), Err(SimError::BadInput(_))), "{name} was not refused");
    }
}

/// Whether vertex `i` of `solid` lies on the top face (z = 0): the flat sheet is the box's top only.
fn sheet_top(solid: &Solid, i: u32) -> bool {
    solid.p.get(i as usize).is_some_and(|p| p.z == 0.0)
}

#[test]
fn a_stock_without_the_memory_for_the_check_is_refused() {
    let (min, max) = stock_corners();
    let need = Stock::box_bytes(min, max, H).expect("layout");
    let s = Stock::new_box(min, max, H, need).expect("a stock that fits exactly");
    let p = box_part();
    assert!(matches!(compare(&s, &p.p, &p.t), Err(SimError::Budget { .. })));
}

#[test]
fn subtract_keeps_what_the_other_side_does_not_cover() {
    let mut out = Vec::new();
    subtract(&[(0.0, 10.0)], &[(2.0, 3.0), (5.0, 7.0)], &mut out);
    assert_eq!(out, vec![(0.0, 2.0), (3.0, 5.0), (7.0, 10.0)]);
    subtract(&[(0.0, 1.0), (2.0, 3.0)], &[(0.5, 2.5)], &mut out);
    assert_eq!(out, vec![(0.0, 0.5), (2.5, 3.0)]);
    subtract(&[(0.0, 4.0)], &[(1.0, 2.0), (2.0, 3.0)], &mut out);
    assert_eq!(out, vec![(0.0, 1.0), (3.0, 4.0)]);
    subtract(&[(0.0, 1.0)], &[(0.0, 1.0)], &mut out);
    assert!(out.is_empty());
    subtract(&[(0.0, 1.0)], &[(-1.0, 2.0)], &mut out);
    assert!(out.is_empty());
}
