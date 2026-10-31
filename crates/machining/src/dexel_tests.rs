use std::time::Instant;

use solidcraft_geom::Vec3;

use super::Stock;
use crate::SimError;
use crate::gcode::{CodeTable, parse};
use crate::sim::simulate;
use crate::sweep::{ArcFixedAxis, FlatTool, LinearFixedAxis};
use crate::toolpath::{Program, Shape, Tool};

const R: f64 = 2.5;
/// The slot's centreline. Off the ray rows of h = 0.2, 0.1 and 0.05, so the walls (SLOT_Y -+ R) are never
/// exactly on a row and the midpoint volume is not biased by a half-row (see `volume()`).
const SLOT_Y: f64 = 10.03;
const L: f64 = 20.0;
const BUDGET: u64 = 1 << 32;
/// Volume tolerance as a fraction of perimeter times h times depth. The test slot is off the ray rows,
/// so the midpoint error is far below this (about 0.4 mm^3 at h = 0.2); a 0.1 mm radius error is 38 mm^3.
const TOL_FACTOR: f64 = 0.1;

fn tool() -> FlatTool {
    FlatTool::new(R, L, Vec3::Z).expect("valid tool")
}

/// A 40 mm slot, 4 mm deep, centred on y = 10, cut from the top (z = 0) down to z = -4.
fn slot() -> LinearFixedAxis {
    LinearFixedAxis { from: Vec3::new(10.0, SLOT_Y, -4.0), to: Vec3::new(50.0, SLOT_Y, -4.0), tool: tool() }
}

fn slot_volume_analytic() -> f64 {
    4.0 * (2.0 * R * 40.0 + std::f64::consts::PI * R * R)
}

fn slot_stock(h: f64) -> Stock {
    Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(60.0, 20.0, 0.0), h, BUDGET).expect("stock")
}

fn box_volume() -> f64 {
    60.0 * 20.0 * 10.0
}

/// A stored coordinate of a ray, converted back to the stock's absolute coordinates.
fn abs(s: &Stock, axis: usize, stored: f32) -> f64 {
    f64::from(stored) + s.min[axis]
}

#[test]
fn slot_floor_is_exact_on_every_interior_z_ray() {
    let mut s = slot_stock(0.2);
    s.apply(&slot(), 1).expect("cut");
    let n = s.cell_counts();
    let mut checked = 0;
    for iu in 0..n[0] {
        for iv in 0..n[1] {
            let (x, y) = (s.pos(0, iu as i64), s.pos(1, iv as i64));
            let segs = s.ray_segs(2, iu, iv);
            if x > 10.5 && x < 49.5 && (y - SLOT_Y).abs() < 2.0 {
                assert_eq!(segs.len(), 1, "one interval left at ({x}, {y})");
                let floor = abs(&s, 2, segs[0].hi);
                assert!((floor + 4.0).abs() < 1e-5, "floor {floor} at ({x}, {y})");
                assert_eq!(segs[0].hi_tag, 1, "the cut made the floor");
                assert_eq!(segs[0].lo_tag, 0, "the bottom is still the stock face");
                checked += 1;
            } else if x < 4.0 || x > 56.0 {
                let top = abs(&s, 2, segs[0].hi);
                assert!(top.abs() < 1e-6, "the top is untouched far from the slot: {top}");
            }
        }
    }
    assert!(checked > 500, "checked {checked} rays");
}

#[test]
fn slot_walls_are_exact_on_x_and_y_rays() {
    let mut s = slot_stock(0.2);
    s.apply(&slot(), 1).expect("cut");
    let n = s.cell_counts();
    // X rays (grid 0, indexed by y then z): the stadium chord at each y, clear of the walls.
    let mut x_checked = 0;
    for iy in 0..n[1] {
        for iz in 0..n[2] {
            let (y, z) = (s.pos(1, iy as i64), s.pos(2, iz as i64));
            let dy = (y - SLOT_Y).abs();
            if dy > 0.05 && dy < R - 0.05 && z > -3.5 && z < -0.5 {
                let h = (R * R - dy * dy).sqrt();
                let segs = s.ray_segs(0, iy, iz);
                assert_eq!(segs.len(), 2, "two material pieces at ({y}, {z})");
                assert!((abs(&s, 0, segs[0].lo) - 0.0).abs() < 1e-5);
                assert!((abs(&s, 0, segs[0].hi) - (10.0 - h)).abs() < 1e-5, "left wall at y={y}");
                assert!((abs(&s, 0, segs[1].lo) - (50.0 + h)).abs() < 1e-5, "right wall at y={y}");
                assert!((abs(&s, 0, segs[1].hi) - 60.0).abs() < 1e-5);
                x_checked += 1;
            }
        }
    }
    assert!(x_checked > 100, "X rays checked: {x_checked}");
    // Y rays (grid 1, indexed by z then x): the walls are at y = 7.5 and 12.5 for x inside the slot.
    let mut y_checked = 0;
    for iz in 0..n[2] {
        for ix in 0..n[0] {
            let (x, z) = (s.pos(0, ix as i64), s.pos(2, iz as i64));
            if x > 10.5 && x < 49.5 && z > -3.5 && z < -0.5 {
                let segs = s.ray_segs(1, iz, ix);
                assert_eq!(segs.len(), 2, "two material pieces at ({x}, {z})");
                assert!((abs(&s, 1, segs[0].hi) - (SLOT_Y - R)).abs() < 1e-5, "near wall at x={x}");
                assert!((abs(&s, 1, segs[1].lo) - (SLOT_Y + R)).abs() < 1e-5, "far wall at x={x}");
                y_checked += 1;
            }
        }
    }
    assert!(y_checked > 100, "Y rays checked: {y_checked}");
}

#[test]
fn slot_volume_converges_to_the_analytic_depth_times_stadium() {
    let analytic_removed = slot_volume_analytic();
    // A 0.1 mm error in the tool radius moves the removed volume by about 38 mm^3 for this slot,
    // so the tolerance (a quarter of perimeter times h times depth) is well below that.
    let perimeter = 2.0 * 40.0 + 2.0 * std::f64::consts::PI * R;
    let mut errs = Vec::new();
    for h in [0.2, 0.1, 0.05] {
        let mut s = slot_stock(h);
        s.apply(&slot(), 1).expect("cut");
        let removed = box_volume() - s.volume();
        let err = (removed - analytic_removed).abs();
        eprintln!("slot h={h}: removed {removed:.4}, analytic {analytic_removed:.4}, error {err:.4}");
        errs.push((h, err, TOL_FACTOR * perimeter * h * 4.0));
    }
    for (h, err, tol) in &errs {
        assert!(err <= tol, "h={h}: error {err} over {tol}");
    }
    assert!(errs[1].1 < errs[0].1 && errs[2].1 < errs[1].1, "error falls as h halves: {errs:?}");
}

#[test]
fn a_slightly_smaller_tool_fails_the_volume_check() {
    // A 0.1 mm radius error changes the removed volume by more than the h = 0.1 tolerance.
    let perimeter = 2.0 * 40.0 + 2.0 * std::f64::consts::PI * R;
    let tol = TOL_FACTOR * perimeter * 0.1 * 4.0;
    let smaller = FlatTool::new(R - 0.1, L, Vec3::Z).expect("tool");
    let sweep = LinearFixedAxis { from: slot().from, to: slot().to, tool: smaller };
    let mut s = slot_stock(0.1);
    s.apply(&sweep, 1).expect("cut");
    let err = (box_volume() - s.volume() - slot_volume_analytic()).abs();
    assert!(err > tol, "a 0.1 mm radius error gave {err}, under the tolerance {tol}");
}

fn pocket_program() -> Vec<crate::toolpath::Move> {
    let text = "G0 X37.5 Y30 Z5\nG1 Z-4 F200\nG3 X37.5 Y30 I-7.5 J0\nG1 X32.5 Y30\nG3 X32.5 Y30 I-2.5 J0";
    parse(text, &CodeTable::default()).moves
}

fn run_pocket(s: &mut Stock) -> Vec<crate::toolpath::Diag> {
    let program = Program { tools: vec![flat_tool(0)], moves: pocket_program(), diags: Vec::new() };
    simulate(&program, s).expect("simulate")
}

fn flat_tool(id: u32) -> Tool {
    Tool { id, shape: Shape::Flat, diameter: 2.0 * R, flute_length: L, shank: Vec::new(), holder: Vec::new(), stickout: 30.0 }
}

#[test]
fn circular_pocket_from_g3_arcs_matches_the_analytic_volume() {
    let analytic = 4.0 * std::f64::consts::PI * 10.0 * 10.0;
    let mut s = Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(60.0, 60.0, 0.0), 0.2, BUDGET).expect("stock");
    let before = s.volume();
    let diags = run_pocket(&mut s);
    assert!(diags.is_empty(), "{diags:?}");
    let removed = before - s.volume();
    let err = (removed - analytic).abs();
    eprintln!("pocket h=0.2: removed {removed:.3}, analytic {analytic:.3}, error {err:.3}");
    let perimeter = 2.0 * std::f64::consts::PI * 10.0;
    assert!(err <= TOL_FACTOR * perimeter * 0.2 * 4.0, "error {err}");
}

#[test]
fn same_program_gives_the_same_state_hash() {
    let mut a = slot_stock(0.25);
    let mut b = slot_stock(0.25);
    run_pocket(&mut a);
    run_pocket(&mut b);
    assert_eq!(a.state_hash(), b.state_hash());
    let mut c = slot_stock(0.25);
    c.apply(&slot(), 1).expect("cut");
    assert_ne!(a.state_hash(), c.state_hash());
}

#[test]
fn copy_on_write_keeps_the_checkpoint() {
    let mut s = slot_stock(0.25);
    let checkpoint = s.clone();
    s.apply(&slot(), 1).expect("cut");
    assert!((checkpoint.volume() - box_volume()).abs() < 1e-6);
    assert!(s.volume() < box_volume());
}

#[test]
fn stock_refuses_bad_sizes_and_budgets() {
    let ok = Vec3::new(0.0, 0.0, 0.0);
    let far = Vec3::new(10.0, 10.0, 10.0);
    assert!(matches!(Stock::new_box(ok, far, 0.0, BUDGET), Err(SimError::BadInput(_))));
    assert!(matches!(Stock::new_box(ok, far, f64::NAN, BUDGET), Err(SimError::BadInput(_))));
    assert!(matches!(Stock::new_box(ok, far, -1.0, BUDGET), Err(SimError::BadInput(_))));
    assert!(matches!(Stock::new_box(far, ok, 1.0, BUDGET), Err(SimError::BadInput(_))));
    assert!(matches!(Stock::new_box(ok, Vec3::new(1e9, 1.0, 1.0), 1.0, BUDGET), Err(SimError::BadInput(_))));
    assert!(matches!(Stock::new_box(ok, far, 1e-9, BUDGET), Err(SimError::BadInput(_))));
    assert!(matches!(Stock::new_box(ok, far, 0.5, 10), Err(SimError::Budget { .. })));
}

#[test]
fn budget_is_enforced_as_cuts_split_rays() {
    // A 1 mm cube at h = 1 is 3 rays of 20 bytes. A cut that splits a ray adds bytes, so a budget
    // with only 8 bytes to spare refuses it, and the stock is unchanged. The estimate is 72 bytes:
    // 3 rays of 20 bytes and 3 tile sentinels of 4.
    let mut s = Stock::new_box(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0), 1.0, 72 + 8).expect("stock fits");
    let hole =
        LinearFixedAxis { from: Vec3::new(0.5, 0.5, 0.4), to: Vec3::new(0.5, 0.5, 0.401), tool: FlatTool::new(0.1, 0.001, Vec3::Z).expect("tool") };
    let (segs, hash) = (s.segments(), s.state_hash());
    assert!(matches!(s.apply(&hole, 1), Err(SimError::Budget { .. })));
    assert_eq!((s.segments(), s.state_hash()), (segs, hash));
}

#[test]
fn too_many_segments_is_an_error_and_leaves_the_stock_alone() {
    let mut s = Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(1.0, 1.0, 0.0), 1.0, BUDGET).expect("stock");
    let mut failed = false;
    for k in 0..400 {
        let z = -9.9 + 0.03 * f64::from(k);
        let hole = LinearFixedAxis {
            from: Vec3::new(0.5, 0.5, z),
            to: Vec3::new(0.5, 0.5, z + 0.001),
            tool: FlatTool::new(0.1, 0.001, Vec3::Z).expect("tool"),
        };
        let (segs_before, hash_before) = (s.segments(), s.state_hash());
        match s.apply(&hole, k + 1) {
            Ok(()) => {}
            Err(SimError::SegmentCap { .. }) => {
                // A failed cut commits nothing: every grid is as it was before this cut.
                assert_eq!(s.segments(), segs_before);
                assert_eq!(s.state_hash(), hash_before);
                failed = true;
                break;
            }
            Err(e) => panic!("{e:?}"),
        }
    }
    assert!(failed, "the cap is reached");
}

#[test]
fn bad_sweep_bounds_do_not_panic() {
    let mut s = slot_stock(0.5);
    let nan = LinearFixedAxis { from: Vec3::new(f64::NAN, 0.0, 0.0), to: Vec3::new(1.0, 1.0, 1.0), tool: tool() };
    assert!(s.apply(&nan, 1).is_ok());
    assert!((s.volume() - box_volume()).abs() < 1e-6);
}

#[test]
fn g_code_slot_simulates_to_the_analytic_floor() {
    let text = "G0 X10 Y10.03 Z5\nG1 Z-4 F100\nG1 X50\nG0 Z5\nG0 X10 Y10.03";
    let program = parse(text, &CodeTable::default());
    assert!(program.diags.is_empty());
    let program = Program { tools: vec![flat_tool(0)], ..program };
    let mut s = slot_stock(0.2);
    let diags = simulate(&program, &mut s).expect("simulate");
    assert!(diags.is_empty(), "{diags:?}");
    let removed = box_volume() - s.volume();
    assert!((removed - slot_volume_analytic()).abs() < TOL_FACTOR * (2.0 * 40.0 + 2.0 * std::f64::consts::PI * R) * 0.2 * 4.0, "removed {removed}");
}

#[test]
fn a_jump_after_home_is_not_cut_and_the_next_move_starts_from_it() {
    // Home is unknown: the move after G28 starts a new path, so the travel to it is not cut.
    let text = "G0 X10 Y10 Z5\nG1 Z-4 F100\nG1 X20\nG28\nG0 X30 Y10 Z5\nG1 X40 Y10 Z-4";
    let program = parse(text, &CodeTable::default());
    assert!(program.diags.iter().all(|d| !d.is_error()), "{:?}", program.diags);
    let program = Program { tools: vec![flat_tool(0)], ..program };
    // Moves: rapid, feed, feed, then the rapid after G28 (G28 itself makes no move).
    let jump = program.moves.iter().position(|m| m.kind == crate::toolpath::MoveKind::Jump).expect("a jump");
    assert_eq!(jump, 3, "the rapid after G28 is a jump");
    let mut s = slot_stock(0.2);
    let diags = simulate(&program, &mut s).expect("simulate");
    assert!(diags.is_empty(), "{diags:?}");
}

#[test]
fn unsupported_shapes_are_reported_once_not_panicked() {
    let text = "G0 X10 Y10 Z5\nG1 Z-4\nG1 X20\nG1 X30";
    let mut program = parse(text, &CodeTable::default());
    program.tools =
        vec![Tool { id: 0, shape: Shape::Ball, diameter: 5.0, flute_length: 20.0, shank: Vec::new(), holder: Vec::new(), stickout: 30.0 }];
    let mut s = slot_stock(0.5);
    let diags = simulate(&program, &mut s).expect("simulate");
    assert_eq!(diags.len(), 1, "{diags:?}");
    assert!(diags[0].msg.contains("not simulated yet"));
    assert!((s.volume() - box_volume()).abs() < 1e-6);
}

#[test]
fn undefined_tool_is_reported() {
    let program = parse("G0 X1\nG1 X2", &CodeTable::default());
    let mut s = slot_stock(0.5);
    let diags = simulate(&program, &mut s).expect("simulate");
    assert!(diags.iter().any(|d| d.msg.contains("not defined")), "{diags:?}");
}

#[test]
fn a_failing_cut_names_its_move() {
    let mut s = Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(1.0, 1.0, 0.0), 1.0, BUDGET).expect("stock");
    let mut moves = Vec::new();
    for k in 0..400 {
        let z = -9.9 + 0.03 * f64::from(k);
        moves.push(crate::toolpath::Move {
            kind: crate::toolpath::MoveKind::Jump,
            tip: [0.5, 0.5, z + 0.001],
            axis: [0.0, 0.0, 1.0],
            joints: None,
            interp: crate::toolpath::Interp::Tcp,
            tool: 0,
            feed: 1.0,
            spindle: 0.0,
            source: 100 + k as u32,
        });
        moves.push(crate::toolpath::Move {
            kind: crate::toolpath::MoveKind::Feed,
            tip: [0.5, 0.5, z],
            axis: [0.0, 0.0, 1.0],
            joints: None,
            interp: crate::toolpath::Interp::Tcp,
            tool: 0,
            feed: 0.0,
            spindle: 0.0,
            source: 100 + k as u32,
        });
    }
    let program = Program {
        tools: vec![Tool { id: 0, shape: Shape::Flat, diameter: 0.2, flute_length: 0.001, shank: Vec::new(), holder: Vec::new(), stickout: 30.0 }],
        moves,
        diags: Vec::new(),
    };
    match simulate(&program, &mut s) {
        Err(SimError::AtMove { index, line, cause }) => {
            assert!(index > 0 && line >= 100, "index {index} line {line}");
            assert!(cause.contains("segments"), "{cause}");
        }
        other => panic!("expected a named failure, got {other:?}"),
    }
}

#[test]
fn slot_timing_for_the_report() {
    let mut s = slot_stock(0.1);
    let t0 = Instant::now();
    s.apply(&slot(), 1).expect("cut");
    eprintln!("one 40 mm slot move at h=0.1: {:?}", t0.elapsed());
}

#[test]
fn arc_sweep_on_the_stock_matches_a_full_circle_pocket_ring() {
    let mut s = Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(60.0, 60.0, 0.0), 0.4, BUDGET).expect("stock");
    let p = Vec3::new(37.5, 30.0, -4.0);
    let arc = ArcFixedAxis::new(p, p, Vec3::new(30.0, 30.0, -4.0), Vec3::Z, true, 0, tool()).expect("arc");
    let before = s.volume();
    s.apply(&arc, 1).expect("cut");
    // One full ring of radius 7.5 with a 2.5 mm tool cuts the annulus 5..10 mm, 4 mm deep.
    let removed = before - s.volume();
    let analytic = 4.0 * std::f64::consts::PI * (10.0 * 10.0 - 5.0 * 5.0);
    eprintln!("ring h=0.4: removed {removed:.3}, analytic {analytic:.3}");
    assert!((removed - analytic).abs() < 0.1 * analytic, "removed {removed}");
}

#[test]
fn a_non_finite_position_is_a_diagnostic_not_a_cut() {
    let mut program = parse("G0 X10 Y10 Z5\nG1 Z-4\nG1 X20", &CodeTable::default());
    program.tools = vec![flat_tool(0)];
    program.moves[1].tip[0] = f64::NAN;
    let mut s = slot_stock(0.5);
    let diags = simulate(&program, &mut s).expect("simulate");
    assert!(diags.iter().any(|d| d.msg.contains("non-finite")), "{diags:?}");
}

#[test]
fn radius_60_arc_move_timing_for_the_report() {
    // A quarter arc of radius 60 with a 5 mm tool, 4 mm deep, at h = 0.1 on a 140 mm square stock.
    let mut s = Stock::new_box(Vec3::new(-70.0, -70.0, -10.0), Vec3::new(70.0, 70.0, 0.0), 0.1, 1 << 34).expect("stock");
    let t = FlatTool::new(2.5, 10.0, Vec3::Z).expect("tool");
    let arc = ArcFixedAxis::new(Vec3::new(60.0, 0.0, -4.0), Vec3::new(0.0, 60.0, -4.0), Vec3::new(0.0, 0.0, -4.0), Vec3::Z, true, 0, t).expect("arc");
    let before = s.volume();
    let t0 = Instant::now();
    for piece in arc.revolutions() {
        s.apply(&piece, 1).expect("cut");
    }
    let elapsed = t0.elapsed();
    let removed = before - s.volume();
    // A quarter annulus between radii 57.5 and 62.5, plus the two end caps (one full disc in all).
    let analytic = 4.0 * (std::f64::consts::FRAC_PI_4 * (62.5f64.powi(2) - 57.5f64.powi(2)) + std::f64::consts::PI * 2.5 * 2.5);
    eprintln!("R=60 quarter arc h=0.1: {elapsed:?}, removed {removed:.2}, analytic {analytic:.2}");
    assert!((removed - analytic).abs() < 0.01 * analytic, "removed {removed} vs {analytic}");
}

/// Many holes on every ray: a piece whose cut overflows the segment cap.
struct Holes;

impl crate::sweep::Sweep for Holes {
    fn bounds(&self) -> solidcraft_geom::Aabb3 {
        solidcraft_geom::Aabb3 { min: Vec3::new(-1.0, -1.0, -11.0), max: Vec3::new(2.0, 2.0, 1.0) }
    }

    fn ray(&self, _o: Vec3, _d: Vec3, _t_range: (f64, f64), out: &mut Vec<(f64, f64)>) {
        for k in 0..300 {
            let z = -9.9 + 0.03 * f64::from(k);
            out.push((z, z + 0.01));
        }
    }
}

#[test]
fn a_multi_piece_cut_that_fails_partway_leaves_the_stock_unchanged() {
    // An arc is several revolution pieces. If a later piece fails, the earlier ones must not stay.
    let mut s = Stock::new_box(Vec3::new(0.0, 0.0, -10.0), Vec3::new(1.0, 1.0, 0.0), 1.0, BUDGET).expect("stock");
    let good =
        LinearFixedAxis { from: Vec3::new(0.5, 0.5, -2.0), to: Vec3::new(0.5, 0.5, -1.0), tool: FlatTool::new(0.1, 0.5, Vec3::Z).expect("tool") };
    let before = (s.segments(), s.state_hash());
    let pieces: [&dyn crate::sweep::Sweep; 2] = [&good, &Holes];
    assert!(s.apply_all(&pieces, 1).is_err());
    assert_eq!((s.segments(), s.state_hash()), before);
}
