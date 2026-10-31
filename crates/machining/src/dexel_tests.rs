use std::time::Instant;

use solidcraft_geom::Vec3;

use super::Stock;
use crate::SimError;
use crate::gcode::{CodeTable, parse};
use crate::sim::simulate;
use crate::sweep::{ArcFixedAxis, FlatTool, LinearFixedAxis};
use crate::toolpath::{Diag, Shape, Tool};

const R: f64 = 2.5;
const L: f64 = 20.0;
const BUDGET: u64 = 1 << 32;

fn tool() -> FlatTool {
    FlatTool::new(R, L, Vec3::Z).expect("valid tool")
}

/// A 40 mm slot, 4 mm deep, centred on y = 10, cut from the top (z = 0) down to z = -4.
fn slot() -> LinearFixedAxis {
    LinearFixedAxis { from: Vec3::new(10.0, 10.0, -4.0), to: Vec3::new(50.0, 10.0, -4.0), tool: tool() }
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
            if x > 10.5 && x < 49.5 && (y - 10.0).abs() < 2.0 {
                assert_eq!(segs.len(), 1, "one interval left at ({x}, {y})");
                assert!((f64::from(segs[0].hi) + 4.0).abs() < 1e-5, "floor {} at ({x}, {y})", segs[0].hi);
                assert_eq!(segs[0].hi_tag, 1, "the cut made the floor");
                assert_eq!(segs[0].lo_tag, 0, "the bottom is still the stock face");
                checked += 1;
            } else if x < 4.0 || x > 56.0 {
                assert_eq!(segs[0].hi, 0.0, "the top is untouched far from the slot");
            }
        }
    }
    assert!(checked > 500, "checked {checked} rays");
}

#[test]
fn slot_volume_converges_to_the_analytic_depth_times_stadium() {
    let analytic_removed = slot_volume_analytic();
    let mut errs = Vec::new();
    for h in [0.2, 0.1, 0.05] {
        let mut s = slot_stock(h);
        s.apply(&slot(), 1).expect("cut");
        let removed = box_volume() - s.volume();
        let err = (removed - analytic_removed).abs();
        eprintln!("slot h={h}: removed {removed:.4}, analytic {analytic_removed:.4}, error {err:.4}");
        errs.push(err);
    }
    // The error is bounded by the perimeter times h times depth, and it does not grow as h falls.
    let perimeter = 2.0 * 40.0 + 2.0 * std::f64::consts::PI * R;
    for (err, h) in errs.iter().zip([0.2, 0.1, 0.05]) {
        assert!(*err <= perimeter * h * 4.0, "h={h}: error {err}");
    }
    assert!(errs[2] <= errs[0] + 1e-9, "{errs:?}");
}

fn pocket_program() -> Vec<crate::toolpath::Move> {
    let text = "G0 X37.5 Y30 Z5\nG1 Z-4 F200\nG3 X37.5 Y30 I-7.5 J0\nG1 X32.5 Y30\nG3 X32.5 Y30 I-2.5 J0";
    parse(text, &CodeTable::default()).moves
}

fn run_pocket(s: &mut Stock) -> Vec<Diag> {
    let program = crate::toolpath::Program { tools: vec![flat_tool(0)], moves: pocket_program(), diags: Vec::new() };
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
    assert!(err <= perimeter * 0.2 * 4.0, "error {err}");
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
    assert_eq!(s.volume(), box_volume());
}

#[test]
fn g_code_slot_simulates_to_the_analytic_floor() {
    let text = "G0 X10 Y10 Z5\nG1 Z-4 F100\nG1 X50\nG0 Z5\nG0 X10 Y10";
    let program = parse(text, &CodeTable::default());
    assert!(program.diags.is_empty());
    let program = crate::toolpath::Program { tools: vec![flat_tool(0)], ..program };
    let mut s = slot_stock(0.2);
    let diags = simulate(&program, &mut s).expect("simulate");
    assert!(diags.is_empty(), "{diags:?}");
    let removed = box_volume() - s.volume();
    // The plunge at the slot end and the rapids above the stock add nothing beyond the slot.
    assert!((removed - slot_volume_analytic()).abs() < 15.0, "removed {removed}");
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
    assert_eq!(s.volume(), box_volume());
}

#[test]
fn undefined_tool_is_reported() {
    let program = parse("G0 X1\nG1 X2", &CodeTable::default());
    let mut s = slot_stock(0.5);
    let diags = simulate(&program, &mut s).expect("simulate");
    assert!(diags.iter().any(|d| d.msg.contains("not defined")), "{diags:?}");
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
