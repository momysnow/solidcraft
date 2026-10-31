use solidcraft_geom::Vec3;

use super::{ArcFixedAxis, FlatTool, LinearFixedAxis, MAX_ARC_TURNS, Sweep, brent_min, fiber};

const R: f64 = 2.5;
const L: f64 = 10.0;
const WIDE: (f64, f64) = (-100.0, 100.0);

fn tool() -> FlatTool {
    FlatTool::new(R, L, Vec3::Z).expect("valid tool")
}

fn line(from: Vec3, to: Vec3) -> LinearFixedAxis {
    LinearFixedAxis { from, to, tool: tool() }
}

/// Sorts and merges intervals, joining any that overlap or are less than `gap` apart.
fn merge(mut v: Vec<(f64, f64)>, gap: f64) -> Vec<(f64, f64)> {
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut out: Vec<(f64, f64)> = Vec::new();
    for (a, b) in v {
        match out.last_mut() {
            Some(last) if a <= last.1 + gap => last.1 = last.1.max(b),
            _ => out.push((a, b)),
        }
    }
    out
}

/// Compares two interval sets: same count, same gaps, endpoints within `tol`.
/// Both sets are merged across gaps under 2e-3 first: a sliver the sampled oracle cannot resolve is
/// not a separate interval.
fn assert_sets(got: &[(f64, f64)], want: &[(f64, f64)], tol: f64, what: &str) {
    let got = merge(got.to_vec(), 2e-3);
    let want = merge(want.to_vec(), 2e-3);
    assert_eq!(got.len(), want.len(), "{what}: got {got:?} want {want:?}");
    for (g, w) in got.iter().zip(&want) {
        assert!((g.0 - w.0).abs() < tol && (g.1 - w.1).abs() < tol, "{what}: got {got:?} want {want:?}");
    }
}

/// Oracle for a linear move: the fiber at each of `n + 1` positions along the move.
fn dense_linear(s: &LinearFixedAxis, o: Vec3, d: Vec3, tr: (f64, f64), n: usize) -> Vec<(f64, f64)> {
    let path = s.to - s.from;
    let mut v = Vec::new();
    for k in 0..=n {
        let p = s.from + path * (k as f64 / n as f64);
        v.extend(fiber(o, d, p, &s.tool).and_then(|(a, b)| clip(a, b, tr)));
    }
    v
}

fn clip(a: f64, b: f64, tr: (f64, f64)) -> Option<(f64, f64)> {
    let (lo, hi) = (a.max(tr.0), b.min(tr.1));
    (lo < hi).then_some((lo, hi))
}

fn ray_set(sweep: &dyn Sweep, o: Vec3, d: Vec3, tr: (f64, f64)) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    sweep.ray(o, d, tr, &mut out);
    merge(out, 1e-9)
}

#[test]
fn parallel_rays_match_the_dense_oracle() {
    let sweep = line(Vec3::new(0.0, 0.0, -5.0), Vec3::new(20.0, 0.0, -5.0));
    for x in [-3.0, -2.4, 0.0, 10.0, 22.0, 25.0] {
        for y in [0.0, 2.0, 2.6, 4.0] {
            for dz in [1.0, -1.0] {
                let o = Vec3::new(x, y, 0.0);
                let d = Vec3::new(0.0, 0.0, dz);
                let want = merge(dense_linear(&sweep, o, d, WIDE, 20000), 1e-9);
                assert_sets(&ray_set(&sweep, o, d, WIDE), &want, 2e-3, "parallel");
                let general = sweep.general(o, d).and_then(|(a, b)| clip(a, b, WIDE));
                let closed = ray_set(&sweep, o, d, WIDE);
                assert_eq!(closed.len(), general.iter().count(), "closed vs envelope at ({x}, {y})");
                for (c, g) in closed.iter().zip(general.iter()) {
                    assert!((c.0 - g.0).abs() < 1e-5 && (c.1 - g.1).abs() < 1e-5, "closed {c:?} envelope {g:?}");
                }
            }
        }
    }
}

#[test]
fn perpendicular_rays_match_the_dense_oracle() {
    let sweep = line(Vec3::new(0.0, 0.0, -5.0), Vec3::new(20.0, 0.0, -5.0));
    for y in [-1.0, 0.5, 2.4, 3.0, 22.0] {
        for z in [-6.0, -3.0, 4.0, 6.0] {
            for d in [Vec3::X, Vec3::Y, Vec3::new(-1.0, 0.0, 0.0)] {
                let o = Vec3::new(-50.0, y, z);
                let want = merge(dense_linear(&sweep, o, d, WIDE, 20000), 1e-9);
                assert_sets(&ray_set(&sweep, o, d, WIDE), &want, 2e-3, "perpendicular");
            }
        }
    }
}

#[test]
fn perpendicular_stadium_chord_matches_geometry() {
    let sweep = line(Vec3::new(0.0, 0.0, -5.0), Vec3::new(20.0, 0.0, -5.0));
    let half = (R * R - 0.25f64).sqrt();
    // The ray along x at y = 0.5 with origin x = -50: t = x + 50.
    assert_sets(&ray_set(&sweep, Vec3::new(-50.0, 0.5, -3.0), Vec3::X, WIDE), &[(50.0 - half, 70.0 + half)], 1e-9, "stadium");
}

#[test]
fn oblique_rays_match_the_dense_oracle() {
    let sweep = line(Vec3::new(0.0, 0.0, -5.0), Vec3::new(20.0, 3.0, -4.0));
    for d in [Vec3::new(1.0, 0.0, 0.3), Vec3::new(0.2, 1.0, 0.5), Vec3::new(-1.0, 0.4, -0.2)] {
        let d = d.normalized().expect("nonzero");
        for o in [Vec3::new(-30.0, 0.5, -2.0), Vec3::new(4.0, -20.0, -4.0), Vec3::new(9.0, 1.0, 0.0)] {
            let want = merge(dense_linear(&sweep, o, d, WIDE, 20000), 1e-9);
            assert_sets(&ray_set(&sweep, o, d, WIDE), &want, 3e-3, "oblique");
        }
    }
}

#[test]
fn rays_exactly_on_a_wall_are_not_cut() {
    // A slot along x at y = 10 with a 2.5 mm tool: the walls are at y = 7.5 and y = 12.5, exact in
    // binary, so these rays are tangent to the tool and the open-set rule must not cut them.
    let slot = line(Vec3::new(0.0, 10.0, -4.0), Vec3::new(40.0, 10.0, -4.0));
    assert!(ray_set(&slot, Vec3::new(20.0, 7.5, 0.0), Vec3::Z, WIDE).is_empty(), "Z ray on the wall");
    assert!(ray_set(&slot, Vec3::new(20.0, 12.5, 0.0), Vec3::Z, WIDE).is_empty(), "Z ray on the far wall");
    assert!(ray_set(&slot, Vec3::new(-50.0, 7.5, -2.0), Vec3::X, WIDE).is_empty(), "X ray along the wall");
    // Just inside the wall the ray is cut over the whole tool height window.
    let inside = ray_set(&slot, Vec3::new(20.0, 7.5 + 1e-6, 0.0), Vec3::Z, WIDE);
    assert_sets(&inside, &[(-4.0, 6.0)], 1e-9, "Z ray just inside");
    // A Y ray across the slot crosses the tool centreline: its chord is exact.
    assert_sets(&ray_set(&slot, Vec3::new(20.0, -50.0, -2.0), Vec3::Y, WIDE), &[(7.5 + 50.0, 12.5 + 50.0)], 1e-9, "Y chord");
}

#[test]
fn slot_walls_are_exact_on_interior_rays() {
    let slot = line(Vec3::new(0.0, 10.0, -4.0), Vec3::new(40.0, 10.0, -4.0));
    // Z ray at y = 9: the cut is the full height window.
    assert_sets(&ray_set(&slot, Vec3::new(20.0, 9.0, 0.0), Vec3::Z, WIDE), &[(-4.0, 6.0)], 1e-12, "Z interior");
    // X ray at y = 9, z = -2: the chord is the stadium's, with the end discs at x = 0 and x = 40.
    let h = (R * R - 1.0f64).sqrt();
    assert_sets(&ray_set(&slot, Vec3::new(-50.0, 9.0, -2.0), Vec3::X, WIDE), &[(50.0 - h, 90.0 + h)], 1e-12, "X chord");
}

#[test]
fn ray_outside_the_height_window_is_not_cut() {
    let slot = line(Vec3::new(0.0, 10.0, -4.0), Vec3::new(40.0, 10.0, -4.0));
    assert!(ray_set(&slot, Vec3::new(-50.0, 10.0, 17.0), Vec3::X, WIDE).is_empty(), "above the tool");
    assert!(ray_set(&slot, Vec3::new(-50.0, 10.0, -5.0), Vec3::X, WIDE).is_empty(), "below the tool tip");
}

#[test]
fn zero_length_move_is_a_static_cylinder() {
    let p = Vec3::new(3.0, 4.0, -2.0);
    let sweep = line(p, p);
    assert_sets(&ray_set(&sweep, Vec3::new(3.0, 4.0, 0.0), Vec3::Z, WIDE), &[(-2.0, 8.0)], 1e-9, "static");
}

#[test]
fn sweep_bounds_contain_the_swept_volume() {
    let sweep = line(Vec3::new(0.0, 0.0, -5.0), Vec3::new(20.0, 0.0, -5.0));
    let b = sweep.bounds();
    assert!(b.min.x <= -R + 1e-9 && b.max.x >= 20.0 + R - 1e-9);
    assert!(b.max.z >= -5.0 + L - 1e-9);
}

#[test]
fn bad_tools_are_refused() {
    assert!(FlatTool::new(0.0, 5.0, Vec3::Z).is_none());
    assert!(FlatTool::new(1.0, -5.0, Vec3::Z).is_none());
    assert!(FlatTool::new(f64::NAN, 5.0, Vec3::Z).is_none());
    assert!(FlatTool::new(1.0, 5.0, Vec3::ZERO).is_none());
}

#[test]
fn brent_finds_a_parabola_minimum() {
    let (x, fx) = brent_min(&|x: f64| (x - 0.3).powi(2) + 1.0, 0.0, 1.0, 1e-9, 25);
    assert!((x - 0.3).abs() < 1e-6);
    assert!((fx - 1.0).abs() < 1e-9);
}

fn full_circle(centre: Vec3, r: f64, z: f64) -> ArcFixedAxis {
    let start = Vec3::new(centre.x + r, centre.y, z);
    ArcFixedAxis::new(start, start, Vec3::new(centre.x, centre.y, z), Vec3::Z, true, 0, tool()).expect("arc")
}

/// Oracle for an arc: the fiber at `n + 1` arc parameters over the whole sweep.
fn dense_arc(arc: &ArcFixedAxis, o: Vec3, d: Vec3, tr: (f64, f64), n: usize) -> Vec<(f64, f64)> {
    let total = arc.sweep.abs();
    let mut v = Vec::new();
    for k in 0..=n {
        let u = total * k as f64 / n as f64;
        v.extend(fiber(o, d, arc.tip(u), &arc.tool).and_then(|(a, b)| clip(a, b, tr)));
    }
    v
}

#[test]
fn full_circle_z_ray_inside_the_annulus_is_cut_over_the_height_window() {
    let arc = full_circle(Vec3::new(30.0, 30.0, -4.0), 7.5, -4.0);
    assert_sets(&ray_set(&arc, Vec3::new(36.0, 30.0, 0.0), Vec3::Z, (-20.0, 20.0)), &[(-4.0, 6.0)], 1e-6, "annulus");
    assert!(ray_set(&arc, Vec3::new(30.0, 30.0, 0.0), Vec3::Z, (-20.0, 20.0)).is_empty(), "the centre");
    assert!(ray_set(&arc, Vec3::new(33.0, 30.0, 0.0), Vec3::Z, (-20.0, 20.0)).is_empty(), "the hole");
}

#[test]
fn full_circle_x_ray_through_the_centre_cuts_two_chords() {
    let arc = full_circle(Vec3::new(30.0, 30.0, -4.0), 7.5, -4.0);
    let set = ray_set(&arc, Vec3::new(0.0, 30.0, -3.0), Vec3::X, (-20.0, 80.0));
    assert_eq!(set.len(), 2, "two separate chords: {set:?}");
    assert!((set[0].0 - 20.0).abs() < 1e-3 && (set[0].1 - 25.0).abs() < 1e-3, "{set:?}");
    assert!((set[1].0 - 35.0).abs() < 1e-3 && (set[1].1 - 40.0).abs() < 1e-3, "{set:?}");
}

#[test]
fn arc_rays_match_the_dense_oracle_as_sets() {
    let arc = ArcFixedAxis::new(Vec3::new(37.5, 30.0, -4.0), Vec3::new(30.0, 37.5, -4.0), Vec3::new(30.0, 30.0, -4.0), Vec3::Z, true, 0, tool())
        .expect("arc");
    let diag = Vec3::new(1.0, 1.0, 0.0).normalized().expect("nonzero");
    for (o, d) in [
        (Vec3::new(0.0, 33.0, -3.0), Vec3::X),
        (Vec3::new(31.0, 0.0, -3.0), Vec3::Y),
        (Vec3::new(36.0, 33.0, 0.0), Vec3::Z),
        (Vec3::new(20.0, 20.0, -3.0), diag),
    ] {
        let want = merge(dense_arc(&arc, o, d, (-20.0, 80.0), 60000), 5e-3);
        assert_sets(&ray_set(&arc, o, d, (-20.0, 80.0)), &want, 3e-3, "arc");
    }
}

#[test]
fn g18_and_g19_arcs_starting_at_non_axis_angles_match_the_oracle() {
    // XZ plane (G18): normal along y, start at about 40 degrees from the x axis.
    let c = Vec3::new(30.0, 0.0, 30.0);
    let start = c + Vec3::new(7.5 * 0.7f64.cos(), 0.0, 7.5 * 0.7f64.sin());
    let end = c + Vec3::new(-7.5, 0.0, 0.0);
    let g18 = ArcFixedAxis::new(start, end, c, Vec3::Y, true, 0, tool()).expect("G18 arc");
    // YZ plane (G19): normal along x, start at about 130 degrees.
    let c = Vec3::new(0.0, 30.0, 30.0);
    let start = c + Vec3::new(0.0, 7.5 * 2.2f64.cos(), 7.5 * 2.2f64.sin());
    let end = c + Vec3::new(0.0, 7.5, 0.0);
    let g19 = ArcFixedAxis::new(start, end, c, Vec3::X, false, 0, tool()).expect("G19 arc");
    for arc in [g18, g19] {
        for (o, d) in [
            (Vec3::new(30.0, -20.0, 30.0), Vec3::Y),
            (Vec3::new(-20.0, 0.3, 28.0), Vec3::X),
            (Vec3::new(20.0, 0.1, 34.0), Vec3::Z),
            (Vec3::new(0.0, 2.0, 30.0), Vec3::Y),
            (Vec3::new(-20.0, 30.0, 29.0), Vec3::X),
        ] {
            let want = merge(dense_arc(&arc, o, d, (-100.0, 100.0), 60000), 5e-3);
            assert_sets(&ray_set(&arc, o, d, (-100.0, 100.0)), &want, 3e-3, "G18/G19");
        }
    }
}

#[test]
fn multi_turn_arc_matches_the_oracle() {
    // One full extra turn on top of a quarter arc: the sweep is 2.5 turns in all.
    let arc = ArcFixedAxis::new(Vec3::new(37.5, 30.0, -4.0), Vec3::new(30.0, 37.5, -4.0), Vec3::new(30.0, 30.0, -4.0), Vec3::Z, true, 1, tool())
        .expect("multi-turn arc");
    assert!(arc.sweep > std::f64::consts::TAU, "sweep {}", arc.sweep);
    for (o, d) in [(Vec3::new(0.0, 33.0, -3.0), Vec3::X), (Vec3::new(36.0, 33.0, 0.0), Vec3::Z)] {
        let want = merge(dense_arc(&arc, o, d, (-20.0, 80.0), 120000), 5e-3);
        assert_sets(&ray_set(&arc, o, d, (-20.0, 80.0)), &want, 3e-3, "multi-turn");
    }
}

#[test]
fn turns_above_the_cap_are_refused() {
    let p = Vec3::new(5.0, 0.0, 0.0);
    let q = Vec3::new(0.0, 5.0, 0.0);
    assert!(ArcFixedAxis::new(p, q, Vec3::ZERO, Vec3::Z, true, MAX_ARC_TURNS, tool()).is_some());
    assert!(ArcFixedAxis::new(p, q, Vec3::ZERO, Vec3::Z, true, MAX_ARC_TURNS + 1, tool()).is_none());
}

#[test]
fn helix_clipped_by_the_stock_top_is_clipped() {
    // Tip climbs from z = -4 to z = 8 over one turn; the stock top is z = 0.
    let arc = ArcFixedAxis::new(Vec3::new(37.5, 30.0, -4.0), Vec3::new(37.5, 30.0, 8.0), Vec3::new(30.0, 30.0, -4.0), Vec3::Z, true, 0, tool())
        .expect("helix");
    let set = ray_set(&arc, Vec3::new(37.5, 30.0, 0.0), Vec3::Z, (-10.0, 0.0));
    assert_eq!(set.len(), 1, "{set:?}");
    assert!((set[0].0 + 4.0).abs() < 1e-6 && (set[0].1 - 0.0).abs() < 1e-12, "{set:?}");
}

#[test]
fn helix_z_ray_spans_the_lift() {
    let arc = ArcFixedAxis::new(Vec3::new(37.5, 30.0, -4.0), Vec3::new(37.5, 30.0, -2.0), Vec3::new(30.0, 30.0, -4.0), Vec3::Z, true, 0, tool())
        .expect("helix");
    assert_sets(&ray_set(&arc, Vec3::new(37.5, 30.0, 0.0), Vec3::Z, (-20.0, 20.0)), &[(-4.0, 8.0)], 1e-3, "helix");
}

#[test]
fn degenerate_arcs_are_refused() {
    let p = Vec3::new(5.0, 0.0, 0.0);
    assert!(ArcFixedAxis::new(p, p, p, Vec3::Z, true, 0, tool()).is_none(), "zero radius");
    assert!(ArcFixedAxis::new(p, p, Vec3::ZERO, Vec3::ZERO, true, 0, tool()).is_none(), "zero normal");
}
