use solidcraft_geom::Vec3;

use super::{ArcFixedAxis, FlatTool, LinearFixedAxis, Sweep, brent_min, fiber};

const R: f64 = 2.5;
const L: f64 = 10.0;

fn tool() -> FlatTool {
    FlatTool::new(R, L, Vec3::Z).expect("valid tool")
}

fn line(from: Vec3, to: Vec3) -> LinearFixedAxis {
    LinearFixedAxis { from, to, tool: tool() }
}

/// Oracle: the hull of the removed interval over `n` samples of the move, by brute force.
fn dense_hull(sweep: &LinearFixedAxis, o: Vec3, d: Vec3, tr: (f64, f64), n: usize) -> Option<(f64, f64)> {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for k in 0..=n {
        let s = k as f64 / n as f64;
        let p = sweep.from + (sweep.to - sweep.from) * s;
        if let Some((a, b)) = fiber(o, d, p, &sweep.tool, tr) {
            lo = lo.min(a);
            hi = hi.max(b);
        }
    }
    (lo <= hi).then_some((lo, hi))
}

fn hull_of(out: &[(f64, f64)]) -> Option<(f64, f64)> {
    let lo = out.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let hi = out.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    (lo <= hi).then_some((lo, hi))
}

fn assert_hull(got: Option<(f64, f64)>, want: Option<(f64, f64)>, tol: f64, what: &str) {
    match (got, want) {
        (None, None) => {}
        (Some((a, b)), Some((c, d))) => {
            assert!((a - c).abs() < tol && (b - d).abs() < tol, "{what}: got ({a}, {b}) want ({c}, {d})");
        }
        other => panic!("{what}: got {other:?}"),
    }
}

#[test]
fn parallel_rays_match_the_dense_oracle() {
    let sweep = line(Vec3::new(0.0, 0.0, -5.0), Vec3::new(20.0, 0.0, -5.0));
    let tr = (-50.0, 50.0);
    for x in [-3.0, -2.4, 0.0, 10.0, 22.0, 25.0] {
        for y in [0.0, 2.0, 2.6, 4.0] {
            for dz in [1.0, -1.0] {
                let o = Vec3::new(x, y, 0.0);
                let d = Vec3::new(0.0, 0.0, dz);
                let mut closed = Vec::new();
                sweep.ray(o, d, tr, &mut closed);
                assert_hull(hull_of(&closed), dense_hull(&sweep, o, d, tr, 20000), 2e-3, "parallel");
                // The envelope is the general method; it must agree with the closed form.
                let general = sweep.general(o, d, tr);
                assert_hull(hull_of(&closed), general, 1e-5, "closed vs envelope");
            }
        }
    }
}

#[test]
fn parallel_ray_interval_is_the_cutting_height_window() {
    // A ray in the middle of the path: the tool cuts z in [-5, 5] there.
    let sweep = line(Vec3::new(0.0, 0.0, -5.0), Vec3::new(20.0, 0.0, -5.0));
    let mut out = Vec::new();
    sweep.ray(Vec3::new(10.0, 0.0, 0.0), Vec3::Z, (-50.0, 50.0), &mut out);
    assert_hull(hull_of(&out), Some((-5.0, 5.0)), 1e-9, "window");
    out.clear();
    sweep.ray(Vec3::new(10.0, 3.0, 0.0), Vec3::Z, (-50.0, 50.0), &mut out);
    assert!(out.is_empty(), "a ray outside the radius is not cut");
}

#[test]
fn perpendicular_rays_match_the_dense_oracle() {
    let sweep = line(Vec3::new(0.0, 0.0, -5.0), Vec3::new(20.0, 0.0, -5.0));
    let tr = (-100.0, 100.0);
    for y in [-1.0, 0.5, 2.4, 3.0, 22.0] {
        for z in [-6.0, -3.0, 4.0, 6.0] {
            for d in [Vec3::X, Vec3::Y, Vec3::new(-1.0, 0.0, 0.0)] {
                let o = Vec3::new(-50.0, y, z);
                let mut closed = Vec::new();
                sweep.ray(o, d, tr, &mut closed);
                // The dense oracle works in t on the ray; the parameter is the same for both.
                assert_hull(hull_of(&closed), dense_hull(&sweep, o, d, tr, 20000), 2e-3, "perpendicular");
            }
        }
    }
}

#[test]
fn perpendicular_stadium_chord_matches_geometry() {
    // The path along x at y = 0 and the ray along x at y = 0.5, inside the tool height window.
    let sweep = line(Vec3::new(0.0, 0.0, -5.0), Vec3::new(20.0, 0.0, -5.0));
    let mut out = Vec::new();
    sweep.ray(Vec3::new(-50.0, 0.5, -3.0), Vec3::X, (-100.0, 100.0), &mut out);
    let half = (R * R - 0.25f64).sqrt();
    assert_hull(hull_of(&out), Some((50.0 - half, 70.0 + half)), 1e-6, "stadium");
}

#[test]
fn oblique_rays_match_the_dense_oracle() {
    let sweep = line(Vec3::new(0.0, 0.0, -5.0), Vec3::new(20.0, 3.0, -4.0));
    let tr = (-100.0, 100.0);
    let dirs = [Vec3::new(1.0, 0.0, 0.3), Vec3::new(0.2, 1.0, 0.5), Vec3::new(-1.0, 0.4, -0.2)];
    for d in dirs {
        let d = d.normalized().expect("nonzero");
        for o in [Vec3::new(-30.0, 0.5, -2.0), Vec3::new(4.0, -20.0, -4.0), Vec3::new(9.0, 1.0, 0.0)] {
            let mut out = Vec::new();
            sweep.ray(o, d, tr, &mut out);
            assert_hull(hull_of(&out), dense_hull(&sweep, o, d, tr, 20000), 3e-3, "oblique");
        }
    }
}

#[test]
fn zero_length_move_is_a_static_cylinder() {
    let p = Vec3::new(3.0, 4.0, -2.0);
    let sweep = line(p, p);
    let mut out = Vec::new();
    sweep.ray(Vec3::new(3.0, 4.0, 0.0), Vec3::Z, (-50.0, 50.0), &mut out);
    assert_hull(hull_of(&out), Some((-2.0, 8.0)), 1e-9, "static");
}

#[test]
fn sweep_bounds_contain_the_swept_volume() {
    let sweep = line(Vec3::new(0.0, 0.0, -5.0), Vec3::new(20.0, 0.0, -5.0));
    let b = sweep.bounds();
    for x in [-2.4, 0.0, 20.0, 22.4] {
        assert!(b.contains(Vec3::new(x, 0.0, 0.0)) || b.min.x <= x && x <= b.max.x);
    }
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

#[test]
fn full_circle_z_ray_inside_the_annulus_is_cut_over_the_height_window() {
    let arc = full_circle(Vec3::new(30.0, 30.0, -4.0), 7.5, -4.0);
    let mut out = Vec::new();
    arc.ray(Vec3::new(36.0, 30.0, 0.0), Vec3::Z, (-20.0, 20.0), &mut out);
    assert_hull(hull_of(&out), Some((-4.0, 6.0)), 1e-6, "annulus");
    out.clear();
    // The centre of the circle is 7.5 from the path, so it is not reached by the tool.
    arc.ray(Vec3::new(30.0, 30.0, 0.0), Vec3::Z, (-20.0, 20.0), &mut out);
    assert!(out.is_empty());
    out.clear();
    // Inside the hole of the annulus, 3 mm from the centre, the path is 4.5 mm away: not reached either.
    arc.ray(Vec3::new(33.0, 30.0, 0.0), Vec3::Z, (-20.0, 20.0), &mut out);
    assert!(out.is_empty());
}

#[test]
fn full_circle_x_ray_through_the_centre_cuts_two_chords() {
    let arc = full_circle(Vec3::new(30.0, 30.0, -4.0), 7.5, -4.0);
    let mut out = Vec::new();
    // Ray along x at y = 30, at a height inside the tool's window.
    arc.ray(Vec3::new(0.0, 30.0, -3.0), Vec3::X, (-20.0, 80.0), &mut out);
    let covered = |x: f64| out.iter().any(|&(a, b)| a <= x && x <= b);
    assert!(covered(22.0), "left chord: {out:?}");
    assert!(covered(38.0), "right chord: {out:?}");
    assert!(!covered(30.0), "the centre is not cut: {out:?}");
    assert!(!covered(16.0) && !covered(44.0));
    let lo = out.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let hi = out.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    assert!((lo - 20.0).abs() < 1e-3 && (hi - 40.0).abs() < 1e-3, "{lo} {hi}");
}

#[test]
fn arc_rays_match_the_dense_oracle() {
    let arc = ArcFixedAxis::new(Vec3::new(37.5, 30.0, -4.0), Vec3::new(30.0, 37.5, -4.0), Vec3::new(30.0, 30.0, -4.0), Vec3::Z, true, 0, tool())
        .expect("arc");
    let total = arc.sweep.abs();
    for (o, d) in [
        (Vec3::new(0.0, 33.0, -3.0), Vec3::X),
        (Vec3::new(31.0, 0.0, -3.0), Vec3::Y),
        (Vec3::new(36.0, 33.0, 0.0), Vec3::Z),
        (Vec3::new(20.0, 20.0, -3.0), Vec3::new(1.0, 1.0, 0.0).normalized().expect("nonzero")),
    ] {
        let mut out = Vec::new();
        arc.ray(o, d, (-20.0, 80.0), &mut out);
        let n = 40000usize;
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for k in 0..=n {
            let u = total * k as f64 / n as f64;
            if let Some((a, b)) = fiber(o, d, arc.tip(u), &arc.tool, (-20.0, 80.0)) {
                lo = lo.min(a);
                hi = hi.max(b);
            }
        }
        // The oracle's hull contains every interval; the sweep's intervals must cover the same hull.
        let got = hull_of(&out);
        match got {
            Some((a, b)) => {
                assert!((a - lo).abs() < 2e-3 && (b - hi).abs() < 2e-3, "{o:?} {d:?}: got ({a}, {b}) oracle ({lo}, {hi})");
            }
            None => assert!(lo > hi, "{o:?} {d:?}: oracle found ({lo}, {hi})"),
        }
    }
}

#[test]
fn helix_z_ray_spans_the_lift() {
    let arc = ArcFixedAxis::new(Vec3::new(37.5, 30.0, -4.0), Vec3::new(37.5, 30.0, -2.0), Vec3::new(30.0, 30.0, -4.0), Vec3::Z, true, 0, tool())
        .expect("helix");
    let mut out = Vec::new();
    arc.ray(Vec3::new(37.5, 30.0, 0.0), Vec3::Z, (-20.0, 20.0), &mut out);
    // The tip climbs from -4 to -2 over one turn: the tool covers [-4, 8] at the top of the climb.
    assert_hull(hull_of(&out), Some((-4.0, 8.0)), 1e-3, "helix");
}

#[test]
fn degenerate_arcs_are_refused() {
    let p = Vec3::new(5.0, 0.0, 0.0);
    assert!(ArcFixedAxis::new(p, p, p, Vec3::Z, true, 0, tool()).is_none(), "zero radius");
    assert!(ArcFixedAxis::new(p, p, Vec3::ZERO, Vec3::ZERO, true, 0, tool()).is_none(), "zero normal");
}
