//! The toolpath of a G-code simulation as lines in the viewport: each move of the program is a rapid
//! link, a retract, a cutting move or a jump, drawn or left out by the View tab, and only the part of the
//! program the display mode asks for (all of it, behind the tool, ahead of it, the current operation or
//! the trail). Lines are built a chunk of moves at a time and kept while their key comes back, so a
//! playhead step rebuilds at most the chunk it is in.

use std::sync::Arc;

use solidcraft_machining::toolpath::{MoveKind, Program};

use crate::gpu::{BlockBuilder, BlockBytes, PaletteIndex};
use crate::sim_view::PathMode;

/// Moves in a cached chunk of lines.
pub const CHUNK: usize = 2048;
/// Moves of the trail behind the tool.
pub const TRAIL: usize = 200;
/// Line segments per full circle when an arc is drawn.
const ARC_SEGMENTS: usize = 32;
/// A rapid whose direction is this close to the tool axis (the sine of the angle) is a retract.
const AXIS_SINE: f64 = 1.0e-6;

/// What a move is, for the View tab.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// A rapid that is not a retract.
    Link,
    /// A rapid that only lifts the tool along its axis.
    Retract,
    /// A move that cuts: a line or an arc at a feed.
    Cut,
    /// A jump with no known path (G28, G30).
    Jump,
}

/// Which kinds are drawn, and whether the move ends are marked.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Flags {
    pub links: bool,
    pub retracts: bool,
    pub cuts: bool,
    pub jumps: bool,
    pub points: bool,
}

impl Flags {
    fn shows(self, k: Kind) -> bool {
        match k {
            Kind::Link => self.links,
            Kind::Retract => self.retracts,
            Kind::Cut => self.cuts,
            Kind::Jump => self.jumps,
        }
    }

    /// The flags as bits, for a cache key.
    pub fn bits(self) -> u8 {
        u8::from(self.links) | u8::from(self.retracts) << 1 | u8::from(self.cuts) << 2 | u8::from(self.jumps) << 3 | u8::from(self.points) << 4
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn len(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// What move `i` is. The first move has no start, so it is no segment (`None`).
pub fn kind_of(program: &Program, i: usize) -> Option<Kind> {
    let m = program.moves.get(i)?;
    let prev = program.moves.get(i.checked_sub(1)?)?;
    Some(match m.kind {
        MoveKind::Feed | MoveKind::Arc { .. } => Kind::Cut,
        MoveKind::Jump => Kind::Jump,
        MoveKind::Rapid => {
            let d = sub(m.tip, prev.tip);
            let l = len(d);
            let axis_len = len(m.axis);
            if l > 0.0 && axis_len > 0.0 && len(cross(d, m.axis)) / (l * axis_len) <= AXIS_SINE && dot(d, m.axis) > 0.0 {
                Kind::Retract
            } else {
                Kind::Link
            }
        }
    })
}

/// The points of move `i`'s way from the previous tip to its own, in the workpiece frame: two for a
/// straight move, more along an arc. Empty for the first move.
pub fn polyline(program: &Program, i: usize) -> Vec<[f64; 3]> {
    let (Some(m), Some(prev)) = (program.moves.get(i), i.checked_sub(1).and_then(|p| program.moves.get(p))) else { return Vec::new() };
    let (a, b) = (prev.tip, m.tip);
    let MoveKind::Arc { centre, normal, ccw, turns } = &m.kind else { return vec![a, b] };
    let n_len = len(*normal);
    if !(n_len > 0.0) {
        return vec![a, b];
    }
    let n = [normal[0] / n_len, normal[1] / n_len, normal[2] / n_len];
    // Radial vectors in the arc's plane, the axial part kept apart (a helix climbs along the normal).
    let radial = |p: [f64; 3]| {
        let d = sub(p, *centre);
        let h = dot(d, n);
        ([d[0] - h * n[0], d[1] - h * n[1], d[2] - h * n[2]], h)
    };
    let ((ra, ha), (rb, hb)) = (radial(a), radial(b));
    let r = len(ra);
    if !(r > 0.0) {
        return vec![a, b];
    }
    let (ua, va) = ([ra[0] / r, ra[1] / r, ra[2] / r], cross(n, [ra[0] / r, ra[1] / r, ra[2] / r]));
    // The angle from a to b about n, counter-clockwise, then taken the way the arc goes.
    let angle_b = dot(rb, va).atan2(dot(rb, ua));
    let mut sweep = angle_b;
    if *ccw {
        if sweep <= 0.0 {
            sweep += std::f64::consts::TAU;
        }
    } else if sweep >= 0.0 {
        sweep -= std::f64::consts::TAU;
    }
    // Start and end on one point: a full circle.
    if len(sub(a, b)) < 1.0e-9 * r.max(1.0) && (hb - ha).abs() < 1.0e-9 {
        sweep = if *ccw { std::f64::consts::TAU } else { -std::f64::consts::TAU };
    }
    let total = sweep + (f64::from(*turns) * std::f64::consts::TAU).copysign(sweep);
    let steps = ((total.abs() / std::f64::consts::TAU * ARC_SEGMENTS as f64).ceil() as usize).clamp(2, 4096);
    let mut out = Vec::with_capacity(steps + 1);
    out.push(a);
    for s in 1..steps {
        let t = s as f64 / steps as f64;
        let (c, sn) = ((total * t).cos(), (total * t).sin());
        let h = ha + (hb - ha) * t;
        out.push(std::array::from_fn(|k| centre[k] + r * (c * ua[k] + sn * va[k]) + h * n[k]));
    }
    out.push(b);
    out
}

/// The moves `[from, to)` a display mode draws: `cursor` moves are done, `total` are in the program, and
/// `operation` is the moves of the operation under the playhead when there is one.
pub fn move_range(mode: PathMode, cursor: usize, total: usize, operation: Option<(usize, usize)>) -> (usize, usize) {
    let cursor = cursor.min(total);
    match mode {
        PathMode::All => (0, total),
        PathMode::Behind => (0, cursor),
        PathMode::Ahead => (cursor, total),
        PathMode::Operation => operation.map_or((0, total), |(a, b)| (a.min(total), b.min(total))),
        PathMode::Trail => (cursor.saturating_sub(TRAIL), cursor),
    }
}

/// The lines of moves `[from, to)` as one block: cutting moves in `cut`, rapids and retracts in `rapid`,
/// jumps in `jump`, and a dot at each move end when `flags.points`. `place` puts a workpiece point in the
/// model.
pub fn build(
    program: &Program,
    (from, to): (usize, usize),
    flags: Flags,
    colours: [PaletteIndex; 3],
    place: impl Fn([f32; 3]) -> [f32; 3],
) -> Arc<BlockBytes> {
    let [cut, rapid, jump] = colours;
    let mut block = BlockBuilder::default();
    let f = |p: [f64; 3]| place([p[0] as f32, p[1] as f32, p[2] as f32]);
    for i in from..to.min(program.moves.len()) {
        let Some(kind) = kind_of(program, i) else { continue };
        if flags.shows(kind) {
            let colour = match kind {
                Kind::Cut => cut,
                Kind::Link | Kind::Retract => rapid,
                Kind::Jump => jump,
            };
            let points = polyline(program, i);
            for w in points.windows(2) {
                if let [a, b] = w {
                    block.line(f(*a), f(*b), colour, 1.6);
                }
            }
        }
        if flags.points
            && let Some(m) = program.moves.get(i)
        {
            let p = f(m.tip);
            // A dot: a segment too short to see as a line, drawn wide.
            block.line(p, [p[0] + 0.01, p[1], p[2]], cut, 4.0);
        }
    }
    block.freeze()
}

#[cfg(test)]
mod tests {
    use super::*;
    use solidcraft_machining::toolpath::{Interp, Move};

    fn mv(kind: MoveKind, tip: [f64; 3]) -> Move {
        Move { kind, tip, axis: [0.0, 0.0, 1.0], joints: None, interp: Interp::default(), tool: 0, feed: 100.0, spindle: 0.0, source: 1 }
    }

    fn program(moves: Vec<Move>) -> Program {
        Program { tools: Vec::new(), moves, diags: Vec::new() }
    }

    #[test]
    fn a_rapid_up_the_axis_is_a_retract_and_a_rapid_sideways_is_a_link() {
        let p = program(vec![
            mv(MoveKind::Rapid, [0.0, 0.0, 0.0]),
            mv(MoveKind::Rapid, [0.0, 0.0, 5.0]),
            mv(MoveKind::Rapid, [10.0, 0.0, 5.0]),
            mv(MoveKind::Rapid, [10.0, 0.0, 0.0]),
            mv(MoveKind::Feed, [20.0, 0.0, 0.0]),
            mv(MoveKind::Jump, [0.0, 0.0, 50.0]),
        ]);
        let kinds: Vec<_> = (0..6).map(|i| kind_of(&p, i)).collect();
        assert_eq!(
            kinds,
            [None, Some(Kind::Retract), Some(Kind::Link), Some(Kind::Link), Some(Kind::Cut), Some(Kind::Jump)],
            "the first move has no start, a plunge down the axis is a link, not a retract"
        );
    }

    #[test]
    fn an_arc_ends_where_the_move_does_and_stays_on_its_circle() {
        let arc = MoveKind::Arc { centre: [0.0, 0.0, 0.0], normal: [0.0, 0.0, 1.0], ccw: true, turns: 0 };
        let p = program(vec![mv(MoveKind::Rapid, [10.0, 0.0, 0.0]), mv(arc, [0.0, 10.0, 0.0])]);
        let pts = polyline(&p, 1);
        assert!(pts.len() > 3, "a quarter circle is more than a chord: {}", pts.len());
        assert_eq!(pts.first(), Some(&[10.0, 0.0, 0.0]));
        assert_eq!(pts.last(), Some(&[0.0, 10.0, 0.0]));
        for q in &pts {
            assert!(((q[0] * q[0] + q[1] * q[1]).sqrt() - 10.0).abs() < 1.0e-9, "{q:?} is off the circle");
        }
        let mid = pts[pts.len() / 2];
        assert!(mid[0] > 0.0 && mid[1] > 0.0, "counter-clockwise from +X to +Y goes through the first quadrant: {mid:?}");
    }

    #[test]
    fn a_clockwise_arc_takes_the_long_way_round_and_a_full_circle_closes() {
        let cw = MoveKind::Arc { centre: [0.0, 0.0, 0.0], normal: [0.0, 0.0, 1.0], ccw: false, turns: 0 };
        let p = program(vec![mv(MoveKind::Rapid, [10.0, 0.0, 0.0]), mv(cw.clone(), [0.0, 10.0, 0.0]), mv(cw, [0.0, 10.0, 0.0])]);
        let mid = polyline(&p, 1)[polyline(&p, 1).len() / 2];
        assert!(mid[0] < 0.0 || mid[1] < 0.0, "clockwise from +X to +Y goes the long way: {mid:?}");
        let full = program(vec![
            mv(MoveKind::Rapid, [10.0, 0.0, 0.0]),
            mv(MoveKind::Arc { centre: [0.0, 0.0, 0.0], normal: [0.0, 0.0, 1.0], ccw: true, turns: 0 }, [10.0, 0.0, 0.0]),
        ]);
        let pts = polyline(&full, 1);
        assert!(pts.len() >= ARC_SEGMENTS, "a full circle is drawn round: {}", pts.len());
    }

    #[test]
    fn the_display_modes_pick_their_moves() {
        let (cursor, total) = (500, 1000);
        assert_eq!(move_range(PathMode::All, cursor, total, None), (0, 1000));
        assert_eq!(move_range(PathMode::Behind, cursor, total, None), (0, 500));
        assert_eq!(move_range(PathMode::Ahead, cursor, total, None), (500, 1000));
        assert_eq!(move_range(PathMode::Trail, cursor, total, None), (300, 500));
        assert_eq!(move_range(PathMode::Trail, 50, total, None), (0, 50));
        assert_eq!(move_range(PathMode::Operation, cursor, total, Some((400, 700))), (400, 700));
        assert_eq!(move_range(PathMode::Operation, cursor, total, None), (0, 1000), "no operation known: the whole program");
        assert_eq!(move_range(PathMode::Behind, 5000, total, None), (0, 1000), "a cursor past the end is clamped");
    }

    #[test]
    fn the_flags_are_distinct_bits() {
        let all = Flags { links: true, retracts: true, cuts: true, jumps: true, points: true };
        assert_eq!(all.bits(), 0b11111);
        assert_eq!(Flags { links: false, ..all }.bits(), 0b11110);
        assert!(!Flags { cuts: false, ..all }.shows(Kind::Cut));
    }
}
