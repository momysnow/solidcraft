//! Runs a program's moves through the tool sweeps into a stock. S1a: flat end mills on a 3-axis
//! tool; rapids cut as well as feeds. Anything else is reported as a diagnostic and skipped.

use std::ops::Range;

use solidcraft_geom::Vec3;

use crate::SimError;
use crate::dexel::Stock;
use crate::sweep::{ArcFixedAxis, FlatTool, LinearFixedAxis, MAX_ARC_TURNS, Sweep};
use crate::toolpath::{Diag, MoveKind, Program, Shape};

const MAX_DIAGS: usize = 1000;

fn finite(p: &[f64; 3]) -> bool {
    p.iter().all(|x| x.is_finite())
}

/// Cuts a program a few moves at a time, keeping what it reported so far: a run can be split (a
/// seek replays from the start, a playback goes move by move) and each cause is still reported once.
#[derive(Debug, Default)]
pub struct Cutter {
    reported: Vec<(u32, &'static str)>,
    diags: Vec<Diag>,
}

impl Cutter {
    pub fn new() -> Cutter {
        Cutter::default()
    }

    /// What could not be simulated so far, one line per cause.
    pub fn diags(&self) -> &[Diag] {
        &self.diags
    }

    pub fn into_diags(self) -> Vec<Diag> {
        self.diags
    }

    fn report(&mut self, tool: u32, source: u32, key: &'static str, msg: String) {
        if !self.reported.contains(&(tool, key)) && self.diags.len() < MAX_DIAGS {
            self.reported.push((tool, key));
            self.diags.push(Diag::error(source, 0, msg));
        }
    }

    /// Cuts the moves in `moves` of `program` into `stock`, move `i` tagged `i + 1`. Move `i` starts
    /// from the tip of move `i - 1`, so a range may start anywhere; a range past the last move stops
    /// at it. The first move and every `Jump` only set where the tool is. An arc is cut one
    /// revolution at a time under the same tag. A failing move returns its error, with the earlier
    /// moves of the range cut.
    pub fn cut(&mut self, program: &Program, stock: &mut Stock, moves: Range<usize>) -> Result<(), SimError> {
        for i in moves {
            let Some(m) = program.moves.get(i) else { break };
            let tag = u32::try_from(i + 1).map_err(|_| SimError::TooManyMoves { max: u32::MAX as usize })?;
            let Some(start) = i.checked_sub(1).and_then(|p| program.moves.get(p)).map(|p| p.tip) else { continue };
            if m.kind == MoveKind::Jump {
                continue;
            }
            if !finite(&start) || !finite(&m.tip) {
                self.report(m.tool, m.source, "finite", format!("non-finite position at line {} skipped", m.source));
                continue;
            }
            if let MoveKind::Arc { centre, normal, .. } = &m.kind
                && (!finite(centre) || !finite(normal))
            {
                self.report(m.tool, m.source, "finite", format!("non-finite arc at line {} skipped", m.source));
                continue;
            }
            let Some(tool) = program.tools.iter().find(|t| t.id == m.tool) else {
                self.report(m.tool, m.source, "missing", format!("tool {} is not defined", m.tool));
                continue;
            };
            if tool.shape != Shape::Flat {
                self.report(m.tool, m.source, "shape", format!("tool {} shape is not simulated yet", m.tool));
                continue;
            }
            if m.axis != [0.0, 0.0, 1.0] {
                self.report(m.tool, m.source, "axis", format!("tilted tool axis is not simulated yet (tool {})", m.tool));
                continue;
            }
            let Some(flat) = FlatTool::new(tool.diameter / 2.0, tool.flute_length, Vec3::Z) else {
                self.report(m.tool, m.source, "size", format!("tool {} has a bad diameter or flute length", m.tool));
                continue;
            };
            let cut = match &m.kind {
                MoveKind::Rapid | MoveKind::Feed => {
                    let sweep = LinearFixedAxis { from: Vec3::from(start), to: Vec3::from(m.tip), tool: flat };
                    stock.apply(&sweep, tag)
                }
                MoveKind::Arc { centre, normal, ccw, turns } => {
                    if *turns > MAX_ARC_TURNS {
                        self.report(
                            m.tool,
                            m.source,
                            "turns",
                            format!("arc at line {} has more than {MAX_ARC_TURNS} turns; not simulated", m.source),
                        );
                        continue;
                    }
                    match ArcFixedAxis::new(Vec3::from(start), Vec3::from(m.tip), Vec3::from(*centre), Vec3::from(*normal), *ccw, *turns, flat) {
                        Some(arc) => {
                            // All revolutions or none: a failing one must not leave the earlier ones cut.
                            let pieces = arc.revolutions();
                            let refs: Vec<&dyn Sweep> = pieces.iter().map(|p| p as &dyn Sweep).collect();
                            stock.apply_all(&refs, tag)
                        }
                        None => {
                            self.report(m.tool, m.source, "arc", format!("degenerate arc at line {} skipped", m.source));
                            continue;
                        }
                    }
                }
                MoveKind::Jump => continue,
            };
            cut.map_err(|e| SimError::AtMove { index: i, line: m.source, cause: e.to_string() })?;
        }
        Ok(())
    }
}

/// Cuts every move into `stock` (see [`Cutter::cut`]). Returns what could not be simulated, one line
/// per cause.
pub fn simulate(program: &Program, stock: &mut Stock) -> Result<Vec<Diag>, SimError> {
    let mut cutter = Cutter::new();
    cutter.cut(program, stock, 0..program.moves.len())?;
    Ok(cutter.into_diags())
}
