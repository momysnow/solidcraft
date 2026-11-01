//! Runs a program's moves through the tool sweeps into a stock. S1a: flat end mills on a 3-axis
//! tool; rapids cut as well as feeds. Anything else is reported as a diagnostic and skipped. A move
//! is cut in steps (one per arc revolution), so a caller can stop between them.

use std::ops::Range;

use solidcraft_geom::Vec3;

use crate::SimError;
use crate::dexel::Stock;
use crate::sweep::{ArcFixedAxis, FlatTool, LinearFixedAxis, MAX_ARC_TURNS};
use crate::toolpath::{Diag, MoveKind, NO_TOOL, Program, Shape};

const MAX_DIAGS: usize = 1000;

fn finite(p: &[f64; 3]) -> bool {
    p.iter().all(|x| x.is_finite())
}

/// One piece of a move's sweep: a line, or one revolution of an arc.
enum Step {
    Line(LinearFixedAxis),
    Arc(ArcFixedAxis),
}

/// A move being cut: its steps, how many are done, and, for a move of more than one step, the copy
/// of the stock its steps go into. The stock takes that copy only after the last step, so a move is
/// cut whole or not at all.
pub struct Cut {
    index: usize,
    source: u32,
    tool: u32,
    tag: u32,
    steps: Vec<Step>,
    done: usize,
    trial: Option<Stock>,
}

impl Cut {
    /// The move's index in the program.
    pub fn index(&self) -> usize {
        self.index
    }

    /// Steps in the move: one, or one per arc revolution.
    pub fn steps(&self) -> usize {
        self.steps.len()
    }

    /// Steps done so far. Until the last one, they are in a copy the stock has not taken.
    pub fn done(&self) -> usize {
        self.done
    }
}

/// What one [`Cutter::step`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Progress {
    /// The move has steps left.
    More,
    /// The move is cut into the stock.
    Cut,
    /// The move needs more segments on a ray than the stock keeps: it is reported and not cut.
    Skipped,
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

    /// Records `diag` once per tool and cause.
    fn report(&mut self, tool: u32, key: &'static str, diag: Diag) {
        if !self.reported.contains(&(tool, key)) && self.diags.len() < MAX_DIAGS {
            self.reported.push((tool, key));
            self.diags.push(diag);
        }
    }

    /// Starts move `index` of `program`. `None` when there is nothing to cut: the first move (it has
    /// no start point), a jump, or a move that cannot be simulated (a diagnostic says why). Only a
    /// move index too large for a tag is an error.
    pub fn start(&mut self, program: &Program, index: usize) -> Result<Option<Cut>, SimError> {
        let Some(m) = program.moves.get(index) else { return Ok(None) };
        let Some(start) = index.checked_sub(1).and_then(|p| program.moves.get(p)).map(|p| p.tip) else { return Ok(None) };
        if m.kind == MoveKind::Jump {
            return Ok(None);
        }
        let tag = u32::try_from(index + 1).map_err(|_| SimError::TooManyMoves { max: u32::MAX as usize })?;
        if m.tool == NO_TOOL {
            let diag = Diag::warning(m.source, 0, format!("no tool is selected before line {}; the move is not cut", m.source));
            self.report(NO_TOOL, "no_tool", diag);
            return Ok(None);
        }
        if !finite(&start) || !finite(&m.tip) {
            self.report(m.tool, "finite", Diag::error(m.source, 0, format!("non-finite position at line {} skipped", m.source)));
            return Ok(None);
        }
        if let MoveKind::Arc { centre, normal, .. } = &m.kind
            && (!finite(centre) || !finite(normal))
        {
            self.report(m.tool, "finite", Diag::error(m.source, 0, format!("non-finite arc at line {} skipped", m.source)));
            return Ok(None);
        }
        let Some(tool) = program.tools.iter().find(|t| t.id == m.tool) else {
            self.report(m.tool, "missing", Diag::error(m.source, 0, format!("tool {} is not defined", m.tool)));
            return Ok(None);
        };
        if tool.shape != Shape::Flat {
            self.report(m.tool, "shape", Diag::error(m.source, 0, format!("tool {} shape is not simulated yet", m.tool)));
            return Ok(None);
        }
        if m.axis != [0.0, 0.0, 1.0] {
            self.report(m.tool, "axis", Diag::error(m.source, 0, format!("tilted tool axis is not simulated yet (tool {})", m.tool)));
            return Ok(None);
        }
        let Some(flat) = FlatTool::new(tool.diameter / 2.0, tool.flute_length, Vec3::Z) else {
            self.report(m.tool, "size", Diag::error(m.source, 0, format!("tool {} has a bad diameter or flute length", m.tool)));
            return Ok(None);
        };
        let steps = match &m.kind {
            MoveKind::Rapid | MoveKind::Feed => {
                vec![Step::Line(LinearFixedAxis { from: Vec3::from(start), to: Vec3::from(m.tip), tool: flat })]
            }
            MoveKind::Arc { centre, normal, ccw, turns } => {
                if *turns > MAX_ARC_TURNS {
                    let msg = format!("arc at line {} has more than {MAX_ARC_TURNS} turns; not simulated", m.source);
                    self.report(m.tool, "turns", Diag::error(m.source, 0, msg));
                    return Ok(None);
                }
                let arc = ArcFixedAxis::new(Vec3::from(start), Vec3::from(m.tip), Vec3::from(*centre), Vec3::from(*normal), *ccw, *turns, flat);
                match arc {
                    Some(arc) => arc.revolutions().into_iter().map(Step::Arc).collect(),
                    None => {
                        self.report(m.tool, "arc", Diag::error(m.source, 0, format!("degenerate arc at line {} skipped", m.source)));
                        return Ok(None);
                    }
                }
            }
            MoveKind::Jump => return Ok(None),
        };
        Ok(Some(Cut { index, source: m.source, tool: m.tool, tag, steps, done: 0, trial: None }))
    }

    /// Cuts the next step of `cut` into `stock`. A move of one step goes straight into the stock,
    /// which is unchanged when the step fails. A longer move's steps go into a copy, which replaces
    /// the stock after the last one. A step over the segment cap skips the whole move, reported. Any
    /// other failure is an error naming the move; the stock keeps the moves before it.
    pub fn step(&mut self, cut: &mut Cut, stock: &mut Stock) -> Result<Progress, SimError> {
        let Some(step) = cut.steps.get(cut.done) else { return Ok(Progress::Cut) };
        let result = if cut.steps.len() == 1 {
            apply_step(step, cut.tag, stock)
        } else {
            let trial = cut.trial.get_or_insert_with(|| (*stock).clone());
            apply_step(step, cut.tag, trial)
        };
        match result {
            Ok(()) => {
                cut.done += 1;
                if cut.done < cut.steps.len() {
                    return Ok(Progress::More);
                }
                if let Some(trial) = cut.trial.take() {
                    *stock = trial;
                }
                Ok(Progress::Cut)
            }
            Err(SimError::SegmentCap { max }) => {
                cut.trial = None;
                let msg = format!("the move at line {} needs more than {max} segments on a ray; not cut", cut.source);
                self.report(cut.tool, "segments", Diag::error(cut.source, 0, msg));
                Ok(Progress::Skipped)
            }
            Err(e) => Err(SimError::AtMove { index: cut.index, line: cut.source, cause: e.to_string() }),
        }
    }

    /// Cuts the moves in `moves` of `program` into `stock`, move `i` tagged `i + 1`. Move `i` starts
    /// from the tip of move `i - 1`, so a range may start anywhere; a range past the last move stops
    /// at it. The first move and every `Jump` only set where the tool is.
    pub fn cut(&mut self, program: &Program, stock: &mut Stock, moves: Range<usize>) -> Result<(), SimError> {
        for i in moves {
            if i >= program.moves.len() {
                break;
            }
            let Some(mut cut) = self.start(program, i)? else { continue };
            loop {
                if self.step(&mut cut, stock)? != Progress::More {
                    break;
                }
            }
        }
        Ok(())
    }
}

fn apply_step(step: &Step, tag: u32, stock: &mut Stock) -> Result<(), SimError> {
    match step {
        Step::Line(sweep) => stock.apply(sweep, tag),
        Step::Arc(sweep) => stock.apply(sweep, tag),
    }
}

/// Cuts every move into `stock` (see [`Cutter::cut`]). Returns what could not be simulated, one line
/// per cause.
pub fn simulate(program: &Program, stock: &mut Stock) -> Result<Vec<Diag>, SimError> {
    let mut cutter = Cutter::new();
    cutter.cut(program, stock, 0..program.moves.len())?;
    Ok(cutter.into_diags())
}
