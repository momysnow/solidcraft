//! Runs a program's moves through the tool sweeps into a stock. S1a: flat end mills on a 3-axis
//! tool; rapids cut as well as feeds. Anything else is reported as a diagnostic and skipped.

use solidcraft_geom::Vec3;

use crate::SimError;
use crate::dexel::Stock;
use crate::sweep::{ArcFixedAxis, FlatTool, LinearFixedAxis, MAX_ARC_TURNS};
use crate::toolpath::{Diag, MoveKind, Program, Shape};

const MAX_DIAGS: usize = 1000;

fn finite(p: &[f64; 3]) -> bool {
    p.iter().all(|x| x.is_finite())
}

/// Cuts every move into `stock`, move `i` tagged `i + 1`. The first move and every `Jump` only set
/// where the tool is. An arc is cut one revolution at a time under the same tag. Returns what could
/// not be simulated, one line per cause.
pub fn simulate(program: &Program, stock: &mut Stock) -> Result<Vec<Diag>, SimError> {
    let mut diags: Vec<Diag> = Vec::new();
    let mut reported: Vec<(u32, &'static str)> = Vec::new();
    let mut push = |diags: &mut Vec<Diag>, tool: u32, source: u32, key: &'static str, msg: String| {
        if !reported.contains(&(tool, key)) && diags.len() < MAX_DIAGS {
            reported.push((tool, key));
            diags.push(Diag::error(source, 0, msg));
        }
    };
    let mut prev: Option<[f64; 3]> = None;
    for (i, m) in program.moves.iter().enumerate() {
        let start = prev.replace(m.tip);
        let tag = u32::try_from(i + 1).map_err(|_| SimError::TooManyMoves { max: u32::MAX as usize })?;
        let Some(start) = start else { continue };
        if m.kind == MoveKind::Jump {
            continue;
        }
        if !finite(&start) || !finite(&m.tip) {
            push(&mut diags, m.tool, m.source, "finite", format!("non-finite position at line {} skipped", m.source));
            continue;
        }
        if let MoveKind::Arc { centre, normal, .. } = &m.kind
            && (!finite(centre) || !finite(normal))
        {
            push(&mut diags, m.tool, m.source, "finite", format!("non-finite arc at line {} skipped", m.source));
            continue;
        }
        let Some(tool) = program.tools.iter().find(|t| t.id == m.tool) else {
            push(&mut diags, m.tool, m.source, "missing", format!("tool {} is not defined", m.tool));
            continue;
        };
        if tool.shape != Shape::Flat {
            push(&mut diags, m.tool, m.source, "shape", format!("tool {} shape is not simulated yet", m.tool));
            continue;
        }
        if m.axis != [0.0, 0.0, 1.0] {
            push(&mut diags, m.tool, m.source, "axis", format!("tilted tool axis is not simulated yet (tool {})", m.tool));
            continue;
        }
        let Some(flat) = FlatTool::new(tool.diameter / 2.0, tool.flute_length, Vec3::Z) else {
            push(&mut diags, m.tool, m.source, "size", format!("tool {} has a bad diameter or flute length", m.tool));
            continue;
        };
        let cut = match &m.kind {
            MoveKind::Rapid | MoveKind::Feed => {
                let sweep = LinearFixedAxis { from: Vec3::from(start), to: Vec3::from(m.tip), tool: flat };
                stock.apply(&sweep, tag)
            }
            MoveKind::Arc { centre, normal, ccw, turns } => {
                if *turns > MAX_ARC_TURNS {
                    push(
                        &mut diags,
                        m.tool,
                        m.source,
                        "turns",
                        format!("arc at line {} has more than {MAX_ARC_TURNS} turns; not simulated", m.source),
                    );
                    continue;
                }
                match ArcFixedAxis::new(Vec3::from(start), Vec3::from(m.tip), Vec3::from(*centre), Vec3::from(*normal), *ccw, *turns, flat) {
                    Some(arc) => {
                        let mut result = Ok(());
                        for piece in arc.revolutions() {
                            result = stock.apply(&piece, tag);
                            if result.is_err() {
                                break;
                            }
                        }
                        result
                    }
                    None => {
                        push(&mut diags, m.tool, m.source, "arc", format!("degenerate arc at line {} skipped", m.source));
                        continue;
                    }
                }
            }
            MoveKind::Jump => continue,
        };
        cut.map_err(|e| SimError::AtMove { index: i, line: m.source, cause: e.to_string() })?;
    }
    Ok(diags)
}
