//! G-code: a streaming parser for the v1 subset, against a code table. Never panics: anything it
//! cannot do becomes a [`Diag`] for its line, and the rest of the program still parses.

use crate::toolpath::{Diag, Interp, Move, MoveKind, Program};

pub const MAX_INPUT_BYTES: usize = 64 << 20;
pub const MAX_LINE_BYTES: usize = 4096;
pub const MAX_MOVES: usize = 5_000_000;
pub const MAX_WORDS: usize = 64;
const MAX_DIAGS: usize = 1000;
const MAX_NUMBER_CHARS: usize = 32;
/// Coordinates, feeds and speeds above this (mm) are rejected, so squares stay finite.
const MAX_VALUE: f64 = 1e9;
const MAX_POSITION: f64 = 1e10;

/// The code table: which G and M numbers mean what. G codes are stored times ten (G90.1 is 901),
/// so every code is an integer. The defaults are the generic 3-axis table.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CodeTable {
    pub rapid: u32,
    pub linear: u32,
    pub arc_cw: u32,
    pub arc_ccw: u32,
    pub plane_xy: u32,
    pub plane_zx: u32,
    pub plane_yz: u32,
    pub inch: u32,
    pub mm: u32,
    pub absolute: u32,
    pub incremental: u32,
    pub arc_centre_absolute: u32,
    pub arc_centre_incremental: u32,
    pub work_offsets: [u32; 6],
    pub machine_coords: u32,
    pub dwell: u32,
    pub cutter_comp_off: u32,
    pub cutter_comp_left: u32,
    pub cutter_comp_right: u32,
    pub tool_length_on: u32,
    pub tool_length_off: u32,
    pub cancel_cycle: u32,
    pub home: [u32; 2],
    pub set_offsets: u32,
    pub canned: [u32; 4],
    pub cycle_mode: [u32; 2],
    /// A `/` at the start of a line skips the line when true; when false the slash is ignored.
    pub block_delete: bool,
}

impl Default for CodeTable {
    fn default() -> Self {
        CodeTable {
            rapid: 0,
            linear: 10,
            arc_cw: 20,
            arc_ccw: 30,
            plane_xy: 170,
            plane_zx: 180,
            plane_yz: 190,
            inch: 200,
            mm: 210,
            absolute: 900,
            incremental: 910,
            arc_centre_absolute: 901,
            arc_centre_incremental: 911,
            work_offsets: [540, 550, 560, 570, 580, 590],
            machine_coords: 530,
            dwell: 40,
            cutter_comp_off: 400,
            cutter_comp_left: 410,
            cutter_comp_right: 420,
            tool_length_on: 430,
            tool_length_off: 490,
            cancel_cycle: 800,
            home: [280, 300],
            set_offsets: 100,
            canned: [730, 810, 820, 830],
            cycle_mode: [980, 990],
            block_delete: false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Motion {
    Rapid,
    Linear,
    ArcCw,
    ArcCcw,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Plane {
    Xy,
    Zx,
    Yz,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GKind {
    Motion(Motion),
    Plane(Plane),
    Inch,
    Mm,
    Absolute,
    Incremental,
    ArcAbsolute,
    ArcIncremental,
    Offset,
    Machine,
    Dwell,
    CutterOff,
    CutterComp,
    ToolLengthOn,
    ToolLengthOff,
    Home,
    Canned,
    CycleMode,
    CancelCycle,
    SetOffsets,
    Unknown,
}

impl CodeTable {
    fn classify(&self, code: u32) -> GKind {
        if code == self.rapid {
            GKind::Motion(Motion::Rapid)
        } else if code == self.linear {
            GKind::Motion(Motion::Linear)
        } else if code == self.arc_cw {
            GKind::Motion(Motion::ArcCw)
        } else if code == self.arc_ccw {
            GKind::Motion(Motion::ArcCcw)
        } else if code == self.plane_xy {
            GKind::Plane(Plane::Xy)
        } else if code == self.plane_zx {
            GKind::Plane(Plane::Zx)
        } else if code == self.plane_yz {
            GKind::Plane(Plane::Yz)
        } else if code == self.inch {
            GKind::Inch
        } else if code == self.mm {
            GKind::Mm
        } else if code == self.absolute {
            GKind::Absolute
        } else if code == self.incremental {
            GKind::Incremental
        } else if code == self.arc_centre_absolute {
            GKind::ArcAbsolute
        } else if code == self.arc_centre_incremental {
            GKind::ArcIncremental
        } else if self.work_offsets.contains(&code) {
            GKind::Offset
        } else if code == self.machine_coords {
            GKind::Machine
        } else if code == self.dwell {
            GKind::Dwell
        } else if code == self.cutter_comp_off {
            GKind::CutterOff
        } else if code == self.cutter_comp_left || code == self.cutter_comp_right {
            GKind::CutterComp
        } else if code == self.tool_length_on {
            GKind::ToolLengthOn
        } else if code == self.tool_length_off {
            GKind::ToolLengthOff
        } else if self.home.contains(&code) {
            GKind::Home
        } else if self.canned.contains(&code) {
            GKind::Canned
        } else if self.cycle_mode.contains(&code) {
            GKind::CycleMode
        } else if code == self.cancel_cycle {
            GKind::CancelCycle
        } else if code == self.set_offsets {
            GKind::SetOffsets
        } else {
            GKind::Unknown
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Word {
    letter: u8,
    value: f64,
    col: u32,
}

#[derive(Clone, Copy, Debug)]
struct State {
    pos: [f64; 3],
    motion: Motion,
    plane: Plane,
    /// 1 for millimetres, 25.4 for inches.
    scale: f64,
    absolute: bool,
    arc_absolute: bool,
    feed: f64,
    rpm: f64,
    spindle_on: bool,
    tool: u32,
    pending_tool: u32,
}

/// Streams a program line by line. Call [`Parser::push_line`] for each line, then [`Parser::finish`].
pub struct Parser<'a> {
    codes: &'a CodeTable,
    state: State,
    moves: Vec<Move>,
    diags: Vec<Diag>,
    line: u32,
    diag_full: bool,
    moves_full: bool,
    warned_tool_length: bool,
    warned_machine: bool,
}

/// Parses a whole program. Input over 64 MB gives one error and no moves.
pub fn parse(text: &str, codes: &CodeTable) -> Program {
    if text.len() > MAX_INPUT_BYTES {
        return Program { tools: Vec::new(), moves: Vec::new(), diags: vec![Diag::error(0, 0, "input is over 64 MB")] };
    }
    let mut parser = Parser::new(codes);
    for line in text.split('\n') {
        parser.push_line(line);
    }
    parser.finish()
}

impl<'a> Parser<'a> {
    pub fn new(codes: &'a CodeTable) -> Self {
        Parser {
            codes,
            state: State {
                pos: [0.0; 3],
                motion: Motion::Rapid,
                plane: Plane::Xy,
                scale: 1.0,
                absolute: true,
                arc_absolute: false,
                feed: 0.0,
                rpm: 0.0,
                spindle_on: false,
                tool: 0,
                pending_tool: 0,
            },
            moves: Vec::new(),
            diags: Vec::new(),
            line: 0,
            diag_full: false,
            moves_full: false,
            warned_tool_length: false,
            warned_machine: false,
        }
    }

    /// The program so far. Tools are left empty: the caller supplies the tool table.
    pub fn finish(self) -> Program {
        Program { tools: Vec::new(), moves: self.moves, diags: self.diags }
    }

    pub fn push_line(&mut self, line: &str) {
        self.line = self.line.saturating_add(1);
        if line.len() > MAX_LINE_BYTES {
            self.err(1, "line is over 4 KB");
            return;
        }
        if let Some(words) = self.tokenize(line) {
            self.block(&words);
        }
    }

    fn err(&mut self, col: u32, msg: impl AsRef<str>) {
        self.push_diag(Diag::error(self.line, col, msg));
    }

    fn warn(&mut self, col: u32, msg: impl AsRef<str>) {
        self.push_diag(Diag::warning(self.line, col, msg));
    }

    fn push_diag(&mut self, d: Diag) {
        if self.diags.len() < MAX_DIAGS {
            self.diags.push(d);
        } else if !self.diag_full {
            self.diag_full = true;
            self.diags.push(Diag::error(self.line, 0, "too many problems; the rest are not listed"));
        }
    }

    /// Splits a line into words. `None` when the line is skipped (block delete) or has an error.
    fn tokenize(&mut self, line: &str) -> Option<Vec<Word>> {
        let b = line.as_bytes();
        let mut words: Vec<Word> = Vec::new();
        let mut i = 0usize;
        let mut started = false;
        while let Some(&c) = b.get(i) {
            let col = u32::try_from(i + 1).unwrap_or(u32::MAX);
            match c {
                b' ' | b'\t' | b'\r' | b'%' => i += 1,
                b';' => break,
                b'(' => match b.get(i..).and_then(|rest| rest.iter().position(|&x| x == b')')) {
                    Some(p) => i += p + 1,
                    None => {
                        self.err(col, "unterminated comment");
                        return None;
                    }
                },
                b'/' if !started => {
                    if self.codes.block_delete {
                        return None;
                    }
                    i += 1;
                }
                b'#' | b'[' | b']' | b'=' | b'<' | b'>' => {
                    self.err(col, "variables and expressions are not supported");
                    return None;
                }
                c if c.is_ascii_alphabetic() => {
                    if b.get(i + 1).is_some_and(|x| x.is_ascii_alphabetic()) {
                        let mut j = i;
                        while b.get(j).is_some_and(|x| x.is_ascii_alphabetic()) {
                            j += 1;
                        }
                        let word = b.get(i..j).map(|s| String::from_utf8_lossy(s).into_owned()).unwrap_or_default();
                        self.err(col, format!("{word} is not supported"));
                        return None;
                    }
                    let letter = c.to_ascii_uppercase();
                    let (value, end) = match number(b, i + 1) {
                        Ok(v) => v,
                        Err(msg) => {
                            self.err(col, format!("word {} {msg}", letter as char));
                            return None;
                        }
                    };
                    if words.len() >= MAX_WORDS {
                        self.err(col, "more than 64 words on a line");
                        return None;
                    }
                    words.push(Word { letter, value, col });
                    i = end;
                    started = true;
                }
                _ => {
                    self.err(col, "unexpected character");
                    return None;
                }
            }
        }
        Some(words)
    }

    fn code_of(&mut self, w: &Word) -> Option<u32> {
        let t = (w.value * 10.0).round();
        if (w.value * 10.0 - t).abs() > 1e-6 || t < 0.0 || t > 1e6 {
            self.err(w.col, format!("G{} is not a code", w.value));
            return None;
        }
        Some(t as u32)
    }

    fn int_of(&mut self, w: &Word) -> Option<u32> {
        let r = w.value.round();
        if (w.value - r).abs() > 1e-9 || !(0.0..=4.0e9).contains(&r) {
            self.err(w.col, format!("{} {} is not a whole number", w.letter as char, w.value));
            return None;
        }
        Some(r as u32)
    }

    fn block(&mut self, words: &[Word]) {
        let mut gs: Vec<(u32, u32)> = Vec::new();
        let mut ms: Vec<(u32, u32)> = Vec::new();
        let mut tool: Option<u32> = None;
        let mut feed: Option<(f64, u32)> = None;
        let mut spindle: Option<f64> = None;
        let mut axes: [Option<f64>; 3] = [None; 3];
        let mut ijk: [Option<f64>; 3] = [None; 3];
        let mut radius: Option<f64> = None;
        for w in words {
            match w.letter {
                b'G' => match self.code_of(w) {
                    Some(c) => gs.push((c, w.col)),
                    None => return,
                },
                b'M' => match self.int_of(w) {
                    Some(c) => ms.push((c, w.col)),
                    None => return,
                },
                b'T' => match self.int_of(w) {
                    Some(c) => tool = Some(c),
                    None => return,
                },
                b'X' => axes[0] = Some(w.value),
                b'Y' => axes[1] = Some(w.value),
                b'Z' => axes[2] = Some(w.value),
                b'I' => ijk[0] = Some(w.value),
                b'J' => ijk[1] = Some(w.value),
                b'K' => ijk[2] = Some(w.value),
                b'R' => radius = Some(w.value),
                b'F' => {
                    if w.value < 0.0 {
                        self.err(w.col, "feed rate is negative");
                        return;
                    }
                    feed = Some((w.value, w.col));
                }
                b'S' => {
                    if w.value < 0.0 {
                        self.err(w.col, "spindle speed is negative");
                        return;
                    }
                    spindle = Some(w.value);
                }
                // Line and program numbers, dwell and length words: read where they are used.
                b'N' | b'O' | b'P' | b'H' => {}
                b'A' | b'B' | b'C' => {
                    self.err(w.col, "rotary axes are not on the generic 3-axis machine");
                    return;
                }
                other => {
                    self.err(w.col, format!("word {} is not supported", other as char));
                    return;
                }
            }
        }

        // Mode codes first, so the rest of the block reads in the modes it sets.
        let mut block_motion: Option<Motion> = None;
        let mut skip_motion = false;
        let mut machine = false;
        for (c, col) in gs {
            match self.codes.classify(c) {
                GKind::Motion(m) => {
                    if block_motion.replace(m).is_some() {
                        self.err(col, "two motion codes in one block");
                        return;
                    }
                    self.state.motion = m;
                }
                GKind::Plane(p) => self.state.plane = p,
                GKind::Inch => self.state.scale = 25.4,
                GKind::Mm => self.state.scale = 1.0,
                GKind::Absolute => self.state.absolute = true,
                GKind::Incremental => self.state.absolute = false,
                GKind::ArcAbsolute => self.state.arc_absolute = true,
                GKind::ArcIncremental => self.state.arc_absolute = false,
                GKind::Offset | GKind::Dwell | GKind::CutterOff | GKind::ToolLengthOff | GKind::CancelCycle => {}
                GKind::Machine => {
                    machine = true;
                    if !self.warned_machine {
                        self.warned_machine = true;
                        self.warn(col, "G53 is taken as workpiece coordinates: no machine offsets in S1a");
                    }
                }
                GKind::CutterComp => self.warn(col, "cutter compensation is not applied: the path is shown uncompensated"),
                GKind::ToolLengthOn => {
                    if !self.warned_tool_length {
                        self.warned_tool_length = true;
                        self.warn(col, "tool length offsets are not applied yet");
                    }
                }
                GKind::Home => {
                    self.warn(col, "G28 and G30 are skipped: no home positions yet");
                    skip_motion = true;
                }
                GKind::Canned => {
                    self.err(col, "canned cycles are not supported yet");
                    return;
                }
                GKind::CycleMode => {
                    self.err(col, "canned cycle modes are not supported yet");
                    return;
                }
                GKind::SetOffsets => {
                    self.err(col, "G10 is not supported yet");
                    return;
                }
                GKind::Unknown => {
                    self.err(col, format!("G{} is not supported", c as f64 / 10.0));
                    return;
                }
            }
        }

        if let Some(t) = tool {
            self.state.pending_tool = t;
        }
        for (m, col) in ms {
            match m {
                0 | 1 | 2 | 30 | 7 | 8 | 9 => {}
                3 | 4 => self.state.spindle_on = true,
                5 => self.state.spindle_on = false,
                6 => self.state.tool = self.state.pending_tool,
                98 | 99 => {
                    self.err(col, "subprograms (M98, M99) are not supported");
                    return;
                }
                _ => self.warn(col, format!("M{m} is ignored")),
            }
        }
        if let Some((f, col)) = feed {
            if f > MAX_VALUE {
                self.err(col, "feed rate is out of range");
                return;
            }
            self.state.feed = f * self.state.scale;
        }
        if let Some(s) = spindle {
            if s > MAX_VALUE {
                self.err(0, "spindle speed is out of range");
                return;
            }
            self.state.rpm = s;
        }
        if skip_motion {
            return;
        }
        match self.state.motion {
            Motion::Rapid | Motion::Linear => {
                if axes.iter().all(|a| a.is_none()) {
                    return;
                }
                let Some(tip) = self.target(axes, machine) else { return };
                let kind = if self.state.motion == Motion::Rapid { MoveKind::Rapid } else { MoveKind::Feed };
                self.push_kind(kind, tip);
            }
            m @ (Motion::ArcCw | Motion::ArcCcw) => self.arc(m, axes, ijk, radius, machine),
        }
    }

    /// The end point of a linear or arc move; a missing axis keeps the current position.
    fn target(&mut self, axes: [Option<f64>; 3], machine: bool) -> Option<[f64; 3]> {
        let mut tip = self.state.pos;
        for (k, a) in axes.iter().enumerate() {
            let Some(v) = *a else { continue };
            let v = v * self.state.scale;
            let absolute = self.state.absolute || machine;
            let c = if absolute { v } else { self.state.pos.get(k).copied().unwrap_or(0.0) + v };
            if !c.is_finite() || c.abs() > MAX_POSITION {
                self.err(0, "coordinate is out of range");
                return None;
            }
            if let Some(slot) = tip.get_mut(k) {
                *slot = c;
            }
        }
        Some(tip)
    }

    fn arc(&mut self, m: Motion, axes: [Option<f64>; 3], ijk: [Option<f64>; 3], radius: Option<f64>, machine: bool) {
        let (u, v, n) = match self.state.plane {
            Plane::Xy => (0usize, 1usize, 2usize),
            Plane::Zx => (2, 0, 1),
            Plane::Yz => (1, 2, 0),
        };
        let ccw = m == Motion::ArcCcw;
        let start = self.state.pos;
        let Some(end) = self.target(axes, machine) else { return };
        let at = |p: [f64; 3], a: usize| p.get(a).copied().unwrap_or(0.0);
        let centre_uv: (f64, f64);
        if let Some(r) = radius {
            if ijk.iter().any(|x| x.is_some()) {
                self.err(0, "arc has both R and I, J, K");
                return;
            }
            let r = r * self.state.scale;
            if r.abs() < 1e-9 {
                self.err(0, "arc radius is zero");
                return;
            }
            let (su, sv) = (at(start, u), at(start, v));
            let (eu, ev) = (at(end, u), at(end, v));
            let (wu, wv) = (eu - su, ev - sv);
            let chord = (wu * wu + wv * wv).sqrt();
            if chord < 1e-9 {
                self.err(0, "R form cannot draw a full circle");
                return;
            }
            if chord > 2.0 * r.abs() + 1e-9 {
                self.err(0, "arc radius is too small for its end points");
                return;
            }
            let h = (r * r - chord * chord / 4.0).max(0.0).sqrt();
            let side = if ccw == (r > 0.0) { 1.0 } else { -1.0 };
            let (pu, pv) = (-wv / chord, wu / chord);
            let (mu, mv) = ((su + eu) / 2.0, (sv + ev) / 2.0);
            centre_uv = (mu + side * h * pu, mv + side * h * pv);
        } else {
            if ijk.iter().all(|x| x.is_none()) {
                self.err(0, "arc has no centre (I, J or K)");
                return;
            }
            let offset = |a: usize, base: f64| match ijk.get(a).copied().flatten() {
                Some(o) if self.state.arc_absolute => o * self.state.scale,
                Some(o) => base + o * self.state.scale,
                None => base,
            };
            let cu = offset(u, at(start, u));
            let cv = offset(v, at(start, v));
            centre_uv = (cu, cv);
            let rs = ((at(start, u) - cu).powi(2) + (at(start, v) - cv).powi(2)).sqrt();
            let re = ((at(end, u) - cu).powi(2) + (at(end, v) - cv).powi(2)).sqrt();
            if rs < 1e-9 {
                self.err(0, "arc radius is zero");
                return;
            }
            if (rs - re).abs() > 0.002f64.max(0.001 * rs) {
                self.err(0, format!("arc end is {:.4} mm off its circle", (rs - re).abs()));
                return;
            }
        }
        let mut centre = start;
        if let Some(c) = centre.get_mut(u) {
            *c = centre_uv.0;
        }
        if let Some(c) = centre.get_mut(v) {
            *c = centre_uv.1;
        }
        let mut normal = [0.0; 3];
        if let Some(x) = normal.get_mut(n) {
            *x = 1.0;
        }
        self.push_kind(MoveKind::Arc { centre, normal, ccw, turns: 0 }, end);
    }

    fn push_kind(&mut self, kind: MoveKind, tip: [f64; 3]) {
        if self.moves.len() >= MAX_MOVES {
            if !self.moves_full {
                self.moves_full = true;
                self.err(0, "more than 5,000,000 moves; the rest are dropped");
            }
            return;
        }
        let feeding = !matches!(kind, MoveKind::Rapid);
        self.moves.push(Move {
            kind,
            tip,
            axis: [0.0, 0.0, 1.0],
            joints: None,
            interp: Interp::Tcp,
            tool: self.state.tool,
            feed: if feeding { self.state.feed } else { 0.0 },
            spindle: if self.state.spindle_on { self.state.rpm } else { 0.0 },
            source: self.line,
        });
        self.state.pos = tip;
    }
}

/// A number after a word letter: optional sign, digits, optional fraction. Returns the value and
/// the index after it. Rejects anything that is not finite or is out of range.
fn number(b: &[u8], start: usize) -> Result<(f64, usize), &'static str> {
    let mut j = start;
    if matches!(b.get(j), Some(b'+') | Some(b'-')) {
        j += 1;
    }
    let mut digits = 0usize;
    while b.get(j).is_some_and(|x| x.is_ascii_digit()) {
        j += 1;
        digits += 1;
    }
    if b.get(j) == Some(&b'.') {
        j += 1;
        while b.get(j).is_some_and(|x| x.is_ascii_digit()) {
            j += 1;
            digits += 1;
        }
    }
    if digits == 0 {
        return Err("has no value");
    }
    if j - start > MAX_NUMBER_CHARS {
        return Err("has a number that is too long");
    }
    let text = b.get(start..j).and_then(|s| std::str::from_utf8(s).ok()).ok_or("has no value")?;
    let value: f64 = text.parse().map_err(|_| "has a bad number")?;
    if !value.is_finite() || value.abs() > MAX_VALUE {
        return Err("is out of range");
    }
    Ok((value, j))
}

#[cfg(test)]
#[path = "gcode_tests.rs"]
mod tests;
