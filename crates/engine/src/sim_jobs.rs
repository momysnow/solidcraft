//! Stock-removal simulation of the session: the setup `sim.setup` made, and the stock as far as its
//! cursor has cut it. Runtime state, like `render_jobs`: outside the design and its undo history.
//! A design that replaces this session (`Session::replace_design`) takes this state with it.
//!
//! shortcut: a backward seek rebuilds the stock from the start and replays it, with no checkpoints
//! yet, so seeking back in a long program is slow. Upgrade to copy-on-write checkpoints (design S2).

use solidcraft_geom::Vec3;
use solidcraft_machining::SimError;
use solidcraft_machining::dexel::Stock;
use solidcraft_machining::sim::{Cut, Cutter, Progress};
use solidcraft_machining::toolpath::{Diag, Program};
use std::sync::atomic::{AtomicU64, Ordering};

use web_time::Instant;

use crate::Session;

/// What `sim.setup` was given. Kept so a backward seek can rebuild the stock from the start.
pub(crate) struct Setup {
    pub min: Vec3,
    pub max: Vec3,
    pub h: f64,
    pub budget_bytes: u64,
    pub program: Program,
}

/// One simulation: the stock, how many moves of the program it has had cut, and what was reported.
pub(crate) struct Job {
    setup: Setup,
    stock: Stock,
    /// Moves completed; the stock has all of them and none after.
    cursor: usize,
    cutter: Cutter,
    /// The box's volume, from which removed volume is counted.
    start_volume: f64,
    /// A move part-cut (one arc revolution at a time). Its steps are in a copy, not in `stock`, until
    /// its last step.
    partial: Option<Cut>,
    /// The failure that stopped the run. A forward run refuses until a backward seek rebuilds.
    error: Option<SimError>,
    /// Changes whenever the stock is built again (see `rebuild`).
    revision: u64,
}

#[derive(Default)]
pub(crate) struct Sim {
    job: Option<Job>,
}

/// Revisions are handed out by the process, not by a session, so one never repeats: not after
/// `file.new`, not in another session.
static REVISIONS: AtomicU64 = AtomicU64::new(0);

/// A revision no stock has had before.
pub(crate) fn next_revision() -> u64 {
    REVISIONS.fetch_add(1, Ordering::Relaxed) + 1
}

impl Job {
    /// The stock of `setup`, uncut. Nothing is dirty: the UI draws the uncut box itself. Refused, never
    /// a panic, when the box or the memory is bad.
    pub fn new(setup: Setup, revision: u64) -> Result<Job, SimError> {
        let stock = Stock::new_box(setup.min, setup.max, setup.h, setup.budget_bytes)?;
        let start_volume = stock.volume();
        Ok(Job { setup, stock, cursor: 0, cutter: Cutter::new(), start_volume, partial: None, error: None, revision })
    }

    /// Moves to `to` (a move count): cuts forward, or rebuilds the stock and cuts from the start when
    /// `to` is behind. Stops early, with `false`, when `until` has passed, but always after one step,
    /// so a call makes progress. A step that fails stops the run: the error is kept, and a forward
    /// run then returns it until a backward seek. Moves are cut whole, so the cursor is at a move
    /// boundary whatever happens.
    pub fn seek(&mut self, to: usize, until: Option<Instant>) -> Result<bool, SimError> {
        if to < self.cursor {
            self.rebuild()?;
        } else if to == self.cursor {
            // A move in progress is not part of `to`, so its copy goes.
            self.partial = None;
        }
        if let Some(e) = self.error.as_ref().filter(|_| self.cursor < to) {
            return Err(e.clone());
        }
        let mut first = true;
        while self.cursor < to {
            if !first && until.is_some_and(|t| Instant::now() >= t) {
                return Ok(false);
            }
            first = false;
            if let Err(e) = self.step() {
                self.partial = None;
                self.error = Some(e.clone());
                return Err(e);
            }
        }
        Ok(true)
    }

    /// Cuts one step: the next piece of the move in progress, or the first step of the next move.
    fn step(&mut self) -> Result<(), SimError> {
        let mut cut = match self.partial.take() {
            Some(cut) => cut,
            None => match self.cutter.start(&self.setup.program, self.cursor)? {
                Some(cut) => cut,
                None => {
                    self.cursor += 1;
                    return Ok(());
                }
            },
        };
        match self.cutter.step(&mut cut, &mut self.stock)? {
            Progress::More => self.partial = Some(cut),
            Progress::Cut | Progress::Skipped => self.cursor += 1,
        }
        Ok(())
    }

    /// Builds the stock again from the setup, uncut, and cuts nothing: the start of a backward seek.
    /// The chunks the old stock had touched are dirty, since they are the box again; nothing else is,
    /// as the new stock is the box where no cut reaches. The revision moves on.
    fn rebuild(&mut self) -> Result<(), SimError> {
        let previous = self.stock.touched_chunks();
        let mut stock = Stock::new_box(self.setup.min, self.setup.max, self.setup.h, self.setup.budget_bytes)?;
        stock.mark_dirty(previous);
        self.stock = stock;
        self.cursor = 0;
        self.cutter = Cutter::new();
        self.partial = None;
        self.error = None;
        self.revision = next_revision();
        Ok(())
    }

    /// Hands out the chunks whose surface changed since the last call, and forgets them.
    pub fn take_dirty(&mut self) -> Vec<[usize; 3]> {
        let dirty = self.stock.dirty_chunks();
        self.stock.clear_dirty();
        dirty
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub fn cursor(&self) -> usize {
        self.cursor
    }

    pub fn total(&self) -> usize {
        self.setup.program.moves.len()
    }

    /// The step count of the move in progress: `(done, steps)`, when a run stopped part-way through.
    pub fn partial(&self) -> Option<(usize, usize)> {
        self.partial.as_ref().map(|c| (c.done(), c.steps()))
    }

    /// Why the run stopped, when a step failed.
    pub fn error(&self) -> Option<&SimError> {
        self.error.as_ref()
    }

    /// The box's volume before any cut, mm^3.
    pub fn start_volume(&self) -> f64 {
        self.start_volume
    }

    /// Material left, mm^3.
    pub fn remaining_volume(&self) -> f64 {
        self.stock.volume()
    }

    /// Material taken away so far, mm^3.
    pub fn removed_volume(&self) -> f64 {
        self.start_volume - self.stock.volume()
    }

    /// Bytes the stock takes, against its budget.
    pub fn memory(&self) -> (u64, u64) {
        self.stock.memory()
    }

    pub fn bounds(&self) -> (Vec3, Vec3) {
        (self.setup.min, self.setup.max)
    }

    pub fn h(&self) -> f64 {
        self.setup.h
    }

    /// The G-code's own problems, then what cutting could not simulate.
    pub fn diagnostics(&self) -> impl Iterator<Item = &Diag> + '_ {
        self.setup.program.diags.iter().chain(self.cutter.diags())
    }
}

impl Sim {
    /// Makes `job` the simulation, dropping the one there was.
    pub fn set(&mut self, job: Job) {
        self.job = Some(job);
    }

    pub fn job(&self) -> Option<&Job> {
        self.job.as_ref()
    }

    pub fn job_mut(&mut self) -> Option<&mut Job> {
        self.job.as_mut()
    }

    /// Drops the simulation; `true` when there was one.
    pub fn clear(&mut self) -> bool {
        self.job.take().is_some()
    }
}

/// Read access for the UI (drawing the stock and the toolpath): the simulation's stock as far as
/// its cursor has cut it, and the program it runs. `None` before `sim.setup`.
impl Session {
    pub fn sim_stock(&self) -> Option<&Stock> {
        self.sim.job().map(|j| &j.stock)
    }

    pub fn sim_program(&self) -> Option<&Program> {
        self.sim.job().map(|j| &j.setup.program)
    }

    /// The stock's revision: it changes when the stock is built again (setup, a backward seek, clear).
    /// 0 without a simulation.
    pub fn sim_revision(&self) -> u64 {
        self.sim.job().map_or(0, Job::revision)
    }

    /// The chunks whose surface changed since the last call, each once, and forgets them: what the UI
    /// remeshes. Empty without a simulation.
    pub fn sim_take_dirty(&mut self) -> Vec<[usize; 3]> {
        self.sim.job_mut().map_or_else(Vec::new, Job::take_dirty)
    }
}
