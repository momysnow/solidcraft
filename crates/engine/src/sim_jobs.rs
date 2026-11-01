//! Stock-removal simulation of the session: the setup `sim.setup` made, and the stock as far as its
//! cursor has cut it. Runtime state, like `render_jobs`: outside the design and its undo history.
//! A design that replaces this session (`Session::replace_design`) takes this state with it.

use solidcraft_geom::Vec3;
use solidcraft_machining::SimError;
use solidcraft_machining::dexel::Stock;
use solidcraft_machining::sim::Cutter;
use solidcraft_machining::toolpath::{Diag, Program};
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
    cursor: usize,
    cutter: Cutter,
    /// The box's volume, from which removed volume is counted.
    start_volume: f64,
}

#[derive(Default)]
pub(crate) struct Sim {
    job: Option<Job>,
}

impl Job {
    /// The stock of `setup`, uncut. Refused, never a panic, when the box or the memory is bad.
    pub fn new(setup: Setup) -> Result<Job, SimError> {
        let stock = Stock::new_box(setup.min, setup.max, setup.h, setup.budget_bytes)?;
        let start_volume = stock.volume();
        Ok(Job { setup, stock, cursor: 0, cutter: Cutter::new(), start_volume })
    }

    /// Moves to `to` (a move count): cuts forward, or rebuilds the stock and cuts from the start
    /// when `to` is behind. Stops early, with `false`, when `until` has passed. A move that cannot
    /// be cut is an error, and the cursor stays before it.
    pub fn seek(&mut self, to: usize, until: Option<Instant>) -> Result<bool, SimError> {
        if to < self.cursor {
            self.stock = Stock::new_box(self.setup.min, self.setup.max, self.setup.h, self.setup.budget_bytes)?;
            self.cutter = Cutter::new();
            self.cursor = 0;
        }
        while self.cursor < to {
            if until.is_some_and(|t| Instant::now() >= t) {
                return Ok(false);
            }
            self.cutter.cut(&self.setup.program, &mut self.stock, self.cursor..self.cursor + 1)?;
            self.cursor += 1;
        }
        Ok(true)
    }

    pub fn cursor(&self) -> usize {
        self.cursor
    }

    pub fn total(&self) -> usize {
        self.setup.program.moves.len()
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
}
