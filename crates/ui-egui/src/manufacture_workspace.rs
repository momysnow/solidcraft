//! Manufacture workspace: process tabs (Milling / Turning / Additive / Inspection / Fabrication)
//! replace the Design ribbon. New Setup opens the SETUP window. Setups live in the left browser.

use crate::SolveApp;

/// Process families shown as the Manufacture workspace's top tabs.
pub const TABS: &[&str] = &["MILLING", "TURNING", "ADDITIVE", "INSPECTION", "FABRICATION"];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum CamMode {
    #[default]
    Milling,
    Turning,
    Additive,
    Inspection,
    Fabrication,
}

impl CamMode {
    pub fn tab(self) -> &'static str {
        match self {
            Self::Milling => "MILLING",
            Self::Turning => "TURNING",
            Self::Additive => "ADDITIVE",
            Self::Inspection => "INSPECTION",
            Self::Fabrication => "FABRICATION",
        }
    }

    pub fn from_tab(tab: &str) -> Option<Self> {
        Some(match tab {
            "MILLING" => Self::Milling,
            "TURNING" => Self::Turning,
            "ADDITIVE" => Self::Additive,
            "INSPECTION" => Self::Inspection,
            "FABRICATION" => Self::Fabrication,
            _ => return None,
        })
    }
}

#[derive(Default)]
pub struct State {
    pub active: bool,
    pub active_setup: String,
    pub cam_mode: CamMode,
    pub setup_dialog: Option<crate::manufacture_setup::SetupDialogState>,
    /// The open operation window (Contour, Pocket, Adaptive).
    pub op_dialog: Option<crate::manufacture_operation::OpDialogState>,
    /// The operation picked in the Browser (its name in the active setup): its toolpath is drawn
    /// in the highlight colour and the others are dimmed.
    pub selected_op: Option<String>,
    /// The active setup's toolpaths, computed off the window thread.
    pub toolpaths: crate::manufacture_toolpaths::Toolpaths,
}

/// The ribbon tab of a running simulation: it replaces the process tabs while the simulation shows.
pub const SIMULATION_TAB: &str = "SIMULATION";

pub fn is_tab(tab: &str) -> bool {
    CamMode::from_tab(tab).is_some() || tab == "MANUFACTURE" || tab == SIMULATION_TAB
}

/// Selects the Simulation tab (Simulate does, when it starts a simulation).
pub fn show_simulation_tab(app: &mut SolveApp) {
    app.ui.tab = SIMULATION_TAB.into();
}

pub fn active(app: &SolveApp) -> bool {
    app.manufacture_workspace.active
}

pub fn show_tab(app: &mut SolveApp, tab: &str) {
    if let Some(mode) = CamMode::from_tab(tab) {
        app.manufacture_workspace.cam_mode = mode;
        app.ui.tab = tab.into();
    }
}

pub fn sync_tab(app: &mut SolveApp) {
    if app.ui.tab == SIMULATION_TAB && crate::sim_stock::simulating(app) {
        return;
    }
    if let Some(mode) = CamMode::from_tab(&app.ui.tab) {
        app.manufacture_workspace.cam_mode = mode;
        return;
    }
    app.ui.tab = app.manufacture_workspace.cam_mode.tab().into();
}

pub fn enter(app: &mut SolveApp) {
    if crate::render_workspace::active(app) {
        let _ = crate::render_workspace::enter(app, false);
    }
    app.manufacture_workspace.active = true;
    if CamMode::from_tab(&app.ui.tab).is_none() {
        app.ui.tab = app.manufacture_workspace.cam_mode.tab().into();
    }
}

pub fn exit(app: &mut SolveApp) {
    if !active(app) {
        return;
    }
    close_floating_dialogs(app);
    app.manufacture_workspace.active = false;
    app.manufacture_workspace.selected_op = None;
    if is_tab(&app.ui.tab) {
        app.ui.tab = "MODEL".into();
    }
}

pub fn reset_document(app: &mut SolveApp) {
    app.manufacture_workspace = State::default();
}

pub fn close_floating_dialogs(app: &mut SolveApp) {
    app.manufacture_workspace.setup_dialog = None;
    app.manufacture_workspace.op_dialog = None;
    app.ui.pick_bodies = false;
}

fn tab_for_command(id: &str) -> Option<&'static str> {
    Some(match id {
        s if s.starts_with("manufacture.turning.") || s.starts_with("manufacture.part_handling.") => "TURNING",
        s if s.starts_with("manufacture.additive.") => "ADDITIVE",
        s if s.starts_with("manufacture.inspection.") => "INSPECTION",
        s if s.starts_with("manufacture.fabrication.") => "FABRICATION",
        s if s.starts_with("manufacture.2d.")
            || s.starts_with("manufacture.3d.")
            || s.starts_with("manufacture.multiaxis.")
            || s == "manufacture.drill"
            || s == "manufacture.hole_recognition" =>
        {
            "MILLING"
        }
        _ => return None,
    })
}

/// Return false to cancel starting `id` after handling it here.
pub fn start_hook(app: &mut SolveApp, id: &str) -> bool {
    match id {
        "manufacture.setup.new" => {
            crate::manufacture_setup::open_new(app);
            false
        }
        "cam.setup" => {
            crate::manufacture_setup::open_new(app);
            false
        }
        "manufacture.post.simulate" => {
            crate::sim_stock::simulate_active(app);
            false
        }
        "manufacture.sim.options" => {
            app.sim_stock.view.open = !app.sim_stock.view.open;
            false
        }
        "manufacture.sim.exit" => {
            crate::sim_stock::exit_simulation(app);
            false
        }
        id if let Some(kind) = crate::manufacture_operation::OpKind::from_command(id) => {
            crate::manufacture_operation::open(app, kind);
            false
        }
        id if id.starts_with("manufacture.") => {
            enter(app);
            if let Some(tab) = tab_for_command(id) {
                show_tab(app, tab);
            }
            // The rest of the ribbon has no window yet: say so instead of doing nothing.
            let label = app.tr(&format!("cmd.{id}")).to_string();
            app.set_status(solidcraft_ui_text::format("manufacture.op.not_available", &[("name", &label)]), true);
            false
        }
        _ => true,
    }
}

pub fn tick(app: &mut SolveApp, ctx: &egui::Context) {
    crate::manufacture_toolpaths::poll(app, ctx);
    if app.manufacture_workspace.setup_dialog.is_none() {
        return;
    }
    if !ctx.input(|i| i.pointer.primary_clicked()) {
        return;
    }
    let Some(pos) = ctx.pointer_interact_pos() else {
        return;
    };
    let Some(rect) = app.viewport.rect else {
        return;
    };
    if !rect.contains(pos) {
        return;
    }
    let proj = crate::viewport::projection(app, rect);
    let _ = crate::manufacture_setup::try_pick_origin(app, &proj, pos);
}

pub fn panel(app: &mut SolveApp, ui: &mut egui::Ui) {
    crate::manufacture_setup::show_dialogs(app, ui);
    crate::manufacture_operation::show_dialog(app, ui);
    crate::sim_view::window(app, ui);
}

pub fn show(app: &mut SolveApp, ui: &mut egui::Ui) {
    crate::viewport::show(app, ui);
}

/// Hide bodies that are not in the setup being edited, or in the setup selected in the browser.
pub fn setup_shows_body(app: &SolveApp, name: &str) -> bool {
    if let Some(state) = app.manufacture_workspace.setup_dialog.as_ref() {
        if state.draft.model_bodies.is_empty() {
            return true;
        }
        return state.draft.model_bodies.iter().any(|body| body == name);
    }
    if !active(app) {
        return true;
    }
    let Some(setup) = app.session.doc.cam.setup(&app.manufacture_workspace.active_setup) else {
        return true;
    };
    if setup.model_bodies.is_empty() {
        return true;
    }
    setup.model_bodies.iter().any(|body| body == name)
}

/// Identifies which bodies the Manufacture setup shows, so a scene cache notices a change: leaving
/// the workspace, another setup, or other body names with the same count all change it.
pub fn setup_body_key(app: &SolveApp) -> u64 {
    use std::hash::{Hash, Hasher};
    let state = &app.manufacture_workspace;
    let mut h = std::collections::hash_map::DefaultHasher::new();
    active(app).hash(&mut h);
    state.setup_dialog.is_some().hash(&mut h);
    state.active_setup.hash(&mut h);
    let bodies = match state.setup_dialog.as_ref() {
        Some(dialog) => Some(&dialog.draft.model_bodies),
        None => app.session.doc.cam.setup(&state.active_setup).map(|s| &s.model_bodies),
    };
    bodies.hash(&mut h);
    h.finish()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::Harness;
    use serde_json::json;
    use solidcraft_engine::geom::prelude::*;

    #[test]
    fn new_setup_opens_the_setup_window_without_creating() {
        let mut h = Harness::new();
        h.step(&json!({"start": "manufacture.setup.new"})).unwrap();
        h.frames(2);
        assert!(active(&h.app));
        assert!(h.app.manufacture_workspace.setup_dialog.is_some());
        assert!(h.app.session.doc.cam.setups.is_empty());
        assert_eq!(h.app.manufacture_workspace.setup_dialog.as_ref().unwrap().name, "Setup1");
        assert!(h.widget("SETUP").is_some() || h.widget("Setup").is_some());
    }

    #[test]
    fn ok_without_bodies_does_not_save() {
        let mut h = Harness::new();
        h.step(&json!({"start": "manufacture.setup.new"})).unwrap();
        h.frames(2);
        h.step(&json!({"widget": "OK"})).unwrap();
        h.frames(2);
        assert!(h.app.session.doc.cam.setups.is_empty());
        assert!(h.app.manufacture_workspace.setup_dialog.is_some());
    }

    #[test]
    fn mill_panel_is_gone() {
        let mut h = Harness::new();
        h.step(&json!({"start": "manufacture.setup.new"})).unwrap();
        h.frames(2);
        assert!(h.widget("Add Setup").is_none());
        assert!(h.widget("Work offset").is_none());
        assert!(h.widget("G54").is_none());
        assert!(h.widget("CAM Setup").is_none());
        assert!(h.widget("New Setup").is_some());
        assert!(crate::scenario::handle_at("row:Setups").is_some());
        assert!(crate::scenario::handle_at("row:No setups yet").is_some());
    }

    #[test]
    fn manufacture_uses_process_tabs_not_the_design_ribbon() {
        let mut h = Harness::new();
        h.step(&json!({"start": "manufacture.setup.new"})).unwrap();
        h.frames(3);
        assert_eq!(crate::workspace::visible_tabs(&h.app), TABS);
        assert_eq!(h.app.ui.tab, "MILLING");
        for tab in ["MODEL", "SURFACE", "ASSEMBLE", "INSERT", "INSPECT", "MANAGE", "VIEW"] {
            assert!(crate::scenario::handle_at(&format!("tab:{tab}")).is_none(), "design tab {tab} must not paint in Manufacture");
        }
        for tab in TABS {
            assert!(crate::scenario::handle_at(&format!("tab:{tab}")).is_some(), "missing process tab {tab}");
        }
        assert!(crate::scenario::handle_at("button:solid.extrude").is_none());
        assert!(crate::scenario::handle_at("button:inspect.zebra").is_none());
        assert!(crate::scenario::handle_at("button:manufacture.setup.new").is_some());
        assert!(crate::scenario::handle_at("button:manufacture.2d.contour").is_some());
        assert!(crate::scenario::handle_at("button:manufacture.turning.face").is_none());
        h.step(&json!({"click": {"handle": "tab:TURNING"}})).unwrap();
        h.frames(2);
        assert_eq!(h.app.ui.tab, "TURNING");
        assert_eq!(h.app.manufacture_workspace.cam_mode, CamMode::Turning);
        assert!(crate::scenario::handle_at("button:manufacture.turning.face").is_some());
        assert!(crate::scenario::handle_at("button:manufacture.2d.contour").is_none());
        assert!(crate::scenario::handle_at("button:solid.extrude").is_none());
    }

    /// Two operations on one setup, with the workspace on.
    fn milled_setup(h: &mut Harness) {
        let s = &mut h.app.session;
        s.execute("cam.tool.define", &json!({"number": 1, "diameter": 6, "flute_length": 20, "feed": 600, "plunge_feed": 200, "stepdown": 3}))
            .unwrap_or_else(|e| panic!("{e}"));
        s.execute("cam.setup", &json!({"name": "Top"})).unwrap_or_else(|e| panic!("{e}"));
        s.execute(
            "cam.op.contour",
            &json!({"setup": "Top", "name": "Outside", "tool": 1, "outer": [[0, 0], [40, 0], [40, 30], [0, 30]], "side": "outside", "bottom": -3, "stepdown": 3}),
        )
        .unwrap_or_else(|e| panic!("{e}"));
        s.execute(
            "cam.op.pocket",
            &json!({"setup": "Top", "name": "Pocket", "tool": 1, "outer": [[5, 5], [35, 5], [35, 25], [5, 25]], "stepover": 2, "bottom": -3, "stepdown": 3}),
        )
        .unwrap_or_else(|e| panic!("{e}"));
        crate::manufacture_workspace::enter(&mut h.app);
        h.app.manufacture_workspace.active_setup = "Top".into();
    }

    /// The Pocket button opens the operation window on the active setup; OK adds the pocket of the
    /// picked sketch with a new tool. A button without a toolpath says so instead of doing nothing.
    #[test]
    fn operation_buttons_open_a_window_that_adds_the_operation() {
        let mut h = Harness::new();
        milled_setup(&mut h);
        h.app.run("sketch.create", json!({"plane": "XY", "name": "Slot"})).unwrap_or_else(|e| panic!("{e}"));
        h.app.run("sketch.circle.center", json!({"center": [20, 15], "diameter": 10})).unwrap_or_else(|e| panic!("{e}"));
        h.app.run("sketch.finish", json!({})).unwrap_or_else(|e| panic!("{e}"));
        h.app.start("manufacture.2d.pocket");
        let state = h.app.manufacture_workspace.op_dialog.clone().unwrap_or_else(|| panic!("Pocket opens its window"));
        assert_eq!((state.kind, state.setup.as_str(), state.geometry), (crate::manufacture_operation::OpKind::Pocket, "Top", 1));
        h.frames(2);
        let state = crate::manufacture_operation::OpDialogState { tool: 1, new_diameter: 4.0, bottom: -2.0, ..state };
        assert!(crate::manufacture_operation::save(&mut h.app, &state), "{:?}", h.app.status);
        let ops = &h.app.session.doc.cam.setup("Top").map(|s| s.operations.clone()).unwrap_or_default();
        assert!(ops.iter().any(|o| o.name() == state.name && o.tool() == 2), "a pocket on the new tool T2");
        h.app.start("manufacture.2d.face");
        assert!(h.app.status.as_ref().is_some_and(|(text, _, error)| *error && text.contains("Face")), "{:?}", h.app.status);
    }

    /// The active setup's toolpaths are computed once a design has been still for a moment, then
    /// drawn as strokes; the picked operation takes its own colour role, the others are dimmed, and
    /// leaving the workspace draws nothing.
    #[test]
    fn toolpaths_are_computed_once_per_design_and_drawn_with_the_pick_highlighted() {
        use crate::manufacture_toolpaths::{Role, computing, key, poll_at, started, strokes, wait};
        let mut h = Harness::new();
        milled_setup(&mut h);
        let ctx = egui::Context::default();
        poll_at(&mut h.app, &ctx, 0.0);
        assert!(!computing(&h.app), "a design that has just changed waits for the debounce");
        poll_at(&mut h.app, &ctx, 200.0);
        assert!(computing(&h.app));
        wait(&mut h.app);
        let all = strokes(&h.app);
        assert!(all.iter().any(|s| s.rapid), "rapids are drawn");
        assert!(all.iter().any(|s| !s.rapid && s.role == Role::Feed), "cutting moves are drawn");
        poll_at(&mut h.app, &ctx, 300.0);
        assert!(!computing(&h.app), "the same design and setup are not computed again");
        let plain = key(&h.app);
        h.app.manufacture_workspace.selected_op = Some("Pocket".into());
        poll_at(&mut h.app, &ctx, 300.0);
        let picked = strokes(&h.app);
        assert_ne!(plain, key(&h.app), "picking an operation changes what is drawn");
        assert!(picked.iter().any(|s| s.role == Role::Selected));
        assert!(picked.iter().any(|s| s.role == Role::Dimmed));
        assert!(!picked.iter().any(|s| s.role == Role::Feed || s.role == Role::Rapid));
        // An edit to the design computes the paths again, after its debounce.
        h.app.session.execute("cam.tool.define", &json!({"number": 2, "diameter": 3, "flute_length": 10})).unwrap_or_else(|e| panic!("{e}"));
        poll_at(&mut h.app, &ctx, 400.0);
        assert!(!computing(&h.app));
        poll_at(&mut h.app, &ctx, 600.0);
        assert!(computing(&h.app));
        wait(&mut h.app);
        crate::manufacture_workspace::exit(&mut h.app);
        poll_at(&mut h.app, &ctx, 700.0);
        assert!(strokes(&h.app).is_empty());
        assert!(h.app.manufacture_workspace.selected_op.is_none());
        assert_eq!(started(&h.app), 2);
    }

    /// Document edits in a row start at most one computation, and the next waits for the running one.
    #[test]
    fn edits_in_a_row_start_one_computation_at_a_time() {
        use crate::manufacture_toolpaths::{computing, poll_at, started, wait};
        let mut h = Harness::new();
        milled_setup(&mut h);
        let ctx = egui::Context::default();
        let mut now = 0.0;
        for n in 0..10 {
            h.app
                .session
                .execute("cam.tool.define", &json!({"number": 2, "diameter": 3.0 + f64::from(n) * 0.1, "flute_length": 10}))
                .unwrap_or_else(|e| panic!("{e}"));
            now += 5.0;
            poll_at(&mut h.app, &ctx, now);
        }
        assert_eq!(started(&h.app), 0, "the design is still changing: nothing starts");
        now += 1000.0;
        poll_at(&mut h.app, &ctx, now);
        assert_eq!(started(&h.app), 1);
        assert!(computing(&h.app));
        // Edits made while it runs start nothing: they wait for it.
        for n in 0..10 {
            h.app
                .session
                .execute("cam.tool.define", &json!({"number": 2, "diameter": 4.0 + f64::from(n) * 0.1, "flute_length": 10}))
                .unwrap_or_else(|e| panic!("{e}"));
            now += 5.0;
            poll_at(&mut h.app, &ctx, now);
            assert_eq!(started(&h.app), 1, "one computation at a time");
        }
        wait(&mut h.app);
        assert_eq!(started(&h.app), 2, "the wanted design starts once the running one reports");
        wait(&mut h.app);
        poll_at(&mut h.app, &ctx, now + 1000.0);
        assert_eq!(started(&h.app), 2, "nothing is computed twice");
    }

    /// A pick of an operation the setup no longer has dims nothing, and is let go.
    #[test]
    fn a_pick_of_an_operation_that_is_gone_dims_nothing() {
        use crate::manufacture_toolpaths::{Role, poll_at, strokes, wait};
        let mut h = Harness::new();
        milled_setup(&mut h);
        let ctx = egui::Context::default();
        poll_at(&mut h.app, &ctx, 0.0);
        poll_at(&mut h.app, &ctx, 200.0);
        wait(&mut h.app);
        h.app.manufacture_workspace.selected_op = Some("Ghost".into());
        poll_at(&mut h.app, &ctx, 300.0);
        assert!(strokes(&h.app).iter().all(|s| s.role != Role::Dimmed && s.role != Role::Selected), "a pick the setup lacks is no pick");
        assert!(h.app.manufacture_workspace.selected_op.is_none(), "the pick is let go");
    }

    /// Each operation is a row under its setup, and a click picks it and clicking it again lets go.
    #[test]
    fn operations_are_rows_under_their_setup_and_a_click_picks_one() {
        let mut h = Harness::new();
        milled_setup(&mut h);
        h.frames(2);
        assert!(crate::scenario::handle_at("row:op:Top/Outside").is_some());
        assert!(crate::scenario::handle_at("row:op:Top/Pocket").is_some());
        h.step(&json!({"click": {"handle": "row:op:Top/Pocket"}})).unwrap_or_else(|e| panic!("{e}"));
        h.frames(2);
        assert_eq!(h.app.manufacture_workspace.selected_op.as_deref(), Some("Pocket"));
        h.step(&json!({"click": {"handle": "row:op:Top/Pocket"}})).unwrap_or_else(|e| panic!("{e}"));
        h.frames(2);
        assert!(h.app.manufacture_workspace.selected_op.is_none());
    }

    /// Two cuts far apart on a 200 x 100 mm stock: the first cut meshes the chunks it reached, and the
    /// second meshes only the chunks it dirtied, while the first cut keeps its surface.
    #[test]
    fn sim_stock_meshes_the_chunks_a_cut_reached_and_only_those_it_dirties() {
        use crate::sim_stock::{meshed_chunks, meshed_count, settle};
        let mut h = Harness::new();
        // The stock is drawn in Manufacture only.
        h.app.manufacture_workspace.active = true;
        let gcode = "T0 M6\nG21 G90\nG0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X40 F300\nG0 Z5\nG0 X180 Y90\nG1 Z-2 F100\nG1 X190 F300\nG0 Z5";
        h.app
            .session
            .execute(
                "sim.setup",
                &json!({"stock": {"box": {"min": [0, 0, -10], "max": [200, 100, 0]}}, "tools": [{"id": 0, "diameter": 6, "flute_length": 20}], "gcode": gcode, "resolution": 0.5}),
            )
            .unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        assert!(meshed_chunks(&h.app).is_empty(), "an uncut stock has no chunk mesh");
        h.app.session.execute("sim.run", &json!({"to": 3})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        let first = meshed_chunks(&h.app);
        let touched = h.app.session.sim_stock().map_or(0, |s| s.touched_chunks().len());
        assert!(!first.is_empty() && first.len() <= touched, "the first cut has a surface: {} of {touched}", first.len());
        assert!(!crate::viewport::build_scene(&h.app).blocks.is_empty(), "the stock is drawn");
        let before = meshed_count(&h.app);
        h.app.session.execute("sim.run", &json!({})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        let added = meshed_count(&h.app) - before;
        let total = h.app.session.sim_stock().map_or(0, |s| s.touched_chunks().len());
        assert!(added > 0 && added < total, "{added} chunks meshed of {total} cut: only the dirty ones");
        let now = meshed_chunks(&h.app);
        assert!(first.iter().all(|c| now.contains(c)), "the first cut keeps its surface");
    }

    /// Outside Manufacture a simulation leaves the bodies shown: the stock is Manufacture's view.
    #[test]
    fn the_stock_hides_the_bodies_in_manufacture_only() {
        let mut h = Harness::new();
        let setup = json!({"stock": {"box": {"min": [0, 0, -10], "max": [60, 20, 0]}}, "tools": [{"id": 0, "diameter": 6, "flute_length": 20}], "gcode": "T0 M6
G21 G90
G0 X10 Y10 Z5
G1 Z-2 F100
G1 X50 F300
G0 Z5", "resolution": 0.5});
        h.app.session.execute("sim.setup", &setup).unwrap_or_else(|e| panic!("{e}"));
        assert!(!crate::sim_stock::hides_bodies(&h.app), "Design keeps its bodies");
        h.app.manufacture_workspace.active = true;
        assert!(crate::sim_stock::hides_bodies(&h.app), "Manufacture shows the stock instead");
    }

    /// A new simulation replaces the stock: its meshes go, and the uncut box covers the whole stock.
    #[test]
    fn a_new_simulation_starts_from_the_uncut_box() {
        use crate::sim_stock::{meshed_chunks, settle};
        let mut h = Harness::new();
        // The stock is drawn in Manufacture only.
        h.app.manufacture_workspace.active = true;
        let setup = json!({"stock": {"box": {"min": [0, 0, -10], "max": [60, 20, 0]}}, "tools": [{"id": 0, "diameter": 6, "flute_length": 20}], "gcode": "T0 M6\nG21 G90\nG0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X50 F300\nG0 Z5", "resolution": 0.1});
        h.app.session.execute("sim.setup", &setup).unwrap_or_else(|e| panic!("{e}"));
        h.app.session.execute("sim.run", &json!({})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        assert!(!meshed_chunks(&h.app).is_empty());
        h.app.session.execute("sim.setup", &setup).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        assert!(meshed_chunks(&h.app).is_empty(), "the new stock is uncut");
        assert!(!crate::viewport::build_scene(&h.app).tris.is_empty(), "the uncut box is drawn");
    }

    /// A cursor step over a move in the air cuts nothing: the stock's blocks keep their keys and their
    /// bytes, so the window uploads none of them again (only the tool moves).
    #[test]
    fn a_cursor_step_that_cuts_nothing_keeps_the_stock_blocks() {
        use crate::sim_stock::settle;
        let mut h = Harness::new();
        // The stock is drawn in Manufacture only.
        h.app.manufacture_workspace.active = true;
        // The toolpath's lines change with the cursor by design; this test is about the stock's blocks.
        h.app.sim_stock.view.path = false;
        let gcode = "T0 M6\nG21 G90\nG0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X40 F300\nG0 Z5\nG0 X180 Y90\nG1 Z-2 F100\nG1 X190 F300\nG0 Z5";
        h.app
            .session
            .execute(
                "sim.setup",
                &json!({"stock": {"box": {"min": [0, 0, -10], "max": [200, 100, 0]}}, "tools": [{"id": 0, "diameter": 6, "flute_length": 20}], "gcode": gcode, "resolution": 0.5}),
            )
            .unwrap_or_else(|e| panic!("{e}"));
        h.app.session.execute("sim.run", &json!({"to": 4})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        let before = crate::viewport::build_scene(&h.app).blocks;
        // Move 4 (G0 X180 Y90) runs at Z 5, above the stock: the cursor moves, the cut does not. (Move 3
        // rises through the cut it just made, which re-meshes its chunks: not a cut-free step.)
        h.app.session.execute("sim.run", &json!({"to": 5})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        let after = crate::viewport::build_scene(&h.app).blocks;
        assert!(!before.is_empty(), "the first cut is drawn as blocks");
        let keys = |blocks: &[crate::gpu::SceneBlock]| blocks.iter().map(|b| b.key).collect::<Vec<_>>();
        assert_eq!(keys(&before), keys(&after), "a cursor step that cuts nothing keeps every stock block");
        assert!(
            before.iter().zip(&after).all(|(a, b)| std::sync::Arc::ptr_eq(&a.bytes, &b.bytes)),
            "the kept blocks keep their bytes, so the GPU keeps their buffers"
        );
    }

    /// The model scene keys on the stock's meshes and on the cursor; the scene key the sketch and highlight
    /// scenes share keeps neither, so a re-mesh or a cursor step leaves them as they were.
    #[test]
    fn the_model_scene_keys_on_the_stock_and_cursor_but_not_the_shared_scene_key() {
        use crate::sim_stock::settle;
        let mut h = Harness::new();
        let gcode = "T0 M6\nG21 G90\nG0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X40 F300\nG0 Z5\nG0 X180 Y90\nG1 Z-2 F100\nG1 X190 F300\nG0 Z5";
        h.app
            .session
            .execute(
                "sim.setup",
                &json!({"stock": {"box": {"min": [0, 0, -10], "max": [200, 100, 0]}}, "tools": [{"id": 0, "diameter": 6, "flute_length": 20}], "gcode": gcode, "resolution": 0.5}),
            )
            .unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        let (scene0, model0) = (crate::viewport::scene_key(&h.app), crate::viewport::model_scene_key(&h.app));
        h.app.session.execute("sim.run", &json!({"to": 3})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        let (scene1, model1) = (crate::viewport::scene_key(&h.app), crate::viewport::model_scene_key(&h.app));
        assert_eq!(scene0, scene1, "a re-mesh leaves the shared scene key as it was");
        assert_ne!(model0, model1, "a re-mesh rebuilds the model scene");
        h.app.session.execute("sim.run", &json!({"to": 4})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        let (scene2, model2) = (crate::viewport::scene_key(&h.app), crate::viewport::model_scene_key(&h.app));
        assert_eq!(scene1, scene2, "a cursor step leaves the shared scene key as it was");
        assert_ne!(model1, model2, "a cursor step rebuilds the model scene");
    }

    /// The Simulate button is on the Post group, and pressing it (the start path) posts the active
    /// setup and sets its stock up; playing then advances the move index and hides the body.
    #[test]
    fn simulate_sets_up_the_active_setup_and_playing_advances_the_moves() {
        let mut h = Harness::new();
        let s = &mut h.app.session;
        s.execute("cam.tool.define", &json!({"number": 1, "diameter": 6, "flute_length": 20, "feed": 600, "plunge_feed": 200, "stepdown": 3}))
            .unwrap_or_else(|e| panic!("{e}"));
        s.execute("cam.setup", &json!({"name": "Top", "stock": {"min": [0, 0, -10], "max": [40, 30, 0]}})).unwrap_or_else(|e| panic!("{e}"));
        s.execute(
            "cam.op.contour",
            &json!({"setup": "Top", "name": "Outside", "tool": 1, "outer": [[0, 0], [40, 0], [40, 30], [0, 30]], "side": "outside", "bottom": -3, "stepdown": 3}),
        )
        .unwrap_or_else(|e| panic!("{e}"));
        crate::manufacture_workspace::enter(&mut h.app);
        h.app.manufacture_workspace.active_setup = "Top".into();
        h.frames(3);
        assert!(crate::scenario::handle_at("button:manufacture.post.simulate").is_some(), "Simulate is on the Post group");
        assert!(!crate::sim_stock::hides_bodies(&h.app));
        crate::sim_stock::simulate_active(&mut h.app);
        assert!(h.app.session.sim_stock().is_some(), "Simulate sets up the active setup's stock");
        assert!(crate::sim_stock::hides_bodies(&h.app), "the body is hidden while the stock shows");
        let (before, total) = h.app.session.sim_progress().unwrap_or((0, 0));
        assert!(total > 0 && before == 0, "a fresh simulation starts at its first move");
        let ctx = egui::Context::default();
        // Playing runs on with real time, a sixth of a machine second per frame at the default speed, so
        // the program takes a few hundred frames: the loop stops at its end, not at a frame count.
        for _ in 0..20_000 {
            crate::sim_stock::poll(&mut h.app, &ctx);
            if h.app.session.sim_time().is_some_and(|(t, total)| t >= total) {
                break;
            }
        }
        let (after, _) = h.app.session.sim_progress().unwrap_or((0, 0));
        assert!(after > before, "playing advances the move index: {before} -> {after}");
    }

    /// Two moves, the second one in the air above the stock: a cut-free program, so each frame's step is
    /// quick and the playhead is never cut short by the frame's budget.
    const TOOL_IN_AIR: &str = "T0 M6\nG21 G90\nG0 X10 Y10 Z5\nG1 X110 Y10 Z5 F1200";
    /// A few moves that cut the stock (the same program as the Simulate tests above).
    const CUTTING: &str = "T0 M6\nG21 G90\nG0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X40 F300\nG0 Z5\nG0 X180 Y90\nG1 Z-2 F100\nG1 X190 F300\nG0 Z5";

    /// Sets up a simulation of `gcode` on a 200 x 100 mm stock, with a 6 mm tool.
    fn sim_setup(h: &mut Harness, gcode: &str) {
        h.app
            .session
            .execute(
                "sim.setup",
                &json!({"stock": {"box": {"min": [0, 0, -10], "max": [200, 100, 0]}}, "tools": [{"id": 0, "diameter": 6, "flute_length": 20}], "gcode": gcode, "resolution": 0.5}),
            )
            .unwrap_or_else(|e| panic!("{e}"));
    }

    /// Playing moves the playhead on by the frame's time, times the speed: one frame at the harness's
    /// own frame time, then another.
    #[test]
    fn playing_moves_the_playhead_by_the_frame_time_times_the_speed() {
        use crate::sim_stock::{next_time, play};
        let mut h = Harness::new();
        h.app.manufacture_workspace.active = true;
        h.app.sim_stock.speed = 20.0;
        sim_setup(&mut h, TOOL_IN_AIR);
        play(&mut h.app);
        let (_, total) = h.app.session.sim_time().unwrap_or((0.0, 0.0));
        let (mut expected, mut time, mut frames) = (0.0, 0.0, 0);
        // A frame may leave the playhead where it was (the engine cuts within the frame's budget): run on,
        // bounded, until two frames have passed and the playhead has moved.
        while frames < 200 && (frames < 2 || time <= 0.0) {
            h.frame();
            frames += 1;
            let dt = f64::from(h.ctx.input(|i| i.stable_dt));
            assert!(dt > 0.0, "the harness gives each frame a time");
            expected = next_time(expected, total, dt, 20.0);
            time = h.app.session.sim_time().map_or(f64::NAN, |(t, _)| t);
            // The engine may cut short of the target within the frame's budget, never past it.
            assert!(time <= expected + 1e-9, "the playhead is {time} after frame {frames}, at most {expected}");
        }
        assert!(time > 0.0, "the playhead moved within 200 frames: {time}");
        assert!(expected > 0.0 && expected < total, "two frames are a step into the program: {expected} of {total}");
    }

    /// The playhead's step is the frame's time capped at 0.1 s, times the speed, and never past the end.
    #[test]
    fn the_playhead_step_caps_the_frame_and_stops_at_the_end() {
        use crate::sim_stock::next_time;
        assert!((next_time(0.0, 100.0, 1.0 / 60.0, 10.0) - 10.0 / 60.0).abs() < 1e-12);
        assert!((next_time(0.0, 100.0, 0.5, 10.0) - 1.0).abs() < 1e-12, "a stalled frame moves the playhead 0.1 s of time");
        assert_eq!(next_time(99.9, 100.0, 0.1, 10.0), 100.0);
    }

    /// Scrubbing back restores the stock of that time: the removed volume and the meshed chunks are those
    /// of a fresh run to the same time, and the playhead stops there.
    #[test]
    fn scrubbing_backward_restores_the_stock_of_that_time() {
        use crate::sim_stock::{meshed_chunks, seek, settle};
        let mut h = Harness::new();
        h.app.manufacture_workspace.active = true;
        sim_setup(&mut h, CUTTING);
        let (_, total) = h.app.session.sim_time().unwrap_or((0.0, 0.0));
        let (hi, lo) = (0.8 * total, 0.3 * total);
        h.app.session.execute("sim.run", &json!({"time": hi})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        seek(&mut h.app, lo);
        let ctx = egui::Context::default();
        for _ in 0..1000 {
            crate::sim_stock::poll(&mut h.app, &ctx);
            if h.app.session.sim_time().is_some_and(|(t, _)| (t - lo).abs() < 1e-9) {
                break;
            }
        }
        settle(&mut h.app);
        assert!(h.app.session.sim_time().is_some_and(|(t, _)| (t - lo).abs() < 1e-9), "the playhead stopped at the seek's time");
        let scrubbed = h.app.session.execute("sim.run", &json!({"time": lo})).unwrap_or_else(|e| panic!("{e}"));

        let mut fresh = Harness::new();
        fresh.app.manufacture_workspace.active = true;
        sim_setup(&mut fresh, CUTTING);
        let direct = fresh.app.session.execute("sim.run", &json!({"time": lo})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut fresh.app);
        let (a, b) = (scrubbed["removed_volume"].as_f64().unwrap_or(f64::NAN), direct["removed_volume"].as_f64().unwrap_or(f64::NAN));
        assert!(b > 0.0 && (a - b).abs() <= 1e-9 * b, "removed {a} after the seek, {b} in a fresh run to the same time");
        assert_eq!(meshed_chunks(&h.app), meshed_chunks(&fresh.app), "the same chunks have a surface");
    }

    /// A plunge, then one long X line at F600 (18 s of cut). Scrubbing back inside that line keeps the engine's
    /// revision, so the UI must remesh what the undo dirtied (`sim_take_dirty`): the surface at 7.0 s is that of
    /// a fresh run to 7.0 s, chunk by chunk, triangle for triangle.
    #[test]
    fn scrubbing_back_inside_one_long_move_keeps_the_revision_and_remeshes_what_it_dirtied() {
        use crate::sim_stock::{meshed_chunks, meshed_triangles, seek, settle};
        const LONG: &str = "T0 M6\nG21 G90\nG0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X190 F600\nG0 Z5";
        let mut h = Harness::new();
        h.app.manufacture_workspace.active = true;
        sim_setup(&mut h, LONG);
        h.app.session.execute("sim.run", &json!({"time": 9.2})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        let revision = h.app.session.sim_revision();
        let (_, tip, _) = h.app.session.sim_tool_pose().unwrap_or_else(|| panic!("the playhead has a tool pose"));
        assert!(tip[0] > 10.0 && tip[0] < 190.0, "9.2 s is inside the long move: {tip:?}");
        let deep = meshed_triangles(&h.app);

        seek(&mut h.app, 7.0);
        let ctx = egui::Context::default();
        for _ in 0..1000 {
            crate::sim_stock::poll(&mut h.app, &ctx);
            if h.app.session.sim_time().is_some_and(|(t, _)| (t - 7.0).abs() < 1e-9) {
                break;
            }
        }
        settle(&mut h.app);
        assert!(h.app.session.sim_time().is_some_and(|(t, _)| (t - 7.0).abs() < 1e-9), "the playhead stopped at 7.0 s");
        assert_eq!(h.app.session.sim_revision(), revision, "a scrub inside one move keeps the stock's revision");
        let (_, tip, _) = h.app.session.sim_tool_pose().unwrap_or_else(|| panic!("the playhead has a tool pose"));
        assert!(tip[0] > 10.0 && tip[0] < 190.0, "7.0 s is still inside the long move: {tip:?}");
        let scrubbed = h.app.session.execute("sim.run", &json!({"time": 7.0})).unwrap_or_else(|e| panic!("{e}"));
        let scrubbed_tris = meshed_triangles(&h.app);
        assert_ne!(scrubbed_tris, deep, "the undo dirtied chunks, and they were remeshed");

        let mut fresh = Harness::new();
        fresh.app.manufacture_workspace.active = true;
        sim_setup(&mut fresh, LONG);
        let direct = fresh.app.session.execute("sim.run", &json!({"time": 7.0})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut fresh.app);
        let (a, b) = (scrubbed["removed_volume"].as_f64().unwrap_or(f64::NAN), direct["removed_volume"].as_f64().unwrap_or(f64::NAN));
        assert!(b > 0.0 && (a - b).abs() <= 1e-9 * b, "removed {a} after the scrub, {b} in a fresh run to 7.0 s");
        assert_eq!(meshed_chunks(&h.app), meshed_chunks(&fresh.app), "the same chunks have a surface");
        assert_eq!(scrubbed_tris, meshed_triangles(&fresh.app), "and each has the same triangles as a fresh run");
    }

    /// Whether the drawn cutting tool has its base fan at `tip`, facing against `axis`.
    fn tool_base_at(app: &SolveApp, tip: [f64; 3], axis: [f64; 3]) -> bool {
        let (tip, axis) = (solidcore_record::Vec3::from(tip), solidcore_record::Vec3::from(axis));
        crate::sim_stock::drawn_tool(app)
            .iter()
            .flatten()
            .any(|(p, n)| (p.x - tip.x).abs() < 1e-6 && (p.y - tip.y).abs() < 1e-6 && (p.z - tip.z).abs() < 1e-6 && n.dot(axis) < -0.999)
    }

    /// The cutting tool is drawn at the engine's pose at the playhead: mid-move, its tip is between the
    /// move's ends, and the model scene is rebuilt as the tool moves.
    #[test]
    fn the_drawn_tool_follows_the_playhead_pose() {
        use crate::sim_stock::settle;
        let mut h = Harness::new();
        sim_setup(&mut h, TOOL_IN_AIR);
        settle(&mut h.app);
        let (_, total) = h.app.session.sim_time().unwrap_or((0.0, 0.0));
        h.app.session.execute("sim.run", &json!({"time": 0.5 * total})).unwrap_or_else(|e| panic!("{e}"));
        let key = crate::viewport::model_scene_key(&h.app);
        let (_, tip, axis) = h.app.session.sim_tool_pose().unwrap_or_else(|| panic!("the playhead has a tool pose"));
        assert!(tip[0] > 10.0 && tip[0] < 110.0, "the tip is mid-move: {tip:?}");
        assert!(tool_base_at(&h.app, tip, axis), "the drawn tool stands at the pose's tip");
        assert!(!tool_base_at(&h.app, [110.0, 10.0, 5.0], axis), "not at the move's end, where the cursor's move ends");

        h.app.session.execute("sim.run", &json!({"time": 0.75 * total})).unwrap_or_else(|e| panic!("{e}"));
        let (_, tip2, axis2) = h.app.session.sim_tool_pose().unwrap_or_else(|| panic!("the playhead has a tool pose"));
        assert!(tip2[0] > tip[0] && tool_base_at(&h.app, tip2, axis2), "the tool moved on along the move");
        assert_ne!(crate::viewport::model_scene_key(&h.app), key, "the tool moved, so the model scene is rebuilt");
    }

    /// Moving the tool within an air move redraws the model scene but keeps every stock block, bytes and
    /// all, and the uncut box's: only the tool's triangles are built again.
    #[test]
    fn the_view_options_change_what_the_scene_draws() {
        use crate::sim_stock::settle;
        let mut h = Harness::new();
        h.app.manufacture_workspace.active = true;
        sim_setup(&mut h, CUTTING);
        settle(&mut h.app);
        h.app.session.execute("sim.run", &json!({"to": 4})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        let count = |h: &Harness| crate::viewport::build_scene(&h.app).blocks.len();
        let all = count(&h);
        let keys = |h: &Harness| crate::viewport::build_scene(&h.app).blocks.iter().map(|b| b.key).collect::<Vec<_>>();
        let (model, plain) = (crate::sim_stock::model_key(&h.app), keys(&h));
        h.app.sim_stock.view.colouring = crate::sim_view::Colouring::Operation;
        assert_ne!(crate::sim_stock::model_key(&h.app), model, "a colouring redraws the scene");
        assert_ne!(keys(&h), plain, "the cut is built again in the operations' colours");
        h.app.sim_stock.view.colouring = crate::sim_view::Colouring::Material;
        assert!(!crate::sim_stock::drawn_tool(&h.app).is_empty(), "the tool shows by default");
        h.app.sim_stock.view.tool = false;
        assert!(crate::sim_stock::drawn_tool(&h.app).is_empty(), "the tool can be hidden");
        h.app.sim_stock.view.tool = true;
        h.app.sim_stock.view.tool_part = crate::sim_view::ToolPart::Cutting;
        let cutting = crate::sim_stock::drawn_tool(&h.app).len();
        h.app.sim_stock.view.tool_part = crate::sim_view::ToolPart::Holder;
        assert!(crate::sim_stock::drawn_tool(&h.app).len() > cutting, "the holder adds to the cutting part");
        h.app.sim_stock.view.stock = false;
        let path_only = count(&h);
        assert!(path_only < all, "hiding the stock drops its blocks: {path_only} < {all}");
        h.app.sim_stock.view.path = false;
        assert!(count(&h) < path_only, "hiding the path drops its blocks too");
    }

    #[test]
    fn a_tool_move_keeps_the_stock_and_box_blocks() {
        use crate::sim_stock::settle;
        let mut h = Harness::new();
        h.app.manufacture_workspace.active = true;
        h.app.sim_stock.view.path = false;
        sim_setup(&mut h, CUTTING);
        settle(&mut h.app);
        // Move 4 (G0 X180 Y90) runs at Z 5, above the stock: the playhead moves along it, nothing is cut.
        h.app.session.execute("sim.run", &json!({"to": 4})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        let before = crate::viewport::build_scene(&h.app).blocks;
        let pose = h.app.session.sim_tool_pose();
        let (time, _) = h.app.session.sim_time().unwrap_or((0.0, 0.0));
        h.app.session.execute("sim.run", &json!({"time": time + 0.01})).unwrap_or_else(|e| panic!("{e}"));
        settle(&mut h.app);
        let after = crate::viewport::build_scene(&h.app).blocks;
        assert_ne!(h.app.session.sim_tool_pose(), pose, "the tool moved");
        assert!(!before.is_empty(), "the cut and the box are drawn as blocks");
        let keys = |blocks: &[crate::gpu::SceneBlock]| blocks.iter().map(|b| b.key).collect::<Vec<_>>();
        assert_eq!(keys(&before), keys(&after), "a tool move keeps every block's key");
        assert!(
            before.iter().zip(&after).all(|(a, b)| std::sync::Arc::ptr_eq(&a.bytes, &b.bytes)),
            "the kept blocks keep their bytes, so the GPU keeps their buffers"
        );
    }
}
