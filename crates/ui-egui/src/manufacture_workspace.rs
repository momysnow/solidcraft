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
    /// The operation picked in the Browser (its name in the active setup): its toolpath is drawn
    /// in the highlight colour and the others are dimmed.
    pub selected_op: Option<String>,
    /// The active setup's toolpaths, computed off the window thread.
    pub toolpaths: crate::manufacture_toolpaths::Toolpaths,
}

pub fn is_tab(tab: &str) -> bool {
    CamMode::from_tab(tab).is_some() || tab == "MANUFACTURE"
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
        id if id.starts_with("manufacture.") => {
            enter(app);
            if let Some(tab) = tab_for_command(id) {
                show_tab(app, tab);
            }
            true
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
    crate::sim_stock::panel(app, ui);
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
        for _ in 0..200 {
            crate::sim_stock::poll(&mut h.app, &ctx);
            if h.app.session.sim_progress().is_some_and(|(at, n)| at == n) {
                break;
            }
        }
        let (after, _) = h.app.session.sim_progress().unwrap_or((0, 0));
        assert!(after > before, "playing advances the move index: {before} -> {after}");
    }
}
