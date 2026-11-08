//! The simulation's view options: what is drawn while a G-code simulation plays, and the window that
//! sets it (the Simulation window). The settings are session state of the window, not
//! part of the design.

use crate::SolveApp;
use crate::i18n::tr;
use crate::theme::Tokens;
use crate::ui_kit;

/// Which parts of the tool are drawn.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ToolPart {
    Cutting,
    Shank,
    #[default]
    Holder,
}

/// Which part of the toolpath is drawn, relative to the playhead.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PathMode {
    All,
    #[default]
    Behind,
    Ahead,
    Operation,
    Trail,
}

/// How the stock is coloured.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Colouring {
    #[default]
    Material,
    Operation,
    Tool,
}

/// The stock material: a tint of the theme's stock colour.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Material {
    #[default]
    Plastic,
    Aluminium,
    Steel,
    Wood,
    Brass,
}

/// Where the camera looks from while the simulation plays.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Viewpoint {
    #[default]
    Model,
    Top,
    Front,
    Right,
    Isometric,
}

/// Every option of the View tab. The defaults show the tool with its holder, the toolpath behind the
/// tool, and the stock.
#[derive(Clone, Debug, PartialEq)]
pub struct ViewOptions {
    /// Whether the window is open.
    pub open: bool,
    pub tool: bool,
    pub tool_part: ToolPart,
    pub tool_transparent: bool,
    /// A marker at the point the program drives (the tool tip).
    pub programmed_point: bool,
    pub path: bool,
    /// Rapid moves that are not retracts.
    pub links: bool,
    /// Rapid moves that only lift the tool along its axis.
    pub retracts: bool,
    pub cuts: bool,
    /// Jumps the machine makes without a known path (G28 and G30).
    pub connections: bool,
    pub points: bool,
    pub path_mode: PathMode,
    pub stock: bool,
    pub colouring: Colouring,
    pub material: Material,
    /// 0 is the coarsest the memory allows, 1 the finest.
    pub precision: f32,
    pub stock_transparent: bool,
    pub rebuild_on_rewind: bool,
    pub viewpoint: Viewpoint,
    pub spindle_direction: bool,
    /// The model's opacity while the simulation shows, 0 to 1.
    pub model_opacity: f32,
}

impl Default for ViewOptions {
    fn default() -> Self {
        ViewOptions {
            open: false,
            tool: true,
            tool_part: ToolPart::default(),
            tool_transparent: false,
            programmed_point: false,
            path: true,
            links: true,
            retracts: true,
            cuts: true,
            connections: false,
            points: true,
            path_mode: PathMode::default(),
            stock: true,
            colouring: Colouring::default(),
            material: Material::default(),
            precision: 1.0,
            stock_transparent: false,
            rebuild_on_rewind: true,
            viewpoint: Viewpoint::default(),
            spindle_direction: true,
            // The model is hidden so the cut shows; raise this to see it through the stock.
            model_opacity: 0.0,
        }
    }
}

impl ViewOptions {
    /// What the stock's colour is multiplied by, per channel, for the material.
    pub fn tint(&self) -> [f32; 3] {
        match self.material {
            Material::Plastic => [1.0, 1.0, 1.0],
            Material::Aluminium => [0.9, 0.95, 1.1],
            Material::Steel => [0.7, 0.72, 0.78],
            Material::Wood => [1.15, 0.9, 0.65],
            Material::Brass => [1.2, 1.0, 0.45],
        }
    }
}

const TOOL_PARTS: [ToolPart; 3] = [ToolPart::Cutting, ToolPart::Shank, ToolPart::Holder];
const PATH_MODES: [PathMode; 5] = [PathMode::All, PathMode::Behind, PathMode::Ahead, PathMode::Operation, PathMode::Trail];
const COLOURINGS: [Colouring; 3] = [Colouring::Material, Colouring::Operation, Colouring::Tool];
const MATERIALS: [Material; 5] = [Material::Plastic, Material::Aluminium, Material::Steel, Material::Wood, Material::Brass];
const VIEWPOINTS: [Viewpoint; 5] = [Viewpoint::Model, Viewpoint::Top, Viewpoint::Front, Viewpoint::Right, Viewpoint::Isometric];

fn index<T: PartialEq + Copy>(all: &[T], value: T) -> usize {
    all.iter().position(|x| *x == value).unwrap_or(0)
}

/// A dropdown over `all`, labelled by the catalog keys in `keys`.
fn pick<T: PartialEq + Copy>(ui: &mut egui::Ui, id: &str, value: &mut T, all: &[T], keys: &[&str]) -> bool {
    let labels: Vec<&str> = keys.iter().map(|k| tr(k)).collect();
    let mut at = index(all, *value);
    let changed = ui_kit::dropdown_picked(ui, id, &mut at, &labels);
    if let Some(v) = all.get(at) {
        *value = *v;
    }
    changed
}

/// A collapsible group of options: the kit's editor disclosure, open at first and remembered by `id`.
fn group(ui: &mut egui::Ui, id: &str, key: &str, add: impl FnOnce(&mut egui::Ui)) {
    ui_kit::editor_section(ui, id, tr(key), true, add);
}

/// An option row: the label on the left, the control on the right.
fn option<R>(ui: &mut egui::Ui, key: &str, control: impl FnOnce(&mut egui::Ui) -> R) -> R {
    ui_kit::row(ui, tr(key), control)
}

/// The simulation's options, in groups that fold. Changes land in `options` at once; the scene reads them
/// each frame.
pub fn view_tab(ui: &mut egui::Ui, o: &mut ViewOptions) {
    group(ui, "sim_view_tool", "sim.view.tool", |ui| {
        option(ui, "sim.view.show", |ui| ui_kit::checkbox(ui, "", &mut o.tool));
        option(ui, "sim.view.tool_part", |ui| {
            pick(ui, "sim_view_tool_part", &mut o.tool_part, &TOOL_PARTS, &["sim.view.part_cutting", "sim.view.part_shank", "sim.view.part_holder"])
        });
        option(ui, "sim.view.transparent", |ui| ui_kit::checkbox(ui, "", &mut o.tool_transparent));
        option(ui, "sim.view.programmed_point", |ui| ui_kit::checkbox(ui, "", &mut o.programmed_point));
    });
    group(ui, "sim_view_path", "sim.view.path", |ui| {
        option(ui, "sim.view.show", |ui| ui_kit::checkbox(ui, "", &mut o.path));
        option(ui, "sim.view.links", |ui| ui_kit::checkbox(ui, "", &mut o.links));
        option(ui, "sim.view.retracts", |ui| ui_kit::checkbox(ui, "", &mut o.retracts));
        option(ui, "sim.view.cuts", |ui| ui_kit::checkbox(ui, "", &mut o.cuts));
        option(ui, "sim.view.connections", |ui| ui_kit::checkbox(ui, "", &mut o.connections));
        option(ui, "sim.view.points", |ui| ui_kit::checkbox(ui, "", &mut o.points));
        option(ui, "sim.view.path_mode", |ui| {
            pick(
                ui,
                "sim_view_path_mode",
                &mut o.path_mode,
                &PATH_MODES,
                &["sim.view.mode_all", "sim.view.mode_behind", "sim.view.mode_ahead", "sim.view.mode_operation", "sim.view.mode_trail"],
            )
        });
    });
    group(ui, "sim_view_stock", "sim.view.stock", |ui| {
        option(ui, "sim.view.show", |ui| ui_kit::checkbox(ui, "", &mut o.stock));
        option(ui, "sim.view.colouring", |ui| {
            pick(
                ui,
                "sim_view_colouring",
                &mut o.colouring,
                &COLOURINGS,
                &["sim.view.colour_material", "sim.view.colour_operation", "sim.view.colour_tool"],
            )
        });
        option(ui, "sim.view.material", |ui| {
            pick(
                ui,
                "sim_view_material",
                &mut o.material,
                &MATERIALS,
                &[
                    "sim.view.material_plastic",
                    "sim.view.material_aluminium",
                    "sim.view.material_steel",
                    "sim.view.material_wood",
                    "sim.view.material_brass",
                ],
            )
        });
        option(ui, "sim.view.precision", |ui| {
            ui_kit::slider(ui, &mut o.precision, 0.0..=1.0, ui_kit::Slider { show_value: false, ..Default::default() })
        });
        option(ui, "sim.view.transparent", |ui| ui_kit::checkbox(ui, "", &mut o.stock_transparent));
        option(ui, "sim.view.rebuild_on_rewind", |ui| ui_kit::checkbox(ui, "", &mut o.rebuild_on_rewind));
        // Not available yet: comparing the stock with the design, and stopping on a collision.
        ui.add_enabled_ui(false, |ui| {
            option(ui, "sim.view.colour_compare", |ui| ui_kit::checkbox(ui, "", &mut false.clone()));
            option(ui, "sim.view.stop_on_collision", |ui| ui_kit::checkbox(ui, "", &mut false.clone()));
        });
    });
    group(ui, "sim_view_view", "sim.view.view", |ui| {
        option(ui, "sim.view.viewpoint", |ui| {
            pick(
                ui,
                "sim_view_viewpoint",
                &mut o.viewpoint,
                &VIEWPOINTS,
                &["sim.view.vp_model", "sim.view.vp_top", "sim.view.vp_front", "sim.view.vp_right", "sim.view.vp_iso"],
            )
        });
        option(ui, "sim.view.spindle_direction", |ui| ui_kit::checkbox(ui, "", &mut o.spindle_direction));
    });
    group(ui, "sim_view_model", "sim.view.model", |ui| {
        option(ui, "sim.view.opacity", |ui| {
            ui_kit::slider(ui, &mut o.model_opacity, 0.0..=1.0, ui_kit::Slider { show_value: false, ..Default::default() })
        });
    });
}

/// The width of the simulation window.
pub(crate) fn window_width() -> f32 {
    Tokens::get().form_card_width * WINDOW_CARDS
}

const WINDOW_CARDS: f32 = 1.15;

/// The simulation window, while it is open: every option in groups that fold. It opens with Simulate and
/// closes with its header's close button or the ribbon's View Options.
pub fn window(app: &mut SolveApp, ui: &mut egui::Ui) {
    if !app.sim_stock.view.open || !crate::sim_stock::simulating(app) {
        return;
    }
    let mut options = app.sim_stock.view.clone();
    // The card stops above the bar along the bottom of the window, under its header and padding.
    // Under the navigation cube, which the card must not cover.
    let top = app.viewport.rect.map_or(120.0, |vp| crate::viewport::axis_rect(app, vp).bottom() + 8.0);
    let bottom = app.shell.history.map_or(ui.ctx().content_rect().bottom(), |bar| bar.top());
    let body = (bottom - top - 40.0 - 24.0 - 12.0).max(120.0);
    let closed = ui_kit::Window::new(egui::Id::new("sim_view_window"), "eye", tr("sim.view.title"))
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-16.0, top))
        .max_body(body)
        .cards(WINDOW_CARDS)
        .show_body(ui.ctx(), |ui| view_tab(ui, &mut options));
    options.open = !closed;
    app.sim_stock.view = options;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_defaults_show_tool_path_and_stock() {
        let o = ViewOptions::default();
        assert!(o.tool && o.path && o.stock);
        assert_eq!((o.tool_part, o.path_mode, o.colouring), (ToolPart::Holder, PathMode::Behind, Colouring::Material));
        assert!(!o.open, "the window opens with Simulate, not by default");
    }

    #[test]
    fn every_option_list_round_trips_through_its_index() {
        for p in TOOL_PARTS {
            assert_eq!(TOOL_PARTS.get(index(&TOOL_PARTS, p)), Some(&p));
        }
        for m in PATH_MODES {
            assert_eq!(PATH_MODES.get(index(&PATH_MODES, m)), Some(&m));
        }
        for m in MATERIALS {
            assert_eq!(MATERIALS.get(index(&MATERIALS, m)), Some(&m));
        }
    }

    #[test]
    fn a_material_tints_the_stock_and_plastic_leaves_it() {
        let plastic = ViewOptions::default().tint();
        assert_eq!(plastic, [1.0, 1.0, 1.0]);
        for m in [Material::Aluminium, Material::Steel, Material::Wood, Material::Brass] {
            let o = ViewOptions { material: m, ..ViewOptions::default() };
            assert_ne!(o.tint(), plastic, "{m:?}");
        }
    }
}
