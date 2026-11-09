//! Operation window: the 2D Contour, Pocket and Adaptive buttons (and 3D Adaptive) open it on the
//! active setup; OK runs `cam.op.contour`, `cam.op.pocket` or `cam.op.adaptive`.

use serde_json::{Value, json};
use solidcraft_engine::doc::FeatureKind;
use solidcraft_ui_text::tr;

use crate::{SolveApp, ui_kit};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpKind {
    Contour,
    Pocket,
    Adaptive,
}

impl OpKind {
    /// The operation a Manufacture ribbon button makes, if SolidCraft computes it.
    pub fn from_command(id: &str) -> Option<Self> {
        Some(match id {
            "manufacture.2d.contour" => Self::Contour,
            "manufacture.2d.pocket" => Self::Pocket,
            "manufacture.2d.adaptive" | "manufacture.3d.adaptive" => Self::Adaptive,
            _ => return None,
        })
    }

    fn command(self) -> &'static str {
        match self {
            Self::Contour => "cam.op.contour",
            Self::Pocket => "cam.op.pocket",
            Self::Adaptive => "cam.op.adaptive",
        }
    }

    fn title_key(self) -> &'static str {
        match self {
            Self::Contour => "cmd.manufacture.2d.contour",
            Self::Pocket => "cmd.manufacture.2d.pocket",
            Self::Adaptive => "cmd.manufacture.2d.adaptive",
        }
    }

    fn icon(self) -> &'static str {
        match self {
            Self::Contour => "cut",
            Self::Pocket | Self::Adaptive => "hole",
        }
    }
}

const SIDES: [&str; 3] = ["outside", "inside", "on"];

#[derive(Clone, Debug)]
pub struct OpDialogState {
    pub kind: OpKind,
    pub setup: String,
    pub name: String,
    /// Index into the setup's tools; one past the end defines a new tool.
    pub tool: usize,
    pub new_diameter: f64,
    pub new_flute_length: f64,
    /// 0 machines the selected faces; 1.. the design's sketches, in timeline order.
    pub geometry: usize,
    pub side: usize,
    pub top: f64,
    pub bottom: f64,
    pub stepdown: f64,
    pub stepover: f64,
}

pub fn open(app: &mut SolveApp, kind: OpKind) {
    crate::manufacture_workspace::enter(app);
    let cam = &app.session.doc.cam;
    let setup = match cam.setup(&app.manufacture_workspace.active_setup) {
        Some(s) => s,
        None => match cam.setups.first() {
            Some(s) => s,
            None => {
                app.set_status(tr("manufacture.op.needs_setup").to_string(), true);
                crate::manufacture_setup::open_new(app);
                return;
            }
        },
    };
    let prefix = tr(kind.title_key()).replace(' ', "");
    let name = (1..=MAX_NAME_TRIES)
        .map(|n| format!("{prefix}{n}"))
        .find(|n| setup.operations.iter().all(|o| o.name() != n))
        .unwrap_or_else(|| prefix.to_string());
    let diameter = cam.tools.first().map_or(6.0, |t| t.diameter);
    let state = OpDialogState {
        kind,
        setup: setup.name.clone(),
        name,
        tool: 0,
        new_diameter: 6.0,
        new_flute_length: 20.0,
        geometry: if selected_faces(app).is_empty() && !sketches(app).is_empty() { 1 } else { 0 },
        side: 0,
        top: 0.0,
        bottom: stock_bottom(setup).unwrap_or(-5.0),
        stepdown: cam.tools.first().and_then(|t| t.stepdown).unwrap_or(diameter * 0.5),
        // Adaptive keeps the tool engaged all round: half the diameter fails in small regions.
        stepover: cam.tools.first().and_then(|t| t.stepover).unwrap_or(diameter * if kind == OpKind::Adaptive { 0.25 } else { 0.5 }),
    };
    app.manufacture_workspace.op_dialog = Some(state);
}

const MAX_NAME_TRIES: usize = 999;

/// The stock's lowest point along the setup's tool axis, measured from the work origin: a through cut.
fn stock_bottom(setup: &solidcraft_engine::doc::cam::CamSetup) -> Option<f64> {
    let s = setup.stock?;
    let (o, z) = (setup.work_origin, setup.tool_axis);
    let mut lo = f64::INFINITY;
    for i in 0..8 {
        let c = [
            if i & 1 == 0 { s.min[0] } else { s.max[0] },
            if i & 2 == 0 { s.min[1] } else { s.max[1] },
            if i & 4 == 0 { s.min[2] } else { s.max[2] },
        ];
        lo = lo.min((c[0] - o[0]) * z[0] + (c[1] - o[1]) * z[1] + (c[2] - o[2]) * z[2]);
    }
    lo.is_finite().then_some(lo)
}

fn sketches(app: &SolveApp) -> Vec<(u64, String)> {
    app.session.doc.features.iter().filter(|f| matches!(f.kind, FeatureKind::Sketch { .. })).map(|f| (f.id, f.name.clone())).collect()
}

/// The selected faces as `cam.op.*` face references: body, persistent face name and the picked point.
fn selected_faces(app: &SolveApp) -> Vec<Value> {
    let Ok(st) = app.session.world_state() else {
        return Vec::new();
    };
    app.session
        .selection
        .iter()
        .filter_map(|sel| match sel {
            solidcraft_engine::Sel::Face { body, index, point } => {
                let b = st.body(body)?;
                let name = solidcraft_engine::doc::naming::face_names(b).get(*index)?.clone();
                Some(json!({"body": body, "name": name, "at": [point.x, point.y, point.z]}))
            }
            _ => None,
        })
        .collect()
}

pub fn show_dialog(app: &mut SolveApp, ui: &mut egui::Ui) {
    let Some(mut state) = app.manufacture_workspace.op_dialog.clone() else {
        return;
    };
    let tools: Vec<String> = app
        .session
        .doc
        .cam
        .tools
        .iter()
        .map(|t| {
            solidcraft_ui_text::format("manufacture.op.tool_row", &[("number", &t.number.to_string()), ("diameter", &format!("{:.2}", t.diameter))])
        })
        .chain(std::iter::once(tr("manufacture.op.new_tool").to_string()))
        .collect();
    let mut geometry: Vec<String> = vec![tr("manufacture.op.selected_faces").to_string()];
    geometry.extend(sketches(app).into_iter().map(|(_, n)| n));
    let faces = selected_faces(app).len();
    let title = format!("{} : {}", tr(state.kind.title_key()), state.setup);

    let (closed, action) =
        ui_kit::Window::new(egui::Id::unique("mfg_op_dialog"), state.kind.icon(), &title)
            .fields()
            .cards(1.2)
            .show(ui.ctx(), true, |ui| ui_kit::scope(ui, |ui| op_fields(ui, &mut state, &tools, &geometry, faces)));
    if closed || action == ui_kit::Action::Cancel {
        app.manufacture_workspace.op_dialog = None;
        return;
    }
    if action == ui_kit::Action::Ok && save(app, &state) {
        app.manufacture_workspace.op_dialog = None;
        return;
    }
    app.manufacture_workspace.op_dialog = Some(state);
}

fn op_fields(ui: &mut egui::Ui, state: &mut OpDialogState, tools: &[String], geometry: &[String], faces: usize) {
    ui_kit::section(ui, "op_tool", tr("manufacture.op.tool"), true, |ui| {
        ui_kit::text_row(ui, tr("manufacture.op.name"), &mut state.name, "Pocket1");
        let labels: Vec<&str> = tools.iter().map(String::as_str).collect();
        state.tool = state.tool.min(labels.len().saturating_sub(1));
        ui_kit::choice_row(ui, tr("manufacture.op.tool"), "op_tool_choice", &mut state.tool, &labels);
        if state.tool + 1 == labels.len() {
            ui_kit::row(ui, tr("manufacture.op.diameter"), |ui| ui_kit::number_field(ui, &mut state.new_diameter, 0.1..=100.0, 0.1, " mm"));
            ui_kit::row(ui, tr("manufacture.op.flute_length"), |ui| ui_kit::number_field(ui, &mut state.new_flute_length, 0.1..=300.0, 0.5, " mm"));
        }
    });
    ui_kit::section(ui, "op_geometry", tr("manufacture.op.geometry"), true, |ui| {
        let labels: Vec<&str> = geometry.iter().map(String::as_str).collect();
        state.geometry = state.geometry.min(labels.len().saturating_sub(1));
        ui_kit::choice_row(ui, tr("manufacture.op.geometry"), "op_geometry_choice", &mut state.geometry, &labels);
        if state.geometry == 0 {
            let note = solidcraft_ui_text::format("manufacture.op.faces_selected", &[("count", &faces.to_string())]);
            ui_kit::text(ui, &note, ui_kit::Text::Hint("op_faces_hint"));
        }
        if state.kind == OpKind::Contour {
            let sides = [tr("manufacture.op.side_outside"), tr("manufacture.op.side_inside"), tr("manufacture.op.side_on")];
            let labels = sides;
            ui_kit::choice_row(ui, tr("manufacture.op.side"), "op_side_choice", &mut state.side, &labels);
        }
    });
    ui_kit::section(ui, "op_heights", tr("manufacture.op.heights"), true, |ui| {
        ui_kit::row(ui, tr("manufacture.op.top"), |ui| ui_kit::number_field(ui, &mut state.top, -1.0e4..=1.0e4, 0.1, " mm"));
        ui_kit::row(ui, tr("manufacture.op.bottom"), |ui| ui_kit::number_field(ui, &mut state.bottom, -1.0e4..=1.0e4, 0.1, " mm"));
    });
    ui_kit::section(ui, "op_passes", tr("manufacture.op.passes"), true, |ui| {
        ui_kit::row(ui, tr("manufacture.op.stepdown"), |ui| ui_kit::number_field(ui, &mut state.stepdown, 0.01..=100.0, 0.1, " mm"));
        if state.kind != OpKind::Contour {
            ui_kit::row(ui, tr("manufacture.op.stepover"), |ui| ui_kit::number_field(ui, &mut state.stepover, 0.01..=100.0, 0.1, " mm"));
        }
    });
}

/// Run the window's command, defining its new tool first; `false` (with the error in the status bar) keeps it open.
pub fn save(app: &mut SolveApp, state: &OpDialogState) -> bool {
    let tool = match app.session.doc.cam.tools.get(state.tool) {
        Some(t) => t.number,
        None => {
            let number = app.session.doc.cam.tools.iter().map(|t| t.number).max().unwrap_or(0) + 1;
            let define = json!({"number": number, "diameter": state.new_diameter, "flute_length": state.new_flute_length});
            if let Err(e) = app.run("cam.tool.define", define) {
                app.set_status(e, true);
                return false;
            }
            number
        }
    };
    let mut p = json!({
        "setup": state.setup, "name": state.name.trim(), "tool": tool,
        "top": state.top, "bottom": state.bottom, "stepdown": state.stepdown,
    });
    if state.geometry == 0 {
        let faces = selected_faces(app);
        if faces.is_empty() {
            app.set_status(tr("manufacture.op.no_geometry").to_string(), true);
            return false;
        }
        p["faces"] = Value::Array(faces);
    } else if let Some((id, _)) = sketches(app).get(state.geometry - 1) {
        p["sketch"] = json!(id);
    }
    match state.kind {
        OpKind::Contour => p["side"] = json!(SIDES.get(state.side).copied().unwrap_or("outside")),
        OpKind::Pocket | OpKind::Adaptive => p["stepover"] = json!(state.stepover),
    }
    match app.run(state.kind.command(), p) {
        Ok(_) => {
            app.manufacture_workspace.active_setup = state.setup.clone();
            app.manufacture_workspace.selected_op = Some(state.name.trim().to_string());
            app.set_status(tr("manufacture.op.saved").to_string(), false);
            true
        }
        Err(e) => {
            app.set_status(e, true);
            false
        }
    }
}
