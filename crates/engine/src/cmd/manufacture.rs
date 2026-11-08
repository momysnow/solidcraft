//! Manufacture workspace chrome. These commands open UI; document writes go through `cam.setup`.
//!
//! Recut of the #1782 ribbon onto the cam document. Op dialogs land in the stacked UI PRs.

use serde_json::{Value, json};
use solidcraft_cam::ToolKind;
use solidcraft_geom::Vec3;

use super::CommandSpec;
use crate::params::{bad, str_};
use crate::{Result, Session};

fn opened(_s: &mut Session, _params: &Value) -> Result<Value> {
    Ok(json!({"opened": true}))
}

const fn chrome(id: &'static str, label: &'static str, panel: &'static str, icon: &'static str, params: &'static str) -> CommandSpec {
    CommandSpec::new(id, label, opened).icon(icon).noundo().doc_only().at("MANUFACTURE", panel).params(params)
}

/// The operations of a posted program: the post writes one comment per operation, `name: T<n> D<d> kind`,
/// in its comment style. Each such line starts a section at its 1-based line. A post with comments off
/// gives none, and the simulation then shows one section.
fn sections_of(gcode: &str) -> Vec<Value> {
    gcode
        .lines()
        .enumerate()
        .filter_map(|(i, line)| {
            let text = line.trim().trim_start_matches(|c| c == '(' || c == ';').trim_end_matches(')').trim();
            let (name, rest) = text.rsplit_once(": T")?;
            let mut parts = rest.split_whitespace();
            let tool = parts.next()?;
            let diameter = parts.next()?;
            (!name.is_empty() && !tool.is_empty() && tool.bytes().all(|b| b.is_ascii_digit()) && diameter.starts_with('D'))
                .then(|| json!({"name": name, "line": i + 1}))
        })
        .collect()
}

/// Posts a setup in memory and sets its G-code up for the simulation: the setup's stock box and its
/// flat end mills, the box in the setup's work frame like the G-code. A 0.25 mm ray spacing is tried
/// first, then coarser up to 2 mm, so a large stock still fits the memory cap. Replies sim.setup's reply
/// plus `frame` {origin, x, y, z}, the work frame in the model. Nothing is written to the design.
fn simulate(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "manufacture.post.simulate";
    let name = str_(p, "setup").map(str::trim).filter(|n| !n.is_empty()).ok_or_else(|| bad(cmd, "`setup` is required"))?.to_string();
    let setup = s.doc.cam.setup(&name).ok_or_else(|| bad(cmd, format!("no setup `{name}`")))?;
    let stock = setup.stock.ok_or_else(|| bad(cmd, format!("setup `{name}` has no stock box")))?;
    // The G-code is in the setup's work frame, so the stock box goes there too: the box around its
    // corners (the same box when the tool axis is a model axis).
    let frame = super::cam::work_frame(setup).map_err(|e| bad(cmd, e))?;
    let (mut min, mut max) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for i in 0..8 {
        let pick = |k: usize| if i >> k & 1 == 0 { stock.min[k] } else { stock.max[k] };
        let w = frame.local(Vec3::new(pick(0), pick(1), pick(2)));
        for (k, v) in [w.x, w.y, w.z].into_iter().enumerate() {
            min[k] = min[k].min(v);
            max[k] = max[k].max(v);
        }
    }
    let posted = s.execute("cam.post", &json!({"setup": name}))?;
    let gcode = posted["gcode"].as_str().unwrap_or_default().to_string();
    let tools: Vec<Value> = s
        .doc
        .cam
        .tools
        .iter()
        .filter(|t| t.kind == ToolKind::FlatEndmill)
        .map(|t| json!({"id": t.number, "diameter": t.diameter, "flute_length": t.flute_length}))
        .collect();
    let sections = sections_of(&gcode);
    let mut last = None;
    for resolution in [0.25, 0.5, 1.0, 2.0] {
        let params =
            json!({"stock": {"box": {"min": min, "max": max}}, "tools": tools, "gcode": gcode, "resolution": resolution, "sections": sections});
        let reply = s.execute("sim.setup", &params);
        let done = reply.is_ok();
        last = Some(reply);
        if done {
            break;
        }
    }
    let mut reply = last.unwrap_or_else(|| Err(bad(cmd, "no ray spacing to try")))?;
    reply["setup"] = json!(name);
    // The stock is in the work frame: the window draws it back in the model with this.
    let o = frame.model(Vec3::ZERO);
    let axis = |w: Vec3| {
        let d = frame.model(w) - o;
        [d.x, d.y, d.z]
    };
    reply["frame"] = json!({"origin": [o.x, o.y, o.z], "x": axis(Vec3::new(1.0, 0.0, 0.0)), "y": axis(Vec3::new(0.0, 1.0, 0.0)), "z": axis(Vec3::new(0.0, 0.0, 1.0))});
    Ok(reply)
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("manufacture.setup.new", "New Setup", opened)
        .icon("origin")
        .noundo()
        .doc_only()
        .at("MANUFACTURE", "SETUP")
        .params("Opens the New Setup dialog. Document writes go through cam.setup."),
    chrome("manufacture.tools.library", "Tool Library", "MANAGE", "library", "Opens the document tool library for the active Manufacture setup."),
    chrome("manufacture.manage.machine_library", "Machines", "MANAGE", "list", "Opens the Machine Library browser."),
    chrome("manufacture.manage.post_library", "Posts", "MANAGE", "export", "Opens the Post Library browser."),
    chrome("manufacture.manage.template_library", "Templates", "MANAGE", "copy", "Opens the Template Library browser."),
    chrome("manufacture.2d.contour", "Contour", "2D", "cut", "Opens a 2D Contour operation on the active Manufacture setup."),
    chrome("manufacture.2d.face", "Face", "2D", "surface", "Opens a 2D Face operation on the active Manufacture setup."),
    chrome("manufacture.2d.adaptive", "Adaptive", "2D", "sweep", "Opens a 2D Adaptive operation on the active Manufacture setup."),
    chrome("manufacture.2d.pocket", "Pocket", "2D", "hole", "Opens a 2D Pocket operation on the active Manufacture setup."),
    chrome("manufacture.2d.slot", "Slot", "2D", "slot", "Opens a 2D Slot operation on the active Manufacture setup."),
    chrome("manufacture.2d.trace", "Trace", "2D", "trace", "Opens a 2D Trace operation on the active Manufacture setup."),
    chrome("manufacture.2d.thread", "Thread", "2D", "thread", "Opens a 2D Thread operation on the active Manufacture setup."),
    chrome("manufacture.2d.bore", "Bore", "2D", "hole", "Opens a 2D Bore operation on the active Manufacture setup."),
    chrome("manufacture.2d.circular", "Circular", "2D", "hole", "Opens a 2D Circular operation on the active Manufacture setup."),
    chrome("manufacture.2d.engrave", "Engrave", "2D", "text", "Opens a 2D Engrave operation on the active Manufacture setup."),
    chrome("manufacture.2d.chamfer", "Chamfer", "2D", "chamfer", "Opens a 2D Chamfer operation on the active Manufacture setup."),
    chrome("manufacture.drill", "Drill", "DRILLING", "hole", "Opens a Drill operation on the active Manufacture setup."),
    chrome("manufacture.hole_recognition", "Hole Recognition", "DRILLING", "hole", "Opens Hole Recognition on the active Manufacture setup."),
    chrome("manufacture.3d.adaptive", "Adaptive", "3D", "sweep", "Opens 3D Adaptive Roughing on the active Manufacture setup."),
    chrome("manufacture.3d.parallel", "Parallel", "3D", "surface", "Opens a 3D Parallel operation on the active Manufacture setup."),
    chrome("manufacture.3d.scallop", "Scallop", "3D", "untrim", "Opens a 3D Scallop operation on the active Manufacture setup."),
    chrome("manufacture.3d.steep", "Steep", "3D", "draft_analysis", "Opens Steep and Shallow on the active Manufacture setup."),
    chrome("manufacture.3d.contour", "3D Contour", "3D", "cut", "Opens a 3D Contour operation on the active Manufacture setup."),
    chrome("manufacture.3d.flat", "Flat", "3D", "surface", "Opens a 3D Flat operation on the active Manufacture setup."),
    chrome("manufacture.multiaxis.swarf", "Swarf", "MULTI-AXIS", "sweep", "Opens a Swarf operation on the active Manufacture setup."),
    chrome("manufacture.multiaxis.rough", "MA Rough", "MULTI-AXIS", "extrude", "Opens Multi-Axis Roughing on the active Manufacture setup."),
    chrome("manufacture.multiaxis.rotary", "Rotary", "MULTI-AXIS", "revolve", "Opens Rotary Parallel on the active Manufacture setup."),
    chrome("manufacture.multiaxis.finish", "MA Finish", "MULTI-AXIS", "fillet", "Opens Multi-Axis Finishing on the active Manufacture setup."),
    chrome("manufacture.multiaxis.contour", "MA Contour", "MULTI-AXIS", "cut", "Opens Multi-Axis Contour on the active Manufacture setup."),
    chrome("manufacture.multiaxis.adv_swarf", "Adv. Swarf", "MULTI-AXIS", "sweep", "Opens Advanced Swarf on the active Manufacture setup."),
    chrome("manufacture.turning.face", "Face", "TURNING", "surface", "Opens a Turning Face operation on the active Manufacture setup."),
    chrome("manufacture.turning.profile", "Profile", "TURNING", "silhouette", "Opens Turning Profile Roughing on the active Manufacture setup."),
    chrome("manufacture.turning.groove", "Groove", "TURNING", "slot", "Opens a Turning Groove operation on the active Manufacture setup."),
    chrome("manufacture.turning.thread", "Thread", "TURNING", "thread", "Opens a Turning Thread operation on the active Manufacture setup."),
    chrome("manufacture.turning.chamfer", "Chamfer", "TURNING", "chamfer", "Opens a Turning Chamfer operation on the active Manufacture setup."),
    chrome("manufacture.turning.part", "Part", "TURNING", "split_body", "Opens a Turning Part operation on the active Manufacture setup."),
    chrome("manufacture.part_handling.grab", "Grab", "PART", "joint", "Opens Subspindle Grab on the active Manufacture setup."),
    chrome("manufacture.part_handling.bar_pull", "Bar Pull", "PART", "move", "Opens a Bar Pull operation on the active Manufacture setup."),
    chrome("manufacture.part_handling.return", "Return", "PART", "update", "Opens Subspindle Return on the active Manufacture setup."),
    chrome("manufacture.part_handling.tool_call", "Tool Call", "PART", "list", "Opens a Tool Call operation on the active Manufacture setup."),
    chrome("manufacture.inspection.wcs", "Probe WCS", "PROBE", "origin", "Opens Probe WCS on the active Manufacture setup."),
    chrome("manufacture.inspection.geometry", "Probe Geom", "PROBE", "measure", "Opens Probe Geometry on the active Manufacture setup."),
    chrome("manufacture.inspection.surface", "Surface", "PROBE", "zebra", "Opens Inspect Surface on the active Manufacture setup."),
    chrome("manufacture.inspection.align", "Align", "PROBE", "align", "Opens Part Alignment on the active Manufacture setup."),
    chrome("manufacture.inspection.manual", "Manual", "PROBE", "select", "Opens Manual Inspection on the active Manufacture setup."),
    chrome("manufacture.inspection.record", "Record", "PROBE", "params", "Opens Record Manual Inspection on the active Manufacture setup."),
    chrome("manufacture.fabrication.nest", "Nest", "CUTTING", "pattern", "Opens Nest planning on the active Manufacture setup."),
    chrome(
        "manufacture.fabrication.profile",
        "2D Profile",
        "CUTTING",
        "cut",
        "Opens a 2D Profile cutting operation on the active Manufacture setup.",
    ),
    chrome(
        "manufacture.additive.toolpath",
        "Additive",
        "ADDITIVE",
        "extrude",
        "Opens an Additive Toolpath operation on the active Manufacture setup.",
    ),
    chrome("manufacture.additive.from_gcode", "From G-code", "ADDITIVE", "export", "Opens From G-code import on the active Manufacture setup."),
    chrome("manufacture.additive.simulate", "Simulate", "ADDITIVE", "camera", "Simulates the Additive toolpath on the active Manufacture setup."),
    chrome("manufacture.modify.trim", "Trim", "MODIFY", "trim", "Opens Trim on the active Manufacture setup."),
    chrome("manufacture.modify.delete_passes", "Del Passes", "MODIFY", "delete", "Opens Delete Passes on the active Manufacture setup."),
    chrome("manufacture.modify.leads_links", "Leads/Links", "MODIFY", "spline", "Opens Leads and Links on the active Manufacture setup."),
    chrome("manufacture.modify.replace_tool", "Replace Tool", "MODIFY", "update", "Opens Replace Tool on the active Manufacture setup."),
    chrome("manufacture.modify.move_entry", "Move Entry", "MODIFY", "move", "Opens Move Entry Positions on the active Manufacture setup."),
    chrome("manufacture.post.process", "Post", "POST", "export", "Opens Post Process on the active Manufacture setup."),
    chrome("manufacture.post.time", "Cycle Time", "POST", "params", "Opens Machining Time on the active Manufacture setup."),
    chrome("manufacture.post.sheet", "Setup Sheet", "POST", "list", "Opens a Setup Sheet for the active Manufacture setup."),
    chrome("manufacture.sim.options", "View Options", "VIEW", "eye", "Opens the simulation's view options: tool, toolpath, stock and view settings."),
    chrome("manufacture.sim.exit", "Exit Simulation", "EXIT", "check", "Ends the simulation and returns to the Manufacture tabs. The stock is dropped; nothing is written to the design."),
    CommandSpec::new("manufacture.post.simulate", "Simulate", simulate)
        .icon("camera")
        .noundo()
        .doc_only()
        .at("MANUFACTURE", "POST")
        .params("setup: name. Posts the setup in memory and sets its G-code up for the stock-removal simulation: the setup's stock box and its flat end mills. Refused without a stock box. Replies the sim.setup reply with the setup name; then sim.run plays it."),
];

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn sections_come_from_the_operation_comments_of_the_post() {
        let g = "(Program)
G21
(Face1: T3 D63 Face mill)
G0 X0
; Pocket A: T1 D6 Flat end mill
G1 X5
(not an op: just text)
";
        let got: Vec<_> = sections_of(g).iter().map(|v| (v["name"].as_str().unwrap_or("").to_string(), v["line"].as_u64().unwrap_or(0))).collect();
        assert_eq!(got, [("Face1".to_string(), 3), ("Pocket A".to_string(), 5)]);
        assert!(
            sections_of(
                "G21
G0 X0
"
            )
            .is_empty(),
            "no comments, no sections"
        );
    }

    #[test]
    fn setup_new_is_display_only() {
        let mut s = Session::default();
        let r = s.execute("manufacture.setup.new", &json!({})).unwrap();
        assert_eq!(r["opened"], true);
        assert!(s.doc.cam.setups.is_empty());
    }

    #[test]
    fn contour_is_display_only() {
        let mut s = Session::default();
        let r = s.execute("manufacture.2d.contour", &json!({})).unwrap();
        assert_eq!(r["opened"], true);
        assert!(s.doc.cam.setups.is_empty());
    }

    #[test]
    fn simulate_sets_up_the_stock_of_a_posted_setup() {
        let mut s = Session::default();
        s.execute("cam.tool.define", &json!({"number": 1, "diameter": 6, "flute_length": 20, "feed": 600, "plunge_feed": 200, "stepdown": 3}))
            .unwrap_or_else(|e| panic!("{e}"));
        s.execute("cam.setup", &json!({"name": "Top", "stock": {"min": [0, 0, -10], "max": [40, 30, 0]}})).unwrap_or_else(|e| panic!("{e}"));
        s.execute(
            "cam.op.contour",
            &json!({"setup": "Top", "name": "Outside", "tool": 1, "outer": [[0, 0], [40, 0], [40, 30], [0, 30]], "side": "outside", "bottom": -3, "stepdown": 3}),
        )
        .unwrap_or_else(|e| panic!("{e}"));
        assert!(s.sim_stock().is_none());
        let reply = s.execute("manufacture.post.simulate", &json!({"setup": "Top"})).unwrap_or_else(|e| panic!("{e}"));
        assert_eq!(reply["setup"], "Top");
        assert!(s.sim_stock().is_some(), "the posted setup has a stock to cut");
        assert!(s.sim_program().is_some_and(|p| !p.moves.is_empty()), "the posted G-code is the program");
        let status = s.execute("sim.status", &json!({})).unwrap_or_else(|e| panic!("{e}"));
        let names: Vec<_> = status["sections"].as_array().map(|a| a.iter().filter_map(|x| x["name"].as_str()).collect()).unwrap_or_default();
        assert_eq!(names, ["Outside"], "one section per operation of the setup: {status}");
        assert!(s.execute("manufacture.post.simulate", &json!({"setup": "Nope"})).is_err());
        assert!(s.execute("manufacture.post.simulate", &json!({})).is_err());
    }

    #[test]
    fn simulate_puts_the_stock_in_the_work_frame_of_the_g_code() {
        // Part zero on the stock's top, as on a mill: the G-code's Z 0 is the model's Z 10.
        let mut s = Session::default();
        s.execute("cam.tool.define", &json!({"number": 1, "diameter": 6, "flute_length": 20, "feed": 600, "plunge_feed": 200, "stepdown": 3}))
            .unwrap_or_else(|e| panic!("{e}"));
        s.execute("cam.setup", &json!({"name": "Top", "work_origin": [0, 0, 10], "stock": {"min": [0, 0, 0], "max": [40, 30, 10]}}))
            .unwrap_or_else(|e| panic!("{e}"));
        s.execute(
            "cam.op.contour",
            &json!({"setup": "Top", "name": "Outside", "tool": 1, "outer": [[0, 0], [40, 0], [40, 30], [0, 30]], "side": "outside", "bottom": -3, "stepdown": 3}),
        )
        .unwrap_or_else(|e| panic!("{e}"));
        let reply = s.execute("manufacture.post.simulate", &json!({"setup": "Top"})).unwrap_or_else(|e| panic!("{e}"));
        assert_eq!(reply["frame"]["origin"], json!([0.0, 0.0, 10.0]));
        let (lo, hi) = (reply["bounds"]["min"][2].as_f64().unwrap_or(f64::NAN), reply["bounds"]["max"][2].as_f64().unwrap_or(f64::NAN));
        assert!((lo + 10.0).abs() < 0.5 && hi.abs() < 0.5, "stock from z {lo} to {hi}, not -10 to 0: {}", reply["bounds"]);
    }

    #[test]
    fn ribbon_icons_are_not_all_settings() {
        let icons: Vec<_> = COMMANDS.iter().map(|c| c.icon).collect();
        assert!(icons.contains(&"cut"));
        assert!(icons.contains(&"hole"));
        assert!(icons.contains(&"origin"));
        assert!(!icons.iter().all(|icon| *icon == "settings"));
    }
}
