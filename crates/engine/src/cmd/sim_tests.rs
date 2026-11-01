//! `sim.*` (cmd/sim.rs) measured against values worked out by hand: a slot cut with a Ø6 flat end
//! mill, seeking back and forth, the refusals and `clear`.

use serde_json::{Value, json};

use crate::Session;
use crate::schema::ParamCheck;

/// A Ø6 flat end mill cuts 2 mm deep along X for 40 mm, in a 60 × 20 × 10 mm box: a stadium of
/// 40 × 6 plus a disc of radius 3, 2 mm deep.
const SLOT: &str = "T0 M6\nG21 G90\nG0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X50 F300\nG0 Z5";
const SLOT_MOVES: u64 = 4;

fn slot_volume() -> f64 {
    (40.0 * 6.0 + std::f64::consts::PI * 3.0 * 3.0) * 2.0
}

fn setup_params(gcode: &str) -> Value {
    json!({
        "stock": {"box": {"min": [0, 0, -10], "max": [60, 20, 0]}},
        "tools": [{"id": 0, "diameter": 6, "flute_length": 20}],
        "gcode": gcode,
        "resolution": 0.1
    })
}

fn call(s: &mut Session, id: &str, params: &Value) -> Value {
    s.execute(id, params).unwrap_or_else(|e| panic!("{id} {params}: {e}"))
}

fn set_up(gcode: &str) -> Session {
    let mut s = Session::default();
    call(&mut s, "sim.setup", &setup_params(gcode));
    s
}

#[test]
fn a_slot_removes_its_analytic_volume() {
    let mut s = set_up(SLOT);
    let status = call(&mut s, "sim.status", &json!({}));
    assert_eq!(status["total"], SLOT_MOVES);
    assert_eq!(status["cursor"], 0);
    assert_eq!(status["removed_volume"], 0.0);
    let run = call(&mut s, "sim.run", &json!({}));
    assert_eq!(run["at"], SLOT_MOVES);
    assert_eq!(run["done"], true);
    let removed = run["removed_volume"].as_f64().unwrap_or(f64::NAN);
    let expected = slot_volume();
    assert!((removed - expected).abs() <= 0.01 * expected, "removed {removed} mm^3, analytic {expected} mm^3");
    let status = call(&mut s, "sim.status", &json!({}));
    let stock = status["stock_volume"].as_f64().unwrap_or(f64::NAN);
    assert!((stock - 12000.0).abs() < 1e-6, "box volume {stock}");
    assert_eq!(status["removed_volume"].as_f64(), Some(removed));
}

#[test]
fn running_back_matches_a_fresh_run() {
    let gcode = format!("{SLOT}\nG0 Y15\nG1 Z-4 F100\nG1 X10 F300\nG0 Z5");
    let mut s = set_up(&gcode);
    call(&mut s, "sim.run", &json!({"to": 7}));
    let back = call(&mut s, "sim.run", &json!({"to": 3}));
    assert_eq!(back["at"], 3);
    assert_eq!(back["done"], false);

    let mut fresh = set_up(&gcode);
    let direct = call(&mut fresh, "sim.run", &json!({"to": 3}));
    assert_eq!(back["removed_volume"], direct["removed_volume"]);
    let hash = |s: &Session| s.sim_stock().map(|st| st.state_hash());
    assert!(hash(&s).is_some());
    assert_eq!(hash(&s), hash(&fresh));
}

#[test]
fn the_default_resolution_is_the_finest_that_fits_half_the_default_cap() {
    let mut s = Session::default();
    let mut params = setup_params(SLOT);
    if let Some(o) = params.as_object_mut() {
        o.remove("resolution");
    }
    let setup = call(&mut s, "sim.setup", &params);
    assert_eq!(setup["h"].as_f64(), Some(0.05));
    assert_eq!(setup["memory_estimate"]["budget"].as_u64(), Some(512 << 20));
    let estimate = setup["memory_estimate"]["bytes"].as_u64().unwrap_or(u64::MAX);
    assert!(estimate <= setup["memory_estimate"]["budget"].as_u64().unwrap_or(0));
}

#[test]
fn setup_lists_the_gcode_problems_and_still_runs() {
    let gcode = format!("{SLOT}\nG123 X5");
    let mut s = Session::default();
    let setup = call(&mut s, "sim.setup", &setup_params(&gcode));
    assert_eq!(setup["diag_count"], 1);
    assert_eq!(setup["diagnostics"][0]["severity"], "error");
    let run = call(&mut s, "sim.run", &json!({}));
    assert_eq!(run["done"], true);
}

#[test]
fn run_and_status_need_a_setup() {
    let mut s = Session::default();
    assert!(s.execute("sim.run", &json!({})).is_err());
    assert!(s.execute("sim.status", &json!({})).is_err());
    assert!(s.sim_stock().is_none());
}

#[test]
fn the_sim_commands_do_not_read_the_model() {
    for id in ["sim.setup", "sim.run", "sim.status", "sim.clear"] {
        let spec = crate::cmd::find_command(id).unwrap_or_else(|| panic!("{id} is not registered"));
        assert!(!spec.reads_model, "{id} reads only its own state, so it must not force a model refresh");
    }
}

#[test]
fn hostile_sim_params_are_refused_not_panicked() {
    let good = setup_params(SLOT);
    let with = |key: &str, value: Value| {
        let mut p = good.clone();
        p[key] = value;
        p
    };
    let mut no_tools = good.clone();
    if let Some(o) = no_tools.as_object_mut() {
        o.remove("tools");
    }
    let flat = |id: u32| json!({"id": id, "diameter": 6, "flute_length": 20});
    let cases: Vec<(&str, Value)> = vec![
        ("no tools key", no_tools),
        ("resolution zero", with("resolution", json!(0))),
        ("resolution negative", with("resolution", json!(-0.1))),
        ("resolution absurdly fine", with("resolution", json!(1e-9))),
        ("inverted box", with("stock", json!({"box": {"min": [0, 0, 0], "max": [-1, 20, 0]}}))),
        ("empty box", with("stock", json!({"box": {"min": [0, 0, 0], "max": [0, 20, 0]}}))),
        ("box beyond 100 m", with("stock", json!({"box": {"min": [0, 0, 0], "max": [1e9, 20, 0]}}))),
        ("box of two numbers", with("stock", json!({"box": {"min": [0, 0], "max": [1, 1]}}))),
        ("memory zero", with("memory_mb", json!(0))),
        ("memory over the cap", with("memory_mb", json!(2048))),
        (
            "memory too small for the grid",
            json!({
                "stock": {"box": {"min": [0, 0, -100], "max": [300, 200, 0]}},
                "tools": [flat(0)],
                "gcode": SLOT, "resolution": 0.05, "memory_mb": 1
            }),
        ),
        ("tool diameter zero", with("tools", json!([{"id": 0, "diameter": 0, "flute_length": 20}]))),
        ("tool negative flute", with("tools", json!([{"id": 0, "diameter": 6, "flute_length": -1}]))),
        ("unknown shape", with("tools", json!([{"id": 0, "diameter": 6, "flute_length": 20, "shape": "hexagon"}]))),
        ("bull without a corner radius", with("tools", json!([{"id": 0, "diameter": 6, "flute_length": 20, "shape": "bull"}]))),
        ("cone without an angle", with("tools", json!([{"id": 0, "diameter": 6, "flute_length": 20, "shape": "cone", "tip_diameter": 0}]))),
        ("corner radius on a ball", with("tools", json!([{"id": 0, "diameter": 6, "flute_length": 20, "shape": "ball", "corner_radius": 1}]))),
        ("tool listed twice", with("tools", json!([flat(0), flat(0)]))),
        ("gcode not text", with("gcode", json!(5))),
    ];
    for (name, params) in cases {
        let mut s = Session::default();
        assert!(s.execute("sim.setup", &params).is_err(), "{name} was accepted");
        assert!(s.sim_stock().is_none(), "{name} left a simulation behind");
    }
    let mut strict = Session { param_check: ParamCheck::Strict, ..Session::default() };
    assert!(strict.execute("sim.setup", &with("sketch", json!(1))).is_err(), "an unknown key was accepted in strict");
    let mut s = set_up(SLOT);
    for params in [json!({"to": 99}), json!({"to": -1}), json!({"to": "begin"}), json!({"budget_s": -1}), json!({"budget_s": "soon"})] {
        assert!(s.execute("sim.run", &params).is_err(), "{params} was accepted");
    }
    assert!(s.execute("sim.status", &json!({"limit": 5000000000u64, "offset": -1})).is_err());
    assert_eq!(call(&mut s, "sim.status", &json!({"limit": 5000000000u64})).get("cursor"), Some(&json!(0)));
}

#[test]
fn clear_drops_the_simulation() {
    let mut s = set_up(SLOT);
    call(&mut s, "sim.run", &json!({}));
    assert_eq!(call(&mut s, "sim.clear", &json!({}))["cleared"], true);
    assert!(s.sim_stock().is_none());
    assert!(s.execute("sim.status", &json!({})).is_err());
    assert_eq!(call(&mut s, "sim.clear", &json!({}))["cleared"], false);
}

/// An arc at Z -9 around the origin, from (10, 0): a full circle of six turns. In a 40 × 40 × 10 box.
fn arc_setup() -> Value {
    json!({
        "stock": {"box": {"min": [-20, -20, -10], "max": [20, 20, 0]}},
        "tools": [{"id": 0, "diameter": 6, "flute_length": 20}],
        "gcode": "T0 M6\nG21 G90\nG0 X10 Y0 Z5\nG1 Z-9 F100\nG3 X10 Y0 I-10 J0 P6 F200\nG0 Z5",
        "resolution": 0.2
    })
}

#[test]
fn a_budgeted_arc_is_cut_one_revolution_per_call() {
    let mut s = Session::default();
    call(&mut s, "sim.setup", &arc_setup());
    // With no budget left, each call still cuts one step: the arc is entered and left part-cut.
    let mut reply = call(&mut s, "sim.run", &json!({"budget_s": 0.0}));
    let mut calls = 1;
    while reply["partial"].is_null() && calls < 50 {
        reply = call(&mut s, "sim.run", &json!({"budget_s": 0.0}));
        calls += 1;
    }
    assert!(!reply["partial"].is_null(), "no call stopped inside the arc: {reply}");
    assert_eq!(reply["at"], 2, "the arc is not in the stock until its last revolution: {reply}");
    assert_eq!(reply["partial"]["step"], 1, "{reply}");
    let steps = reply["partial"]["steps"].as_u64().unwrap_or(0);
    assert!(steps >= 2, "{reply}");
    let mut done = reply;
    let mut more = 0;
    while done["done"] != json!(true) {
        done = call(&mut s, "sim.run", &json!({"budget_s": 0.0}));
        more += 1;
        assert!(more <= 2 * steps + 4, "the run does not finish: {done}");
    }
    let mut whole = Session::default();
    call(&mut whole, "sim.setup", &arc_setup());
    call(&mut whole, "sim.run", &json!({}));
    assert_eq!(s.sim_stock().map(|st| st.state_hash()), whole.sim_stock().map(|st| st.state_hash()));
}

#[test]
fn a_budget_error_stops_the_run_and_status_says_why() {
    // At 0.2 mm the empty box is about 1.0 MB of a 1 MB cap, so the first cut that splits rays
    // overflows it. The run stops there with the error kept: a forward run then refuses again and
    // does not retry the same move.
    let mut s = Session::default();
    let params = json!({
        "stock": {"box": {"min": [0, 0, -10], "max": [60, 20, 0]}},
        "tools": [{"id": 0, "diameter": 6, "flute_length": 20}],
        "gcode": "T0 M6\nG0 X10 Y10 Z5\nG1 Z-8 F100\nG1 X50 F300\nG0 Z5",
        "resolution": 0.2, "memory_mb": 1
    });
    call(&mut s, "sim.setup", &params);
    let err = s.execute("sim.run", &json!({})).expect_err("the cut must stop on the budget");
    assert!(err.to_string().contains("byte budget"), "{err}");
    let status = call(&mut s, "sim.status", &json!({}));
    assert!(status["error"].as_str().unwrap_or("").contains("byte budget"), "the stop is not in the status: {status}");
    let cursor = status["cursor"].clone();
    assert!(s.execute("sim.run", &json!({})).is_err(), "a stopped run must not retry on its own");
    let after = call(&mut s, "sim.status", &json!({}));
    assert_eq!(after["cursor"], cursor);
    // A backward seek clears the stop and the run goes on from the start.
    let back = call(&mut s, "sim.run", &json!({"to": 0}));
    assert_eq!(back["at"], 0);
    assert!(call(&mut s, "sim.status", &json!({}))["error"].is_null());
}

#[test]
fn a_rapid_before_the_first_tool_change_is_not_an_error() {
    let gcode = "G0 X0 Y0 Z5\nT1 M6\nG0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X30 F300\nG0 Z5";
    let params = json!({
        "stock": {"box": {"min": [0, 0, -10], "max": [60, 20, 0]}},
        "tools": [{"id": 1, "diameter": 6, "flute_length": 20}],
        "gcode": gcode, "resolution": 0.1
    });
    let mut s = Session::default();
    call(&mut s, "sim.setup", &params);
    let run = call(&mut s, "sim.run", &json!({}));
    assert_eq!(run["done"], true);
    assert!(run["removed_volume"].as_f64().unwrap_or(0.0) > 0.0, "{run}");
}

#[test]
fn a_library_listing_extra_flat_and_ball_tools_is_accepted() {
    let params = json!({
        "stock": {"box": {"min": [0, 0, -10], "max": [60, 20, 0]}},
        "tools": [
            {"id": 1, "diameter": 6, "flute_length": 20},
            {"id": 2, "diameter": 3, "flute_length": 10},
            {"id": 3, "diameter": 5, "flute_length": 15, "shape": "ball"}
        ],
        "gcode": "T1 M6\nG0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X30 F300\nG0 Z5",
        "resolution": 0.1
    });
    let mut s = Session::default();
    call(&mut s, "sim.setup", &params);
    let run = call(&mut s, "sim.run", &json!({}));
    assert_eq!(run["done"], true);
    assert!(run["removed_volume"].as_f64().unwrap_or(0.0) > 0.0, "{run}");
}

#[test]
fn a_program_in_rs274_shape_runs() {
    // The header, M5, the tool change, the length offset and an arc, as the usual post writes them.
    let gcode = "%\nO0001 (SLOT TEST)\nN10 G21 G90 G17\nN20 M5\nN30 T1 M6\nN40 G43 H1\nN50 G0 X10 Y10 Z5\nN60 G1 Z-2 F100\nN70 G2 X20 Y10 I5 J0 F200\nN80 G0 Z5\nN90 M5\nM30\n%";
    let params = json!({
        "stock": {"box": {"min": [0, 0, -10], "max": [60, 20, 0]}},
        "tools": [{"id": 1, "diameter": 6, "flute_length": 20}],
        "gcode": gcode, "resolution": 0.1
    });
    let mut s = Session::default();
    call(&mut s, "sim.setup", &params);
    let run = call(&mut s, "sim.run", &json!({}));
    assert_eq!(run["done"], true);
    assert!(run["removed_volume"].as_f64().unwrap_or(0.0) > 0.0, "{run}");
}

#[test]
fn initial_tool_cuts_a_program_without_a_tool_change() {
    let params = json!({
        "stock": {"box": {"min": [0, 0, -10], "max": [60, 20, 0]}},
        "tools": [{"id": 1, "diameter": 6, "flute_length": 20}],
        "gcode": "G0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X30 F300\nG0 Z5",
        "initial_tool": 1, "resolution": 0.1
    });
    let mut s = Session::default();
    call(&mut s, "sim.setup", &params);
    let run = call(&mut s, "sim.run", &json!({}));
    assert!(run["removed_volume"].as_f64().unwrap_or(0.0) > 0.0, "{run}");
}

#[test]
fn a_move_with_an_undefined_tool_is_a_diagnostic_not_a_setup_error() {
    let params = json!({
        "stock": {"box": {"min": [0, 0, -10], "max": [60, 20, 0]}},
        "tools": [{"id": 1, "diameter": 6, "flute_length": 20}],
        "gcode": "T1 M6\nG0 X10 Y10 Z5\nG1 Z-2 F100\nT2 M6\nG1 X30 F300",
        "resolution": 0.1
    });
    let mut s = Session::default();
    let setup = call(&mut s, "sim.setup", &params);
    assert_eq!(setup["diag_count"], 0, "{setup}");
    let run = call(&mut s, "sim.run", &json!({}));
    assert_eq!(run["done"], true);
    let status = call(&mut s, "sim.status", &json!({}));
    assert_eq!(status["diag_count"], 1, "{status}");
    assert!(status["diagnostics"][0]["msg"].as_str().unwrap_or("").contains("tool 2"), "{status}");
}

#[test]
fn reached_says_the_target_was_cut_and_done_says_the_program_was() {
    let mut s = set_up(SLOT);
    let part = call(&mut s, "sim.run", &json!({"to": 2}));
    assert_eq!((part["reached"].clone(), part["done"].clone()), (json!(true), json!(false)), "{part}");
    let end = call(&mut s, "sim.run", &json!({}));
    assert_eq!((end["reached"].clone(), end["done"].clone()), (json!(true), json!(true)), "{end}");
}

#[test]
fn a_zero_budget_still_cuts_a_move() {
    let mut s = set_up(SLOT);
    let run = call(&mut s, "sim.run", &json!({"budget_s": 0.0}));
    assert!(run["at"].as_u64().unwrap_or(0) >= 1, "a zero budget must still make progress: {run}");
}

#[test]
fn a_backward_seek_redraws_the_whole_surface() {
    // Two slots, the second 160 mm along X. Seeking back to move 4 restores the material of the second
    // slot, so its chunks must be dirty for the UI to redraw, not only the chunks the cut touched.
    let gcode = "T0 M6\nG21 G90\nG0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X20 F300\nG0 Z5\nG0 X170 Y10 Z5\nG1 Z-2 F100\nG1 X180 F300\nG0 Z5";
    let params = json!({
        "stock": {"box": {"min": [0, 0, -10], "max": [200, 20, 0]}},
        "tools": [{"id": 0, "diameter": 6, "flute_length": 20}],
        "gcode": gcode, "resolution": 0.1
    });
    let mut s = Session::default();
    call(&mut s, "sim.setup", &params);
    let end = call(&mut s, "sim.run", &json!({}));
    assert_eq!(end["done"], true);
    // The UI has drawn the cut.
    s.sim_take_dirty();
    let back = call(&mut s, "sim.run", &json!({"to": 4}));
    assert_ne!(back["revision"], end["revision"], "a rebuild changes the revision: {back}");
    assert_eq!(Some(back["revision"].as_u64().unwrap_or(0)), Some(s.sim_revision()));
    let dirty = s.sim_take_dirty();
    // Cells of 0.1 mm, 32 to a chunk: x = 175 is about chunk 54.
    assert!(dirty.iter().any(|c| (53..=55).contains(&c[0])), "the second slot's chunks must redraw: {dirty:?}");
    assert!(s.sim_take_dirty().is_empty(), "taking the dirty chunks forgets them");
}
