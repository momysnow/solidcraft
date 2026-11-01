//! `sim.*` (cmd/sim.rs) measured against values worked out by hand: a slot cut with a Ø6 flat end
//! mill, seeking back and forth, the refusals and `clear`.

use serde_json::{Value, json};

use crate::Session;
use crate::schema::ParamCheck;

/// A Ø6 flat end mill cuts 2 mm deep along X for 40 mm, in a 60 × 20 × 10 mm box: a stadium of
/// 40 × 6 plus a disc of radius 3, 2 mm deep.
const SLOT: &str = "G21 G90\nG0 X10 Y10 Z5\nG1 Z-2 F100\nG1 X50 F300\nG0 Z5";
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
fn the_default_resolution_is_the_finest_that_fits_the_cap() {
    let mut s = Session::default();
    let mut params = setup_params(SLOT);
    if let Some(o) = params.as_object_mut() {
        o.remove("resolution");
    }
    let setup = call(&mut s, "sim.setup", &params);
    assert_eq!(setup["h"].as_f64(), Some(0.05));
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
                "tools": [{"id": 0, "diameter": 6, "flute_length": 20}],
                "gcode": SLOT, "resolution": 0.05, "memory_mb": 1
            }),
        ),
        ("tool diameter zero", with("tools", json!([{"id": 0, "diameter": 0, "flute_length": 20}]))),
        ("tool negative flute", with("tools", json!([{"id": 0, "diameter": 6, "flute_length": -1}]))),
        ("tool shape ball", with("tools", json!([{"id": 0, "diameter": 6, "flute_length": 20, "shape": "ball"}]))),
        ("tool the G-code does not list", with("tools", json!([{"id": 1, "diameter": 6, "flute_length": 20}]))),
        (
            "tool listed but never used",
            with("tools", json!([{"id": 0, "diameter": 6, "flute_length": 20}, {"id": 3, "diameter": 6, "flute_length": 20}])),
        ),
        ("tool listed twice", with("tools", json!([{"id": 0, "diameter": 6, "flute_length": 20}, {"id": 0, "diameter": 6, "flute_length": 20}]))),
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
