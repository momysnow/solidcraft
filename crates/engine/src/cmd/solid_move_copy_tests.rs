//! `solid.move` with `copy: true`: the original body stays where it was, a new body is
//! pushed at the transform, and an older design file (no `copy` key at all) still loads.
use super::*;

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap()
}

fn cube20(s: &mut Session, name: &str) {
    run(s, "solid.box", json!({"length": 20, "width": 20, "height": 20, "body_name": name}));
}

fn volume(body: &solidcraft_doc::ModelBody) -> f64 {
    solidcraft_kernel::measure(&body.body).unwrap().volume
}

/// A box of 20 mm with its corner at `corner`, as a body of its own.
fn cube20_at(s: &mut Session, name: &str, corner: [i32; 3]) {
    run(s, "solid.box", json!({"length": 20, "width": 20, "height": 20, "body_name": name, "corner": corner}));
}

/// The copy of `source`: the body that sits where the source landed after the move. Which
/// copy is which is read from the geometry, not from the order the bodies are listed in.
fn copy_of(s: &Session, source: &str, translate: Vec3) -> String {
    let state = s.model.state();
    let from = state.body(source).expect("the source body is in the model").mesh().bounds().min;
    let want = Vec3::new(from.x + translate.x, from.y + translate.y, from.z + translate.z);
    state
        .bodies
        .iter()
        .find(|b| b.name != source && b.mesh().bounds().min.dist(want) < 1e-6)
        .map(|b| b.name.clone())
        .expect("the move must have copied the body")
}

/// The name of the body a command made, as the command reported it.
fn created(reply: &Value) -> String {
    reply["created_bodies"].as_array().and_then(|a| a.first()).and_then(Value::as_str).expect("the command must report the body it made").to_string()
}

#[test]
fn copy_keeps_the_original_and_adds_a_transformed_body() {
    let mut s = Session::default();
    cube20(&mut s, "Cube");
    run(&mut s, "solid.move", json!({"bodies": ["Cube"], "translate": [30, 0, 0], "copy": true}));
    let state = s.model.state();
    assert_eq!(state.bodies.len(), 2, "a copy move must add one body, not replace the original");

    let original = state.body("Cube").unwrap();
    let original_bounds = original.mesh().bounds();
    assert!(original_bounds.min.dist(Vec3::ZERO) < 1e-6, "the original body must stay at its own position: {original_bounds:?}");
    assert!(original_bounds.max.dist(Vec3::new(20.0, 20.0, 20.0)) < 1e-6, "{original_bounds:?}");

    let copy = state.bodies.iter().find(|b| b.name != "Cube").unwrap();
    let copy_bounds = copy.mesh().bounds();
    assert!(copy_bounds.min.dist(Vec3::new(30.0, 0.0, 0.0)) < 1e-6, "the copy must land at the translated position: {copy_bounds:?}");
    assert!(copy_bounds.max.dist(Vec3::new(50.0, 20.0, 20.0)) < 1e-6, "{copy_bounds:?}");

    let original_volume = volume(original);
    let copy_volume = volume(copy);
    assert!((original_volume - 8000.0).abs() < 1e-3, "original volume: {original_volume}");
    assert!((copy_volume - 8000.0).abs() < 1e-3, "copy volume: {copy_volume}");
    assert!((original_volume - copy_volume).abs() < 1e-6, "the copy must have the same mass properties as the source");

    let total: f64 = state.bodies.iter().map(volume).sum();
    assert!((total - 16000.0).abs() < 1e-3, "total volume across both bodies: {total}");
}

#[test]
fn without_copy_the_move_still_replaces_the_body_in_place() {
    let mut s = Session::default();
    cube20(&mut s, "Cube");
    run(&mut s, "solid.move", json!({"bodies": ["Cube"], "translate": [30, 0, 0]}));
    let state = s.model.state();
    assert_eq!(state.bodies.len(), 1, "a plain move must not create a second body");
    let moved = state.body("Cube").unwrap();
    let bounds = moved.mesh().bounds();
    assert!(bounds.min.dist(Vec3::new(30.0, 0.0, 0.0)) < 1e-6, "{bounds:?}");
}

#[test]
fn a_copied_body_keeps_the_appearance_of_the_body_it_came_from() {
    let mut s = Session::default();
    cube20(&mut s, "Cube");
    run(&mut s, "appearance.assign", json!({"bodies": ["Cube"], "appearance": "ABS - Blue"}));
    let reply = run(&mut s, "solid.move", json!({"bodies": ["Cube"], "translate": [30, 0, 0], "copy": true}));
    let copy = created(&reply);
    let source = s.doc.appearances.bodies.get("Cube").expect("the source body keeps its look").clone();
    assert_eq!(s.doc.appearances.bodies.get(&copy), Some(&source), "the copy must keep the appearance of the body it came from");
}

/// Editing a copying Move from the History redefines it on a scratch session, where the old Move
/// is still in the timeline as a suppressed placeholder (#7741).
#[test]
fn a_copying_move_can_be_edited_from_the_history() {
    let mut s = Session::default();
    cube20(&mut s, "Cube");
    run(&mut s, "appearance.assign", json!({"bodies": ["Cube"], "appearance": "ABS - Blue"}));
    let reply = run(&mut s, "solid.move", json!({"bodies": ["Cube"], "translate": [30, 0, 0], "copy": true}));
    let feature = reply["feature"].clone();
    let copy = created(&reply);
    let edit = json!({"feature": feature, "command": "solid.move", "params": {"bodies": ["Cube"], "translate": [40, 0, 0], "copy": true}});
    s.execute("timeline.redefine", &edit).expect("editing a copying move must keep copying");
    assert_eq!(s.model.state().bodies.len(), 2, "the edited move still makes one copy");
    assert_eq!(copy_of(&s, "Cube", Vec3::new(40.0, 0.0, 0.0)), copy, "the copy keeps its name and moves to the new place");
    let source = s.doc.appearances.bodies.get("Cube").expect("the source body keeps its look").clone();
    assert_eq!(s.doc.appearances.bodies.get(&copy), Some(&source), "the edited copy keeps the look of the body it came from");
}

#[test]
fn a_copied_body_keeps_the_material_of_the_body_it_came_from() {
    let mut s = Session::default();
    cube20(&mut s, "Cube");
    run(&mut s, "material.assign", json!({"bodies": ["Cube"], "material": "Aluminum"}));
    let reply = run(&mut s, "solid.move", json!({"bodies": ["Cube"], "translate": [30, 0, 0], "copy": true}));
    let copy = created(&reply);
    let source = s.doc.materials.get("Cube").expect("the source body keeps its material").clone();
    assert_eq!(s.doc.materials.get(&copy), Some(&source), "the copy must keep the material of the body it came from");
}

#[test]
fn each_copied_body_keeps_the_look_and_the_material_of_the_body_it_came_from() {
    let mut s = Session::default();
    cube20(&mut s, "Alpha");
    cube20_at(&mut s, "Beta", [50, 0, 0]);
    run(&mut s, "appearance.assign", json!({"bodies": ["Alpha"], "appearance": "ABS - Blue"}));
    run(&mut s, "appearance.assign", json!({"bodies": ["Beta"], "appearance": "ABS - Red"}));
    run(&mut s, "material.assign", json!({"bodies": ["Alpha"], "material": "Aluminum"}));
    run(&mut s, "material.assign", json!({"bodies": ["Beta"], "material": "Steel"}));

    // One move, two bodies: each copy keeps the look of the body it came from, not the look of
    // the body copied before it.
    run(&mut s, "solid.move", json!({"bodies": ["Beta", "Alpha"], "translate": [30, 0, 0], "copy": true}));
    assert_eq!(s.model.state().bodies.len(), 4, "two sources and their two copies");

    let translate = Vec3::new(30.0, 0.0, 0.0);
    let alpha = copy_of(&s, "Alpha", translate);
    let beta = copy_of(&s, "Beta", translate);
    assert_ne!(alpha, beta, "the two bodies must have two different copies");

    let alpha_look = s.doc.appearances.bodies.get("Alpha").expect("the source body keeps its look").name.clone();
    let beta_look = s.doc.appearances.bodies.get("Beta").expect("the source body keeps its look").name.clone();
    assert_ne!(alpha_look, beta_look, "the two bodies must have different looks for this to mean anything");
    assert_eq!(
        s.doc.appearances.bodies.get(&alpha).map(|l| l.name.clone()),
        Some(alpha_look.clone()),
        "the copy of Alpha must keep the look of Alpha"
    );
    assert_eq!(s.doc.appearances.bodies.get(&beta).map(|l| l.name.clone()), Some(beta_look.clone()), "the copy of Beta must keep the look of Beta");
    assert_eq!(s.doc.materials.get(&alpha).map(String::as_str), Some("Aluminum"), "the copy of Alpha must keep the material of Alpha");
    assert_eq!(s.doc.materials.get(&beta).map(String::as_str), Some("Steel"), "the copy of Beta must keep the material of Beta");
}

#[test]
fn an_older_design_with_no_copy_key_at_all_still_loads_and_moves_in_place() {
    let mut s = Session::default();
    cube20(&mut s, "Cube");
    run(&mut s, "solid.move", json!({"bodies": ["Cube"], "translate": [5, 0, 0]}));
    let saved = s.doc.to_json();
    assert!(!saved.contains("\"copy\""), "a move without the flag must serialize with no `copy` key, as an older file would read: {saved}");

    // Simulate a design saved before this feature existed: strip the key even if present, then
    // reopen through the same loader a real design file goes through.
    let mut old: Value = serde_json::from_str(&saved).unwrap();
    let features = old.get_mut("features").and_then(Value::as_array_mut).expect("features array");
    let mut touched = false;
    for f in features.iter_mut() {
        if f.get("type").and_then(Value::as_str) == Some("move")
            && let Some(obj) = f.as_object_mut()
        {
            obj.remove("copy");
            touched = true;
        }
    }
    assert!(touched, "the move feature must be present in the saved design");

    let migrated = solidcraft_doc::Document::from_json(&old.to_string()).unwrap();
    let reopened = Session::new(migrated);
    let state = reopened.model.state();
    assert_eq!(state.bodies.len(), 1, "a pre-copy design must still move the one body in place, never duplicate it");
    let moved = state.body("Cube").unwrap();
    assert!(moved.mesh().bounds().min.dist(Vec3::new(5.0, 0.0, 0.0)) < 1e-6);
}

/// A component `Part` holding a 20 mm cube at the origin, placed once at the root.
fn part_with_cube(s: &mut Session) -> (u64, u64) {
    let made = run(s, "component.create", json!({"name": "Part", "grounded": false}));
    cube20(s, "Peg");
    run(s, "component.activate", json!({"component": "root"}));
    (made["component"].as_u64().unwrap(), made["occurrence"].as_u64().unwrap())
}

/// The component an occurrence is an instance of.
fn component_of(s: &Session, occurrence: u64) -> u64 {
    s.doc.occurrences.iter().find(|o| o.id == occurrence).expect("the occurrence exists").component
}

/// The lowest corner of the first body a placement shows, in the world.
fn world_min(s: &Session, path: &[u64]) -> Vec3 {
    let names = s.occurrence_bodies(path, true).unwrap();
    s.world_state().body(&names[0]).unwrap().mesh().bounds().min
}

#[test]
fn copying_a_picked_component_makes_an_independent_component_at_the_moved_place() {
    let mut s = Session::default();
    let (part, occurrence) = part_with_cube(&mut s);
    let before = s.doc.clone();
    let source = world_min(&s, &[occurrence]);
    let reply = run(&mut s, "solid.move", json!({"bodies": [], "occurrences": [[occurrence]], "copy": true, "translate": [50, 0, 0]}));
    let copy = reply["occurrences"][0].as_u64().expect("the move reports the copy it made");
    let copied = component_of(&s, copy);
    assert_ne!(copied, part, "an independent copy of the component, not another instance of it");
    let name = &s.doc.components.iter().find(|c| c.id == copied).expect("the copy is a component").name;
    assert_eq!(name, "Part (2)", "named the way Paste New names its copies");
    assert!(world_min(&s, &[copy]).dist(Vec3::new(source.x + 50.0, source.y, source.z)) < 1e-6, "the copy lands at the moved place");
    assert!(world_min(&s, &[occurrence]).dist(source) < 1e-6, "the original component stays where it was");
    assert_eq!(s.occurrence_bodies(&[occurrence], true).unwrap().len(), 1, "the original keeps its body");
    run(&mut s, "edit.undo", json!({}));
    assert_eq!(s.doc, before, "one undo removes the copy");
}

#[test]
fn a_copied_component_turns_with_the_move() {
    let mut s = Session::default();
    let (_, occurrence) = part_with_cube(&mut s);
    let reply = run(
        &mut s,
        "solid.move",
        json!({"bodies": [], "occurrences": [[occurrence]], "copy": true, "translate": [0, 0, 0], "axis": [0, 0, 1], "angle": "90 deg"}),
    );
    let copy = reply["occurrences"][0].as_u64().unwrap();
    // A quarter turn about the origin takes the cube's x from 0..20 to -20..0 and y to 0..20.
    assert!(world_min(&s, &[copy]).dist(Vec3::new(-20.0, 0.0, 0.0)) < 1e-6, "the copy turns with the move");
    assert!(world_min(&s, &[occurrence]).dist(Vec3::ZERO) < 1e-6, "the original does not turn");
}

#[test]
fn bodies_and_a_component_copied_together_are_one_undo_step() {
    let mut s = Session::default();
    let (_, occurrence) = part_with_cube(&mut s);
    cube20_at(&mut s, "Top", [0, 0, 50]);
    let before = s.doc.clone();
    let count = s.model.state().bodies.len();
    let reply = run(&mut s, "solid.move", json!({"bodies": ["Top"], "occurrences": [[occurrence]], "copy": true, "translate": [50, 0, 0]}));
    assert_eq!(reply["occurrences"].as_array().map(Vec::len), Some(1));
    assert_eq!(s.model.state().bodies.len(), count + 2, "a copy of the body and of the component");
    run(&mut s, "edit.undo", json!({}));
    assert_eq!(s.doc, before, "one undo removes both copies");
}

#[test]
fn components_move_only_as_copies() {
    let mut s = Session::default();
    let (_, occurrence) = part_with_cube(&mut s);
    let before = s.doc.clone();
    let error = s.execute("solid.move", &json!({"bodies": [], "occurrences": [[occurrence]], "translate": [50, 0, 0]})).unwrap_err();
    assert!(error.to_string().contains("copy"), "{error}");
    assert_eq!(s.doc, before, "a refused move changes nothing");
}
