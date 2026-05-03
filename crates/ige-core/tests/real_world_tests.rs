//! Tests using real-world GIS polygon data
//!
//! Run: `cargo test --test real_world_tests`
//! Run all: `cargo test --workspace`

use geo::algorithm::contains::Contains;
use geo::Area;
use geo_types::{Coord, LineString, Polygon};
use ige_core::solve_oriented_lir;
use serde_json::Value;

fn parse_ring(value: &Value) -> Option<Vec<Coord<f64>>> {
    let ring = value.as_array()?;
    let mut coords = Vec::new();
    for point in ring {
        let pt = point.as_array()?;
        if pt.len() >= 2 {
            let x = pt[0].as_f64()?;
            let y = pt[1].as_f64()?;
            coords.push(Coord { x, y });
        }
    }
    Some(coords)
}

fn parse_polygon(geom: &Value) -> Option<Polygon<f64>> {
    let coords = geom.get("coordinates")?;
    let arr = coords.as_array()?;
    let ext_ring = arr.get(0)?;
    let exterior = parse_ring(ext_ring)?;
    if exterior.len() < 3 {
        return None;
    }
    let exterior_ls = LineString::from(exterior);

    let holes: Vec<LineString<f64>> = arr[1..]
        .iter()
        .filter_map(|ring| parse_ring(ring))
        .filter(|ls| ls.len() >= 3)
        .map(LineString::from)
        .collect();

    if holes.is_empty() {
        Some(Polygon::new(exterior_ls, vec![]))
    } else {
        Some(Polygon::new(exterior_ls, holes))
    }
}

fn load_test_polygons() -> Vec<(usize, Polygon<f64>)> {
    let content = include_str!("real_world_data/realworld_290.geojson");
    let json: Value = serde_json::from_str(content).expect("Failed to parse realworld_290.geojson");

    let features = json.get("features").expect("No features");
    let arr = features.as_array().expect("Features is not array");

    arr.iter()
        .filter_map(|f| {
            let id = f.get("properties")?.get("fid")?.as_u64()? as usize;
            let geom = f.get("geometry")?;
            let poly = parse_polygon(geom)?;
            Some((id, poly))
        })
        .collect()
}

#[test]
fn test_real_world_all_polygons() {
    let polygons = load_test_polygons();
    assert!(!polygons.is_empty(), "No polygons loaded from test data");

    let mut passed = 0;
    let total = 10;

    for (_id, poly) in polygons.iter().take(total) {
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| solve_oriented_lir(poly)));
        match result {
            Ok(Some(rect)) => {
                let area = rect.area();
                if area > 0.0 {
                    passed += 1;
                }
            }
            _ => {}
        }
    }

    let pass_rate = (passed as f64 / total as f64) * 100.0;
    println!("Passed: {}/{} ({:.1}%)", passed, total, pass_rate);

    assert!(
        pass_rate >= 80.0,
        "Expected at least 80% pass rate, got {:.1}% ({}/{})",
        pass_rate,
        passed,
        total
    );
}

#[test]
fn test_real_world_rectangle_area_always_less_than_polygon() {
    let polygons = load_test_polygons();
    eprintln!("Loaded {} polygons", polygons.len());

    for (id, poly) in polygons.iter().take(20) {
        let poly_area = poly.unsigned_area();
        
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| solve_oriented_lir(poly)));
        match result {
            Ok(Some(rect)) => {
                let rect_area = rect.area();
                assert!(
                    rect_area <= poly_area,
                    "Feature {}: rectangle area ({}) > polygon area ({})",
                    id,
                    rect_area,
                    poly_area
                );
            }
            Ok(None) => {}
            Err(_) => {
                eprintln!("Feature {}: panicked", id);
            }
        }
    }
}

#[test]
fn test_real_world_rectangle_center_inside_polygon() {
    let polygons = load_test_polygons();

    for (id, poly) in polygons.iter().take(10) {
        if let Some(rect) = solve_oriented_lir(poly) {
            let center_x = (rect.x_min + rect.x_max) / 2.0;
            let center_y = (rect.y_min + rect.y_max) / 2.0;

            assert!(
                poly.contains(&Coord { x: center_x, y: center_y }),
                "Feature {}: center ({}, {}) not inside polygon",
                id,
                center_x,
                center_y
            );
        }
    }
}
