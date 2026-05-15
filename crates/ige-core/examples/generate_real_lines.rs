//! Generate random line features inside real-world polygons.
//! Each line is tagged with the polygon's `fid` so it can be matched
//! and overlaid on the correct polygon during visualization.
//!
//! Usage:
//!   cargo run --package ige-core --example generate_real_lines
//!   cargo run --package ige-core --example generate_real_lines -- --count 3000 --output target/ige_output/real_lines.geojson

use rand::{Rng, SeedableRng};
use rand::rngs::StdRng;
use serde_json::{json, Value};
use std::fs;

fn poly_bbox(coords: &[Vec<[f64; 2]>]) -> Option<(f64, f64, f64, f64)> {
    let ring = coords.first()?;
    let mut min_x = f64::MAX; let mut min_y = f64::MAX;
    let mut max_x = f64::MIN; let mut max_y = f64::MIN;
    for pt in ring {
        min_x = min_x.min(pt[0]); min_y = min_y.min(pt[1]);
        max_x = max_x.max(pt[0]); max_y = max_y.max(pt[1]);
    }
    Some((min_x, min_y, max_x, max_y))
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut count = 3000;
    let mut seed = 42;
    let mut output_path = "target/ige_output/real_lines.geojson".to_string();
    let mut data_path = "crates/ige-core/tests/real_world_data/realworld_290.geojson".to_string();

    for i in 0..args.len() {
        if args[i] == "--count" && i + 1 < args.len() { count = args[i + 1].parse().unwrap_or(3000); }
        if args[i] == "--seed" && i + 1 < args.len() { seed = args[i + 1].parse().unwrap_or(42); }
        if args[i] == "--output" && i + 1 < args.len() { output_path = args[i + 1].clone(); }
        if args[i] == "--data" && i + 1 < args.len() { data_path = args[i + 1].clone(); }
    }

    let content = fs::read_to_string(&data_path).expect("Failed to read real-world data");
    let json: Value = serde_json::from_str(&content).expect("Failed to parse GeoJSON");
    let features = json.get("features").expect("No features").as_array().expect("Not an array");

    let mut rng = StdRng::seed_from_u64(seed);
    let mut output_features: Vec<Value> = Vec::new();
    let per_poly = (count as f64 / features.len() as f64).ceil() as usize;

    for feature in features {
        let fid = feature
            .get("properties").and_then(|p| p.get("fid")).and_then(|v| v.as_u64()).unwrap_or(0) as usize;
        let geom = feature.get("geometry").and_then(|g| g.get("coordinates")).and_then(|c| c.as_array());
        let coords: Vec<Vec<[f64; 2]>> = match geom {
            Some(arr) => arr.iter().filter_map(|ring| {
                ring.as_array().map(|pts| pts.iter().filter_map(|pt| {
                    let a = pt.as_array()?;
                    Some([a[0].as_f64()?, a[1].as_f64()?])
                }).collect())
            }).collect(),
            None => continue,
        };
        let Some((bx0, by0, bx1, by1)) = poly_bbox(&coords) else { continue; };
        let span_x = bx1 - bx0;
        let span_y = by1 - by0;
        if span_x < 1.0 || span_y < 1.0 { continue; }

        for i in 0..per_poly {
            let x1 = rng.gen_range(bx0..bx1);
            let y1 = rng.gen_range(by0..by1);
            let x2 = x1 + rng.gen_range(-span_x * 0.3..span_x * 0.3);
            let y2 = y1 + rng.gen_range(-span_y * 0.3..span_y * 0.3);
            // Clamp to bbox
            let x2 = x2.clamp(bx0, bx1);
            let y2 = y2.clamp(by0, by1);

            output_features.push(json!({
                "type": "Feature",
                "properties": { "id": i + 1, "cluster_id": fid, "fid": fid },
                "geometry": { "type": "LineString", "coordinates": [[x1, y1], [x2, y2]] }
            }));
        }
    }

    let collection = json!({
        "type": "FeatureCollection",
        "name": format!("Real Lines ({} features)", output_features.len()),
        "features": output_features
    });

    fs::create_dir_all(std::path::Path::new(&output_path).parent().unwrap()).unwrap();
    fs::write(&output_path, serde_json::to_string_pretty(&collection).unwrap()).unwrap();
    println!("Generated {} line features for {} polygons to {}", output_features.len(), features.len(), output_path);
}