//! Generate clustered line features for LER testing.
//!
//! Each cluster is a group of lines close together. The bounding box of each
//! cluster defines the LER search region.
//!
//! Usage:
//!   cargo run --package ige-core --example generate_line_features
//!   cargo run --package ige-core --example generate_line_features -- --count 300 --clusters 10 --output target/ige_output/random_lines.geojson

use rand::{Rng, SeedableRng};
use rand::rngs::StdRng;

fn line_length(p1: &[f64; 2], p2: &[f64; 2]) -> f64 {
    let dx = p2[0] - p1[0];
    let dy = p2[1] - p1[1];
    (dx * dx + dy * dy).sqrt()
}

fn generate_cluster(
    rng: &mut StdRng,
    cx: f64,
    cy: f64,
    spread: f64,
    lines_per_cluster: usize,
    min_length: f64,
) -> Vec<Vec<Vec<f64>>> {
    let mut lines = Vec::new();
    for _ in 0..lines_per_cluster {
        let mut attempts = 0;
        loop {
            let x1 = rng.gen_range((cx - spread)..(cx + spread));
            let y1 = rng.gen_range((cy - spread)..(cy + spread));
            let x2 = rng.gen_range((cx - spread)..(cx + spread));
            let y2 = rng.gen_range((cy - spread)..(cy + spread));
            let len = line_length(&[x1, y1], &[x2, y2]);
            attempts += 1;
            if len >= min_length || attempts > 100 {
                lines.push(vec![vec![x1, y1], vec![x2, y2]]);
                break;
            }
        }
    }
    lines
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut count = 9000;
    let mut num_clusters = 300;
    let mut seed = 42;
    let mut output_path = "target/ige_output/random_lines.geojson".to_string();
    let world_bounds = (5.0, 5.0, 95.0, 95.0);
    let cluster_spread = 8.0;
    let min_length = 3.0;

    for i in 0..args.len() {
        if args[i] == "--count" && i + 1 < args.len() {
            count = args[i + 1].parse().unwrap_or(300);
        }
        if args[i] == "--clusters" && i + 1 < args.len() {
            num_clusters = args[i + 1].parse().unwrap_or(10);
        }
        if args[i] == "--seed" && i + 1 < args.len() {
            seed = args[i + 1].parse().unwrap_or(42);
        }
        if args[i] == "--output" && i + 1 < args.len() {
            output_path = args[i + 1].clone();
        }
    }

    let mut rng = StdRng::seed_from_u64(seed);
    let lines_per_cluster = (count / num_clusters).max(1);
    let mut global_id = 0;
    let mut features_array: Vec<serde_json::Value> = Vec::new();

    let mut cluster_stats: Vec<(String, f64, f64, f64, f64)> = Vec::new();

    for cluster_idx in 0..num_clusters {
        let cx = rng.gen_range(world_bounds.0..world_bounds.2);
        let cy = rng.gen_range(world_bounds.1..world_bounds.3);
        let actual_count = if cluster_idx == num_clusters - 1 {
            count - global_id
        } else {
            lines_per_cluster
        };

        let lines = generate_cluster(&mut rng, cx, cy, cluster_spread, actual_count, min_length);

        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;

        for line in &lines {
            for pt in line {
                min_x = min_x.min(pt[0]);
                min_y = min_y.min(pt[1]);
                max_x = max_x.max(pt[0]);
                max_y = max_y.max(pt[1]);
            }
        }

        cluster_stats.push((
            format!("Cluster {}", cluster_idx + 1),
            min_x, min_y, max_x, max_y,
        ));

        for line in &lines {
            global_id += 1;
            let mut feature_map = serde_json::Map::new();
            feature_map.insert("type".to_string(), serde_json::Value::String("Feature".to_string()));

            let mut properties = serde_json::Map::new();
            properties.insert("id".to_string(), serde_json::Value::Number(serde_json::Number::from(global_id)));
            properties.insert("name".to_string(), serde_json::Value::String(format!("Line {}", global_id)));
            properties.insert("cluster_id".to_string(), serde_json::Value::Number(serde_json::Number::from(cluster_idx + 1)));
            feature_map.insert("properties".to_string(), serde_json::Value::Object(properties));

            let mut geometry = serde_json::Map::new();
            geometry.insert("type".to_string(), serde_json::Value::String("LineString".to_string()));

            let coord_array: Vec<serde_json::Value> = line.iter().map(|c| {
                serde_json::Value::Array(
                    c.iter().map(|v| serde_json::Value::Number(serde_json::Number::from_f64(*v).unwrap())).collect()
                )
            }).collect();
            geometry.insert("coordinates".to_string(), serde_json::Value::Array(coord_array));
            feature_map.insert("geometry".to_string(), serde_json::Value::Object(geometry));

            features_array.push(serde_json::Value::Object(feature_map));
        }
    }

    let mut collection_map = serde_json::Map::new();
    collection_map.insert("type".to_string(), serde_json::Value::String("FeatureCollection".to_string()));
    collection_map.insert("name".to_string(), serde_json::Value::String("Clustered Line Features".to_string()));

    let mut metadata = serde_json::Map::new();
    metadata.insert("num_clusters".to_string(), serde_json::Value::Number(serde_json::Number::from(num_clusters)));
    metadata.insert("cluster_spread".to_string(), serde_json::Value::Number(serde_json::Number::from_f64(cluster_spread).unwrap()));
    let stats: Vec<serde_json::Value> = cluster_stats.iter().map(|(name, x0, y0, x1, y1)| {
        serde_json::json!({
            "name": name,
            "bbox": [x0, y0, x1, y1]
        })
    }).collect();
    metadata.insert("clusters".to_string(), serde_json::Value::Array(stats));
    collection_map.insert("metadata".to_string(), serde_json::Value::Object(metadata));
    collection_map.insert("features".to_string(), serde_json::Value::Array(features_array));

    let collection = serde_json::Value::Object(collection_map);

    std::fs::create_dir_all(std::path::Path::new(&output_path).parent().unwrap()).unwrap();
    let json_str = serde_json::to_string_pretty(&collection).unwrap();
    std::fs::write(&output_path, &json_str).unwrap();

    println!("Generated {} line features in {} clusters to {}", global_id, num_clusters, output_path);
    for (name, x0, y0, x1, y1) in &cluster_stats {
        println!("  {}: bbox=({:.1},{:.1})-({:.1},{:.1})", name, x0, y0, x1, y1);
    }
}