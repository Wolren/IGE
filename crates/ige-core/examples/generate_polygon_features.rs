//! Generate clustered polygon obstacles for LER testing.
//!
//! Each cluster is a group of small polygons close together. The bounding box
//! of each cluster defines the LER search region, with the polygons as obstacles.
//!
//! Usage:
//!   cargo run --package ige-core --example generate_polygon_features
//!   cargo run --package ige-core --example generate_polygon_features -- --count 300 --clusters 10 --output target/ige_output/random_polygons.geojson

use rand::{Rng, SeedableRng};
use rand::rngs::StdRng;
use serde_json::{json, Value};

fn random_polygon_coords(rng: &mut StdRng, cx: f64, cy: f64, spread: f64) -> Vec<Vec<Vec<f64>>> {
    let n_sides = rng.gen_range(3..=6);
    let mut ring: Vec<Vec<f64>> = Vec::new();
    for _ in 0..n_sides {
        let x = rng.gen_range((cx - spread)..(cx + spread));
        let y = rng.gen_range((cy - spread)..(cy + spread));
        ring.push(vec![x, y]);
    }
    // Close the ring
    ring.push(ring[0].clone());
    vec![ring]
}

fn generate_cluster(
    rng: &mut StdRng,
    cx: f64,
    cy: f64,
    cluster_spread: f64,
    poly_spread: f64,
    polys_per_cluster: usize,
) -> Vec<Vec<Vec<Vec<f64>>>> {
    (0..polys_per_cluster)
        .map(|_| {
            let ox = rng.gen_range(-cluster_spread..cluster_spread);
            let oy = rng.gen_range(-cluster_spread..cluster_spread);
            random_polygon_coords(rng, cx + ox, cy + oy, poly_spread)
        })
        .collect()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut count = 9000;
    let mut num_clusters = 300;
    let mut seed = 42;
    let mut output_path = "target/ige_output/random_polygons.geojson".to_string();
    let world_bounds = (5.0, 5.0, 95.0, 95.0);
    let cluster_spread = 8.0;
    let poly_spread = 2.0;

    for i in 0..args.len() {
        if args[i] == "--count" && i + 1 < args.len() {
            count = args[i + 1].parse().unwrap_or(100);
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
    let polys_per_cluster = (count / num_clusters).max(1);
    let mut global_id = 0;
    let mut features_array: Vec<Value> = Vec::new();

    for cluster_idx in 0..num_clusters {
        let cx = rng.gen_range(world_bounds.0..world_bounds.2);
        let cy = rng.gen_range(world_bounds.1..world_bounds.3);
        let actual_count = if cluster_idx == num_clusters - 1 {
            count - global_id
        } else {
            polys_per_cluster
        };

        let polys = generate_cluster(&mut rng, cx, cy, cluster_spread, poly_spread, actual_count);

        let mut min_x = f64::MAX; let mut min_y = f64::MAX;
        let mut max_x = f64::MIN; let mut max_y = f64::MIN;
        for poly in &polys {
            for ring in poly {
                for pt in ring {
                    min_x = min_x.min(pt[0]); min_y = min_y.min(pt[1]);
                    max_x = max_x.max(pt[0]); max_y = max_y.max(pt[1]);
                }
            }
        }
        eprintln!("  Cluster {}: {} polys, bbox=({:.1},{:.1})-({:.1},{:.1})",
            cluster_idx + 1, actual_count, min_x, min_y, max_x, max_y);

        for poly_coords in &polys {
            global_id += 1;
            let feature = json!({
                "type": "Feature",
                "properties": {
                    "id": global_id,
                    "name": format!("Polygon {}", global_id),
                    "cluster_id": cluster_idx + 1
                },
                "geometry": {
                    "type": "Polygon",
                    "coordinates": poly_coords
                }
            });
            features_array.push(feature);
        }
    }

    let collection = json!({
        "type": "FeatureCollection",
        "name": "Clustered Polygon Features",
        "features": features_array
    });

    std::fs::create_dir_all(std::path::Path::new(&output_path).parent().unwrap()).unwrap();
    std::fs::write(&output_path, serde_json::to_string_pretty(&collection).unwrap()).unwrap();
    println!("Generated {} polygon features in {} clusters to {}", global_id, num_clusters, output_path);
}