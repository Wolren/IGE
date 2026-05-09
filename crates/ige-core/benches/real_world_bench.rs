//! Benchmarks using real-world GIS polygon data
//!
//! Run: `cargo bench --package ige-core --test real_world`
//! Run all: `cargo bench --package ige-core`

#[cfg(feature = "dhat")]
#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

use criterion::{criterion_group, criterion_main, Criterion};
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

pub fn load_real_world_polygons() -> Vec<Polygon<f64>> {
    let content = include_str!("../tests/real_world_data/realworld_290.geojson");
    let json: Value = serde_json::from_str(content).expect("Failed to parse realworld_290.geojson");

    let features = json.get("features").expect("No features");
    let arr = features.as_array().expect("Features is not array");

    arr.iter()
        .filter_map(|f| {
            let geom = f.get("geometry")?;
            parse_polygon(geom)
        })
        .collect()
}

pub fn get_polygon_stats() -> (usize, f64, f64) {
    let polygons = load_real_world_polygons();
    let total_area: f64 = polygons.iter().map(|p| p.unsigned_area()).sum();
    let avg_area = total_area / polygons.len() as f64;
    (polygons.len(), total_area, avg_area)
}

fn benchmark_real_world_single(c: &mut Criterion) {
    let polygons = load_real_world_polygons();
    let mut group = c.benchmark_group("real_world_single");

    for (i, poly) in polygons.iter().enumerate().take(20) {
        let name = format!("polygon_{}", i + 1);
        group.bench_function(&name, |b| {
            b.iter(|| solve_oriented_lir(poly));
        });
    }

    group.finish();
}

fn benchmark_real_world_batch(c: &mut Criterion) {
    let polygons = load_real_world_polygons();
    let mut group = c.benchmark_group("real_world_batch");

    let sizes = [10, 50, 100, 290];
    for size in sizes {
        let name = format!("{} polygons", size);
        group.bench_function(&name, |b| {
            b.iter(|| {
                for poly in polygons.iter().take(size) {
                    solve_oriented_lir(poly);
                }
            });
        });
    }

    group.finish();
}

fn benchmark_real_world_all(c: &mut Criterion) {
    let polygons = load_real_world_polygons();
    c.bench_function("all_290_polygons", |b| {
        b.iter(|| {
            for poly in &polygons {
                solve_oriented_lir(poly);
            }
        });
    });
}

criterion_group!(
    benches,
    benchmark_real_world_single,
    benchmark_real_world_batch,
    benchmark_real_world_all
);
criterion_main!(benches);
