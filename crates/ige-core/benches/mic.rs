//! MIC benchmarks with criterion HTML chart output.
//!
//! Run: cargo bench --package ige-core --features geos -- mic
//! Then open target/criterion/mic*/report/index.html in a browser.

use criterion::{criterion_group, criterion_main, Criterion};
use geo_types::{Coord, LineString, Polygon};
use ige_core::solvers::mic::{maximum_inscribed_circle, MicEngine, MicOptions, RobustMode};

fn make_polygon(exterior: &[(f64, f64)], holes: &[&[(f64, f64)]]) -> Polygon<f64> {
    let ext = LineString::from(
        exterior.iter()
            .map(|(x, y)| Coord { x: *x, y: *y })
            .collect::<Vec<_>>(),
    );
    let interiors = holes.iter()
        .map(|ring| LineString::from(
            ring.iter().map(|(x, y)| Coord { x: *x, y: *y }).collect::<Vec<_>>(),
        ))
        .collect::<Vec<_>>();
    Polygon::new(ext, interiors)
}

fn fixtures() -> Vec<(&'static str, Polygon<f64>)> {
    vec![
        ("20x20 square",
            make_polygon(&[(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0), (0.0, 0.0)], &[])),
        ("C-shape concave",
            make_polygon(&[
                (0.0, 0.0), (14.0, 0.0), (14.0, 12.0), (9.0, 12.0),
                (9.0, 4.0), (5.0, 4.0), (5.0, 12.0), (0.0, 12.0), (0.0, 0.0),
            ], &[])),
        ("rect with hole",
            make_polygon(&[(0.0, 0.0), (30.0, 0.0), (30.0, 22.0), (0.0, 22.0), (0.0, 0.0)],
                &[&[(9.0, 7.0), (21.0, 7.0), (21.0, 15.0), (9.0, 15.0), (9.0, 7.0)]])),
        ("40x4 thin rect",
            make_polygon(&[(0.0, 0.0), (40.0, 0.0), (40.0, 4.0), (0.0, 4.0), (0.0, 0.0)], &[])),
        ("regular hexagon",
            make_polygon(&[
                (10.0, 0.0), (5.0, 8.66), (-5.0, 8.66), (-10.0, 0.0),
                (-5.0, -8.66), (5.0, -8.66), (10.0, 0.0),
            ], &[])),
        ("right triangle",
            make_polygon(&[(0.0, 0.0), (12.0, 0.0), (0.0, 9.0), (0.0, 0.0)], &[])),
    ]
}

fn exact_opts() -> MicOptions {
    MicOptions { engine: MicEngine::ExactOnly, robust_mode: RobustMode::Filtered }
}

#[cfg(feature = "geos")]
fn geos_opts() -> MicOptions {
    MicOptions { engine: MicEngine::FallbackOnly, robust_mode: RobustMode::Filtered }
}

fn bench_mic_exact(c: &mut Criterion) {
    let mut group = c.benchmark_group("mic");
    group.sample_size(100);

    for (name, poly) in fixtures() {
        group.bench_function(format!("exact/{}", name), |b| {
            b.iter(|| maximum_inscribed_circle(&poly, &exact_opts()));
        });
    }

    group.finish();
}

#[cfg(feature = "geos")]
fn bench_mic_geos(c: &mut Criterion) {
    let mut group = c.benchmark_group("mic");
    group.sample_size(100);

    for (name, poly) in fixtures() {
        group.bench_function(format!("geos/{}", name), |b| {
            b.iter(|| maximum_inscribed_circle(&poly, &geos_opts()));
        });
    }

    group.finish();
}

criterion_group!(benches, bench_mic_exact);

#[cfg(feature = "geos")]
criterion_group!(geos_benches, bench_mic_geos);

#[cfg(feature = "geos")]
criterion_main!(benches, geos_benches);

#[cfg(not(feature = "geos"))]
criterion_main!(benches);
