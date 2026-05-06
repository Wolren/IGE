//! Benchmarks for LIR Oriented solver
//!
//! Run: `cargo bench --package ige-core --test lir_oriented_bench`
//! Run all: `cargo bench --package ige-core`

use criterion::{criterion_group, criterion_main, Criterion};
use geo_types::{coord, LineString, Polygon};
use ige_core::solvers::lir::oriented::{
    solve_lir_oriented, LirOrientedOptions,
};

fn test_shapes() -> Vec<Polygon<f64>> {
    vec![
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:20.0, y:0.0},
                coord! {x:20.0, y:5.0},
                coord! {x:0.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        ),
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:8.0, y:0.0},
                coord! {x:8.0, y:2.0},
                coord! {x:3.0, y:2.0},
                coord! {x:3.0, y:8.0},
                coord! {x:0.0, y:8.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        ),
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:1.0},
                coord! {x:11.0, y:8.0},
                coord! {x:4.0, y:11.0},
                coord! {x:-1.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        ),
    ]
}

fn benchmark_lir_oriented_standard(c: &mut Criterion) {
    let shapes = test_shapes();
    let mut opts = LirOrientedOptions::default();
    opts.use_parallel_field = false;

    c.bench_function("lir_oriented_standard_batch", |b| {
        b.iter(|| {
            for poly in &shapes {
                let _ = solve_lir_oriented(poly, &opts);
            }
        });
    });
}

fn benchmark_lir_oriented_parallel(c: &mut Criterion) {
    let shapes = test_shapes();
    let mut opts = LirOrientedOptions::default();
    opts.use_parallel_field = true;

    c.bench_function("lir_oriented_parallel_batch", |b| {
        b.iter(|| {
            for poly in &shapes {
                let _ = solve_lir_oriented(poly, &opts);
            }
        });
    });
}

fn benchmark_lir_oriented_sa(c: &mut Criterion) {
    let shapes = test_shapes();
    let mut opts = LirOrientedOptions::default();
    opts.use_simulated_annealing = true;

    c.bench_function("lir_oriented_sa_batch", |b| {
        b.iter(|| {
            for poly in &shapes {
                let _ = solve_lir_oriented(poly, &opts);
            }
        });
    });
}

criterion_group!(
    benches,
    benchmark_lir_oriented_standard,
    benchmark_lir_oriented_parallel,
    benchmark_lir_oriented_sa
);
criterion_main!(benches);
