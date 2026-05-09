//! Benchmarks for LER Axis-Aligned solver
//!
//! Run: `cargo bench --package ige-core --test ler_axis_aligned_bench`

#[cfg(feature = "dhat")]
#[global_allocator]
static ALLOC: dhat::Alloc = dhat::Alloc;

use criterion::{criterion_group, criterion_main, Criterion};
use geo_types::{coord, LineString, Polygon};
use ige_core::solvers::ler::axis_aligned::solve_ler_axis_aligned_exact;
use ige_core::solvers::ler::LerOptions;

fn rp(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            coord! { x: x0, y: y0 },
            coord! { x: x1, y: y0 },
            coord! { x: x1, y: y1 },
            coord! { x: x0, y: y1 },
            coord! { x: x0, y: y0 },
        ]),
        vec![],
    )
}

fn unit_square() -> Polygon<f64> {
    rp(0.0, 0.0, 1.0, 1.0)
}

fn square_10x10() -> Polygon<f64> {
    rp(0.0, 0.0, 10.0, 10.0)
}

fn square_100x100() -> Polygon<f64> {
    rp(0.0, 0.0, 100.0, 100.0)
}

fn rectangle_100x10() -> Polygon<f64> {
    rp(0.0, 0.0, 100.0, 10.0)
}

fn l_shaped() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            coord! { x: 0.0, y: 0.0 },
            coord! { x: 10.0, y: 0.0 },
            coord! { x: 10.0, y: 3.0 },
            coord! { x: 3.0, y: 3.0 },
            coord! { x: 3.0, y: 10.0 },
            coord! { x: 0.0, y: 10.0 },
            coord! { x: 0.0, y: 0.0 },
        ]),
        vec![],
    )
}

fn u_shaped() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            coord! { x: 0.0, y: 0.0 },
            coord! { x: 10.0, y: 0.0 },
            coord! { x: 10.0, y: 10.0 },
            coord! { x: 7.0, y: 10.0 },
            coord! { x: 7.0, y: 4.0 },
            coord! { x: 3.0, y: 4.0 },
            coord! { x: 3.0, y: 10.0 },
            coord! { x: 0.0, y: 10.0 },
            coord! { x: 0.0, y: 0.0 },
        ]),
        vec![],
    )
}

fn with_hole() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            coord! { x: 0.0, y: 0.0 },
            coord! { x: 10.0, y: 0.0 },
            coord! { x: 10.0, y: 10.0 },
            coord! { x: 0.0, y: 10.0 },
            coord! { x: 0.0, y: 0.0 },
        ]),
        vec![LineString::from(vec![
            coord! { x: 2.0, y: 2.0 },
            coord! { x: 8.0, y: 2.0 },
            coord! { x: 8.0, y: 8.0 },
            coord! { x: 2.0, y: 8.0 },
            coord! { x: 2.0, y: 2.0 },
        ])],
    )
}

fn make_obstacles(n: usize, sz: f64) -> Vec<Polygon<f64>> {
    (0..n)
        .map(|i| rp(i as f64 * 2.0, 0.0, i as f64 * 2.0 + sz, 10.0))
        .collect()
}

fn benchmark_no_obstacles(c: &mut Criterion) {
    let mut group = c.benchmark_group("ler_no_obstacles");

    group.bench_function("unit_square_1x1", |b| {
        let poly = unit_square();
        b.iter(|| solve_ler_axis_aligned_exact(&poly, &[], &LerOptions::default()));
    });

    group.bench_function("square_10x10", |b| {
        let poly = square_10x10();
        b.iter(|| solve_ler_axis_aligned_exact(&poly, &[], &LerOptions::default()));
    });

    group.bench_function("square_100x100", |b| {
        let poly = square_100x100();
        b.iter(|| solve_ler_axis_aligned_exact(&poly, &[], &LerOptions::default()));
    });

    group.bench_function("rectangle_100x10", |b| {
        let poly = rectangle_100x10();
        b.iter(|| solve_ler_axis_aligned_exact(&poly, &[], &LerOptions::default()));
    });

    group.finish();
}

fn benchmark_concave_polygons(c: &mut Criterion) {
    let mut group = c.benchmark_group("ler_concave");

    group.bench_function("l_shaped", |b| {
        let poly = l_shaped();
        b.iter(|| solve_ler_axis_aligned_exact(&poly, &[], &LerOptions::default()));
    });

    group.bench_function("u_shaped", |b| {
        let poly = u_shaped();
        b.iter(|| solve_ler_axis_aligned_exact(&poly, &[], &LerOptions::default()));
    });

    group.bench_function("with_hole", |b| {
        let poly = with_hole();
        b.iter(|| solve_ler_axis_aligned_exact(&poly, &[], &LerOptions::default()));
    });

    group.finish();
}

fn benchmark_obstacle_counts(c: &mut Criterion) {
    let mut group = c.benchmark_group("ler_obstacle_counts");

    for n in [1, 5, 10, 20] {
        let name = format!("{} obstacles", n);
        let obstacles = make_obstacles(n, 0.1);
        group.bench_function(&name, |b| {
            let poly = square_10x10();
            b.iter(|| solve_ler_axis_aligned_exact(&poly, &obstacles, &LerOptions::default()));
        });
    }

    group.finish();
}

fn benchmark_aspect_ratio(c: &mut Criterion) {
    let mut group = c.benchmark_group("ler_aspect_ratio");
    let poly = square_10x10();

    for ratio in [0.0, 1.0, 2.0, 5.0] {
        let name = if ratio == 0.0 {
            "unlimited".to_string()
        } else {
            format!("{:.0}:1", ratio)
        };
        group.bench_function(&name, |b| {
            let opts = LerOptions {
                max_ratio: ratio,
                ..Default::default()
            };
            b.iter(|| solve_ler_axis_aligned_exact(&poly, &[], &opts));
        });
    }

    group.finish();
}

criterion_group!(
    benches,
    benchmark_no_obstacles,
    benchmark_concave_polygons,
    benchmark_obstacle_counts,
    benchmark_aspect_ratio
);
criterion_main!(benches);