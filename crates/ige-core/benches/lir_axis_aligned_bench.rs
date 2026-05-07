//! Benchmarks for LIR Axis-Aligned solver
//!
//! Run: `cargo bench --package ige-core --test lir_axis_aligned_bench`
//! Run all: `cargo bench --package ige-core`

use criterion::{criterion_group, criterion_main, Criterion};
use geo_types::{Coord, LineString, Polygon};
use ige_core::{
    solve_axis_rect_grid_with_backend,
    solve_oriented_lir,
    MaskBackend,
    SolverOptions,
};

fn make_polygon(coords: &[(f64, f64)]) -> Polygon<f64> {
    let ext: Vec<Coord<f64>> = coords.iter().map(|(x, y)| Coord { x: *x, y: *y }).collect();
    Polygon::new(LineString::from(ext), vec![])
}

fn unit_square() -> Polygon<f64> {
    make_polygon(&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
}

fn rectangle_10x1() -> Polygon<f64> {
    make_polygon(&[(0.0, 0.0), (10.0, 0.0), (10.0, 1.0), (0.0, 1.0)])
}

fn triangle() -> Polygon<f64> {
    make_polygon(&[(0.0, 0.0), (3.0, 0.0), (1.5, 3.0)])
}

fn pentagon() -> Polygon<f64> {
    make_polygon(&[(0.0, 0.0), (2.0, 0.0), (2.5, 1.5), (1.0, 2.5), (-0.5, 1.5)])
}

fn concave_l_shape() -> Polygon<f64> {
    make_polygon(&[
        (0.0, 0.0),
        (4.0, 0.0),
        (4.0, 1.0),
        (2.0, 1.0),
        (2.0, 3.0),
        (4.0, 3.0),
        (4.0, 4.0),
        (0.0, 4.0),
    ])
}

fn concave_u_shape() -> Polygon<f64> {
    make_polygon(&[
        (0.0, 0.0),
        (4.0, 0.0),
        (4.0, 4.0),
        (3.0, 4.0),
        (3.0, 1.0),
        (1.0, 1.0),
        (1.0, 4.0),
        (0.0, 4.0),
    ])
}

fn zigzag() -> Polygon<f64> {
    make_polygon(&[
        (0.0, 0.0),
        (1.0, 0.5),
        (2.0, 0.0),
        (3.0, 0.5),
        (4.0, 0.0),
        (4.0, 1.0),
        (3.0, 1.5),
        (2.0, 1.0),
        (1.0, 1.5),
        (0.0, 1.0),
    ])
}

fn regular_polygon(n: usize, radius: f64) -> Polygon<f64> {
    use std::f64::consts::PI;
    let mut coords = Vec::with_capacity(n + 1);
    for i in 0..n {
        let angle = 2.0 * PI * (i as f64) / (n as f64);
        coords.push((radius * angle.cos(), radius * angle.sin()));
    }
    coords.push(coords[0]);
    make_polygon(&coords)
}

fn benchmark_basic_shapes(c: &mut Criterion) {
    let mut group = c.benchmark_group("basic_shapes");

    group.bench_function("unit_square_1x1", |b| {
        let poly = unit_square();
        b.iter(|| solve_oriented_lir(&poly));
    });

    group.bench_function("rectangle_10x1", |b| {
        let poly = rectangle_10x1();
        b.iter(|| solve_oriented_lir(&poly));
    });

    group.bench_function("triangle", |b| {
        let poly = triangle();
        b.iter(|| solve_oriented_lir(&poly));
    });

    group.bench_function("pentagon", |b| {
        let poly = pentagon();
        b.iter(|| solve_oriented_lir(&poly));
    });

    group.finish();
}

fn benchmark_concave_polygons(c: &mut Criterion) {
    let mut group = c.benchmark_group("concave_polygons");

    group.bench_function("l_shape", |b| {
        let poly = concave_l_shape();
        b.iter(|| solve_oriented_lir(&poly));
    });

    group.bench_function("u_shape", |b| {
        let poly = concave_u_shape();
        b.iter(|| solve_oriented_lir(&poly));
    });

    group.bench_function("zigzag", |b| {
        let poly = zigzag();
        b.iter(|| solve_oriented_lir(&poly));
    });

    group.finish();
}

fn benchmark_rotation(c: &mut Criterion) {
    let mut group = c.benchmark_group("rotation");
    let poly = pentagon();

    for angle in [0.0, 15.0, 30.0, 45.0, 60.0, 90.0] {
        let _name = format!("{:.0}deg", angle);
        group.bench_function(&_name, |b| {
            let _opts = SolverOptions {
                rotation_degrees: angle,
                ..Default::default()
            };
            b.iter(|| solve_oriented_lir(&poly));
        });
    }

    group.finish();
}

fn benchmark_aspect_ratio(c: &mut Criterion) {
    let mut group = c.benchmark_group("aspect_ratio");
    let poly = pentagon();

    for ratio in [0.0, 1.0, 2.0, 5.0, 10.0] {
        let _name = if ratio == 0.0 {
            "unlimited".to_string()
        } else {
            format!("{:.0}:1", ratio)
        };
        group.bench_function(&_name, |b| {
            let _opts = SolverOptions {
                max_aspect_ratio: ratio,
                ..Default::default()
            };
            b.iter(|| solve_oriented_lir(&poly));
        });
    }

    group.finish();
}

fn benchmark_polygon_sizes(c: &mut Criterion) {
    let mut group = c.benchmark_group("polygon_sizes");
    let radius = 10.0;

    for n in [4, 6, 8, 10, 20] {
        let name = format!("{} vertices", n);
        group.bench_function(&name, |b| {
            let poly = regular_polygon(n, radius);
            b.iter(|| solve_oriented_lir(&poly));
        });
    }

    group.finish();
}

fn benchmark_axis_grid_backends(c: &mut Criterion) {
    let mut group = c.benchmark_group("axis_grid_backends");
    let poly = concave_l_shape();
    let grid_steps = 80usize;
    let max_ratio = 0.0;
    let min_ratio = 0.0;

    group.bench_function("cpu", |b| {
        b.iter(|| solve_axis_rect_grid_with_backend(&poly, grid_steps, max_ratio, min_ratio, MaskBackend::Cpu));
    });

    #[cfg(feature = "gpu")]
    group.bench_function("gpu_sdf", |b| {
        b.iter(|| solve_axis_rect_grid_with_backend(&poly, grid_steps, max_ratio, min_ratio, MaskBackend::GpuSdf));
    });

    #[cfg(feature = "gpu")]
    group.bench_function("gpu_grid", |b| {
        b.iter(|| solve_axis_rect_grid_with_backend(&poly, grid_steps, max_ratio, min_ratio, MaskBackend::GpuGridBatch));
    });

    #[cfg(feature = "gpu")]
    group.bench_function("auto", |b| {
        b.iter(|| solve_axis_rect_grid_with_backend(&poly, grid_steps, max_ratio, min_ratio, MaskBackend::Auto));
    });

    group.finish();
}

criterion_group!(
    benches,
    benchmark_basic_shapes,
    benchmark_concave_polygons,
    benchmark_rotation,
    benchmark_aspect_ratio,
    benchmark_polygon_sizes,
    benchmark_axis_grid_backends
);
criterion_main!(benches);
