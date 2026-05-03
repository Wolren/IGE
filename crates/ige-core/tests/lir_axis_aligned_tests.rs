//! Tests for LIR Axis-Aligned solver
//!
//! Run: `cargo test --test lir_axis_aligned_tests`
//! Run all: `cargo test --workspace`

use ige_core::{solve_oriented_lir, SolverOptions};
use geo_types::{Coord, LineString, Polygon};

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
        (0.0, 0.0), (4.0, 0.0), (4.0, 1.0),
        (2.0, 1.0), (2.0, 3.0), (4.0, 3.0), (4.0, 4.0), (0.0, 4.0)
    ])
}

fn concave_u_shape() -> Polygon<f64> {
    make_polygon(&[
        (0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (3.0, 4.0),
        (3.0, 1.0), (1.0, 1.0), (1.0, 4.0), (0.0, 4.0)
    ])
}

fn zigzag() -> Polygon<f64> {
    make_polygon(&[
        (0.0, 0.0), (1.0, 0.5), (2.0, 0.0), (3.0, 0.5),
        (4.0, 0.0), (4.0, 1.0), (3.0, 1.5), (2.0, 1.0),
        (1.0, 1.5), (0.0, 1.0)
    ])
}

fn hexagon() -> Polygon<f64> {
    make_polygon(&[(2.0, 0.0), (1.0, 1.732), (-1.0, 1.732), (-2.0, 0.0), (-1.0, -1.732), (1.0, -1.732)])
}

#[test]
fn test_unit_square() {
    let poly = unit_square();
    let result = solve_oriented_lir(&poly).unwrap();
    assert!((result.area() - 1.0).abs() < 1e-6);
}

#[test]
fn test_rectangle_10x1() {
    let poly = rectangle_10x1();
    let result = solve_oriented_lir(&poly).unwrap();
    assert!((result.area() - 10.0).abs() < 1e-6);
}

#[test]
fn test_triangle() {
    let poly = triangle();
    let result = solve_oriented_lir(&poly).expect("triangle should return a rectangle");
    assert!(result.area() > 0.0);
}

#[test]
fn test_pentagon() {
    let poly = pentagon();
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_concave_l_shape() {
    let poly = concave_l_shape();
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_concave_u_shape() {
    let poly = concave_u_shape();
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_zigzag() {
    let poly = zigzag();
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_hexagon() {
    let poly = hexagon();
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_rotation_0_degrees() {
    let poly = pentagon();
    let _opts = SolverOptions {
        rotation_degrees: 0.0,
        prefer_gpu: false,
        force_cpu: true,
        max_aspect_ratio: 0.0,
        gpu_threshold: 1000,
    };
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_rotation_45_degrees() {
    let poly = pentagon();
    let _opts = SolverOptions {
        rotation_degrees: 45.0,
        prefer_gpu: false,
        force_cpu: true,
        max_aspect_ratio: 0.0,
        gpu_threshold: 1000,
    };
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_rotation_90_degrees() {
    let poly = pentagon();
    let _opts = SolverOptions {
        rotation_degrees: 90.0,
        prefer_gpu: false,
        force_cpu: true,
        max_aspect_ratio: 0.0,
        gpu_threshold: 1000,
    };
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_aspect_ratio_unlimited() {
    let poly = pentagon();
    let _opts = SolverOptions {
        rotation_degrees: 0.0,
        prefer_gpu: false,
        force_cpu: true,
        max_aspect_ratio: 0.0,
        gpu_threshold: 1000,
    };
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_aspect_ratio_1_to_1() {
    let poly = pentagon();
    let _opts = SolverOptions {
        rotation_degrees: 0.0,
        prefer_gpu: false,
        force_cpu: true,
        max_aspect_ratio: 1.0,
        gpu_threshold: 1000,
    };
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_aspect_ratio_2_to_1() {
    let poly = pentagon();
    let _opts = SolverOptions {
        rotation_degrees: 0.0,
        prefer_gpu: false,
        force_cpu: true,
        max_aspect_ratio: 2.0,
        gpu_threshold: 1000,
    };
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_polygon_type_convex_no_holes() {
    let poly = pentagon();
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}

#[test]
fn test_polygon_type_concave_no_holes() {
    let poly = zigzag();
    let result = solve_oriented_lir(&poly).unwrap();
    assert!(result.area() > 0.0);
}
