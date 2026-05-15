//! Tests for LER with line obstacles - 300 random line features
//!
//! Run: `cargo test --test ler_line_obstacles_tests`

use geo_types::{coord, LineString, Polygon};
use ige_core::solve_ler_axis_aligned_with_lines;
use rand::{Rng, SeedableRng};
use rand::rngs::StdRng;

fn sample_polygon() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            coord! { x: 0.0, y: 0.0 },
            coord! { x: 100.0, y: 0.0 },
            coord! { x: 100.0, y: 100.0 },
            coord! { x: 0.0, y: 100.0 },
            coord! { x: 0.0, y: 0.0 },
        ]),
        vec![],
    )
}

fn generate_random_line(rng: &mut StdRng, bounds: (f64, f64, f64, f64)) -> LineString<f64> {
    let (min_x, min_y, max_x, max_y) = bounds;
    let x1 = rng.gen_range(min_x..max_x);
    let y1 = rng.gen_range(min_y..max_y);
    let x2 = rng.gen_range(min_x..max_x);
    let y2 = rng.gen_range(min_y..max_y);
    LineString::from(vec![coord! { x: x1, y: y1 }, coord! { x: x2, y: y2 }])
}

fn generate_random_lines(count: usize, seed: u64, bounds: (f64, f64, f64, f64)) -> Vec<LineString<f64>> {
    let mut rng = StdRng::seed_from_u64(seed);
    (0..count).map(|_| generate_random_line(&mut rng, bounds)).collect()
}

#[test]
fn test_ler_with_300_random_line_obstacles() {
    let poly = sample_polygon();
    let line_obstacles = generate_random_lines(300, 42, (0.0, 0.0, 100.0, 100.0));
    let result = solve_ler_axis_aligned_with_lines(&poly, &[], &line_obstacles, 1.0, &opts());
    assert!(result.is_ok());
    let result = result.unwrap();
    assert!(result.area > 0.0, "Expected positive area, got {}", result.area);
}

#[test]
fn test_ler_line_obstacles_smaller_space() {
    let poly = Polygon::new(
        LineString::from(vec![
            coord! { x: 0.0, y: 0.0 },
            coord! { x: 50.0, y: 0.0 },
            coord! { x: 50.0, y: 50.0 },
            coord! { x: 0.0, y: 50.0 },
            coord! { x: 0.0, y: 0.0 },
        ]),
        vec![],
    );
    let line_obstacles = generate_random_lines(300, 123, (0.0, 0.0, 50.0, 50.0));
    let result = solve_ler_axis_aligned_with_lines(&poly, &[], &line_obstacles, 0.5, &opts());
    assert!(result.is_ok());
    let result = result.unwrap();
    assert!(result.area > 0.0, "Expected positive area, got {}", result.area);
}

#[test]
fn test_ler_mixed_polygon_and_line_obstacles() {
    let poly = sample_polygon();
    let polygon_obstacles = vec![
        Polygon::new(LineString::from(vec![
            coord! { x: 10.0, y: 10.0 },
            coord! { x: 20.0, y: 10.0 },
            coord! { x: 20.0, y: 20.0 },
            coord! { x: 10.0, y: 20.0 },
            coord! { x: 10.0, y: 10.0 },
        ]), vec![]),
    ];
    let line_obstacles = generate_random_lines(300, 456, (0.0, 0.0, 100.0, 100.0));
    let result = solve_ler_axis_aligned_with_lines(&poly, &polygon_obstacles, &line_obstacles, 1.0, &opts());
    assert!(result.is_ok());
    let result = result.unwrap();
    assert!(result.area >= 0.0, "Expected non-negative area");
}

#[test]
fn test_ler_lines_dense_coverage() {
    let poly = Polygon::new(
        LineString::from(vec![
            coord! { x: 0.0, y: 0.0 },
            coord! { x: 30.0, y: 0.0 },
            coord! { x: 30.0, y: 30.0 },
            coord! { x: 0.0, y: 30.0 },
            coord! { x: 0.0, y: 0.0 },
        ]),
        vec![],
    );
    let line_obstacles = generate_random_lines(300, 789, (0.0, 0.0, 30.0, 30.0));
    let result = solve_ler_axis_aligned_with_lines(&poly, &[], &line_obstacles, 2.0, &opts());
    assert!(result.is_ok());
    let result = result.unwrap();
    eprintln!("Dense coverage result area: {}", result.area);
}

#[test]
fn test_ler_various_thicknesses() {
    let poly = sample_polygon();
    let line_obstacles = generate_random_lines(100, 999, (0.0, 0.0, 100.0, 100.0));
    for thickness in [0.1, 0.5, 1.0, 2.0, 5.0] {
        let result = solve_ler_axis_aligned_with_lines(&poly, &[], &line_obstacles, thickness, &opts());
        assert!(result.is_ok(), "Failed with thickness {}", thickness);
        let result = result.unwrap();
        eprintln!("Thickness {}: area {}", thickness, result.area);
    }
}

#[test]
fn test_ler_reproducibility_with_seed() {
    let line_obstacles = generate_random_lines(300, 11111, (0.0, 0.0, 100.0, 100.0));
    let poly = sample_polygon();
    let result1 = solve_ler_axis_aligned_with_lines(&poly, &[], &line_obstacles, 1.0, &opts()).unwrap();
    let result2 = solve_ler_axis_aligned_with_lines(&poly, &[], &line_obstacles, 1.0, &opts()).unwrap();
    assert!((result1.area - result2.area).abs() < 1e-10, "Results should be reproducible");
}

#[test]
fn test_ler_line_obstacles_boundary() {
    let poly = sample_polygon();
    let boundary_lines = vec![
        LineString::from(vec![coord! { x: 25.0, y: 0.0 }, coord! { x: 25.0, y: 100.0 }]),
        LineString::from(vec![coord! { x: 75.0, y: 0.0 }, coord! { x: 75.0, y: 100.0 }]),
        LineString::from(vec![coord! { x: 0.0, y: 25.0 }, coord! { x: 100.0, y: 25.0 }]),
        LineString::from(vec![coord! { x: 0.0, y: 75.0 }, coord! { x: 100.0, y: 75.0 }]),
    ];
    let result = solve_ler_axis_aligned_with_lines(&poly, &[], &boundary_lines, 1.0, &opts()).unwrap();
    assert!(result.area > 0.0);
    eprintln!("Boundary lines area: {}", result.area);
}

#[test]
fn test_ler_300_lines_stress() {
    let poly = sample_polygon();
    for seed in 0..5 {
        let line_obstacles = generate_random_lines(300, seed * 1000, (0.0, 0.0, 100.0, 100.0));
        let result = solve_ler_axis_aligned_with_lines(&poly, &[], &line_obstacles, 1.0, &opts());
        assert!(result.is_ok(), "Failed with seed {}", seed);
        let result = result.unwrap();
        eprintln!("Seed {}: area {}", seed, result.area);
    }
}

fn opts() -> ige_core::LerOptions {
    ige_core::LerOptions {
        max_ratio: 0.0,
        min_ratio: 0.0,
        grid_coarse: 60,
        top_k: 5,
        always_return: true,
    }
}

#[allow(dead_code)]
fn solve_ler_with_lines(
    poly: &Polygon<f64>,
    polygon_obstacles: &[Polygon<f64>],
    line_obstacles: &[LineString<f64>],
    line_thickness: f64,
) -> ige_core::Result<ige_core::LerResult> {
    solve_ler_axis_aligned_with_lines(poly, polygon_obstacles, line_obstacles, line_thickness, &opts())
}