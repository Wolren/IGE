//! Tests for Maximum Inscribed Circle (MIC) solver
//!
//! Run: `cargo test --test mic_tests`
//! Run all: `cargo test --workspace`

use geo::Contains;
use geo_types::{Coord, LineString, MultiPolygon, Polygon};
use ige_core::solvers::mic::{
    maximum_inscribed_circle, maximum_inscribed_circle_multipolygon, MicEngine, MicOptions,
    RobustMode,
};

fn make_polygon(coords: &[(f64, f64)]) -> Polygon<f64> {
    let ext: Vec<Coord<f64>> = coords.iter().map(|(x, y)| Coord { x: *x, y: *y }).collect();
    Polygon::new(LineString::from(ext), vec![])
}

fn square(size: f64) -> Polygon<f64> {
    make_polygon(&[(0.0, 0.0), (size, 0.0), (size, size), (0.0, size), (0.0, 0.0)])
}

fn filtered_exact() -> MicOptions {
    MicOptions {
        engine: MicEngine::ExactOnly,
        robust_mode: RobustMode::Filtered,
    }
}

#[test]
fn mic_square_exact_radius_and_center() {
    let poly = square(10.0);
    let result = maximum_inscribed_circle(&poly, &filtered_exact()).expect("MIC solve failed");
    assert!((result.radius - 5.0).abs() < 1e-8);
    assert!((result.center.x() - 5.0).abs() < 1e-8);
    assert!((result.center.y() - 5.0).abs() < 1e-8);
}

#[test]
fn mic_rectangle_exact_radius() {
    let poly = make_polygon(&[(0.0, 0.0), (8.0, 0.0), (8.0, 2.0), (0.0, 2.0), (0.0, 0.0)]);
    let result = maximum_inscribed_circle(&poly, &filtered_exact()).expect("MIC solve failed");
    assert!((result.radius - 1.0).abs() < 1e-8);
    assert!((result.center.y() - 1.0).abs() < 1e-8);
}

#[test]
fn mic_hole_polygon_center_is_in_domain() {
    let outer = LineString::from(vec![
        Coord { x: 0.0, y: 0.0 },
        Coord { x: 12.0, y: 0.0 },
        Coord { x: 12.0, y: 12.0 },
        Coord { x: 0.0, y: 12.0 },
        Coord { x: 0.0, y: 0.0 },
    ]);
    let hole = LineString::from(vec![
        Coord { x: 4.0, y: 4.0 },
        Coord { x: 8.0, y: 4.0 },
        Coord { x: 8.0, y: 8.0 },
        Coord { x: 4.0, y: 8.0 },
        Coord { x: 4.0, y: 4.0 },
    ]);
    let poly = Polygon::new(outer, vec![hole]);
    let result = maximum_inscribed_circle(&poly, &filtered_exact()).expect("MIC solve failed");
    assert!(poly.contains(&result.center), "center must be in polygon domain");
    assert!(result.radius > 0.0);
}

#[test]
fn mic_multipolygon_selects_largest_component() {
    let multi = MultiPolygon(vec![square(2.0), square(8.0)]);
    let result = maximum_inscribed_circle_multipolygon(&multi, &filtered_exact())
        .expect("MIC multipolygon solve failed");
    assert_eq!(result.component_index, Some(1));
    assert!((result.radius - 4.0).abs() < 1e-8);
}
