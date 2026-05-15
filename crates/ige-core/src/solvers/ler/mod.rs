//! Largest Empty Rectangle (LER) solvers.
//!
//! LER finds the largest axis-aligned or oriented rectangle that fits inside
//! a polygon while remaining completely empty (containing no obstacles).
//! This is complementary to LIR (Largest Inscribed Rectangle).

pub mod axis_aligned;
pub mod oriented;

use geo_types::{Polygon, LineString};
use crate::shared::{Rectangle, Result};

/// Configuration for LER solvers.
#[derive(Debug, Clone)]
pub struct LerOptions {
    /// Max aspect ratio (longer/shorter side); 0.0 = unconstrained.
    pub max_ratio: f64,
    /// Min aspect ratio (longer/shorter side); 0.0 = unconstrained.
    pub min_ratio: f64,
    /// Grid resolution for coarse search.
    pub grid_coarse: usize,
    /// Number of top candidates to refine.
    pub top_k: usize,
    /// If true, return best-effort result even if certification fails.
    pub always_return: bool,
}

impl Default for LerOptions {
    fn default() -> Self {
        Self {
            max_ratio: 0.0,
            min_ratio: 0.0,
            grid_coarse: 60,
            top_k: 5,
            always_return: true,
        }
    }
}

/// Result of an LER solve.
#[derive(Debug, Clone)]
pub struct LerResult {
    /// The largest empty rectangle (axis-aligned bounding box).
    pub rect: Option<Rectangle>,
    /// The oriented rectangle as a polygon (if oriented).
    pub rect_polygon: Option<Polygon<f64>>,
    /// Area of the empty rectangle.
    pub area: f64,
    /// Rotation angle in degrees (for oriented version).
    pub angle_deg: f64,
    /// True if result is best-effort rather than certified.
    pub best_effort: bool,
}

impl LerResult {
    pub fn empty() -> Self {
        Self {
            rect: None,
            rect_polygon: None,
            area: 0.0,
            angle_deg: 0.0,
            best_effort: false,
        }
    }
}

impl Default for LerResult {
    fn default() -> Self {
        Self::empty()
    }
}

/// Solve largest empty rectangle with axis-aligned constraints.
///
/// # Arguments
/// * `poly` - Input polygon defining the free space
/// * `obstacles` - Optional collection of obstacle polygons to avoid
/// * `options` - Solver configuration
///
/// # Returns
/// A `LerResult` with the largest empty rectangle.
pub fn solve_ler_axis_aligned(
    poly: &Polygon<f64>,
    obstacles: &[Polygon<f64>],
    options: &LerOptions,
) -> Result<LerResult> {
    super::ler::axis_aligned::solve_ler_axis_aligned_exact(poly, obstacles, options)
}

/// Solve largest empty rectangle with axis-aligned constraints, including line obstacles.
///
/// # Arguments
/// * `poly` - Input polygon defining the free space
/// * `polygon_obstacles` - Optional collection of obstacle polygons to avoid
/// * `line_obstacles` - Optional collection of line obstacles to avoid
/// * `line_thickness` - Thickness to use for line obstacles (creates buffered obstacles)
/// * `options` - Solver configuration
///
/// # Returns
/// A `LerResult` with the largest empty rectangle.
pub fn solve_ler_axis_aligned_with_lines(
    poly: &Polygon<f64>,
    polygon_obstacles: &[Polygon<f64>],
    line_obstacles: &[LineString<f64>],
    line_thickness: f64,
    options: &LerOptions,
) -> Result<LerResult> {
    super::ler::axis_aligned::solve_ler_axis_aligned_with_lines(poly, polygon_obstacles, line_obstacles, line_thickness, options)
}

/// Solve using exact O(n log² n) divide-and-conquer for point obstacles.
pub fn solve_ler_axis_aligned_points_dc(
    poly: &Polygon<f64>,
    points: &[geo_types::Coord<f64>],
    options: &LerOptions,
) -> Result<LerResult> {
    super::ler::axis_aligned::point_dc::solve_ler_points_dc(poly, points, options)
}

/// Solve using the O(n log n) plane-sweep algorithm for point obstacles.
/// Uses a balanced BST to track y-intervals, sweeping x left-to-right.
/// `points` are the obstacle point coordinates.
pub fn solve_ler_axis_aligned_points_sweep(
    poly: &Polygon<f64>,
    points: &[geo_types::Coord<f64>],
    options: &LerOptions,
) -> Result<LerResult> {
    super::ler::axis_aligned::point_sweep::solve_ler_points_sweep(poly, points, options)
}

/// Solve with exact line obstacles (no thickness approximation).
/// Each LineString segment blocks rectangles precisely along its intersection.
pub fn solve_ler_axis_aligned_with_lines_exact(
    poly: &Polygon<f64>,
    polygon_obstacles: &[Polygon<f64>],
    line_obstacles: &[LineString<f64>],
    options: &LerOptions,
) -> Result<LerResult> {
    let mut inputs: Vec<axis_aligned::ObstacleInput> = Vec::new();
    for p in polygon_obstacles {
        inputs.push(axis_aligned::ObstacleInput::Polygon(p.clone()));
    }
    for ls in line_obstacles {
        let pts: Vec<geo_types::Coord<f64>> = ls.coords().copied().collect();
        for pair in pts.windows(2) {
            let line = LineString::from(vec![pair[0], pair[1]]);
            inputs.push(axis_aligned::ObstacleInput::Line(line));
        }
    }
    solve_ler_axis_aligned_mixed(poly, &inputs, options)
}

/// Unified solver: accepts points, lines, and polygons as obstacles with automatic detection.
///
/// # Arguments
/// * `poly` - Input polygon defining the free space
/// * `obstacles` - Mixed obstacle types (`ObstacleInput::Point`, `ObstacleInput::Line`, `ObstacleInput::Polygon`)
/// * `options` - Solver configuration
///
/// # Returns
/// A `LerResult` with the largest empty rectangle.
pub fn solve_ler_axis_aligned_mixed(
    poly: &Polygon<f64>,
    obstacles: &[axis_aligned::ObstacleInput],
    options: &LerOptions,
) -> Result<LerResult> {
    super::ler::axis_aligned::solve_ler_axis_aligned_mixed(poly, obstacles, options)
}

/// Solve largest empty rectangle with free orientation.
///
/// # Arguments
/// * `poly` - Input polygon defining the free space
/// * `obstacles` - Optional collection of obstacle polygons to avoid
/// * `options` - Solver configuration
///
/// # Returns
/// A `LerResult` with the largest empty rectangle.
pub fn solve_ler_oriented(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerOptions,
) -> Result<LerResult> {
    Err(crate::shared::LirError::NotSupported("LER oriented not yet implemented".to_string()))
}