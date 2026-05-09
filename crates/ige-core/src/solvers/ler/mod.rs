//! Largest Empty Rectangle (LER) solvers.
//!
//! LER finds the largest axis-aligned or oriented rectangle that fits inside
//! a polygon while remaining completely empty (containing no obstacles).
//! This is complementary to LIR (Largest Inscribed Rectangle).

pub mod axis_aligned;
pub mod oriented;

use geo_types::Polygon;
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