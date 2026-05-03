//! CPU-based solvers for oriented largest inscribed rectangles.

use geo_types::Polygon;

pub use crate::shared::{Rectangle, SolverOptions};

pub use crate::solvers::lir::axis_aligned::{
    AxisAlignedOptions,
    detect_polygon_type,
};

pub use crate::geometry::rotate_polygon;
pub use crate::solvers::mic::{maximum_inscribed_circle, maximum_inscribed_circle_multipolygon, MicEngine, MicError, MicOptions, MicResult, MicUsedEngine, RobustMode};

pub use crate::solvers::lir::approximate::solve_lir_approximate_oriented;
pub use crate::solvers::lir::approximate::parallel::solve_lir_approximate_oriented_parallel;
pub use crate::solvers::lir::approximate::{LirApproxOrientedOptions, LirApproxOrientedResult};

/// Convenience wrapper: solve axis-aligned with default options.
/// For full control use `AxisAlignedOptions` with `solvers::lir::axis_aligned::solve_vertex_grid`.
pub fn solve_oriented_lir(poly: &Polygon<f64>) -> Option<Rectangle> {
    crate::solvers::lir::axis_aligned::solve_vertex_grid(poly, &AxisAlignedOptions::default())
}

/// Solve the largest axis-aligned rectangle in a polygon.
/// Uses the vertex-coordinate grid solver (cell-center PIP + LRIH + contraction).
pub fn solve_axis_aligned(poly: &Polygon<f64>, options: &AxisAlignedOptions) -> Option<Rectangle> {
    crate::solvers::lir::axis_aligned::solve_vertex_grid(poly, options)
}
