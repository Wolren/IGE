//! Inscribed Geometry Engine (IGE) - Largest Inscribed Rectangle algorithms

pub mod algorithms;
pub mod shared;
pub mod tuning;

pub mod solvers;

#[cfg(feature = "gpu")]
pub mod gpu;

pub use algorithms::LirSolver;
pub use solvers::lir::axis_aligned::{solve_vertex_grid, AxisAlignedOptions, detect_polygon_type};
pub use solvers::lir::axis_aligned::{
    MaskBackend,
    solve_axis_rect_bcrs_with_backend,
    solve_axis_rect_grid_with_backend,
};
pub use solvers::lir::oriented::{solve_lir_oriented, LirOrientedOptions, LirOrientedResult};
pub use solvers::lir::oriented::parallel::solve_lir_oriented_parallel;
pub use solvers::mic::{
    maximum_inscribed_circle,
    maximum_inscribed_circle_multipolygon,
    MicEngine,
    MicError,
    MicOptions,
    MicResult,
    MicUsedEngine,
    RobustMode,
};

pub use shared::{PolygonType, LirError, Result, Rectangle, SolverOptions, rotate_polygon, AlgorithmCategory, AlgorithmPrecision, AlgorithmSpeed, SolverBackend};

pub use geo_types::Polygon;

pub fn solve_oriented_lir(poly: &Polygon<f64>) -> Option<Rectangle> {
    solve_lir_oriented(poly, &LirOrientedOptions::default())
        .ok()
        .and_then(|r| r.rect)
}

pub fn solve_axis_aligned(poly: &Polygon<f64>, options: &AxisAlignedOptions) -> Option<Rectangle> {
    solvers::lir::axis_aligned::solve_vertex_grid(poly, options)
}
