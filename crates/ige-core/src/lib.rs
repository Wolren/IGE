//! Inscribed Geometry Engine (IGE) - Largest Inscribed Rectangle algorithms

#![cfg_attr(feature = "simd", feature(portable_simd))]

pub mod algorithms;
pub mod shared;
pub mod tuning;

pub mod solvers;

#[cfg(feature = "gpu")]
pub mod gpu;
mod prelude;

pub use algorithms::LirSolver;

// LIR solvers
pub use solvers::lir::axis_aligned::{solve_vertex_grid, AxisAlignedOptions, detect_polygon_type};
pub use solvers::lir::axis_aligned::{
    MaskBackend,
    solve_axis_rect_bcrs_with_backend,
    solve_axis_rect_grid_with_backend,
};
pub use solvers::lir::oriented::{solve_lir_oriented, LirOrientedOptions, LirOrientedResult};
pub use solvers::lir::oriented::parallel::solve_lir_oriented_parallel;

// MIC solvers
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

// LER solvers
pub use solvers::ler::{LerOptions, LerResult, solve_ler_axis_aligned, solve_ler_oriented};

// Nesting solvers
pub use solvers::nesting::{NestingOptions, NestingResult, solve_nesting, solve_nesting_convex};

// LER + LIR combined solvers
pub use solvers::ler_lir::{LerLirOptions, LerLirResult, solve_ler_lir, solve_ler_lir_axis_aligned};

// OBB solvers
pub use solvers::obb::{ObbOptions, ObbResult, solve_obb, solve_obb_constrained};

pub use shared::{PolygonType, LirError, Result, Rectangle, SolverOptions, rotate_polygon, AlgorithmCategory, AlgorithmPrecision, AlgorithmSpeed, SolverBackend};

pub use geo_types::Polygon;

pub fn solve_oriented_lir(poly: &Polygon<f64>) -> Option<Rectangle> {
    solve_lir_oriented(poly, &LirOrientedOptions::default())
        .ok()
        .and_then(|r| r.rect)
}

pub fn solve_axis_aligned(poly: &Polygon<f64>, options: &AxisAlignedOptions) -> Option<Rectangle> {
    solve_vertex_grid(poly, options)
}
