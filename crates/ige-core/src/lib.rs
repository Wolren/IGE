//! Inscribed Geometry Engine (IGE) - Largest Inscribed Rectangle algorithms

pub mod algorithms;
pub mod cpu;
pub mod geometry;
pub mod shared;
pub mod tuning;

pub mod solvers;

#[cfg(feature = "gpu")]
pub mod gpu;

pub use cpu::{solve_lir_approximate_oriented_parallel, solve_oriented_lir, solve_axis_aligned, AxisAlignedOptions, Rectangle, SolverOptions, detect_polygon_type, rotate_polygon};
pub use solvers::lir::approximate::{solve_lir_approximate_oriented, LirApproxOrientedOptions, LirApproxOrientedResult};
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

pub use shared::{PolygonType, LirError, Result};
pub use shared::{AlgorithmCategory, AlgorithmPrecision, AlgorithmSpeed, SolverBackend};
