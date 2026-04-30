//! Inscribed Geometry Engine (IGE) - Largest Inscribed Rectangle algorithms

pub mod algorithms;
pub mod cpu;
pub mod geometry;
pub mod shared;
pub mod solvers;

#[cfg(feature = "gpu")]
pub mod gpu;
pub mod bcrs;

pub use cpu::{solve_oriented_lir, Rectangle, SolverOptions, detect_polygon_type, rotate_polygon};

pub use shared::{PolygonType, LirError, Result};
pub use shared::{AlgorithmCategory, AlgorithmPrecision, AlgorithmSpeed, SolverBackend};