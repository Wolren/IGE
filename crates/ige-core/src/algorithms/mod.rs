//! Core algorithm traits and implementations for LIR solvers.
//!
//! Provides a unified interface for switching between different solver implementations
//! and backends (CPU/GPU).

use crate::shared::{
    AlgorithmCategory, AlgorithmPrecision, AlgorithmSpeed, LirError, PolygonType, Rectangle,
    Result, SolverOptions,
};
use geo_types::Polygon;

pub trait LirSolver: Send + Sync {
    fn name(&self) -> &'static str;
    fn category(&self) -> AlgorithmCategory;
    fn precision(&self) -> AlgorithmPrecision;
    fn speed(&self) -> AlgorithmSpeed;
    fn polygon_type(&self) -> Option<PolygonType>;
    fn solve(&self, polygon: &Polygon<f64>, options: &SolverOptions) -> Result<Rectangle>;
}

pub mod cpu {
    //! CPU-based solver implementations.

    use super::*;
    use crate::shared::SolverOptions;
    use crate::solvers::lir::oriented::{self, LirOrientedOptions};
    use crate::solvers::lir::axis_aligned::{self, AxisAlignedOptions};
    use geo_types::Polygon;

    pub struct AxisAlignedSolver {
        opts: AxisAlignedOptions,
    }

    impl AxisAlignedSolver {
        pub fn new(opts: AxisAlignedOptions) -> Self {
            Self { opts }
        }
    }

    impl LirSolver for AxisAlignedSolver {
        fn name(&self) -> &'static str {
            "axis-aligned-cpu"
        }

        fn category(&self) -> AlgorithmCategory {
            AlgorithmCategory::AxisAligned
        }

        fn precision(&self) -> AlgorithmPrecision {
            AlgorithmPrecision::Approx
        }

        fn speed(&self) -> AlgorithmSpeed {
            AlgorithmSpeed::Standard
        }

        fn polygon_type(&self) -> Option<PolygonType> {
            None
        }

        fn solve(&self, polygon: &Polygon<f64>, _options: &SolverOptions) -> Result<Rectangle> {
            axis_aligned::solve_vertex_grid(polygon, &self.opts)
                .map(|rect| rect)
                .ok_or(LirError::NoRectangleFound)
        }
    }

    pub struct OrientedSolver {
        opts: LirOrientedOptions,
    }

    impl OrientedSolver {
        pub fn new(opts: LirOrientedOptions) -> Self {
            Self { opts }
        }
    }

    impl LirSolver for OrientedSolver {
        fn name(&self) -> &'static str {
            "oriented-cpu"
        }

        fn category(&self) -> AlgorithmCategory {
            AlgorithmCategory::Oriented
        }

        fn precision(&self) -> AlgorithmPrecision {
            AlgorithmPrecision::Approx
        }

        fn speed(&self) -> AlgorithmSpeed {
            AlgorithmSpeed::Fast
        }

        fn polygon_type(&self) -> Option<PolygonType> {
            None
        }

        fn solve(&self, polygon: &Polygon<f64>, _options: &SolverOptions) -> Result<Rectangle> {
            oriented::solve_lir_oriented(polygon, &self.opts)
                .map(|result| result.rect)
                .and_then(|opt| opt.ok_or(LirError::NoRectangleFound))
        }
    }
}

#[cfg(feature = "gpu")]
pub mod gpu {
    use super::*;
    use crate::shared::SolverOptions;
    use geo_types::Polygon;

    pub struct GpuSolver {
        _opts: crate::solvers::lir::axis_aligned::AxisAlignedOptions,
    }

    impl GpuSolver {
        pub fn new(opts: crate::solvers::lir::axis_aligned::AxisAlignedOptions) -> Self {
            Self { _opts: opts }
        }
    }

    impl LirSolver for GpuSolver {
        fn name(&self) -> &'static str {
            "gpu-oriented-lir"
        }

        fn category(&self) -> AlgorithmCategory {
            AlgorithmCategory::Oriented
        }

        fn precision(&self) -> AlgorithmPrecision {
            AlgorithmPrecision::Approx
        }

        fn speed(&self) -> AlgorithmSpeed {
            AlgorithmSpeed::Fast
        }

        fn polygon_type(&self) -> Option<PolygonType> {
            None
        }

        fn solve(&self, _polygon: &Polygon<f64>, _options: &SolverOptions) -> Result<Rectangle> {
            Err(LirError::NotSupported(
                "GPU solver not yet implemented".to_string(),
            ))
        }
    }
}
