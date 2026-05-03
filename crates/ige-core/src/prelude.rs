//! Standard prelude for LIRiAP

pub use crate::algorithms::{
    AlgorithmCategory, AlgorithmPrecision, AlgorithmSpeed, LirAlgorithm, LirError, PolygonType, SolverOptions,
};

pub use crate::cpu::{solve_oriented_lir, Rectangle};
pub use crate::solvers::mic::{
    maximum_inscribed_circle,
    maximum_inscribed_circle_multipolygon,
    MicEngine,
    MicError,
    MicOptions,
    MicResult,
    MicUsedEngine,
    RobustMode,
};

pub use crate::geometry::{detect_polygon_type, rotate_polygon};

#[cfg(feature = "gpu")]
pub use crate::gpu;
