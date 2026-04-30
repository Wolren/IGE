//! CPU-based solvers for oriented largest inscribed rectangles.

pub use crate::shared::{Rectangle, SolverOptions};

pub use crate::solvers::{
    solve_vertex_grid as solve_oriented_lir,
    detect_polygon_type,
};

pub use crate::geometry::rotate_polygon;

pub use crate::bcrs::solve_bcrs;
pub use crate::bcrs::{BcrsOptions, BcrsResult};