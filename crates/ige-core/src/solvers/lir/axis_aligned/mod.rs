//! Axis-aligned largest inscribed rectangle algorithms.
//!
//! All solvers in this module find the largest axis-aligned rectangle
//! inscribed in a polygon (in the polygon's local coordinate frame).

pub mod vertex_grid;
pub mod exact;
pub mod histogram;
pub mod grid;
pub mod sdf;
pub mod containment;

pub use vertex_grid::{solve_vertex_grid, AxisAlignedOptions, detect_polygon_type};
pub use exact::solve_axis_exact;
pub use histogram::{lrih, lrih_vp};
pub use grid::{solve_axis_rect_grid, solve_axis_rect_bcrs as solve_axis_rect_fine};
pub use sdf::{best_effort_shrink, certify_rect, polygon_sdf, rect_sdf_max};
pub use containment::{rect_fully_contained, contract_rect_to_boundary};
