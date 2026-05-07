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
pub use grid::{
    MaskBackend,
    solve_axis_rect_bcrs as solve_axis_rect_fine,
    solve_axis_rect_bcrs_with_backend,
    solve_axis_rect_grid,
    solve_axis_rect_grid_with_backend,
};
pub use sdf::{best_effort_shrink, certify_rect, polygon_sdf, rect_sdf_max};
pub use containment::{rect_fully_contained, contract_rect_to_boundary};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxisAlignedSolver {
    VertexGrid,
    Exact,
    UniformGrid,
}

impl Default for AxisAlignedSolver {
    fn default() -> Self {
        Self::VertexGrid
    }
}

impl AxisAlignedSolver {
    pub fn solve(&self, poly: &geo_types::Polygon<f64>, options: &AxisAlignedOptions) -> Option<crate::shared::Rectangle> {
        match self {
            AxisAlignedSolver::VertexGrid => vertex_grid::solve_vertex_grid(poly, options),
            AxisAlignedSolver::Exact => exact::solve_axis_exact(poly, options),
            AxisAlignedSolver::UniformGrid => {
                let result = grid::solve_axis_rect_grid(poly, options.max_grid, options.max_ratio, options.min_ratio)?;
                Some(crate::shared::Rectangle {
                    x_min: result.0,
                    y_min: result.1,
                    x_max: result.2,
                    y_max: result.3,
                })
            }
        }
    }
}
