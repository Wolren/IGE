//! LIR Approximate Oriented — Largest Inscribed Rectangle Approximate Solver with SDF-guided expansion.
//!
//! Full Rust port of `bcrs_fast_worker.py` Stages 1–7.
//! Optional GPU acceleration hooks are behind the `"gpu"` feature flag.

pub mod candidates;
pub mod certify;
pub mod expand;
pub mod fallback;
pub mod fast;
pub mod parallel;
pub mod polish;
pub mod prepare;
pub mod refine;

use geo::BoundingRect;
use geo_types::{Point, Polygon};

use crate::shared::{LirError, Rectangle, Result};

#[cfg(feature = "gpu")]
use crate::gpu::GpuContext;
pub(crate) use candidates::heuristic_candidates;
pub(crate) use certify::certify_and_adjust;
pub use expand::expand_rect_to_boundary;
pub use fast::maybe_fast_path;
pub use parallel::solve_lir_approximate_oriented_parallel;
pub use prepare::{prepare_polygon, simplify_for_solve};
pub(crate) use refine::refine_best_candidate;

// ─── Public types ─────────────────────────────────────────────────────────

/// Configuration for the LIR approximate oriented solver.
#[derive(Debug, Clone)]
pub struct LirApproxOrientedOptions {
    /// Max aspect ratio (longer/shorter side); 0.0 = unconstrained.
    pub max_ratio: f64,
    /// Coarse grid resolution used for heuristic seeding and Brent polish.
    pub grid_coarse: usize,
    /// Fine grid resolution used in conservative fallback.
    pub grid_fine: usize,
    /// Number of top heuristic candidates forwarded to stages 4–6.
    pub top_k: usize,
    /// If true, return best-effort result even if certification fails.
    pub always_return: bool,
    /// If true, use the parallel ray-shooting candidate-field solver.
    pub use_parallel_field: bool,
    /// Half-width (degrees) for the Brent golden-section polish.
    pub polish_halwidth_deg: f64,
    /// Convergence tolerance for Brent polish (degrees).
    pub polish_xatol_deg: f64,
    /// Prune margin for angle upper-bound pruning.
    pub prune_margin: f64,
    /// Angle offset tried around each polished angle (degrees).
    pub angle_delta_deg: f64,
    /// Number of angle variants to keep per candidate.
    pub top_trials: usize,
    /// SDF certification epsilon.
    pub cert_eps: f64,
    /// Max shrink fraction during certification.
    pub cert_max_shrink: f64,
    /// Minimum angles to pad (parallel field).
    pub field_min_angles: usize,
    /// Step size for regular angle padding (parallel field).
    pub field_angle_step: usize,
    /// Max vertex coords per axis before uniform fallback.
    pub field_max_coords: usize,
    /// GPU context for accelerated SDF evaluation.
    #[cfg(feature = "gpu")]
    pub gpu_ctx: Option<std::sync::Arc<GpuContext>>,
}

impl Default for LirApproxOrientedOptions {
    fn default() -> Self {
        Self {
            max_ratio: 0.0,
            grid_coarse: crate::tuning::GRID_COARSE,
            grid_fine: crate::tuning::GRID_FINE,
            top_k: crate::tuning::TOP_K,
            always_return: true,
            use_parallel_field: false,
            polish_halwidth_deg: crate::tuning::POLISH_HALFWIDTH,
            polish_xatol_deg: crate::tuning::POLISH_XATOL,
            prune_margin: crate::tuning::PRUNE_MARGIN,
            angle_delta_deg: crate::tuning::ANGLE_DELTA,
            top_trials: crate::tuning::TOP_TRIALS,
            cert_eps: crate::tuning::CERT_EPS,
            cert_max_shrink: crate::tuning::CERT_MAX_SHRINK,
            field_min_angles: crate::tuning::FIELD_MIN_ANGLES,
            field_angle_step: crate::tuning::FIELD_ANGLE_STEP,
            field_max_coords: crate::tuning::FIELD_MAX_COORDS,
            #[cfg(feature = "gpu")]
            gpu_ctx: None,
        }
    }
}

/// Result of a LIR approximate oriented solve, including per-stage area gains for diagnostics.
#[derive(Debug, Clone)]
pub struct LirApproxOrientedResult {
    /// Best inscribed rectangle in world frame (AABB — axis-aligned bounding box).
    /// For the actual oriented rectangle, use `rect_polygon`.
    pub rect: Option<Rectangle>,
    /// The actual oriented rectangle as a polygon (rotated in world frame).
    /// None when no solution was found.
    pub rect_polygon: Option<Polygon<f64>>,
    /// Actual certified area.
    pub area: f64,
    /// Rotation angle that produced the best result [degrees].
    pub angle_deg: f64,
    /// True if the result is best-effort rather than strictly certified.
    pub best_effort: bool,
    /// Area after Stage 2 (coarse grid seed).
    pub s2_area: f64,
    /// Area after Stage 4 (BCRS vertex-coordinate solve).
    pub s4_area: f64,
    /// Area after Stage 5 (SDF-guided expansion).
    pub s5_area: f64,
}

impl LirApproxOrientedResult {
    pub fn empty() -> Self {
        Self {
            rect: None,
            rect_polygon: None,
            area: 0.0,
            angle_deg: 0.0,
            best_effort: false,
            s2_area: 0.0,
            s4_area: 0.0,
            s5_area: 0.0,
        }
    }
}

impl Default for LirApproxOrientedResult {
    fn default() -> Self {
        Self::empty()
    }
}

// ─── Internal candidate struct ─────────────────────────────────────────────

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub(crate) struct AngleCandidate {
    angle: f64,
    area: f64,
    rect_rot: (f64, f64, f64, f64), // (x0, y0, x1, y1) in rotated frame
    rect_world_bounds: (f64, f64, f64, f64),
    center: Point<f64>,
}

// ─── Rectangle frame helpers ──────────────────────────────────────────────


/// Solve the largest inscribed rectangle using BCRS + SDF pipeline.
///
/// # Arguments
/// * `poly` - Input polygon (must be valid, non-empty, area > 0)
/// * `options` - Solver configuration
///
/// # Returns
/// A `LirApproxOrientedResult` with the best rectangle (AABB in world frame), area, angle, etc.
pub fn solve_lir_approximate_oriented(poly: &Polygon<f64>, options: &LirApproxOrientedOptions) -> Result<LirApproxOrientedResult> {
    // Dispatch to parallel solver when the flag is set
    if options.use_parallel_field {
        return parallel::solve_lir_approximate_oriented_parallel(poly, options);
    }

    // Stage 0: Geometry preparation
    let poly = prepare_polygon(poly.clone()).ok_or(LirError::InvalidPolygon(
        "Polygon has <3 vertices or zero area".to_string(),
    ))?;

    // Fast path: simple convex shapes
    if let Some((rect_poly, area, angle, _)) = maybe_fast_path(&poly, options.max_ratio) {
        let bb = rect_poly.bounding_rect().unwrap();
        return Ok(LirApproxOrientedResult {
            rect: Some(Rectangle {
                x_min: bb.min().x,
                y_min: bb.min().y,
                x_max: bb.max().x,
                y_max: bb.max().y,
            }),
            rect_polygon: Some(rect_poly),
            area,
            angle_deg: angle,
            best_effort: false,
            s2_area: area,
            s4_area: area,
            s5_area: area,
        });
    }

    // Stage 1: Geometry preparation (simplification done inside)
    let angle_step = 5usize;

    // Stage 2: Heuristic candidates
    let candidates = heuristic_candidates(
        &poly,
        angle_step,
        options.grid_coarse,
        options.max_ratio,
        options.top_k,
    );

    if candidates.is_empty() {
        return Err(LirError::NoRectangleFound);
    }

    let s2_area = candidates.first().map(|c| c.area).unwrap_or(0.0);

    // Stages 3–7: Refine best candidate
    let result = refine_best_candidate(
        &poly,
        &candidates,
        options.grid_coarse,
        options.grid_fine,
        options.max_ratio,
        options.always_return,
    );

    match result {
        Some((rect, area, angle, _ratio, _rank, _gain, used_best_effort)) => {
            let bb = rect.bounding_rect().unwrap();
            Ok(LirApproxOrientedResult {
                rect: Some(Rectangle {
                    x_min: bb.min().x,
                    y_min: bb.min().y,
                    x_max: bb.max().x,
                    y_max: bb.max().y,
                }),
                rect_polygon: Some(rect),
                area,
                angle_deg: angle,
                best_effort: used_best_effort,
                s2_area,
                s4_area: area,
                s5_area: area,
            })
        }
        None => Err(LirError::NoRectangleFound),
    }
}

// ─── Worker entry point (compatible with Python signature) ─────────────────

/// Stateless worker entry point, mirrors `_worker_process_feature`.
///
/// Returns `(area, angle_deg, ratio, cand_rank, s2_gain, best_effort)` on success.
pub fn worker_process_feature(
    poly: &Polygon<f64>,
    _angle_step: usize,
    grid_coarse: usize,
    grid_fine: usize,
    max_ratio: f64,
    top_k: usize,
    always_return: bool,
) -> Option<(Rectangle, f64, f64, f64, usize, f64, bool)> {
    let options = LirApproxOrientedOptions {
        max_ratio,
        grid_coarse,
        grid_fine,
        top_k,
        always_return,
        use_parallel_field: false,
        polish_halwidth_deg: crate::tuning::POLISH_HALFWIDTH,
        polish_xatol_deg: crate::tuning::POLISH_XATOL,
        prune_margin: crate::tuning::PRUNE_MARGIN,
        angle_delta_deg: crate::tuning::ANGLE_DELTA,
        top_trials: crate::tuning::TOP_TRIALS,
        cert_eps: crate::tuning::CERT_EPS,
        cert_max_shrink: crate::tuning::CERT_MAX_SHRINK,
        field_min_angles: crate::tuning::FIELD_MIN_ANGLES,
        field_angle_step: crate::tuning::FIELD_ANGLE_STEP,
        field_max_coords: crate::tuning::FIELD_MAX_COORDS,
        #[cfg(feature = "gpu")]
        gpu_ctx: None,
    };

    let result = solve_lir_approximate_oriented(poly, &options).ok()?;

    Some((
        result.rect?,
        result.area,
        result.angle_deg,
        result.s5_area / (result.s2_area.max(1e-12)),
        if result.s5_area > 0.0 { 0 } else { 0 },
        result.s5_area - result.s2_area,
        result.best_effort,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    fn square_10x10() -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:10.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        )
    }

    #[test]
    fn lir_approximate_oriented_solve_square() {
        let poly = square_10x10();
        let result = solve_lir_approximate_oriented(&poly, &LirApproxOrientedOptions::default()).unwrap();
        assert!(result.area > 80.0, "area too small: {}", result.area);
        assert!(result.rect.is_some());
    }

    #[test]
    fn lir_approximate_oriented_solve_rectangle() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:20.0, y:0.0},
                coord! {x:20.0, y:5.0},
                coord! {x:0.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let result = solve_lir_approximate_oriented(&poly, &LirApproxOrientedOptions::default()).unwrap();
        assert!((result.area - 100.0).abs() < 10.0, "area={}", result.area);
    }

    #[test]
    fn lir_approximate_oriented_triangle_finds_rect() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let result = solve_lir_approximate_oriented(&poly, &LirApproxOrientedOptions::default());
        assert!(result.is_ok(), "LIR Approximate Oriented should find a rect in a triangle");
    }

    #[test]
    fn empty_result() {
        let result = LirApproxOrientedResult::empty();
        assert!(result.rect.is_none());
        assert_eq!(result.area, 0.0);
    }
}
