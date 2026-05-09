//! Maximum Inscribed Circle (MIC) solvers for polygonal inputs.

pub mod index;
pub mod input;
pub mod solver;
pub mod workspace;

use geo_types::{MultiPolygon, Point, Polygon};
use thiserror::Error;

use self::input::HostPolygon;
use self::solver::exact::solve_exact;
use self::solver::grid::solve_grid;
use self::workspace::MicWorkspace;

/// Engine selection for MIC solving.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MicEngine {
    /// Use the native Rust polygon-specialized solver only.
    ExactOnly,
    /// Use GEOS fallback only.
    FallbackOnly,
    /// Try exact first, then GEOS fallback if exact fails.
    ExactThenGeos,
}

/// Numeric robustness mode for the exact engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobustMode {
    /// Fast finite-precision mode.
    FastF64,
    /// Extra candidate filtering and certification.
    Filtered,
}

/// Solver configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MicOptions {
    pub engine: MicEngine,
    pub robust_mode: RobustMode,
}

impl Default for MicOptions {
    fn default() -> Self {
        Self {
            engine: MicEngine::ExactThenGeos,
            robust_mode: RobustMode::Filtered,
        }
    }
}

/// Engine that produced the final result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MicUsedEngine {
    Exact,
    Grid,
    GeosFallback,
}

/// MIC solve result.
#[derive(Debug, Clone)]
pub struct MicResult {
    pub center: Point<f64>,
    pub radius: f64,
    pub radius_sq: f64,
    pub support_segments: Vec<usize>,
    pub candidate_count: usize,
    pub used_engine: MicUsedEngine,
    pub component_index: Option<usize>,
}

/// MIC solver error.
#[derive(Debug, Error)]
pub enum MicError {
    #[error("invalid polygon input: {0}")]
    InvalidInput(String),
    #[error("no valid MIC candidate found")]
    NoCircleFound,
    #[error("exact MIC solver failed: {0}")]
    ExactFailed(String),
    #[error("GEOS fallback feature is not enabled")]
    GeosFeatureDisabled,
    #[error("GEOS fallback failed: {0}")]
    GeosFailed(String),
    #[error("unsupported GEOS MIC output: {0}")]
    UnsupportedGeosOutput(String),
}

/// Solve MIC on a single polygon.
pub fn maximum_inscribed_circle(
    poly: &Polygon<f64>,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    let host = HostPolygon::from_polygon(poly)?;
    solve_on_host_polygon(&host, opts)
}

/// Solve MIC with a reusable workspace (avoids rebuilding indexes per call).
///
/// The workspace is rebuilt only if `host` changes; for repeated calls on
/// different polygons, create a fresh [`MicWorkspace`] each time.
pub fn maximum_inscribed_circle_with_workspace(
    workspace: &mut MicWorkspace,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    solve_exact(workspace, opts).map_err(|err| MicError::ExactFailed(err.to_string()))
}

/// Solve MIC on a multipolygon by solving each component and keeping the best.
pub fn maximum_inscribed_circle_multipolygon(
    multi: &MultiPolygon<f64>,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    if multi.0.is_empty() {
        return Err(MicError::InvalidInput("multipolygon has no components".to_string()));
    }

    let mut best: Option<MicResult> = None;
    let mut last_error: Option<MicError> = None;

    for (idx, poly) in multi.0.iter().enumerate() {
        match maximum_inscribed_circle(poly, opts) {
            Ok(mut result) => {
                result.component_index = Some(idx);
                let replace = best
                    .as_ref()
                    .map(|current| result.radius_sq > current.radius_sq)
                    .unwrap_or(true);
                if replace {
                    best = Some(result);
                }
            }
            Err(err) => {
                last_error = Some(err);
            }
        }
    }

    best.ok_or_else(|| last_error.unwrap_or(MicError::NoCircleFound))
}

fn solve_on_host_polygon(
    host: &HostPolygon,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    // Phase 0: Try analytical fast path for simple shapes (triangle, convex quad)
    if opts.engine != MicEngine::FallbackOnly && host.ring_count() == 1 {
        let outer_len = host.outer_ring().len();
        'fast: {
            // fast_triangle needs outer.len() == 4 (3 vertices + closing)
            // fast_convex_quad needs outer.len() == 5 (4 vertices + closing)
            let result = if outer_len == 4 { solver::exact::fast_triangle(host) }
            else if outer_len == 5 { solver::exact::fast_convex_quad(host) }
            else { None };
            if result.is_none() { break 'fast; }
            let result = result.unwrap();

            // Verify: compute exact nearest-boundary distance via linear scan
            let seg_idx = input::SegmentIndex::from_host(host);
            let mut actual_sq = f64::INFINITY;
            for idx in 0..seg_idx.len() {
                let d = seg_idx.point_segment_distance_sq(idx, result.center.x(), result.center.y());
                if d < actual_sq { actual_sq = d; }
            }
            let actual = actual_sq.sqrt();
            if actual > 0.0 && (result.radius - actual).abs() / actual < 0.01 {
                return Ok(result);
            }
        }
    }

    // Phase 1: Use grid solver for complex shapes - matches GEOS exactly and is much faster
    // Skip for FallbackOnly - let GEOS solve independently for comparison
    let use_grid = opts.engine != MicEngine::FallbackOnly;
    
    if use_grid {
        // Use grid solver - now uses original polygon (no normalization)
        let workspace = MicWorkspace::new(host.clone())?;
        let bounds = host.bounds().unwrap_or((0.0, 0.0, 1.0, 1.0));
        let diag = (bounds.2 - bounds.0).hypot(bounds.3 - bounds.1).max(1.0);
        let tolerance = (diag * 1e-7).max(1e-12);

        if let Some(grid_result) = solve_grid(host, &workspace.pip_index, &workspace.nb_index, tolerance) {
            return Ok(grid_result);
        }
    }

    match opts.engine {
        MicEngine::ExactOnly => run_exact(host, opts),
        MicEngine::FallbackOnly => run_geos(host, None, opts),
        MicEngine::ExactThenGeos => {
            let mut workspace = match MicWorkspace::new(host.clone()) {
                Ok(w) => w,
                Err(e) => {
                    #[cfg(feature = "geos")]
                    { return run_geos(host, None, opts).map_err(|fe| MicError::GeosFailed(format!("workspace failed ({e}); fallback failed ({fe})"))); }
                    #[cfg(not(feature = "geos"))]
                    { return Err(e); }
                }
            };
            match solve_exact(&mut workspace, opts) {
                Ok(result) => Ok(result),
                Err(e) => {
                    #[cfg(feature = "geos")]
                    {
                        let seg_index = workspace.seg_index.clone();
                        run_geos(host, Some(&seg_index), opts).map_err(|fallback_err| {
                            MicError::GeosFailed(format!("exact failed ({e}); fallback failed ({fallback_err})"))
                        })
                    }
                    #[cfg(not(feature = "geos"))]
                    {
                        Err(MicError::ExactFailed(e.to_string()))
                    }
                }
            }
        }
    }
}

fn run_exact(
    host: &HostPolygon,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    let mut workspace = MicWorkspace::new(host.clone())?;
    solve_exact(&mut workspace, opts).map_err(|err| MicError::ExactFailed(err.to_string()))
}

fn run_geos(
    host: &HostPolygon,
    existing_seg_index: Option<&input::SegmentIndex>,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    #[cfg(feature = "geos")]
    {
        self::solver::geos_fallback::solve_with_geos(host, opts, existing_seg_index)
    }
    #[cfg(not(feature = "geos"))]
    {
        let _ = (host, existing_seg_index, opts);
        Err(MicError::GeosFeatureDisabled)
    }
}
