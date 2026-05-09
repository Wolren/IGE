//! C bindings for Inscribed Geometry Engine (IGE)
//!
//! Provides a C-compatible API for calling IGE from C, C++, or any language
//! with C FFI support.

use libc::{c_double, c_int, size_t};
use ige_core::{solve_axis_aligned, AxisAlignedOptions, rotate_polygon};
use ige_core::solvers::lir::oriented::{solve_lir_oriented, LirOrientedOptions};
use ige_core::solvers::mic::{maximum_inscribed_circle, MicEngine, MicOptions, MicUsedEngine, RobustMode};
use geo::BoundingRect;
use geo_types::{Coord, LineString, Polygon};
use std::slice;

/// C-compatible rectangle result
#[repr(C)]
pub struct IgeRectangle {
    pub x_min: c_double,
    pub y_min: c_double,
    pub x_max: c_double,
    pub y_max: c_double,
}

/// C-compatible MIC result.
#[repr(C)]
pub struct IgeMicResult {
    pub center_x: c_double,
    pub center_y: c_double,
    pub radius: c_double,
    pub radius_sq: c_double,
    pub used_engine: c_int, // 0 = exact, 1 = geos fallback
    pub candidate_count: size_t,
    pub component_index: c_int, // -1 when not multipolygon solve
}

/// C-compatible MIC options.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IgeMicOptions {
    pub engine: c_int,      // 0 = ExactOnly, 1 = FallbackOnly, 2 = ExactThenGeos
    pub robust_mode: c_int, // 0 = FastF64, 1 = Filtered
}

impl Default for IgeMicOptions {
    fn default() -> Self {
        Self {
            engine: 2,
            robust_mode: 1,
        }
    }
}

/// C-compatible solver options
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IgeOptions {
    pub rotation_degrees: c_double,
    pub prefer_gpu: c_int,
    pub force_cpu: c_int,
    pub max_aspect_ratio: c_double,
    pub min_aspect_ratio: c_double,
    pub use_parallel_field: c_int,
    pub use_simulated_annealing: c_int,
    pub use_bootstrap_seeds: c_int,
    pub use_pca_axes: c_int,
}

impl Default for IgeOptions {
    fn default() -> Self {
        Self {
            rotation_degrees: 0.0,
            prefer_gpu: 1,
            force_cpu: 0,
            max_aspect_ratio: 0.0,
            min_aspect_ratio: 0.0,
            use_parallel_field: 0,
            use_simulated_annealing: 0,
            use_bootstrap_seeds: 0,
            use_pca_axes: 0,
        }
    }
}

// ─── Axis-aligned solver ──────────────────────────────────────────────────

/// C-compatible axis-aligned solver options
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IgeAxisAlignedOptions {
    pub max_aspect_ratio: c_double,
    pub min_aspect_ratio: c_double,
}

impl Default for IgeAxisAlignedOptions {
    fn default() -> Self {
        Self { max_aspect_ratio: 0.0, min_aspect_ratio: 0.0 }
    }
}

impl From<IgeAxisAlignedOptions> for AxisAlignedOptions {
    fn from(opts: IgeAxisAlignedOptions) -> Self {
        AxisAlignedOptions { max_ratio: opts.max_aspect_ratio, min_ratio: opts.min_aspect_ratio, max_grid: 51 }
    }
}

// ─── LER solver (placeholder) ─────────────────────────────────────────────────

#[repr(C)]
#[derive(Clone, Copy)]
pub struct IgeLerOptions {
    pub max_aspect_ratio: c_double,
    pub min_aspect_ratio: c_double,
    pub grid_coarse: usize,
    pub top_k: usize,
    pub always_return: c_int,
}

impl Default for IgeLerOptions {
    fn default() -> Self {
        Self {
            max_aspect_ratio: 0.0,
            min_aspect_ratio: 0.0,
            grid_coarse: 60,
            top_k: 5,
            always_return: 1,
        }
    }
}

#[no_mangle]
pub unsafe extern "C" fn ige_ler_options_default() -> IgeLerOptions {
    IgeLerOptions::default()
}

// ─── Nesting solver (placeholder) ─────────────────────────────────────────

#[repr(C)]
#[derive(Clone, Copy)]
pub struct IgeNestingOptions {
    pub max_aspect_ratio: c_double,
    pub min_aspect_ratio: c_double,
    pub max_vertices: usize,
    pub grid_coarse: usize,
    pub prefer_convex: c_int,
}

impl Default for IgeNestingOptions {
    fn default() -> Self {
        Self {
            max_aspect_ratio: 0.0,
            min_aspect_ratio: 0.0,
            max_vertices: 100,
            grid_coarse: 60,
            prefer_convex: 1,
        }
    }
}

#[repr(C)]
pub struct IgeNestingResult {
    pub area: c_double,
    pub fill_ratio: c_double,
}

#[no_mangle]
pub unsafe extern "C" fn ige_nesting_options_default() -> IgeNestingOptions {
    IgeNestingOptions::default()
}

// ─── LER + LIR solver (placeholder) ───────────────────────────────────────

#[repr(C)]
#[derive(Clone, Copy)]
pub struct IgeLerLirOptions {
    pub max_aspect_ratio: c_double,
    pub min_aspect_ratio: c_double,
    pub grid_coarse: usize,
    pub top_k: usize,
    pub always_return: c_int,
    pub axis_aligned_only: c_int,
}

impl Default for IgeLerLirOptions {
    fn default() -> Self {
        Self {
            max_aspect_ratio: 0.0,
            min_aspect_ratio: 0.0,
            grid_coarse: 60,
            top_k: 5,
            always_return: 1,
            axis_aligned_only: 0,
        }
    }
}

#[repr(C)]
pub struct IgeLerLirResult {
    pub lir_area: c_double,
    pub ler_area: c_double,
}

#[no_mangle]
pub unsafe extern "C" fn ige_ler_lir_options_default() -> IgeLerLirOptions {
    IgeLerLirOptions::default()
}

// ─── OBB solver (placeholder) ─────────────────────────────────────────────

#[repr(C)]
#[derive(Clone, Copy)]
pub struct IgeObbOptions {
    pub max_aspect_ratio: c_double,
    pub min_aspect_ratio: c_double,
    pub angle_samples: usize,
    pub use_pca: c_int,
    pub use_refinement: c_int,
    pub xatol_deg: c_double,
}

impl Default for IgeObbOptions {
    fn default() -> Self {
        Self {
            max_aspect_ratio: 0.0,
            min_aspect_ratio: 0.0,
            angle_samples: 90,
            use_pca: 1,
            use_refinement: 1,
            xatol_deg: 0.1,
        }
    }
}

#[repr(C)]
pub struct IgeObbResult {
    pub area: c_double,
    pub angle_deg: c_double,
    pub width: c_double,
    pub height: c_double,
    pub aspect_ratio: c_double,
    pub fill_ratio: c_double,
}

#[no_mangle]
pub unsafe extern "C" fn ige_obb_options_default() -> IgeObbOptions {
    IgeObbOptions::default()
}

fn mic_engine_from_raw(value: c_int) -> MicEngine {
    match value {
        0 => MicEngine::ExactOnly,
        1 => MicEngine::FallbackOnly,
        2 => MicEngine::ExactThenGeos,
        _ => MicEngine::ExactThenGeos,
    }
}

fn robust_mode_from_raw(value: c_int) -> RobustMode {
    match value {
        0 => RobustMode::FastF64,
        1 => RobustMode::Filtered,
        _ => RobustMode::Filtered,
    }
}

fn used_engine_to_raw(value: MicUsedEngine) -> c_int {
    match value {
        MicUsedEngine::Exact => 0,
        MicUsedEngine::GeosFallback => 1,
        MicUsedEngine::Grid => 2,
    }
}

/// Solve for the largest axis-aligned rectangle (C API)
///
/// # Safety
///
/// - `coords` must point to a valid array of `coords_len` doubles
/// - Coordinates are interpreted as [x0, y0, x1, y1, x2, y2, ...]
/// - `result` must point to a valid IgeRectangle
/// - Returns 0 on success, -1 on error
#[no_mangle]
pub unsafe extern "C" fn ige_solve_axis_aligned(
    coords: *const c_double,
    coords_len: size_t,
    options: *const IgeAxisAlignedOptions,
    result: *mut IgeRectangle,
) -> c_int {
    if coords.is_null() || result.is_null() {
        return -1;
    }
    if coords_len < 6 || !coords_len.is_multiple_of(2) {
        return -1;
    }

    let coord_slice = slice::from_raw_parts(coords, coords_len);
    let mut geo_coords = Vec::with_capacity(coords_len / 2);
    for i in (0..coords_len).step_by(2) {
        geo_coords.push(Coord {
            x: coord_slice[i],
            y: coord_slice[i + 1],
        });
    }

    let exterior = LineString::from(geo_coords);
    let polygon = Polygon::new(exterior, vec![]);
    let opts: AxisAlignedOptions = if options.is_null() {
        IgeAxisAlignedOptions::default()
    } else {
        unsafe { *options }
    }.into();

    match solve_axis_aligned(&polygon, &opts) {
        Some(rect) => {
            *result = IgeRectangle {
                x_min: rect.x_min,
                y_min: rect.y_min,
                x_max: rect.x_max,
                y_max: rect.y_max,
            };
            0
        }
        None => -1,
    }
}

/// Get default axis-aligned solver options
#[no_mangle]
pub unsafe extern "C" fn ige_axis_aligned_options_default() -> IgeAxisAlignedOptions {
    IgeAxisAlignedOptions::default()
}

/// Get default MIC options.
#[no_mangle]
pub unsafe extern "C" fn ige_mic_options_default() -> IgeMicOptions {
    IgeMicOptions::default()
}

/// Solve for maximum inscribed circle (C API).
///
/// # Safety
///
/// - `coords` must point to a valid array of `coords_len` doubles
/// - Coordinates are interpreted as [x0, y0, x1, y1, x2, y2, ...]
/// - `result` must point to a valid IgeMicResult
/// - Returns 0 on success, -1 on error
#[no_mangle]
pub unsafe extern "C" fn ige_solve_mic(
    coords: *const c_double,
    coords_len: size_t,
    options: *const IgeMicOptions,
    result: *mut IgeMicResult,
) -> c_int {
    if coords.is_null() || result.is_null() {
        return -1;
    }
    if coords_len < 6 || !coords_len.is_multiple_of(2) {
        return -1;
    }

    let coord_slice = slice::from_raw_parts(coords, coords_len);
    let mut geo_coords = Vec::with_capacity(coords_len / 2);
    for i in (0..coords_len).step_by(2) {
        geo_coords.push(Coord {
            x: coord_slice[i],
            y: coord_slice[i + 1],
        });
    }

    let exterior = LineString::from(geo_coords);
    let polygon = Polygon::new(exterior, vec![]);

    let raw_opts = if options.is_null() {
        IgeMicOptions::default()
    } else {
        unsafe { *options }
    };
    let mic_opts = MicOptions {
        engine: mic_engine_from_raw(raw_opts.engine),
        robust_mode: robust_mode_from_raw(raw_opts.robust_mode),
    };

    match maximum_inscribed_circle(&polygon, &mic_opts) {
        Ok(mic) => {
            *result = IgeMicResult {
                center_x: mic.center.x(),
                center_y: mic.center.y(),
                radius: mic.radius,
                radius_sq: mic.radius_sq,
                used_engine: used_engine_to_raw(mic.used_engine),
                candidate_count: mic.candidate_count as size_t,
                component_index: mic.component_index.map(|x| x as c_int).unwrap_or(-1),
            };
            0
        }
        Err(_) => -1,
    }
}

/// Solve for largest oriented inscribed rectangle (C API)
///
/// # Safety
///
/// - `coords` must point to a valid array of `coords_len` doubles
/// - Coordinates are interpreted as [x0, y0, x1, y1, x2, y2, ...]
/// - `result` must point to a valid IgeRectangle
/// - Returns 0 on success, -1 on error
///
/// # Example
///
/// ```c
/// IgeRectangle rect;
/// IgeOptions opts = ige_options_default();
/// double coords[] = {0, 0, 10, 0, 10, 10, 0, 10, 0, 0};
/// int status = ige_solve(coords, 10, NULL, &opts, &rect);
/// if (status == 0) {
///     printf("Area: %f\n", (rect.x_max - rect.x_min) * (rect.y_max - rect.y_min));
/// }
/// ```
#[no_mangle]
pub unsafe extern "C" fn ige_solve(
    coords: *const c_double,
    coords_len: size_t,
    options: *const IgeOptions,
    result: *mut IgeRectangle,
) -> c_int {
    if coords.is_null() || result.is_null() {
        return -1;
    }

    if coords_len < 6 || !coords_len.is_multiple_of(2) {
        return -1; // Need at least 3 coordinate pairs
    }

    // Convert coordinates
    let coord_slice = slice::from_raw_parts(coords, coords_len);
    let mut geo_coords = Vec::with_capacity(coords_len / 2);

    for i in (0..coords_len).step_by(2) {
        geo_coords.push(Coord {
            x: coord_slice[i],
            y: coord_slice[i + 1],
        });
    }

    let exterior = LineString::from(geo_coords);
    let polygon = Polygon::new(exterior, vec![]);

    // Get options
    let opts = if options.is_null() {
        IgeOptions::default()
    } else {
        unsafe { *options }
    };
    let rotation = opts.rotation_degrees;
    let working_polygon = if rotation.abs() > 1e-12 {
        rotate_polygon(&polygon, rotation)
    } else {
        polygon.clone()
    };
    let mut lir_opts = LirOrientedOptions::default();
    lir_opts.max_ratio = opts.max_aspect_ratio;
    lir_opts.min_ratio = opts.min_aspect_ratio;
    lir_opts.use_parallel_field = opts.use_parallel_field != 0;
    lir_opts.use_simulated_annealing = opts.use_simulated_annealing != 0;
    lir_opts.use_bootstrap_seeds = opts.use_bootstrap_seeds != 0;
    lir_opts.use_pca_axes = opts.use_pca_axes != 0;

    let solve_result = solve_lir_oriented(&working_polygon, &lir_opts);
    match solve_result {
        Ok(res) => {
            let mut rect_poly = match res.rect_polygon {
                Some(r) => r,
                None => return -1,
            };
            if rotation.abs() > 1e-12 {
                rect_poly = rotate_polygon(&rect_poly, -rotation);
            }
            let bb = match rect_poly.bounding_rect() {
                Some(b) => b,
                None => return -1,
            };
            *result = IgeRectangle {
                x_min: bb.min().x,
                y_min: bb.min().y,
                x_max: bb.max().x,
                y_max: bb.max().y,
            };
            0
        }
        Err(_) => -1,
    }
}

/// Get default solver options
///
/// # Safety
///
/// This function is safe to call with no pointer arguments.
#[no_mangle]
pub unsafe extern "C" fn ige_options_default() -> IgeOptions {
    IgeOptions::default()
}

/// Calculate rectangle area
#[no_mangle]
pub unsafe extern "C" fn ige_rectangle_area(rect: *const IgeRectangle) -> c_double {
    if rect.is_null() {
        return 0.0;
    }

    let r = &*rect;
    (r.x_max - r.x_min) * (r.y_max - r.y_min)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_c_api_square() {
        let coords = [0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0, 0.0, 0.0];
        let mut rect = IgeRectangle {
            x_min: 0.0,
            y_min: 0.0,
            x_max: 0.0,
            y_max: 0.0,
        };

        let opts = unsafe { ige_options_default() };

        let status = unsafe {
            ige_solve(
                coords.as_ptr(),
                coords.len(),
                &opts as *const _,
                &mut rect as *mut _,
            )
        };

        assert_eq!(status, 0);

        let area = unsafe { ige_rectangle_area(&rect as *const _) };
        assert!((area - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_c_api_axis_aligned_square() {
        let coords = [0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0, 0.0, 0.0];
        let mut rect = IgeRectangle {
            x_min: 0.0,
            y_min: 0.0,
            x_max: 0.0,
            y_max: 0.0,
        };

        let opts = unsafe { ige_axis_aligned_options_default() };

        let status = unsafe {
            ige_solve_axis_aligned(
                coords.as_ptr(),
                coords.len(),
                &opts as *const _,
                &mut rect as *mut _,
            )
        };

        assert_eq!(status, 0);

        let area = unsafe { ige_rectangle_area(&rect as *const _) };
        assert!((area - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_c_api_axis_aligned_triangle() {
        let coords = [0.0, 0.0, 10.0, 0.0, 0.0, 10.0];
        let mut rect = IgeRectangle {
            x_min: 0.0,
            y_min: 0.0,
            x_max: 0.0,
            y_max: 0.0,
        };

        let status = unsafe {
            ige_solve_axis_aligned(
                coords.as_ptr(),
                coords.len(),
                std::ptr::null(),
                &mut rect as *mut _,
            )
        };

        assert_eq!(status, 0);
        assert!((rect.x_max - rect.x_min) * (rect.y_max - rect.y_min) > 0.0);
    }

    #[test]
    fn test_c_api_mic_square() {
        let coords = [0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0, 0.0, 0.0];
        let mut result = IgeMicResult {
            center_x: 0.0,
            center_y: 0.0,
            radius: 0.0,
            radius_sq: 0.0,
            used_engine: -1,
            candidate_count: 0,
            component_index: -1,
        };

        let mut opts = unsafe { ige_mic_options_default() };
        opts.engine = 0; // ExactOnly
        opts.robust_mode = 1; // Filtered

        let status = unsafe {
            ige_solve_mic(
                coords.as_ptr(),
                coords.len(),
                &opts as *const _,
                &mut result as *mut _,
            )
        };

        assert_eq!(status, 0);
        assert!((result.center_x - 5.0).abs() < 1e-7);
        assert!((result.center_y - 5.0).abs() < 1e-7);
        assert!((result.radius - 5.0).abs() < 1e-7);
    }
}
