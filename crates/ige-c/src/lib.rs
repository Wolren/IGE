//! C bindings for Inscribed Geometry Engine (IGE)
//!
//! Provides a C-compatible API for calling IGE from C, C++, or any language
//! with C FFI support.

use libc::{c_double, c_int, size_t};
use ige_core::{solve_axis_aligned, AxisAlignedOptions, rotate_polygon};
use ige_core::bcrs::{solve_bcrs, BcrsOptions};
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

/// C-compatible solver options
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IgeOptions {
    pub rotation_degrees: c_double,
    pub prefer_gpu: c_int,
    pub force_cpu: c_int,
    pub max_aspect_ratio: c_double,
    pub use_parallel_field: c_int,
}

impl Default for IgeOptions {
    fn default() -> Self {
        Self {
            rotation_degrees: 0.0,
            prefer_gpu: 1,
            force_cpu: 0,
            max_aspect_ratio: 0.0,
            use_parallel_field: 0,
        }
    }
}

// ─── Axis-aligned solver ──────────────────────────────────────────────────

/// C-compatible axis-aligned solver options
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IgeAxisAlignedOptions {
    pub max_aspect_ratio: c_double,
}

impl Default for IgeAxisAlignedOptions {
    fn default() -> Self {
        Self { max_aspect_ratio: 0.0 }
    }
}

impl From<IgeAxisAlignedOptions> for AxisAlignedOptions {
    fn from(opts: IgeAxisAlignedOptions) -> Self {
        AxisAlignedOptions { max_ratio: opts.max_aspect_ratio }
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
    let mut bcrs_opts = BcrsOptions::default();
    bcrs_opts.max_ratio = opts.max_aspect_ratio;
    bcrs_opts.use_parallel_field = opts.use_parallel_field != 0;

    let solve_result = solve_bcrs(&working_polygon, &bcrs_opts);
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
}
