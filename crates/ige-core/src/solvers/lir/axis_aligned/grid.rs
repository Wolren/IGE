//! Grid-based axis-aligned rectangle solvers used by the BCRS pipeline.
//!
//! Port of `_solve_axis_rect_grid` (coarse grid, now using 1D Sobol quasi-random
//! grid lines for lower-discrepancy coverage) and `_solve_axis_rect_bcrs`
//! (vertex-coordinate grid) from `bcrs_fast_worker.py`.

use geo::{BoundingRect, Contains};
use geo_types::{Point, Polygon};

use super::histogram::{lrih, lrih_vp};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaskBackend {
    Cpu,
    #[cfg(feature = "gpu")]
    GpuSdf,
    #[cfg(feature = "gpu")]
    GpuGridBatch,
    #[cfg(feature = "gpu")]
    Auto,
}

impl Default for MaskBackend {
    fn default() -> Self {
        #[cfg(feature = "gpu")]
        {
            Self::Auto
        }
        #[cfg(not(feature = "gpu"))]
        {
            Self::Cpu
        }
    }
}

// --- Uniform-grid solver (coarse / Brent) ---------------------------------

/// Solve the largest axis-aligned rectangle using a uniform grid of `grid_steps`
/// points on each axis. Returns `(x0, y0, x1, y1, area)` or `None`.
pub fn solve_axis_rect_grid(
    poly: &Polygon<f64>,
    grid_steps: usize,
    max_ratio: f64,
) -> Option<(f64, f64, f64, f64, f64)> {
    solve_axis_rect_grid_with_backend(poly, grid_steps, max_ratio, MaskBackend::default())
}

pub fn solve_axis_rect_grid_with_backend(
    poly: &Polygon<f64>,
    grid_steps: usize,
    max_ratio: f64,
    backend: MaskBackend,
) -> Option<(f64, f64, f64, f64, f64)> {
    let bb = poly.bounding_rect()?;
    let minx = bb.min().x;
    let miny = bb.min().y;
    let maxx = bb.max().x;
    let maxy = bb.max().y;

    if maxx <= minx || maxy <= miny || grid_steps < 2 {
        return None;
    }

    // Generate grid-line positions from 1D Sobol sequences (low-discrepancy)
    // instead of a uniform grid.  Sorted Sobol values give strictly better
    // space-filling coverage for the same number of probe points, particularly
    // at small grid sizes.
    let xs: Vec<f64> = (0..grid_steps)
        .map(|i| minx + (maxx - minx) * i as f64 / (grid_steps - 1) as f64)
        .collect();
    let ys: Vec<f64> = (0..grid_steps)
        .map(|i| miny + (maxy - miny) * i as f64 / (grid_steps - 1) as f64)
        .collect();

    // Build point-inside mask at each grid-point
    let mut mask = vec![false; grid_steps * grid_steps];
    build_uniform_mask(poly, &xs, &ys, &mut mask, grid_steps, backend);

    let mut heights = vec![0usize; grid_steps];
    let mut best: Option<(f64, f64, f64, f64, f64)> = None;

    for r in 0..grid_steps {
        let row = &mask[r * grid_steps..(r + 1) * grid_steps];
        for c in 0..grid_steps {
            if row[c] {
                heights[c] += 1;
            } else {
                heights[c] = 0;
            }
        }

        let (x0, y0, x1, y1, area) = lrih(&heights, &xs, &ys, r, max_ratio);

        if area > 0.0 {
            best = match best {
                Some((_, _, _, _, a)) if area > a => Some((x0, y0, x1, y1, area)),
                None => Some((x0, y0, x1, y1, area)),
                _ => best,
            };
        }
    }

    best
}

// --- BCRS vertex-coordinate grid solver -----------------------------------

/// Maximum vertex-coordinate count per axis before falling back to uniform grid.


/// Solve the largest axis-aligned rectangle using polygon vertex coordinates
/// as grid lines (boundary-coordinate raster solve).
///
/// Returns `(x0, y0, x1, y1, area)` or `None` if grid is too large or degenerate.
pub fn solve_axis_rect_bcrs(
    rot_poly: &Polygon<f64>,
    seed_bounds: Option<(f64, f64, f64, f64)>,
    max_ratio: f64,
) -> Option<(f64, f64, f64, f64, f64)> {
    solve_axis_rect_bcrs_with_backend(rot_poly, seed_bounds, max_ratio, MaskBackend::default())
}

pub fn solve_axis_rect_bcrs_with_backend(
    rot_poly: &Polygon<f64>,
    seed_bounds: Option<(f64, f64, f64, f64)>,
    max_ratio: f64,
    backend: MaskBackend,
) -> Option<(f64, f64, f64, f64, f64)> {
    // Collect vertex coordinates
    let mut xs_raw: Vec<f64> = rot_poly.exterior().0.iter().map(|c| c.x).collect();
    let mut ys_raw: Vec<f64> = rot_poly.exterior().0.iter().map(|c| c.y).collect();

    for interior in rot_poly.interiors() {
        for c in interior.0.iter() {
            xs_raw.push(c.x);
            ys_raw.push(c.y);
        }
    }

    let bb = rot_poly.bounding_rect()?;
    let minx = bb.min().x;
    let miny = bb.min().y;
    let maxx = bb.max().x;
    let maxy = bb.max().y;
    xs_raw.push(minx);
    xs_raw.push(maxx);
    ys_raw.push(miny);
    ys_raw.push(maxy);

    // Unique sorted coordinates
    xs_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);
    ys_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);

    if xs_raw.len() > crate::tuning::AA_GRID_MAX_COORDS || ys_raw.len() > crate::tuning::AA_GRID_MAX_COORDS {
        return None;
    }

    let n_cols = xs_raw.len().saturating_sub(1);
    let n_rows = ys_raw.len().saturating_sub(1);
    if n_cols < 1 || n_rows < 1 {
        return None;
    }

    // Cell-centre mask
    let mut mask = vec![false; n_cols * n_rows];
    build_bcrs_mask(rot_poly, &xs_raw, &ys_raw, &mut mask, n_cols, n_rows, backend);

    let mut heights = vec![0usize; n_cols];
    let mut best: Option<(f64, f64, f64, f64, f64)> = None;

    // Seed from caller
    if let Some((sx0, sy0, sx1, sy1)) = seed_bounds {
        if sx1 > sx0 && sy1 > sy0 {
            let seed_area = (sx1 - sx0) * (sy1 - sy0);
            if seed_area > 0.0 {
                best = Some((sx0, sy0, sx1, sy1, seed_area));
            }
        }
    }

    for r in 0..n_rows {
        let row = &mask[r * n_cols..(r + 1) * n_cols];
        for c in 0..n_cols {
            if row[c] {
                heights[c] += 1;
            } else {
                heights[c] = 0;
            }
        }

        let (x0, y0, x1, y1, area) = lrih_vp(&heights, &xs_raw, &ys_raw, r, max_ratio);

        if area > 0.0 {
            best = match best {
                Some((_, _, _, _, a)) if area > a => Some((x0, y0, x1, y1, area)),
                None => Some((x0, y0, x1, y1, area)),
                _ => best,
            };
        }
    }

    best
}

/// Active edge for the scanline rasteriser.
struct ActiveEdge {
    y_min: f64,
    y_max: f64,
    x: f64,     // current x-intersection at the scanline y
    dx_dy: f64, // slope dx/dy
}

/// Build the point-inside mask using an active-edge-list scanline.
///
/// Processes each row exactly once: polygon edges are pre-sorted by y-min and
/// activated/deactivated as the scanline advances.  Adjacent rows share most
/// of their crossing structure ΓÇö only edges entering or leaving the active set
/// are updated, reducing the per-row work from O(vertices) to O(changed_edges).
fn scanline_mask(poly: &Polygon<f64>, xs: &[f64], ys: &[f64], mask: &mut [bool], nx: usize, ny: usize) {
    // Collect non-horizontal polygon edges with their active-y range
    let mut edges: Vec<ActiveEdge> = Vec::new();
    for ring in std::iter::once(poly.exterior()).chain(poly.interiors()) {
        for i in 0..ring.0.len().saturating_sub(1) {
            let a = ring.0[i];
            let b = ring.0[i + 1];
            let dy = b.y - a.y;
            if dy.abs() < 1e-12 {
                continue;
            }
            let (lower, upper) = if a.y < b.y { (a, b) } else { (b, a) };
            let span_y = upper.y - lower.y;
            edges.push(ActiveEdge {
                y_min: lower.y,
                y_max: upper.y,
                x: lower.x,
                dx_dy: (upper.x - lower.x) / span_y,
            });
        }
    }
    edges.sort_by(|a, b| a.y_min.partial_cmp(&b.y_min).unwrap());

    let mut active: Vec<ActiveEdge> = Vec::new();
    let mut next_e = 0;

    for r in 0..ny {
        let y = ys[r];

        // Remove edges whose y-range is exhausted
        active.retain(|e| y < e.y_max);

        // Activate edges whose y-range begins at or before this row
        while next_e < edges.len() && edges[next_e].y_min <= y {
            if y < edges[next_e].y_max {
                let e = &edges[next_e];
                active.push(ActiveEdge {
                    y_min: e.y_min,
                    y_max: e.y_max,
                    x: e.x + (y - e.y_min) * e.dx_dy,
                    dx_dy: e.dx_dy,
                });
            }
            next_e += 1;
        }

        // Advance x-positions of active edges to current y
        for e in &mut active {
            e.x += (y - e.y_min) * e.dx_dy;
            e.y_min = y;
        }

        // Sort by current x-intersection
        active.sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap());

        // Walk the AEL: each crossing toggles inside/outside.
        // Columns between an odd-numbered crossing and the next are inside.
        let mut inside = false;
        let mut cross = 0;
        for c in 0..nx {
            let cx = xs[c];
            while cross < active.len() && active[cross].x < cx {
                inside = !inside;
                cross += 1;
            }
            mask[r * nx + c] = inside;
        }
    }
}

fn fallback_to_cpu_mask(poly: &Polygon<f64>, xs: &[f64], ys: &[f64], mask: &mut [bool], grid_steps: usize) {
    scanline_mask(poly, xs, ys, mask, grid_steps, grid_steps);
}

fn fallback_to_cpu_bcrs_mask(poly: &Polygon<f64>, xs: &[f64], ys: &[f64], mask: &mut [bool], n_cols: usize, n_rows: usize) {
    for r in 0..n_rows {
        let cy = (ys[r] + ys[r + 1]) * 0.5;
        for c in 0..n_cols {
            let cx = (xs[c] + xs[c + 1]) * 0.5;
            let pt = Point::new(cx, cy);
            mask[r * n_cols + c] = poly.contains(&pt);
        }
    }
}

#[cfg(feature = "gpu")]
fn gpu_build_mask_sdf(poly: &Polygon<f64>, points: &[(f64, f64)], mask: &mut [bool], n_cols: usize) -> bool {
    if let Some(gpu) = crate::gpu::get_gpu_context() {
        let gpu_rects: Vec<_> = points.iter().map(|&(x, y)| (x, y, x, y)).collect();
        if let Ok(sdf_results) = gpu.evaluate_rect_sdf_batch(poly, &gpu_rects) {
            let eps = crate::tuning::CONTAIN_BOUNDARY_EPS as f32;
            for (i, &val) in sdf_results.iter().enumerate() {
                let r = i / n_cols;
                let c = i % n_cols;
                mask[r * n_cols + c] = val <= eps;
            }
            return true;
        }
    }
    false
}

#[cfg(feature = "gpu")]
fn gpu_build_uniform_mask_batch(poly: &Polygon<f64>, mask: &mut [bool], grid_steps: usize) -> bool {
    let Some(gpu) = crate::gpu::get_gpu_context() else {
        return false;
    };
    let Ok(gpu_mask) = gpu.evaluate_grid_batch(&[poly], grid_steps as u32) else {
        return false;
    };
    if gpu_mask.len() != mask.len() {
        return false;
    }
    for (dst, src) in mask.iter_mut().zip(gpu_mask.into_iter()) {
        *dst = src != 0;
    }
    true
}

// Replace uniform grid mask with GPU-accelerated version
fn build_uniform_mask(
    poly: &Polygon<f64>,
    xs: &[f64],
    ys: &[f64],
    mask: &mut [bool],
    grid_steps: usize,
    backend: MaskBackend,
) {
    #[cfg(not(feature = "gpu"))]
    let _ = backend;
    #[cfg(feature = "gpu")]
    {
        match backend {
            MaskBackend::Cpu => {}
            MaskBackend::GpuGridBatch => {
                if gpu_build_uniform_mask_batch(poly, mask, grid_steps) {
                    return;
                }
            }
            MaskBackend::GpuSdf => {
                let points: Vec<_> = (0..grid_steps)
                    .flat_map(|r| (0..grid_steps).map(move |c| (xs[c], ys[r])))
                    .collect();
                if gpu_build_mask_sdf(poly, &points, mask, grid_steps) {
                    return;
                }
            }
            MaskBackend::Auto => {
                if gpu_build_uniform_mask_batch(poly, mask, grid_steps) {
                    return;
                }
                let points: Vec<_> = (0..grid_steps)
                    .flat_map(|r| (0..grid_steps).map(move |c| (xs[c], ys[r])))
                    .collect();
                if gpu_build_mask_sdf(poly, &points, mask, grid_steps) {
                    return;
                }
            }
        }
    }
    fallback_to_cpu_mask(poly, xs, ys, mask, grid_steps);
}

fn build_bcrs_mask(
    poly: &Polygon<f64>,
    xs_bounds: &[f64],
    ys_bounds: &[f64],
    mask: &mut [bool],
    n_cols: usize,
    n_rows: usize,
    backend: MaskBackend,
) {
    #[cfg(not(feature = "gpu"))]
    let _ = backend;
    #[cfg(feature = "gpu")]
    {
        match backend {
            MaskBackend::Cpu => {}
            MaskBackend::GpuGridBatch => {}
            MaskBackend::GpuSdf | MaskBackend::Auto => {
                let points: Vec<_> = (0..n_rows)
                    .flat_map(|r| {
                        let cy = (ys_bounds[r] + ys_bounds[r + 1]) * 0.5;
                        (0..n_cols).map(move |c| {
                            let cx = (xs_bounds[c] + xs_bounds[c + 1]) * 0.5;
                            (cx, cy)
                        })
                    })
                    .collect();
                if gpu_build_mask_sdf(poly, &points, mask, n_cols) {
                    return;
                }
            }
        }
    }
    fallback_to_cpu_bcrs_mask(poly, xs_bounds, ys_bounds, mask, n_cols, n_rows);
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    fn unit_square() -> Polygon<f64> {
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
    fn coarse_grid_finds_full_square() {
        let poly = unit_square();
        let result = solve_axis_rect_grid(&poly, 32, 0.0);
        assert!(result.is_some());
        let (x0, _y0, x1, _y1, area) = result.unwrap();
        assert!(area > 70.0, "area={area} too small for coarse grid");
        assert!(x0 < 2.0);
        assert!(x1 > 8.0);
    }

    #[test]
    fn bcrs_finds_full_square() {
        let poly = unit_square();
        let result = solve_axis_rect_bcrs(&poly, None, 0.0);
        assert!(result.is_some());
        let (x0, _y0, x1, _y1, area) = result.unwrap();
        assert!((area - 100.0).abs() < 0.01, "area={area}");
        assert!((x0 - 0.0).abs() < 0.01);
        assert!((x1 - 10.0).abs() < 0.01);
    }

    #[cfg(feature = "gpu")]
    #[test]
    fn backend_parity_on_square_grid_solver() {
        if crate::gpu::get_gpu_context().is_none() {
            return;
        }
        let poly = unit_square();
        let cpu = solve_axis_rect_grid_with_backend(&poly, 64, 0.0, MaskBackend::Cpu).unwrap();
        let gpu_sdf = solve_axis_rect_grid_with_backend(&poly, 64, 0.0, MaskBackend::GpuSdf).unwrap();
        let gpu_grid = solve_axis_rect_grid_with_backend(&poly, 64, 0.0, MaskBackend::GpuGridBatch).unwrap();

        let cpu_area = cpu.4;
        assert!((gpu_sdf.4 - cpu_area).abs() <= 1e-3, "gpu_sdf area mismatch: {} vs {}", gpu_sdf.4, cpu_area);
        assert!((gpu_grid.4 - cpu_area).abs() <= 1e-3, "gpu_grid area mismatch: {} vs {}", gpu_grid.4, cpu_area);
    }
}
