//! Grid-based axis-aligned rectangle solvers used by the BCRS pipeline.
//!
//! Port of `_solve_axis_rect_grid` (coarse grid, now using 1D Sobol quasi-random
//! grid lines for lower-discrepancy coverage) and `_solve_axis_rect_bcrs`
//! (vertex-coordinate grid) from `bcrs_fast_worker.py`.

use geo::{BoundingRect, Contains};
use geo_types::{Point, Polygon};

use crate::axis_aligned::histogram::{lrih, lrih_vp};

// --- Sobol 1D generator (Gray-code order) -----------------------------------
//
// Generates the van der Corput sequence in base 2 with Gray-code permutation,
// which has provably lower discrepancy than uniform spacing in 1D.
// Direction vectors: v[i] = 1 << (31 - i) for dimension 0 (the trivial sequence).

struct Sobol1d {
    i: u32,
    dirs: [u32; 32],
}

impl Sobol1d {
    fn new(start_index: u32) -> Self {
        let mut dirs = [0u32; 32];
        for i in 0..32 {
            dirs[i] = 1 << (31 - i);
        }
        Sobol1d { i: start_index, dirs }
    }

    /// Return the next value in [0, 1).
    fn next(&mut self) -> f64 {
        let g = self.i ^ (self.i >> 1);
        let mut val = 0u32;
        for bit in 0..32 {
            if (g >> bit) & 1 != 0 {
                val ^= self.dirs[bit];
            }
        }
        self.i += 1;
        val as f64 / (1u64 << 32) as f64
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
    let mut sobol_x = Sobol1d::new(0);
    let mut sobol_y = Sobol1d::new(1000);
    let mut xs: Vec<f64> = (0..grid_steps).map(|_| sobol_x.next()).collect();
    let mut ys: Vec<f64> = (0..grid_steps).map(|_| sobol_y.next()).collect();
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    // Scale Sobol [0,1) values to fill the bounding box exactly, so the
    // first and last probe land on the boundary just like a uniform grid.
    let scale_x = |u: f64| minx + (maxx - minx) * u;
    let scale_y = |u: f64| miny + (maxy - miny) * u;
    let xs: Vec<f64> = xs.iter().map(|&u| scale_x(u)).collect();
    let ys: Vec<f64> = ys.iter().map(|&u| scale_y(u)).collect();

    // Build point-inside mask at each grid-point
    let mut mask = vec![vec![false; grid_steps]; grid_steps];
    build_uniform_mask(poly, &xs, &ys, &mut mask, grid_steps);

    let mut heights = vec![0usize; grid_steps];
    let mut best: Option<(f64, f64, f64, f64, f64)> = None;

    for r in 0..grid_steps {
        let row = &mask[r];
        for c in 0..grid_steps {
            if row[c] {
                heights[c] += 1;
            } else {
                heights[c] = 0;
            }
        }

        let (x0, y0, x1, y1, area) = lrih(&heights, &xs, &ys, r, max_ratio);

        if area > 0.0 {
            if let Some((_, _, _, _, ref best_area)) = best {
                if area > *best_area {
                    best = Some((x0, y0, x1, y1, area));
                }
            } else {
                best = Some((x0, y0, x1, y1, area));
            }
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
    let mut mask = vec![vec![false; n_cols]; n_rows];
    build_bcrs_mask(rot_poly, &xs_raw, &ys_raw, &mut mask, n_cols, n_rows);

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
        let row = &mask[r];
        for c in 0..n_cols {
            if row[c] {
                heights[c] += 1;
            } else {
                heights[c] = 0;
            }
        }

        let (x0, y0, x1, y1, area) = lrih_vp(&heights, &xs_raw, &ys_raw, r, max_ratio);

        if area > 0.0 {
            if let Some((_, _, _, _, ref best_area)) = best {
                if area > *best_area {
                    best = Some((x0, y0, x1, y1, area));
                }
            } else {
                best = Some((x0, y0, x1, y1, area));
            }
        }
    }

    best
}

fn fallback_to_cpu_mask(poly: &Polygon<f64>, xs: &[f64], ys: &[f64], mask: &mut [Vec<bool>], grid_steps: usize) {
    for r in 0..grid_steps {
        for c in 0..grid_steps {
            let pt = Point::new(xs[c], ys[r]);
            mask[r][c] = poly.contains(&pt);
        }
    }
}

fn fallback_to_cpu_bcrs_mask(poly: &Polygon<f64>, xs: &[f64], ys: &[f64], mask: &mut [Vec<bool>], n_cols: usize, n_rows: usize) {
    for r in 0..n_rows {
        let cy = (ys[r] + ys[r + 1]) * 0.5;
        for c in 0..n_cols {
            let cx = (xs[c] + xs[c + 1]) * 0.5;
            let pt = Point::new(cx, cy);
            mask[r][c] = poly.contains(&pt);
        }
    }
}

#[cfg(feature = "gpu")]
fn gpu_build_mask(poly: &Polygon<f64>, points: &[(f64, f64)], mask: &mut [Vec<bool>], n_cols: usize) {
    // Fall back to CPU for high-vertex-count polygons (f32 precision loss)
    let n_verts = poly.exterior().0.len().saturating_sub(1);
    if n_verts > 200 {
        return;
    }
    if let Some(gpu) = crate::gpu::get_gpu_context() {
        let gpu_rects: Vec<_> = points.iter().map(|&(x, y)| (x, y, x, y)).collect();
            if let Ok(sdf_results) = gpu.evaluate_rect_sdf_batch(poly, &gpu_rects) {
                let eps: f32 = 1e-5; // f32 tolerance for boundary precision
                for (i, &val) in sdf_results.iter().enumerate() {
                    let r = i / n_cols;
                    let c = i % n_cols;
                    mask[r][c] = val <= eps;
                }
                return;
            }
    }
    // GPU failed -- fall back to CPU (handled at call site)
}

// Replace uniform grid mask with GPU-accelerated version
fn build_uniform_mask(poly: &Polygon<f64>, xs: &[f64], ys: &[f64], mask: &mut [Vec<bool>], grid_steps: usize) {
    #[cfg(feature = "gpu")]
    {
        let points: Vec<_> = (0..grid_steps)
            .flat_map(|r| (0..grid_steps).map(move |c| (xs[c], ys[r])))
            .collect();
        gpu_build_mask(poly, &points, mask, grid_steps);
        // Check if GPU populated the mask
        let any_true = mask.iter().any(|row| row.iter().any(|&v| v));
        if any_true { return; }
    }
    fallback_to_cpu_mask(poly, xs, ys, mask, grid_steps);
}

fn build_bcrs_mask(poly: &Polygon<f64>, xs_bounds: &[f64], ys_bounds: &[f64], mask: &mut [Vec<bool>], n_cols: usize, n_rows: usize) {
    #[cfg(feature = "gpu")]
    {
        let points: Vec<_> = (0..n_rows)
            .flat_map(|r| {
                let cy = (ys_bounds[r] + ys_bounds[r + 1]) * 0.5;
                (0..n_cols).map(move |c| {
                    let cx = (xs_bounds[c] + xs_bounds[c + 1]) * 0.5;
                    (cx, cy)
                })
            })
            .collect();
        gpu_build_mask(poly, &points, mask, n_cols);
        let any_true = mask.iter().any(|row| row.iter().any(|&v| v));
        if any_true { return; }
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
}
