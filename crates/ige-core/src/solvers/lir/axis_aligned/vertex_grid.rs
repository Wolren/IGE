//! Vertex-grid exact LIR solver (Daniels et al. 1997).
//!
//! The largest axis-aligned rectangle inscribed in a simple polygon always has
//! at least two sides aligned to vertex coordinates.
//!
//! Procedure:
//! 1. Sort unique vertex x/y coordinates, augment with midpoints.
//! 2. Scanline even-odd test per row (O(v^2)) -- fast cell-centre classification.
//! 3. Largest-rectangle-in-histogram sweep (O(v^2)).
//! 4. Post-verification: if the candidate overflows (concave boundary cells),
//!    a per-side binary contraction guarantees containment while maximising area.

use geo::{Area, BoundingRect, Contains, ConvexHull};
use geo_types::{Point, Polygon};
use ordered_float::OrderedFloat;
use std::collections::HashSet;

use super::containment::contract_rect_to_boundary;
use crate::shared::{PolygonType, Rectangle};

fn largest_rect_in_histogram(
    heights: &[usize],
    xs: &[f64],
    ys: &[f64],
    row_idx: usize,
) -> (f64, f64, f64, f64, f64) {
    let n = heights.len();
    let mut stack: Vec<(usize, usize)> = Vec::new();
    let mut best_area = 0.0;
    let mut best_rect = (0.0, 0.0, 0.0, 0.0);

    for col in 0..=n {
        let h = if col < n { heights[col] } else { 0 };
        let mut start = col;
        while let Some(&(sc, sh)) = stack.last() {
            if sh <= h { break; }
            stack.pop();
            let x0 = xs[sc];
            let x1 = xs[col.min(xs.len() - 1)];
            let y0 = ys[(row_idx + 1).saturating_sub(sh)];
            let y1 = ys[(row_idx + 1).min(ys.len() - 1)];
            let width = x1 - x0;
            let height = y1 - y0;
            if width > 0.0 && height > 0.0 {
                let area = width * height;
                if area > best_area {
                    best_area = area;
                    best_rect = (x0, y0, x1, y1);
                }
            }
            start = sc;
        }
        if col < n {
            stack.push((start, h));
        }
    }
    (best_rect.0, best_rect.1, best_rect.2, best_rect.3, best_area)
}

fn subdivide_coords(coords: &[f64], levels: u32) -> Vec<f64> {
    if coords.len() < 2 || levels == 0 {
        return coords.to_vec();
    }
    let n_parts = 1 << levels;
    let step = 1.0 / n_parts as f64;
    let mut result = Vec::with_capacity((coords.len() - 1) * n_parts + 1);
    for i in 0..coords.len() {
        result.push(coords[i]);
        if i < coords.len() - 1 {
            let a = coords[i];
            let b = coords[i + 1];
            for j in 1..n_parts {
                let t = j as f64 * step;
                result.push(a + (b - a) * t);
            }
        }
    }
    result
}

fn compute_row_intervals(poly: &Polygon<f64>, xs: &[f64], ys: &[f64]) -> Vec<Vec<(usize, usize)>> {
    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 {
        return vec![];
    }

    let mut intervals: Vec<Vec<(usize, usize)>> = vec![vec![]; n_rows];

    let exterior: Vec<(f64, f64)> = poly.exterior().0.iter()
        .map(|c| (c.x, c.y)).collect();
    let interiors: Vec<Vec<(f64, f64)>> = poly.interiors().iter()
        .map(|ring| ring.0.iter().map(|c| (c.x, c.y)).collect())
        .collect();

    let mut intersect_xs: Vec<f64> = Vec::new();

    for row in 0..n_rows {
        let y = (ys[row] + ys[row + 1]) * 0.5;
        intersect_xs.clear();

        for i in 0..exterior.len() - 1 {
            let (x1, y1) = exterior[i];
            let (x2, y2) = exterior[i + 1];
            if (y1 <= y && y2 > y) || (y2 <= y && y1 > y) {
                let t = (y - y1) / (y2 - y1);
                intersect_xs.push(x1 + t * (x2 - x1));
            }
        }
        for ring in &interiors {
            for i in 0..ring.len() - 1 {
                let (x1, y1) = ring[i];
                let (x2, y2) = ring[i + 1];
                if (y1 <= y && y2 > y) || (y2 <= y && y1 > y) {
                    let t = (y - y1) / (y2 - y1);
                    intersect_xs.push(x1 + t * (x2 - x1));
                }
            }
        }

        intersect_xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        for i in (0..intersect_xs.len()).step_by(2) {
            if i + 1 < intersect_xs.len() {
                let x_left = intersect_xs[i];
                let x_right = intersect_xs[i + 1];
                let mut col_start = None;
                let mut col_end = None;
                for col in 0..n_cols {
                    let cx = (xs[col] + xs[col + 1]) * 0.5;
                    if cx > x_left && cx < x_right {
                        if col_start.is_none() { col_start = Some(col); }
                        col_end = Some(col + 1);
                    }
                }
                if let (Some(start), Some(end)) = (col_start, col_end) {
                    intervals[row].push((start, end));
                }
            }
        }
    }
    intervals
}

/// Configuration for the axis-aligned vertex-grid solver.
#[derive(Debug, Clone)]
pub struct AxisAlignedOptions {
    pub max_ratio: f64,
    pub max_grid: usize,
}

impl Default for AxisAlignedOptions {
    fn default() -> Self {
        Self {
            max_ratio: 0.0,
            max_grid: crate::tuning::GRID_COARSE,
        }
    }
}

fn clamp_aspect_ratio(x0: f64, y0: f64, x1: f64, y1: f64, max_ratio: f64) -> (f64, f64, f64, f64) {
    if max_ratio <= 0.0 {
        return (x0, y0, x1, y1);
    }
    let rw = x1 - x0;
    let rh = y1 - y0;
    if rw <= 0.0 || rh <= 0.0 {
        return (x0, y0, x1, y1);
    }
    let ls = rw.max(rh);
    let ss = rw.min(rh);
    if ss > 0.0 && ls / ss > max_ratio {
        let nl = ss * max_ratio;
        if rw >= rh {
            let cx = (x0 + x1) * 0.5;
            (cx - nl * 0.5, y0, cx + nl * 0.5, y1)
        } else {
            let cy = (y0 + y1) * 0.5;
            (x0, cy - nl * 0.5, x1, cy + nl * 0.5)
        }
    } else {
        (x0, y0, x1, y1)
    }
}
pub fn solve_vertex_grid(poly: &Polygon<f64>, options: &AxisAlignedOptions) -> Option<Rectangle> {
    let mut x_coords = HashSet::new();
    let mut y_coords = HashSet::new();

    for coord in poly.exterior().0.iter() {
        x_coords.insert(OrderedFloat(coord.x));
        y_coords.insert(OrderedFloat(coord.y));
    }
    for interior in poly.interiors() {
        for coord in interior.0.iter() {
            x_coords.insert(OrderedFloat(coord.x));
            y_coords.insert(OrderedFloat(coord.y));
        }
    }

    let mut xs: Vec<f64> = x_coords.into_iter().map(|f| f.into_inner()).collect();
    let mut ys: Vec<f64> = y_coords.into_iter().map(|f| f.into_inner()).collect();
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());

    // Adaptive sub-division: more levels for low vertex-count polygons to ensure
    // sufficient grid resolution for the LRIH to find the optimal rectangle.
    let n_unique = xs.len().min(ys.len());
    let levels = if n_unique <= 4 { crate::tuning::AA_SUBDIV_LEVELS_HIGH as u32 } else if n_unique <= crate::tuning::AA_SMALL_VERTEX_CUTOFF { crate::tuning::AA_SUBDIV_LEVELS_MED as u32 } else { crate::tuning::AA_SUBDIV_LEVELS_LOW as u32 };

    xs = subdivide_coords(&xs, levels);
    ys = subdivide_coords(&ys, levels);

    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 {
        return None;
    }

    // Stage 1 -- cell classification
    let mut mask = vec![false; n_cols * n_rows];

    if n_unique <= crate::tuning::AA_SMALL_VERTEX_CUTOFF {
        // Low vertex count: use exact Point-in-Polygon for every cell centre.
        // Guarantees correct classification for simple shapes (triangles, narrow
        // polygons) where the even-odd scanline can misclassify near boundaries.
        for row in 0..n_rows {
            let cy = (ys[row] + ys[row + 1]) * 0.5;
            for col in 0..n_cols {
                let cx = (xs[col] + xs[col + 1]) * 0.5;
                mask[row * n_cols + col] = poly.contains(&Point::new(cx, cy));
            }
        }
    } else {
        // High vertex count: scanline even-odd per row (O(v^2)).
        let row_intervals = compute_row_intervals(poly, &xs, &ys);
        for row in 0..n_rows {
            for (col_start, col_end) in &row_intervals[row] {
                for col in *col_start..*col_end {
                    if col < n_cols {
                        mask[row * n_cols + col] = true;
                    }
                }
            }
        }
    }

    // Stage 2 -- histogram sweep (O(v^2))
    let mut heights = vec![0; n_cols];
    let mut best_area = 0.0;
    let mut best_rect: Option<Rectangle> = None;

    for row in 0..n_rows {
        for col in 0..n_cols {
            if mask[row * n_cols + col] {
                heights[col] += 1;
            } else {
                heights[col] = 0;
            }
        }
        let (x0, y0, x1, y1, area) = largest_rect_in_histogram(&heights, &xs, &ys, row);
        if area > best_area {
            best_area = area;
            let (cx0, cy0, cx1, cy1) = clamp_aspect_ratio(x0, y0, x1, y1, options.max_ratio);
            best_rect = Some(Rectangle { x_min: cx0, y_min: cy0, x_max: cx1, y_max: cy1 });
        }
    }

    // Stage 3 -- geometric verification + per-side contraction
    let contracted = best_rect.and_then(|r| {
        contract_rect_to_boundary(poly, r.x_min, r.y_min, r.x_max, r.y_max)
            .map(|(x0, y0, x1, y1)| Rectangle { x_min: x0, y_min: y0, x_max: x1, y_max: y1 })
    });

    // Stage 4 -- bounding-box fallback: start from the full axis-aligned bounding
    // box and contract inward. This handles cases where the scanline mask produces
    // a sub-optimal candidate position (common in narrow/simple shapes).
    let bb_candidate = poly.bounding_rect().and_then(|bb| {
        let x0 = bb.min().x;
        let y0 = bb.min().y;
        let x1 = bb.max().x;
        let y1 = bb.max().y;
        if x1 - x0 < 1e-12 || y1 - y0 < 1e-12 {
            return None;
        }
        contract_rect_to_boundary(poly, x0, y0, x1, y1)
            .map(|(x0, y0, x1, y1)| Rectangle { x_min: x0, y_min: y0, x_max: x1, y_max: y1 })
    });

    match (contracted, bb_candidate) {
        (Some(vg), Some(bb)) => {
            if bb.area() > vg.area() { Some(bb) } else { Some(vg) }
        }
        (Some(vg), None) => Some(vg),
        (None, Some(bb)) => Some(bb),
        (None, None) => None,
    }.map(|r| {
        let (x0, y0, x1, y1) = clamp_aspect_ratio(r.x_min, r.y_min, r.x_max, r.y_max, options.max_ratio);
        Rectangle { x_min: x0, y_min: y0, x_max: x1, y_max: y1 }
    })
}

/// Detect polygon type (convex/concave, holes/no-holes)
pub fn detect_polygon_type(poly: &Polygon<f64>) -> PolygonType {
    let has_holes = !poly.interiors().is_empty();
    let hull_area = poly.convex_hull().unsigned_area();
    let poly_area = poly.unsigned_area();
    let is_convex = (hull_area - poly_area).abs() / poly_area.max(1e-14) < 1e-6;
    match (is_convex, has_holes) {
        (true, false) => PolygonType::ConvexNoHoles,
        (true, true) => PolygonType::ConvexWithHoles,
        (false, false) => PolygonType::ConcaveNoHoles,
        (false, true) => PolygonType::ConcaveWithHoles,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    #[test]
    fn test_vertex_grid_square() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! { x: 0.0, y: 0.0 },
                coord! { x: 10.0, y: 0.0 },
                coord! { x: 10.0, y: 10.0 },
                coord! { x: 0.0, y: 10.0 },
                coord! { x: 0.0, y: 0.0 },
            ]),
            vec![],
        );
        let rect = solve_vertex_grid(&poly, &AxisAlignedOptions::default()).unwrap();
        assert!((rect.area() - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_polygon_type_detection() {
        let square = Polygon::new(
            LineString::from(vec![
                coord! { x: 0.0, y: 0.0 },
                coord! { x: 10.0, y: 0.0 },
                coord! { x: 10.0, y: 10.0 },
                coord! { x: 0.0, y: 10.0 },
                coord! { x: 0.0, y: 0.0 },
            ]),
            vec![],
        );
        assert_eq!(detect_polygon_type(&square), PolygonType::ConvexNoHoles);
    }
}
