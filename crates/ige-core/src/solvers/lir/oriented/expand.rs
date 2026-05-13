//! SDF-guided boundary expansion.
//!
//! Port of `_expand_rect_to_boundary` from `bcrs_fast_worker.py`.
//! Uses the signed-distance field at edge midpoints to bound binary search
//! for the maximum expansion of each side.

use geo::{BoundingRect, Contains};
use geo_types::{Coord, LineString, Point, Polygon};

use super::super::axis_aligned::sdf::{polygon_sdf, sdf_gradient};

const BINARY_STEPS: usize = crate::tuning::EXPAND_BINARY_STEPS;
const EXPAND_ITERS: usize = crate::tuning::EXPAND_ITERS;
const SDF_PROBES: usize = 5;
const GRADIENT_STEPS: usize = crate::tuning::GRADIENT_EXPAND_STEPS;
const GRADIENT_STEP_SIZE: f64 = crate::tuning::GRADIENT_EXPAND_STEP_SIZE;
const GRADIENT_GRADIENT_STEP: f64 = crate::tuning::GRADIENT_EXPAND_GRADIENT_STEP;
const GRADIENT_MAX_DIST: f64 = crate::tuning::GRADIENT_EXPAND_MAX_DIST;
const GRADIENT_MARGIN: f64 = crate::tuning::GRADIENT_EXPAND_MARGIN;

/// Sample the SDF at `probes` points along a vertical line (fixed x, varying y)
/// using recursive SDF evaluation: the Lipschitz property (|SDF(a)-SDF(b)| ≤ |a-b|)
/// means a far-inside evaluation guarantees nearby probes are also inside,
/// eliminating redundant distance computations.
///
/// Evaluates probes left-to-right.  After each evaluation at position yᵢ with
/// SDF = dᵢ, any probe within distance dᵢ of yᵢ is guaranteed inside (SDF > 0)
/// and is skipped.  The returned value is a conservative lower bound on the
/// minimum SDF across all probes — safe for use as the binary-search ceiling.
fn multi_probe_sdf_v(
    poly: &Polygon<f64>,
    x_fixed: f64,
    y_lo: f64,
    y_hi: f64,
    probes: usize,
) -> f64 {
    if probes == 0 {
        return f64::MAX;
    }
    let span = y_hi - y_lo;
    let mut min_sdf = f64::MAX;
    let mut last_y = f64::NAN;
    let mut last_sdf = f64::NAN;

    for i in 0..probes {
        let t = (i as f64 + 0.5) / probes as f64;
        let y = y_lo + span * t;

        // Lipschitz skip: if last evaluated probe guarantees this point is inside
        if last_sdf.is_finite() {
            let dist = (y - last_y).abs();
            if last_sdf - dist > 0.0 {
                // Conservative bound: actual SDF(y) ≥ last_sdf - dist
                let bound = last_sdf - dist;
                if bound < min_sdf {
                    min_sdf = bound;
                }
                continue;
            }
        }

        let sdf = polygon_sdf(poly, x_fixed, y);
        last_y = y;
        last_sdf = sdf;
        if sdf < min_sdf {
            min_sdf = sdf;
        }
    }
    min_sdf
}

/// Sample the SDF at `probes` points along a horizontal line (fixed y, varying x)
/// using the same Lipschitz-skip optimisation.
fn multi_probe_sdf_h(
    poly: &Polygon<f64>,
    y_fixed: f64,
    x_lo: f64,
    x_hi: f64,
    probes: usize,
) -> f64 {
    if probes == 0 {
        return f64::MAX;
    }
    let span = x_hi - x_lo;
    let mut min_sdf = f64::MAX;
    let mut last_x = f64::NAN;
    let mut last_sdf = f64::NAN;

    for i in 0..probes {
        let t = (i as f64 + 0.5) / probes as f64;
        let x = x_lo + span * t;

        if last_sdf.is_finite() {
            let dist = (x - last_x).abs();
            if last_sdf - dist > 0.0 {
                let bound = last_sdf - dist;
                if bound < min_sdf {
                    min_sdf = bound;
                }
                continue;
            }
        }

        let sdf = polygon_sdf(poly, x, y_fixed);
        last_x = x;
        last_sdf = sdf;
        if sdf < min_sdf {
            min_sdf = sdf;
        }
    }
    min_sdf
}

/// Pre-built spatial index for fast rect-covers queries.
///
/// Builds once per `expand_rect_to_boundary` call. Sorts polygon edges by
/// their x-range minimum and uses binary search + AABB pre-filter to find
/// candidate edges that might intersect the query rect, instead of scanning
/// all N polygon edges for every binary search step.
struct CoversIndex {
    edges_a: Vec<Coord<f64>>,
    edges_b: Vec<Coord<f64>>,
    xmin: Vec<f64>,
    xmax: Vec<f64>,
    ymin: Vec<f64>,
    ymax: Vec<f64>,
    /// Indices into the above arrays, sorted by `xmin`.
    order: Vec<usize>,
}

impl CoversIndex {
    fn from_polygon(poly: &Polygon<f64>) -> Self {
        let mut edges_a = Vec::new();
        let mut edges_b = Vec::new();

        let mut add_ring = |ring: &LineString<f64>| {
            let n = ring.0.len();
            for i in 0..n.saturating_sub(1) {
                edges_a.push(ring.0[i]);
                edges_b.push(ring.0[i + 1]);
            }
        };
        add_ring(poly.exterior());
        for hole in poly.interiors() {
            add_ring(hole);
        }

        let n = edges_a.len();
        let mut xmin = Vec::with_capacity(n);
        let mut xmax = Vec::with_capacity(n);
        let mut ymin = Vec::with_capacity(n);
        let mut ymax = Vec::with_capacity(n);
        for i in 0..n {
            let (a, b) = (edges_a[i], edges_b[i]);
            xmin.push(a.x.min(b.x));
            xmax.push(a.x.max(b.x));
            ymin.push(a.y.min(b.y));
            ymax.push(a.y.max(b.y));
        }

        let mut order: Vec<usize> = (0..n).collect();
        order.sort_unstable_by(|&i, &j| xmin[i].partial_cmp(&xmin[j]).unwrap());

        CoversIndex { edges_a, edges_b, xmin, xmax, ymin, ymax, order }
    }

    /// Returns `true` if any polygon edge crosses one of the four rect edges.
    fn has_crossing(&self, x0: f64, y0: f64, x1: f64, y1: f64) -> bool {
        if self.xmin.is_empty() {
            return false;
        }

        let rect_edges = [
            (Coord { x: x0, y: y0 }, Coord { x: x1, y: y0 }),
            (Coord { x: x1, y: y0 }, Coord { x: x1, y: y1 }),
            (Coord { x: x1, y: y1 }, Coord { x: x0, y: y1 }),
            (Coord { x: x0, y: y1 }, Coord { x: x0, y: y0 }),
        ];

        for &idx in &self.order {
            if self.xmin[idx] > x1 {
                break;
            }
            if self.xmax[idx] < x0 || self.ymax[idx] < y0 || self.ymin[idx] > y1 {
                continue;
            }
            let (a, b) = (self.edges_a[idx], self.edges_b[idx]);
            for &(ra, rb) in &rect_edges {
                if segments_intersect(ra, rb, a, b) {
                    return true;
                }
            }
        }
        false
    }
}

/// Proper geometric containment check: verifies all 4 corners are inside AND
/// no rect edge intersects the polygon boundary. Equivalent to Shapely's
/// `prep.covers(box(x0,y0,x1,y1))`.
///
/// Uses a pre-built spatial index for the edge-crossing test to avoid
/// O(n) ring traversal on every call.
fn rect_covers(index: &CoversIndex, poly: &Polygon<f64>, x0: f64, y0: f64, x1: f64, y1: f64) -> bool {
    if x1 - x0 < 1e-12 || y1 - y0 < 1e-12 {
        return false;
    }

    // Stage 1: fast corner check (4 point-in-polygon tests)
    let corners = [
        Point::new(x0, y0),
        Point::new(x1, y0),
        Point::new(x1, y1),
        Point::new(x0, y1),
    ];
    if !corners.iter().all(|p| poly.contains(p)) {
        return false;
    }

    // Stage 2: use the spatial index for edge-crossing check
    !index.has_crossing(x0, y0, x1, y1)
}

fn segments_intersect(
    a: Coord<f64>,
    b: Coord<f64>,
    c: Coord<f64>,
    d: Coord<f64>,
) -> bool {
    fn orientation(p: Coord<f64>, q: Coord<f64>, r: Coord<f64>) -> f64 {
        (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
    }

    let o1 = orientation(a, b, c);
    let o2 = orientation(a, b, d);
    let o3 = orientation(c, d, a);
    let o4 = orientation(c, d, b);

    // General case: segments straddle
    if o1 * o2 < 0.0 && o3 * o4 < 0.0 {
        return true;
    }

    // Collinear boundary cases -- consider as non-intersecting for containment
    // (the corner check already verifies endpoints are fine; collinear overlaps
    // on the boundary are acceptable for `covers`)
    false
}

fn clamp_aspect_ratio(mut x0: f64, mut y0: f64, mut x1: f64, mut y1: f64, max_ratio: f64, min_ratio: f64) -> (f64, f64, f64, f64) {
    let rw = x1 - x0;
    let rh = y1 - y0;
    if rw <= 0.0 || rh <= 0.0 {
        return (x0, y0, x1, y1);
    }
    let ls = rw.max(rh);
    let ss = rw.min(rh);
    let current_ratio = ls / ss;
    if max_ratio > 0.0 && current_ratio > max_ratio {
        let nl = ss * max_ratio;
        if rw >= rh {
            let cx = (x0 + x1) * 0.5;
            x0 = cx - nl * 0.5;
            x1 = cx + nl * 0.5;
        } else {
            let cy = (y0 + y1) * 0.5;
            y0 = cy - nl * 0.5;
            y1 = cy + nl * 0.5;
        }
    } else if min_ratio > 0.0 && current_ratio < min_ratio {
        let nl = ss * min_ratio;
        if rw >= rh {
            let cx = (x0 + x1) * 0.5;
            x0 = cx - nl * 0.5;
            x1 = cx + nl * 0.5;
        } else {
            let cy = (y0 + y1) * 0.5;
            y0 = cy - nl * 0.5;
            y1 = cy + nl * 0.5;
        }
    }
    (x0, y0, x1, y1)
}

pub fn expand_rect_to_boundary(
    rot_poly: &Polygon<f64>,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    max_ratio: f64,
    min_ratio: f64,
) -> (f64, f64, f64, f64) {
    // Build spatial index once for all rect_covers queries
    let idx = CoversIndex::from_polygon(rot_poly);

    let bb = match rot_poly.bounding_rect() {
        Some(b) => b,
        None => return (x0, y0, x1, y1),
    };
    let minx = bb.min().x;
    let miny = bb.min().y;
    let maxx = bb.max().x;
    let maxy = bb.max().y;

    let mut x0 = x0;
    let mut y0 = y0;
    let mut x1 = x1;
    let mut y1 = y1;

    // Shrink to valid start if seed slightly exceeds bounds
    if !rect_covers(&idx, rot_poly, x0, y0, x1, y1) {
        let cx_c = (x0 + x1) * 0.5;
        let cy_c = (y0 + y1) * 0.5;
        let hw = (x1 - x0) * 0.5;
        let hh = (y1 - y0) * 0.5;
        let mut lo = 0.0_f64;
        let mut hi = 1.0_f64;
        for _ in 0..36 {
            let mid = (lo + hi) * 0.5;
            if rect_covers(&idx, rot_poly, cx_c - hw * mid, cy_c - hh * mid, cx_c + hw * mid, cy_c + hh * mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        if lo < 1e-9 {
            return (x0, y0, x1, y1);
        }
        x0 = cx_c - hw * lo;
        y0 = cy_c - hh * lo;
        x1 = cx_c + hw * lo;
        y1 = cy_c + hh * lo;
    }

    for _ in 0..EXPAND_ITERS {
        let mut any_changed = false;

        // Sort sides by gap size (largest first) for faster convergence
        let gap_left = if x0 > minx { x0 - minx } else { 0.0 };
        let gap_right = if x1 < maxx { maxx - x1 } else { 0.0 };
        let gap_bottom = if y0 > miny { y0 - miny } else { 0.0 };
        let gap_top = if y1 < maxy { maxy - y1 } else { 0.0 };

        let mut expansions: [(usize, f64); 4] = [
            (0, gap_left), (1, gap_right), (2, gap_bottom), (3, gap_top),
        ];
        expansions.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        for &(side, _) in &expansions {
            match side {
                0 if x0 > minx => { // Left
                    let min_sdf = multi_probe_sdf_v(rot_poly, x0, y0, y1, SDF_PROBES);
                    let hi_d = if min_sdf < 0.0 { gap_left.min(min_sdf.abs()) } else { gap_left };
                    if hi_d > 1e-12 {
                        let mut lo_d = 0.0_f64;
                        let mut hi_d = hi_d;
                        for _ in 0..BINARY_STEPS {
                            let mid = (lo_d + hi_d) * 0.5;
                            if rect_covers(&idx, rot_poly, x0 - mid, y0, x1, y1) {
                                lo_d = mid;
                            } else {
                                hi_d = mid;
                            }
                        }
                        if lo_d > 1e-10 { x0 -= lo_d; any_changed = true; }
                    }
                }
                1 if x1 < maxx => { // Right
                    let min_sdf = multi_probe_sdf_v(rot_poly, x1, y0, y1, SDF_PROBES);
                    let hi_d = if min_sdf < 0.0 { gap_right.min(min_sdf.abs()) } else { gap_right };
                    if hi_d > 1e-12 {
                        let mut lo_d = 0.0_f64;
                        let mut hi_d = hi_d;
                        for _ in 0..BINARY_STEPS {
                            let mid = (lo_d + hi_d) * 0.5;
                            if rect_covers(&idx, rot_poly, x0, y0, x1 + mid, y1) {
                                lo_d = mid;
                            } else {
                                hi_d = mid;
                            }
                        }
                        if lo_d > 1e-10 { x1 += lo_d; any_changed = true; }
                    }
                }
                2 if y0 > miny => { // Bottom — fix y, vary x horizontally
                    let min_sdf = multi_probe_sdf_h(rot_poly, y0, x0, x1, SDF_PROBES);
                    let hi_d = if min_sdf < 0.0 { gap_bottom.min(min_sdf.abs()) } else { gap_bottom };
                    if hi_d > 1e-12 {
                        let mut lo_d = 0.0_f64;
                        let mut hi_d = hi_d;
                        for _ in 0..BINARY_STEPS {
                            let mid = (lo_d + hi_d) * 0.5;
                            if rect_covers(&idx, rot_poly, x0, y0 - mid, x1, y1) {
                                lo_d = mid;
                            } else {
                                hi_d = mid;
                            }
                        }
                        if lo_d > 1e-10 { y0 -= lo_d; any_changed = true; }
                    }
                }
                3 if y1 < maxy => { // Top
                    let min_sdf = multi_probe_sdf_h(rot_poly, y1, x0, x1, SDF_PROBES);
                    let hi_d = if min_sdf < 0.0 { gap_top.min(min_sdf.abs()) } else { gap_top };
                    if hi_d > 1e-12 {
                        let mut lo_d = 0.0_f64;
                        let mut hi_d = hi_d;
                        for _ in 0..BINARY_STEPS {
                            let mid = (lo_d + hi_d) * 0.5;
                            if rect_covers(&idx, rot_poly, x0, y0, x1, y1 + mid) {
                                lo_d = mid;
                            } else {
                                hi_d = mid;
                            }
                        }
                        if lo_d > 1e-10 { y1 += lo_d; any_changed = true; }
                    }
                }
                _ => {}
            }
        }

        if !any_changed { break; }
    }

    (x0, y0, x1, y1) = clamp_aspect_ratio(x0, y0, x1, y1, max_ratio, min_ratio);

    (x0, y0, x1, y1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    #[test]
    fn expand_doesnt_exceed_bounds() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:10.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let (x0, y0, x1, y1) = expand_rect_to_boundary(&poly, 1.0, 1.0, 9.0, 9.0, 0.0, 0.0);
        assert!(x0.abs() < 1e-6, "x0={x0}");
        assert!(y0.abs() < 1e-6, "y0={y0}");
        assert!((x1 - 10.0).abs() < 1e-6, "x1={x1}");
        assert!((y1 - 10.0).abs() < 1e-6, "y1={y1}");
    }

    #[test]
    fn rect_covers_concave_rejects_overflow() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:10.0},
                coord! {x:5.0, y:5.0}, // indentation
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let idx = CoversIndex::from_polygon(&poly);
        // This rect has all corners inside but its edge crosses the indentation
        assert!(!rect_covers(&idx, &poly, 1.0, 1.0, 9.0, 8.0));
    }

    #[test]
    fn rect_covers_simple_pass() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:10.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let idx = CoversIndex::from_polygon(&poly);
        assert!(rect_covers(&idx, &poly, 2.0, 2.0, 8.0, 8.0));
    }
}

/// SDF-based gradient descent: walk from rectangle center toward boundary
/// and re-expand to find potentially better solutions.
pub fn expand_rect_gradient(
    rot_poly: &Polygon<f64>,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    max_ratio: f64,
    min_ratio: f64,
) -> (f64, f64, f64, f64) {
    let bb = match rot_poly.bounding_rect() {
        Some(b) => b,
        None => return (x0, y0, x1, y1),
    };
    let minx = bb.min().x;
    let miny = bb.min().y;
    let maxx = bb.max().x;
    let maxy = bb.max().y;

    // First do normal expansion
    let expanded = expand_rect_to_boundary(rot_poly, x0, y0, x1, y1, max_ratio, min_ratio);

    let (ex0, ey0, ex1, ey1) = expanded;
    let cx = (ex0 + ex1) * 0.5;
    let cy = (ey0 + ey1) * 0.5;

    // Try walking toward boundary and expanding from there
    let mut best_rect = expanded;
    let mut best_area = (ex1 - ex0) * (ey1 - ey0);

    // Walk in each cardinal-ish direction and re-expand
    for i in 0..GRADIENT_STEPS {
        let angle = (i as f64) * std::f64::consts::PI / (GRADIENT_STEPS as f64 * 2.0);
        let dx = angle.cos();
        let dy = angle.sin();

        // Walk from center toward this direction
        let walk_dist = GRADIENT_STEP_SIZE * (i as f64 + 1.0) * 0.5;
        let nx = cx + dx * walk_dist;
        let ny = cy + dy * walk_dist;

        // Skip if outside polygon
        if polygon_sdf(rot_poly, nx, ny) > 0.0 {
            continue;
        }

        // Compute gradient at this point
        let (gx, gy) = sdf_gradient(rot_poly, nx, ny);
        let grad_len = (gx * gx + gy * gy).sqrt();
        if grad_len < 1e-10 {
            continue;
        }

        // Normalize gradient
        let gnx = gx / grad_len;
        let gny = gy / grad_len;

        // Walk along gradient toward boundary
        let mut px = nx;
        let mut py = ny;
        let mut dist = 0.0;
        while polygon_sdf(rot_poly, px, py) < 0.0 && dist < GRADIENT_MAX_DIST {
            px += gnx * GRADIENT_GRADIENT_STEP;
            py += gny * GRADIENT_GRADIENT_STEP;
            dist += GRADIENT_GRADIENT_STEP;
        }

        // Expand from this new point as center
        let margin_x = (ex1 - ex0) * GRADIENT_MARGIN;
        let margin_y = (ey1 - ey0) * GRADIENT_MARGIN;

        let start_x0 = (px - margin_x).max(minx);
        let start_y0 = (py - margin_y).max(miny);
        let start_x1 = (px + margin_x).min(maxx);
        let start_y1 = (py + margin_y).min(maxy);

        if start_x1 <= start_x0 || start_y1 <= start_y0 {
            continue;
        }

        let reexpanded = expand_rect_to_boundary(rot_poly, start_x0, start_y0, start_x1, start_y1, max_ratio, min_ratio);

        let (rx0, ry0, rx1, ry1) = reexpanded;
        let area = (rx1 - rx0) * (ry1 - ry0);

        if area > best_area {
            best_area = area;
            best_rect = reexpanded;
        }
    }

    best_rect
}
