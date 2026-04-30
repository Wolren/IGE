//! SDF-guided boundary expansion.
//!
//! Port of `_expand_rect_to_boundary` from `bcrs_fast_worker.py`.
//! Uses the signed-distance field at edge midpoints to bound binary search
//! for the maximum expansion of each side.

use geo::{BoundingRect, Contains};
use geo_types::{Coord, LineString, Point, Polygon};

use crate::bcrs::sdf::polygon_sdf;

const BINARY_STEPS: usize = 24;
const EXPAND_ITERS: usize = 3;

/// Proper geometric containment check: verifies all 4 corners are inside AND
/// no rect edge intersects the polygon boundary. Equivalent to Shapely's
/// `prep.covers(box(x0,y0,x1,y1))`.
fn rect_covers(poly: &Polygon<f64>, x0: f64, y0: f64, x1: f64, y1: f64) -> bool {
    if x1 - x0 < 1e-12 || y1 - y0 < 1e-12 {
        return false;
    }

    // Stage 1: fast corner check (catches most rejections early)
    let corners = [
        Point::new(x0, y0),
        Point::new(x1, y0),
        Point::new(x1, y1),
        Point::new(x0, y1),
    ];
    if !corners.iter().all(|p| poly.contains(p)) {
        return false;
    }

    // Stage 2: check no rect edge crosses the polygon boundary
    // Build rect edges as line segments
    let edges = [
        (Coord { x: x0, y: y0 }, Coord { x: x1, y: y0 }), // bottom
        (Coord { x: x1, y: y0 }, Coord { x: x1, y: y1 }), // right
        (Coord { x: x1, y: y1 }, Coord { x: x0, y: y1 }), // top
        (Coord { x: x0, y: y1 }, Coord { x: x0, y: y0 }), // left
    ];

    // Check against exterior ring
    if rect_edges_intersect(&edges, poly.exterior()) {
        return false;
    }

    // Check against interior rings (holes)
    for interior in poly.interiors() {
        if rect_edges_intersect(&edges, interior) {
            return false;
        }
    }

    true
}

fn rect_edges_intersect(edges: &[(Coord<f64>, Coord<f64>); 4], ring: &LineString<f64>) -> bool {
    let n = ring.0.len();
    if n < 2 {
        return false;
    }
    for i in 0..n - 1 {
        let p1 = ring.0[i];
        let p2 = ring.0[i + 1];
        for &(a, b) in edges.iter() {
            if segments_intersect(a, b, p1, p2) {
                return true;
            }
        }
    }
    false
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

    // Collinear boundary cases — consider as non-intersecting for containment
    // (the corner check already verifies endpoints are fine; collinear overlaps
    // on the boundary are acceptable for `covers`)
    false
}

fn clamp_aspect_ratio(mut x0: f64, mut y0: f64, mut x1: f64, mut y1: f64, max_ratio: f64) -> (f64, f64, f64, f64) {
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
) -> (f64, f64, f64, f64) {
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
    if !rect_covers(rot_poly, x0, y0, x1, y1) {
        let cx_c = (x0 + x1) * 0.5;
        let cy_c = (y0 + y1) * 0.5;
        let hw = (x1 - x0) * 0.5;
        let hh = (y1 - y0) * 0.5;
        let mut lo = 0.0_f64;
        let mut hi = 1.0_f64;
        for _ in 0..36 {
            let mid = (lo + hi) * 0.5;
            if rect_covers(rot_poly, cx_c - hw * mid, cy_c - hh * mid, cx_c + hw * mid, cy_c + hh * mid) {
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
        // Left
        if x0 > minx {
            let sdf = polygon_sdf(rot_poly, x0, (y0 + y1) * 0.5);
            let hi_d = if sdf < 0.0 {
                (x0 - minx).min(sdf.abs())
            } else {
                x0 - minx
            };
            if hi_d > 1e-12 {
                let mut lo_d = 0.0_f64;
                let mut hi_d = hi_d;
                for _ in 0..BINARY_STEPS {
                    let mid = (lo_d + hi_d) * 0.5;
                    if rect_covers(rot_poly, x0 - mid, y0, x1, y1) {
                        lo_d = mid;
                    } else {
                        hi_d = mid;
                    }
                }
                x0 -= lo_d;
            }
        }

        // Right
        if x1 < maxx {
            let sdf = polygon_sdf(rot_poly, x1, (y0 + y1) * 0.5);
            let hi_d = if sdf < 0.0 {
                (maxx - x1).min(sdf.abs())
            } else {
                maxx - x1
            };
            if hi_d > 1e-12 {
                let mut lo_d = 0.0_f64;
                let mut hi_d = hi_d;
                for _ in 0..BINARY_STEPS {
                    let mid = (lo_d + hi_d) * 0.5;
                    if rect_covers(rot_poly, x0, y0, x1 + mid, y1) {
                        lo_d = mid;
                    } else {
                        hi_d = mid;
                    }
                }
                x1 += lo_d;
            }
        }

        // Bottom
        if y0 > miny {
            let sdf = polygon_sdf(rot_poly, (x0 + x1) * 0.5, y0);
            let hi_d = if sdf < 0.0 {
                (y0 - miny).min(sdf.abs())
            } else {
                y0 - miny
            };
            if hi_d > 1e-12 {
                let mut lo_d = 0.0_f64;
                let mut hi_d = hi_d;
                for _ in 0..BINARY_STEPS {
                    let mid = (lo_d + hi_d) * 0.5;
                    if rect_covers(rot_poly, x0, y0 - mid, x1, y1) {
                        lo_d = mid;
                    } else {
                        hi_d = mid;
                    }
                }
                y0 -= lo_d;
            }
        }

        // Top
        if y1 < maxy {
            let sdf = polygon_sdf(rot_poly, (x0 + x1) * 0.5, y1);
            let hi_d = if sdf < 0.0 {
                (maxy - y1).min(sdf.abs())
            } else {
                maxy - y1
            };
            if hi_d > 1e-12 {
                let mut lo_d = 0.0_f64;
                let mut hi_d = hi_d;
                for _ in 0..BINARY_STEPS {
                    let mid = (lo_d + hi_d) * 0.5;
                    if rect_covers(rot_poly, x0, y0, x1, y1 + mid) {
                        lo_d = mid;
                    } else {
                        hi_d = mid;
                    }
                }
                y1 += lo_d;
            }
        }
    }

    (x0, y0, x1, y1) = clamp_aspect_ratio(x0, y0, x1, y1, max_ratio);

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
        let (x0, y0, x1, y1) = expand_rect_to_boundary(&poly, 1.0, 1.0, 9.0, 9.0, 0.0);
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
        // This rect has all corners inside but its edge crosses the indentation
        assert!(!rect_covers(&poly, 1.0, 1.0, 9.0, 8.0));
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
        assert!(rect_covers(&poly, 2.0, 2.0, 8.0, 8.0));
    }
}
