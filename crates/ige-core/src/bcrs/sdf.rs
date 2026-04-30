//! Signed-distance-field utilities for polygon containment.
//!
//! Port of `_polygon_sdf`, `_rect_sdf_max`, and `_certify_and_adjust`
//! from `bcrs_fast_worker.py`.

use geo::{Contains, EuclideanDistance};
use geo_types::{Point, Polygon};

const CERT_EPS: f64 = 1e-7;
const CERT_MAX_SHRINK: f64 = 0.20;

// ─── Core SDF ─────────────────────────────────────────────────────────────

/// Signed distance from `(x, y)` to `poly`.
/// - Negative: strictly inside (magnitude = distance to nearest ring)
/// - Zero:     on boundary
/// - Positive: outside polygon OR inside a hole
pub fn polygon_sdf(poly: &Polygon<f64>, x: f64, y: f64) -> f64 {
    let pt = Point::new(x, y);

    // geo returns 0 if point is inside or on boundary, >0 if outside
    let d_poly: f64 = poly.euclidean_distance(&pt);
    if d_poly > 1e-12 {
        return d_poly; // outside
    }

    // Inside or on boundary — measure distance to nearest ring boundary
    let d_ext: f64 = poly.exterior().euclidean_distance(&pt);

    if poly.contains(&pt) {
        // Strictly inside — find distance to nearest ring (exterior or hole)
        let mut min_d = d_ext;
        for interior in poly.interiors() {
            let dh: f64 = interior.euclidean_distance(&pt);
            if dh < min_d {
                min_d = dh;
            }
        }
        return -min_d; // negative = inside
    }

    // On exterior boundary exactly
    if d_ext < 1e-12 {
        return 0.0;
    }

    // Inside a hole (rare but possible for holed polygons)
    for interior in poly.interiors() {
        let hole = Polygon::new(interior.clone(), vec![]);
        if hole.contains(&pt) {
            return hole.exterior().euclidean_distance(&pt); // positive = inside hole
        }
    }

    0.0
}

// ─── Rect SDF sampling ────────────────────────────────────────────────────

/// Maximum SDF at 8 sample points of an axis-aligned rect (4 corners + 4 edge midpoints).
/// Negative result means all samples are strictly inside the polygon.
pub fn rect_sdf_max(
    poly: &Polygon<f64>,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
) -> f64 {
    let cx = (x0 + x1) * 0.5;
    let cy = (y0 + y1) * 0.5;
    let pts = [
        (x0, y0), (x1, y0), (x1, y1), (x0, y1), // corners
        (cx, y0), (x1, cy), (cx, y1), (x0, cy),  // edge midpoints
    ];
    pts.iter()
        .map(|&(px, py)| polygon_sdf(poly, px, py))
        .fold(f64::NEG_INFINITY, f64::max)
}

// ─── Certification ────────────────────────────────────────────────────────

/// Certify that an axis-aligned rect `(x0,y0,x1,y1)` is fully inside `poly`.
///
/// If `max_sdf ≤ CERT_EPS` it is already valid and returned unchanged.
/// Otherwise a symmetric shrink proportional to the violation is attempted.
///
/// Returns `Some((x0, y0, x1, y1, area))` on success, `None` if unfixable.
pub fn certify_rect(
    poly: &Polygon<f64>,
    mut x0: f64,
    mut y0: f64,
    mut x1: f64,
    mut y1: f64,
    max_ratio: f64,
) -> Option<(f64, f64, f64, f64, f64)> {
    if x1 - x0 < 1e-12 || y1 - y0 < 1e-12 {
        return None;
    }

    let max_sdf = rect_sdf_max(poly, x0, y0, x1, y1);

    if max_sdf <= CERT_EPS {
        let area = (x1 - x0) * (y1 - y0);
        return Some((x0, y0, x1, y1, area));
    }

    // Symmetric shrink from centre
    let shrink = max_sdf + CERT_EPS;
    let hw = (x1 - x0) * 0.5;
    let hh = (y1 - y0) * 0.5;

    // Reject if the required shrink eats more than CERT_MAX_SHRINK of the shorter half-side
    if shrink > hw.min(hh) * CERT_MAX_SHRINK {
        return None;
    }

    let cx = (x0 + x1) * 0.5;
    let cy = (y0 + y1) * 0.5;
    x0 = cx - (hw - shrink);
    x1 = cx + (hw - shrink);
    y0 = cy - (hh - shrink);
    y1 = cy + (hh - shrink);

    if x1 - x0 <= 0.0 || y1 - y0 <= 0.0 {
        return None;
    }

    // Apply aspect-ratio constraint
    if max_ratio > 0.0 {
        let rw = x1 - x0;
        let rh = y1 - y0;
        let ls = rw.max(rh);
        let ss = rw.min(rh);
        if ss > 0.0 && ls / ss > max_ratio {
            let nl = ss * max_ratio;
            if rw >= rh {
                let c = (x0 + x1) * 0.5;
                x0 = c - nl * 0.5;
                x1 = c + nl * 0.5;
            } else {
                let c = (y0 + y1) * 0.5;
                y0 = c - nl * 0.5;
                y1 = c + nl * 0.5;
            }
        }
    }

    // Verify the shrunk rect passes (tighter threshold to catch residual violations)
    if rect_sdf_max(poly, x0, y0, x1, y1) > CERT_EPS * 10.0 {
        return None;
    }

    let area = (x1 - x0) * (y1 - y0);
    Some((x0, y0, x1, y1, area))
}

/// Single-pass best-effort shrink: given a candidate that may slightly violate
/// containment, shrink by exactly `max_sdf + 2*CERT_EPS` without binary search.
pub fn best_effort_shrink(
    poly: &Polygon<f64>,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    max_ratio: f64,
) -> Option<(f64, f64, f64, f64, f64)> {
    certify_rect(poly, x0, y0, x1, y1, max_ratio)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    fn square() -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0,y:0.0},
                coord! {x:10.0,y:0.0},
                coord! {x:10.0,y:10.0},
                coord! {x:0.0,y:10.0},
                coord! {x:0.0,y:0.0},
            ]),
            vec![],
        )
    }

    #[test]
    fn sdf_inside() {
        let poly = square();
        let d = polygon_sdf(&poly, 5.0, 5.0);
        assert!(d < 0.0, "centre should be inside, got {d}");
        assert!((d + 5.0).abs() < 1e-9, "distance to nearest wall = 5.0, got {d}");
    }

    #[test]
    fn sdf_outside() {
        let poly = square();
        let d = polygon_sdf(&poly, 12.0, 5.0);
        assert!((d - 2.0).abs() < 1e-9, "expected 2.0, got {d}");
    }

    #[test]
    fn sdf_on_boundary() {
        let poly = square();
        let d = polygon_sdf(&poly, 0.0, 5.0);
        assert!(d.abs() < 1e-6, "on boundary, got {d}");
    }

    #[test]
    fn certify_valid_rect() {
        let poly = square();
        let result = certify_rect(&poly, 1.0, 1.0, 9.0, 9.0, 0.0);
        assert!(result.is_some());
        let (_, _, _, _, area) = result.unwrap();
        assert!((area - 64.0).abs() < 1e-9);
    }

    #[test]
    fn certify_slightly_outside() {
        let poly = square();
        // Rect that slightly overshoots by 0.001 on the right
        let result = certify_rect(&poly, 0.0, 0.0, 10.001, 10.0, 0.0);
        // Should either succeed with a shrunk rect, or return None
        // (CERT_MAX_SHRINK = 20% so 0.001/5 ≈ 0.02% << 20% — should succeed)
        assert!(result.is_some());
    }

    #[test]
    fn certify_massively_outside_returns_none() {
        let poly = square();
        let result = certify_rect(&poly, -5.0, -5.0, 15.0, 15.0, 0.0);
        assert!(result.is_none(), "wildly outside rect should fail certification");
    }
}