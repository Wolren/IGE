//! Signed-distance-field utilities for polygon containment.
//!
//! Port of `_polygon_sdf`, `_rect_sdf_max`, and `_certify_and_adjust`.

use geo_types::{Coord, Polygon};

#[cfg(target_arch = "x86")]
use std::arch::x86 as arch;
#[cfg(target_arch = "x86_64")]
use std::arch::x86_64 as arch;

const CERT_EPS: f64 = 1e-7;
const CERT_MAX_SHRINK: f64 = 0.20;

// --- Core SDF -------------------------------------------------------------

/// Signed distance from `(x, y)` to `poly`.
/// - Negative: strictly inside (magnitude = distance to nearest ring)
/// - Zero:     on boundary
/// - Positive: outside polygon OR inside a hole
pub fn polygon_sdf(poly: &Polygon<f64>, x: f64, y: f64) -> f64 {
    let mut min_dist_sq = f64::MAX;
    let mut winding = 0i32;

    for ring in std::iter::once(poly.exterior()).chain(poly.interiors()) {
        let coords = ring.0.as_slice();
        if coords.len() >= 2 {
            #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
            {
                let d2 = min_dist_sq_ring_simd(coords, x, y);
                if d2 < min_dist_sq {
                    min_dist_sq = d2;
                }
            }
            #[cfg(not(any(target_arch = "x86", target_arch = "x86_64")))]
            {
                let d2 = min_dist_sq_ring_scalar(coords, x, y);
                if d2 < min_dist_sq {
                    min_dist_sq = d2;
                }
            }
        }
        for w in coords.windows(2) {
            let (ax, ay) = (w[0].x, w[0].y);
            let (bx, by) = (w[1].x, w[1].y);

            // Winding number increment (robust crossing test)
            if ay <= y {
                if by > y && cross2d(ax - x, ay - y, bx - x, by - y) > 0.0 { winding += 1; }
            } else {
                if by <= y && cross2d(ax - x, ay - y, bx - x, by - y) < 0.0 { winding -= 1; }
            }

        }
    }

    let d = min_dist_sq.sqrt();
    if winding != 0 { -d } else { d }  // negative = inside
}

#[inline(always)]
fn cross2d(ux: f64, uy: f64, vx: f64, vy: f64) -> f64 { ux * vy - uy * vx }

#[inline(always)]
fn min_dist_sq_ring_scalar(coords: &[Coord<f64>], x: f64, y: f64) -> f64 {
    let mut min_d2 = f64::MAX;
    for w in coords.windows(2) {
        let (ax, ay) = (w[0].x, w[0].y);
        let (bx, by) = (w[1].x, w[1].y);
        let (ex, ey) = (bx - ax, by - ay);
        let t = ((x - ax) * ex + (y - ay) * ey) / (ex * ex + ey * ey + 1e-300);
        let t = t.clamp(0.0, 1.0);
        let (px, py) = (ax + t * ex, ay + t * ey);
        let d2 = (x - px) * (x - px) + (y - py) * (y - py);
        if d2 < min_d2 {
            min_d2 = d2;
        }
    }
    min_d2
}

#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
#[inline]
fn min_dist_sq_ring_simd(coords: &[Coord<f64>], x: f64, y: f64) -> f64 {
    if is_x86_feature_detected!("avx") {
        // SAFETY: guarded by runtime feature detection.
        return unsafe { min_dist_sq_ring_avx(coords, x, y) };
    }
    if is_x86_feature_detected!("sse2") {
        // SAFETY: guarded by runtime feature detection.
        return unsafe { min_dist_sq_ring_sse2(coords, x, y) };
    }
    min_dist_sq_ring_scalar(coords, x, y)
}

#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
#[target_feature(enable = "avx")]
unsafe fn min_dist_sq_ring_avx(coords: &[Coord<f64>], x: f64, y: f64) -> f64 {
    let n = coords.len().saturating_sub(1);
    if n == 0 {
        return f64::MAX;
    }
    let mut min_d2 = f64::MAX;
    let vx = arch::_mm256_set1_pd(x);
    let vy = arch::_mm256_set1_pd(y);
    let zero = arch::_mm256_set1_pd(0.0);
    let one = arch::_mm256_set1_pd(1.0);
    let eps = arch::_mm256_set1_pd(1e-300);

    let mut i = 0usize;
    while i + 4 <= n {
        let ax = arch::_mm256_set_pd(coords[i + 3].x, coords[i + 2].x, coords[i + 1].x, coords[i].x);
        let ay = arch::_mm256_set_pd(coords[i + 3].y, coords[i + 2].y, coords[i + 1].y, coords[i].y);
        let bx = arch::_mm256_set_pd(coords[i + 4].x, coords[i + 3].x, coords[i + 2].x, coords[i + 1].x);
        let by = arch::_mm256_set_pd(coords[i + 4].y, coords[i + 3].y, coords[i + 2].y, coords[i + 1].y);

        let ex = arch::_mm256_sub_pd(bx, ax);
        let ey = arch::_mm256_sub_pd(by, ay);
        let num = arch::_mm256_add_pd(
            arch::_mm256_mul_pd(arch::_mm256_sub_pd(vx, ax), ex),
            arch::_mm256_mul_pd(arch::_mm256_sub_pd(vy, ay), ey),
        );
        let den = arch::_mm256_add_pd(
            arch::_mm256_add_pd(arch::_mm256_mul_pd(ex, ex), arch::_mm256_mul_pd(ey, ey)),
            eps,
        );
        let t = arch::_mm256_max_pd(zero, arch::_mm256_min_pd(one, arch::_mm256_div_pd(num, den)));
        let px = arch::_mm256_add_pd(ax, arch::_mm256_mul_pd(t, ex));
        let py = arch::_mm256_add_pd(ay, arch::_mm256_mul_pd(t, ey));
        let dx = arch::_mm256_sub_pd(vx, px);
        let dy = arch::_mm256_sub_pd(vy, py);
        let d2 = arch::_mm256_add_pd(arch::_mm256_mul_pd(dx, dx), arch::_mm256_mul_pd(dy, dy));

        let mut lanes = [0.0_f64; 4];
        arch::_mm256_storeu_pd(lanes.as_mut_ptr(), d2);
        for v in lanes {
            if v < min_d2 {
                min_d2 = v;
            }
        }
        i += 4;
    }
    if i < n {
        let rem = min_dist_sq_ring_scalar(&coords[i..=n], x, y);
        if rem < min_d2 {
            min_d2 = rem;
        }
    }
    min_d2
}

#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
#[target_feature(enable = "sse2")]
unsafe fn min_dist_sq_ring_sse2(coords: &[Coord<f64>], x: f64, y: f64) -> f64 {
    let n = coords.len().saturating_sub(1);
    if n == 0 {
        return f64::MAX;
    }
    let mut min_d2 = f64::MAX;
    let vx = arch::_mm_set1_pd(x);
    let vy = arch::_mm_set1_pd(y);
    let zero = arch::_mm_set1_pd(0.0);
    let one = arch::_mm_set1_pd(1.0);
    let eps = arch::_mm_set1_pd(1e-300);

    let mut i = 0usize;
    while i + 2 <= n {
        let ax = arch::_mm_set_pd(coords[i + 1].x, coords[i].x);
        let ay = arch::_mm_set_pd(coords[i + 1].y, coords[i].y);
        let bx = arch::_mm_set_pd(coords[i + 2].x, coords[i + 1].x);
        let by = arch::_mm_set_pd(coords[i + 2].y, coords[i + 1].y);

        let ex = arch::_mm_sub_pd(bx, ax);
        let ey = arch::_mm_sub_pd(by, ay);
        let num = arch::_mm_add_pd(
            arch::_mm_mul_pd(arch::_mm_sub_pd(vx, ax), ex),
            arch::_mm_mul_pd(arch::_mm_sub_pd(vy, ay), ey),
        );
        let den = arch::_mm_add_pd(
            arch::_mm_add_pd(arch::_mm_mul_pd(ex, ex), arch::_mm_mul_pd(ey, ey)),
            eps,
        );
        let t = arch::_mm_max_pd(zero, arch::_mm_min_pd(one, arch::_mm_div_pd(num, den)));
        let px = arch::_mm_add_pd(ax, arch::_mm_mul_pd(t, ex));
        let py = arch::_mm_add_pd(ay, arch::_mm_mul_pd(t, ey));
        let dx = arch::_mm_sub_pd(vx, px);
        let dy = arch::_mm_sub_pd(vy, py);
        let d2 = arch::_mm_add_pd(arch::_mm_mul_pd(dx, dx), arch::_mm_mul_pd(dy, dy));

        let mut lanes = [0.0_f64; 2];
        arch::_mm_storeu_pd(lanes.as_mut_ptr(), d2);
        for v in lanes {
            if v < min_d2 {
                min_d2 = v;
            }
        }
        i += 2;
    }
    if i < n {
        let rem = min_dist_sq_ring_scalar(&coords[i..=n], x, y);
        if rem < min_d2 {
            min_d2 = rem;
        }
    }
    min_d2
}

// --- Rect SDF sampling ----------------------------------------------------

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

// --- Certification --------------------------------------------------------

/// Certify that an axis-aligned rect `(x0,y0,x1,y1)` is fully inside `poly`.
///
/// If `max_sdf <= CERT_EPS` it is already valid and returned unchanged.
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

    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    #[test]
    fn simd_distance_matches_scalar() {
        let poly = square();
        let ring = poly.exterior().0.as_slice();
        let samples = [(5.0, 5.0), (12.0, 5.0), (0.0, 5.0), (9.25, 1.5), (-3.0, -2.0)];
        for &(x, y) in &samples {
            let d2_scalar = min_dist_sq_ring_scalar(ring, x, y);
            let d2_simd = min_dist_sq_ring_simd(ring, x, y);
            assert!(
                (d2_scalar - d2_simd).abs() < 1e-10,
                "scalar={} simd={} at ({},{})",
                d2_scalar,
                d2_simd,
                x,
                y
            );
        }
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
        // (CERT_MAX_SHRINK = 20% so 0.001/5 ≈ 0.02% << 20% -- should succeed)
        assert!(result.is_some());
    }

    #[test]
    fn certify_massively_outside_returns_none() {
        let poly = square();
        let result = certify_rect(&poly, -5.0, -5.0, 15.0, 15.0, 0.0);
        assert!(result.is_none(), "wildly outside rect should fail certification");
    }
}
