//! Brent angle-polish and BCRS expansion.
//!
//! OpenEvolve target: ``--target bcrs/polish.rs --mode balanced``
//!
//! Stage 3: golden-section search for optimal angle within +/-POLISH_HALFWIDTH.
//! Stage 4+5: BCRS vertex-coordinate solve + SDF-guided boundary expansion.

use std::collections::HashMap;

use geo_types::Polygon;
use ordered_float::OrderedFloat;

use crate::axis_aligned::solve_axis_rect_bcrs;
use super::AngleCandidate;
use crate::bcrs::expand::expand_rect_to_boundary;
use crate::geometry::rotate_polygon;

/// Golden-section search to refine a candidate angle within +/-POLISH_HALFWIDTH.
/// Caches coarse-grid area evaluations to avoid redundant work.
/// Returns `(polished_candidate, final_bracket_width)` where the bracket width
/// indicates the curvature of the area-vs-angle function at the optimum
/// (narrow bracket = peaked, wide bracket = flat).
pub(crate) fn polish_angle(
    poly: &Polygon<f64>,
    cand: &AngleCandidate,
    grid_coarse: usize,
    max_ratio: f64,
    cache: &mut HashMap<OrderedFloat<f64>, f64>,
) -> (AngleCandidate, f64) {
    let angle_0 = cand.angle;
    let lo = (angle_0 - crate::tuning::POLISH_HALFWIDTH).max(0.0);
    let hi = (angle_0 + crate::tuning::POLISH_HALFWIDTH).min(90.0);

    if hi - lo < crate::tuning::POLISH_XATOL * 2.0 {
        return (cand.clone(), hi - lo);
    }

    let mut neg_area = |a: f64| -> f64 {
        let key = OrderedFloat((a * 10000.0).round() / 10000.0);
        if let Some(&cached) = cache.get(&key) {
            return -cached;
        }
        let rot = rotate_polygon(poly, -a);
        // Use BCRS (vertex-coordinate-based, exact precision) instead of the
        // grid for polish evaluations.  The grid at GRID_COARSE=32 cannot
        // distinguish angles closer than ~1°; BCRS evaluates exact vertex
        // positions and SDF, giving sub-0.01° angular resolution.
        let area = match solve_axis_rect_bcrs(&rot, None, max_ratio) {
            Some((_, _, _, _, a)) => a,
            None => 0.0,
        };
        cache.insert(key, area);
        -area
    };

    let phi = (5.0_f64.sqrt() - 1.0) / 2.0;
    let mut a = lo;
    let mut b = hi;
    let mut c = b - phi * (b - a);
    let mut d = a + phi * (b - a);
    let mut fc = neg_area(c);
    let mut fd = neg_area(d);

    for _ in 0..60 {
        if (b - a).abs() < crate::tuning::POLISH_XATOL {
            break;
        }
        if fc < fd {
            b = d; d = c; fd = fc;
            c = b - phi * (b - a);
            fc = neg_area(c);
        } else {
            a = c; c = d; fc = fd;
            d = a + phi * (b - a);
            fd = neg_area(d);
        }
    }

    let bracket_width = b - a;
    let best_angle = (a + b) * 0.5;
    if (best_angle - angle_0).abs() > 0.005 {
        let mut new_cand = cand.clone();
        new_cand.angle = best_angle;
        new_cand.area = -neg_area(best_angle);
        (new_cand, bracket_width)
    } else {
        (cand.clone(), bracket_width)
    }
}

/// Stage 4+5: BCRS vertex-coordinate solve followed by SDF expansion.
pub(crate) fn bcrs_expand_at_angle(
    rot_poly: &Polygon<f64>,
    seed_bounds: Option<(f64, f64, f64, f64)>,
    max_ratio: f64,
) -> Option<((f64, f64, f64, f64), f64)> {
    let bcrs_result = solve_axis_rect_bcrs(rot_poly, seed_bounds, max_ratio);
    let (bx0, by0, bx1, by1) = match bcrs_result {
        Some((x0, y0, x1, y1, a)) if a > 0.0 => (x0, y0, x1, y1),
        _ => match seed_bounds {
            Some((sx0, sy0, sx1, sy1)) if sx1 > sx0 && sy1 > sy0 => (sx0, sy0, sx1, sy1),
            _ => return None,
        },
    };
    let (bx0, by0, bx1, by1) = expand_rect_to_boundary(rot_poly, bx0, by0, bx1, by1, max_ratio);
    let area = (bx1 - bx0) * (by1 - by0);
    if area <= 0.0 { None } else { Some(((bx0, by0, bx1, by1), area)) }
}
