//! LIR Approximate Oriented refinement orchestrator: Stage 3-7 pipeline orchestrator.
//!
//! OpenEvolve target: ``--target lir_approximate_oriented/refine.rs --mode balanced``
//!
//! Takes the top-k angle candidates from ``heuristic_candidates``, polishes
//! each with golden-section search (Stage 3), runs LIR Approximate Oriented + SDF expansion
//! (Stage 4-5), certifies (Stage 6), and picks the best.

use std::collections::HashMap;

use geo_types::{Coord, LineString, Point, Polygon};
use ordered_float::OrderedFloat;

use super::AngleCandidate;
use crate::lir_axis_aligned::solve_axis_rect_grid;
use crate::lir_approximate_oriented::certify::{certify_and_adjust, best_effort_shrink_to_cover, rect_sdf_max_poly};
use crate::lir_approximate_oriented::fallback::conservative_inner_fallback;
use crate::lir_approximate_oriented::polish::{expand_at_angle, polish_angle};
use crate::geometry::rotate_polygon;

#[derive(Debug, Clone)]
struct CertifiedCandidate {
    rect: Polygon<f64>,
    area: f64,
    angle: f64,
    ratio: f64,
    rank: usize,
    stage2_gain: f64,
    used_best_effort: bool,
}

fn rotate_point(x: f64, y: f64, angle_deg: f64, origin: &Point<f64>) -> Coord<f64> {
    let rad = angle_deg.to_radians();
    let cos_a = rad.cos();
    let sin_a = rad.sin();
    let dx = x - origin.x();
    let dy = y - origin.y();
    Coord {
        x: origin.x() + dx * cos_a - dy * sin_a,
        y: origin.y() + dx * sin_a + dy * cos_a,
    }
}

#[derive(Debug, Clone)]
struct AngleTrial {
    angle: f64,
    rot_poly: Polygon<f64>,
    seed_bounds: Option<(f64, f64, f64, f64)>,
    seed_area: f64,
}

/// Refine the top-k candidates through Brent polish, BCRS solve,
/// SDF expansion, and certification. Returns the best certified rect.
pub(crate) fn refine_best_candidate(
    poly: &Polygon<f64>,
    candidates: &[AngleCandidate],
    grid_coarse: usize,
    grid_fine: usize,
    max_ratio: f64,
    always_return: bool,
) -> Option<(Polygon<f64>, f64, f64, f64, usize, f64, bool)> {
    let mut certified: Vec<CertifiedCandidate> = Vec::new();
    let mut fallback_best: Option<(Polygon<f64>, f64, f64, usize)> = None;
    let mut stage3_cache: HashMap<OrderedFloat<f64>, f64> = HashMap::new();

    for (rank, cand) in candidates.iter().enumerate() {
        let area_s1 = cand.area;
        let centroid = Point::new(cand.center.x(), cand.center.y());
        let orig_angle = cand.angle;

        let (cand_a, bracket_width) = polish_angle(poly, cand, grid_coarse, max_ratio, &mut stage3_cache);
        let brent_angle = cand_a.angle;

        // Adaptive delta: scale by curvature of area-vs-angle at the optimum.
        // Flat function (wide bracket) → larger delta to sample more broadly.
        // Peaked function (narrow bracket) → keep tight to avoid degenerate angles.
        let curvature = (bracket_width / crate::tuning::POLISH_XATOL).clamp(0.5, 3.0);
        let adaptive_delta = crate::tuning::ANGLE_DELTA * curvature;

        let mut angles_to_try = vec![orig_angle];
        for delta in &[0.0, adaptive_delta, -adaptive_delta] {
            let a_try = brent_angle + delta;
            if a_try >= 0.0 && a_try <= 90.0 && angles_to_try.iter().all(|&x| (a_try - x).abs() > 0.01) {
                angles_to_try.push(a_try);
            }
        }

        let mut trial_data: Vec<AngleTrial> = Vec::new();
        for &angle_try in &angles_to_try {
            let rot_poly = rotate_polygon(poly, -angle_try);
            let seed = solve_axis_rect_grid(&rot_poly, grid_coarse, max_ratio);
            let seed_bounds = seed.map(|(x0, y0, x1, y1, _)| (x0, y0, x1, y1));
            let seed_area = seed.map(|(_, _, _, _, a)| a).unwrap_or(0.0);
            trial_data.push(AngleTrial { angle: angle_try, rot_poly, seed_bounds, seed_area });
        }

        if trial_data.is_empty() { continue; }

        let mut selected: Vec<&AngleTrial> = vec![&trial_data[0]];
        let mut others: Vec<&AngleTrial> = trial_data[1..].iter().collect();
        others.sort_by(|a, b| b.seed_area.partial_cmp(&a.seed_area).unwrap_or(std::cmp::Ordering::Equal));
        for t in others {
            if selected.len() >= crate::tuning::TOP_TRIALS { break; }
            selected.push(t);
        }

        let mut best_raw_r: Option<Polygon<f64>> = None;
        let mut best_raw_a = 0.0_f64;
        let mut best_angle_this = orig_angle;

        for trial in selected {
            let result = expand_at_angle(&trial.rot_poly, trial.seed_bounds, max_ratio);
            if let Some(((rx0, ry0, rx1, ry1), area_rot)) = result {
                if area_rot <= 0.0 { continue; }
                let make_poly = || Polygon::new(
                    LineString::from(vec![
                        rotate_point(rx0, ry0, trial.angle, &centroid),
                        rotate_point(rx1, ry0, trial.angle, &centroid),
                        rotate_point(rx1, ry1, trial.angle, &centroid),
                        rotate_point(rx0, ry1, trial.angle, &centroid),
                        rotate_point(rx0, ry0, trial.angle, &centroid),
                    ]), vec![],
                );
                if area_rot > best_raw_a + 1e-6 {
                    best_raw_a = area_rot;
                    best_raw_r = Some(make_poly());
                    best_angle_this = trial.angle;
                } else if best_raw_r.is_none() {
                    best_raw_a = area_rot;
                    best_raw_r = Some(make_poly());
                    best_angle_this = trial.angle;
                }
            }
        }

        let best_raw_r = match best_raw_r { Some(r) => r, None => continue };
        if fallback_best.is_none() || best_raw_a > fallback_best.as_ref().unwrap().1 {
            fallback_best = Some((best_raw_r.clone(), best_raw_a, best_angle_this, rank));
        }

        let mut used_best_effort = false;
        let (final_rect, final_area) = match certify_and_adjust(poly, &best_raw_r, max_ratio, crate::tuning::CERT_EPS, crate::tuning::CERT_MAX_SHRINK) {
            Some((r, a)) => (r, a),
            None if always_return => match best_effort_shrink_to_cover(poly, &best_raw_r, max_ratio, crate::tuning::CERT_EPS) {
                Some((r, a)) => { used_best_effort = true; (r, a) }
                None => continue,
            },
            None => continue,
        };

        let post_sdf = rect_sdf_max_poly(poly, &final_rect);
        let (final_rect, final_area) = if post_sdf > crate::tuning::CERT_EPS {
            match certify_and_adjust(poly, &final_rect, max_ratio, crate::tuning::CERT_EPS, crate::tuning::CERT_MAX_SHRINK) {
                Some((r, a)) => (r, a),
                None => continue,
            }
        } else { (final_rect, final_area) };

        let coords: Vec<_> = final_rect.exterior().0.iter().collect();
        let w = ((coords[1].x - coords[0].x).powi(2) + (coords[1].y - coords[0].y).powi(2)).sqrt();
        let h = ((coords[2].x - coords[1].x).powi(2) + (coords[2].y - coords[1].y).powi(2)).sqrt();
        let ratio = if w.min(h) > 0.0 { w.max(h) / w.min(h) } else { 1.0 };

        certified.push(CertifiedCandidate { rect: final_rect, area: final_area, angle: best_angle_this, ratio, rank, stage2_gain: final_area - area_s1, used_best_effort });
    }

    if certified.is_empty() {
        if always_return {
            if let Some((rect_fb, _area_fb, angle_fb, rank_fb)) = fallback_best {
                if let Some((r, a)) = best_effort_shrink_to_cover(poly, &rect_fb, max_ratio, crate::tuning::CERT_EPS) {
                    let coords: Vec<_> = r.exterior().0.iter().collect();
                    let w = ((coords[1].x - coords[0].x).powi(2) + (coords[1].y - coords[0].y).powi(2)).sqrt();
                    let h = ((coords[2].x - coords[1].x).powi(2) + (coords[2].y - coords[1].y).powi(2)).sqrt();
                    let ratio = if w.min(h) > 0.0 { w.max(h) / w.min(h) } else { 1.0 };
                    return Some((r, a, angle_fb, ratio, rank_fb, a, true));
                }
            }
        }
        let centroid_fb = candidates.first().map(|c| Point::new(c.center.x(), c.center.y()))?;
        let n_rescue = candidates.len().min(8);
        if n_rescue < 3 { return None; }
        let rescue_angs: Vec<f64> = candidates[..n_rescue].iter().map(|c| c.angle).collect();
        if let Some((r, a, ang)) = conservative_inner_fallback(poly, grid_fine, max_ratio, centroid_fb, &rescue_angs) {
            let coords: Vec<_> = r.exterior().0.iter().collect();
            let w = ((coords[1].x - coords[0].x).powi(2) + (coords[1].y - coords[0].y).powi(2)).sqrt();
            let h = ((coords[2].x - coords[1].x).powi(2) + (coords[2].y - coords[1].y).powi(2)).sqrt();
            let ratio = if w.min(h) > 0.0 { w.max(h) / w.min(h) } else { 1.0 };
            return Some((r, a, ang, ratio, 0, a, true));
        }
        return None;
    }

    let best = certified.into_iter().max_by(|a, b| a.area.partial_cmp(&b.area).unwrap())?;
    Some((best.rect, best.area, best.angle, best.ratio, best.rank, best.stage2_gain, best.used_best_effort))
}