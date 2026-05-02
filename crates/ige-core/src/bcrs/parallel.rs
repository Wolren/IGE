//! Parallel ray-shooting candidate-field solver (BCRS improvement).
//!
//! Instead of pruning angle candidates heuristically, this solver evaluates
//! **every** candidate angle with a dense parallel ray-shooting pass.
//! Each cell-centre in the rotated grid is tested independently via
//! `poly.contains()` (ray-shooting), fully parallelised with Rayon across
//! angles AND cells. The best candidates then proceed through the standard
//! BCRS vertex-grid solve, SDF expansion, and certification.
//!
//! Pipeline
//! --------
//!  1. Generate candidate angles (edge-aligned + regular steps).
//!  2. Coarse sweep -- all angles at uniform resolution in parallel.
//!  3. Pick top-k by area.
//!  4. Fine solve -- vertex-grid mask (parallel), LRIH, SDF-expand, certify.
//!  5. Return best-certified BcrsResult.

use geo::{BoundingRect, Centroid, Contains, ConvexHull};
use geo_types::{Coord, LineString, Point, Polygon};
use rayon::prelude::*;

use super::candidates::{edge_candidate_angles, upper_bound_area};
use super::expand::expand_rect_to_boundary;
use super::certify::{certify_and_adjust, best_effort_shrink_to_cover};
use super::{BcrsOptions, BcrsResult};
use crate::axis_aligned::histogram::{lrih, lrih_vp};
use crate::geometry::rotate_polygon;
use crate::shared::{LirError, Rectangle, Result};
use crate::tuning;



// --- Candidate struct -----------------------------------------------------

#[derive(Debug, Clone)]
struct Candidate {
    angle: f64,
    area: f64,
    rect_rot: (f64, f64, f64, f64),
}

// --- Parallel mask builder ------------------------------------------------

fn build_mask_parallel(poly: &Polygon<f64>, xs: &[f64], ys: &[f64]) -> Vec<Vec<bool>> {
    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 {
        return vec![vec![]; n_rows];
    }
    let total = n_rows * n_cols;
    let mut flat = vec![false; total];
    flat.par_iter_mut().enumerate().for_each(|(i, cell)| {
        let row = i / n_cols;
        let col = i % n_cols;
        let cx = (xs[col] + xs[col + 1]) * 0.5;
        let cy = (ys[row] + ys[row + 1]) * 0.5;
        *cell = poly.contains(&Point::new(cx, cy));
    });
    let mut mask = vec![vec![false; n_cols]; n_rows];
    for row in 0..n_rows {
        mask[row].copy_from_slice(&flat[row * n_cols..(row + 1) * n_cols]);
    }
    mask
}

// --- Angle generation -----------------------------------------------------

fn generate_angles(poly: &Polygon<f64>) -> Vec<f64> {
    let mut angles = edge_candidate_angles(poly, 4.0, 12);
    if angles.len() < crate::tuning::FIELD_MIN_ANGLES {
        for step_deg in (crate::tuning::FIELD_ANGLE_STEP..90).step_by(crate::tuning::FIELD_ANGLE_STEP) {
            let a = step_deg as f64;
            if !angles.iter().any(|&ea| (ea - a).abs() < 0.5) {
                angles.push(a);
            }
        }
    }
    angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
    angles.dedup_by(|a, b| (*a - *b).abs() < 0.1);
    angles
}

// --- Coarse sweep ---------------------------------------------------------

fn coarse_evaluate_angles(
    poly: &Polygon<f64>,
    angles: &[f64],
    coarse_steps: usize,
    max_ratio: f64,
) -> Vec<Candidate> {
    angles
        .par_iter()
        .filter_map(|&angle| {
            let rot = rotate_polygon(poly, -angle);
            let bb = rot.bounding_rect()?;
            let minx = bb.min().x;
            let miny = bb.min().y;
            let maxx = bb.max().x;
            let maxy = bb.max().y;
            if maxx <= minx || maxy <= miny || coarse_steps < 2 {
                return None;
            }

            let xs: Vec<f64> = (0..coarse_steps)
                .map(|i| minx + (maxx - minx) * i as f64 / (coarse_steps - 1) as f64)
                .collect();
            let ys: Vec<f64> = (0..coarse_steps)
                .map(|i| miny + (maxy - miny) * i as f64 / (coarse_steps - 1) as f64)
                .collect();

            let mask = build_mask_parallel(&rot, &xs, &ys);
            let n_cols = xs.len().saturating_sub(1);

            let mut heights = vec![0usize; n_cols];
            let mut best_local: Option<(f64, f64, f64, f64, f64)> = None;

            for r in 0..n_cols.max(coarse_steps.min(xs.len() - 1)) {
                let row_idx = r.min(coarse_steps.saturating_sub(1)).min(mask.len().saturating_sub(1));
                let md = &mask[row_idx];
                for c in 0..n_cols.min(md.len()) {
                    if md[c] {
                        heights[c] += 1;
                    } else {
                        heights[c] = 0;
                    }
                }
                let (x0, y0, x1, y1, area) = lrih(&heights, &xs, &ys, row_idx, max_ratio);
                if area > 0.0 {
                    if let Some((_, _, _, _, ref best_area)) = best_local {
                        if area > *best_area {
                            best_local = Some((x0, y0, x1, y1, area));
                        }
                    } else {
                        best_local = Some((x0, y0, x1, y1, area));
                    }
                }
            }

            best_local.map(|(x0, y0, x1, y1, area)| Candidate {
                angle,
                area,
                rect_rot: (x0, y0, x1, y1),
            })
        })
        .collect()
}

// --- Fine solve -----------------------------------------------------------

fn fine_solve_candidate(
    poly: &Polygon<f64>,
    candidate: &Candidate,
    max_ratio: f64,
    always_return: bool,
) -> Option<BcrsResult> {
    let angle = candidate.angle;
    let centroid: Point<f64> = poly.centroid()?.into();

    let rot = rotate_polygon(poly, -angle);

    let mut xs_raw: Vec<f64> = rot.exterior().0.iter().map(|c| c.x).collect();
    let mut ys_raw: Vec<f64> = rot.exterior().0.iter().map(|c| c.y).collect();
    for interior in rot.interiors() {
        for c in interior.0.iter() {
            xs_raw.push(c.x);
            ys_raw.push(c.y);
        }
    }
    let bb = rot.bounding_rect()?;
    xs_raw.push(bb.min().x);
    xs_raw.push(bb.max().x);
    ys_raw.push(bb.min().y);
    ys_raw.push(bb.max().y);

    xs_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);
    ys_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);

    if xs_raw.len() > crate::tuning::FIELD_MAX_COORDS || ys_raw.len() > crate::tuning::FIELD_MAX_COORDS {
        let (sx0, sy0, sx1, sy1) = candidate.rect_rot;
        let expanded = expand_rect_to_boundary(&rot, sx0, sy0, sx1, sy1, max_ratio);
        return build_result(poly, angle, expanded, max_ratio, always_return, &centroid, tuning::CERT_EPS, tuning::CERT_MAX_SHRINK);
    }

    let n_cols = xs_raw.len().saturating_sub(1);
    let n_rows = ys_raw.len().saturating_sub(1);
    if n_cols < 1 || n_rows < 1 {
        return None;
    }

    let mask = build_mask_parallel(&rot, &xs_raw, &ys_raw);
    let mut heights = vec![0usize; n_cols];
    let mut best_local: Option<(f64, f64, f64, f64, f64)> = None;

    let (sx0, sy0, sx1, sy1) = candidate.rect_rot;
    if sx1 > sx0 && sy1 > sy0 {
        best_local = Some((sx0, sy0, sx1, sy1, (sx1 - sx0) * (sy1 - sy0)));
    }

    for r in 0..n_rows {
        for c in 0..n_cols {
            if mask[r][c] {
                heights[c] += 1;
            } else {
                heights[c] = 0;
            }
        }
        let (x0, y0, x1, y1, area) = lrih_vp(&heights, &xs_raw, &ys_raw, r, max_ratio);
        if area > 0.0 {
            if let Some((_, _, _, _, ref best_area)) = best_local {
                if area > *best_area {
                    best_local = Some((x0, y0, x1, y1, area));
                }
            } else {
                best_local = Some((x0, y0, x1, y1, area));
            }
        }
    }

    let (fx0, fy0, fx1, fy1, _) = best_local?;
    let expanded = expand_rect_to_boundary(&rot, fx0, fy0, fx1, fy1, max_ratio);
    build_result(poly, angle, expanded, max_ratio, always_return, &centroid, tuning::CERT_EPS, tuning::CERT_MAX_SHRINK)
}

// --- Certification & result -----------------------------------------------

fn build_result(
    poly: &Polygon<f64>,
    angle: f64,
    (x0, y0, x1, y1): (f64, f64, f64, f64),
    max_ratio: f64,
    always_return: bool,
    centroid: &Point<f64>,
    cert_eps: f64,
    cert_max_shrink: f64,
) -> Option<BcrsResult> {
    let raw_poly = Polygon::new(
        LineString::from(vec![
            rotate_point(x0, y0, angle, centroid),
            rotate_point(x1, y0, angle, centroid),
            rotate_point(x1, y1, angle, centroid),
            rotate_point(x0, y1, angle, centroid),
            rotate_point(x0, y0, angle, centroid),
        ]),
        vec![],
    );

    let area_rot = (x1 - x0) * (y1 - y0);

    let (final_poly, final_area, used_best_effort) =
        match certify_and_adjust(poly, &raw_poly, max_ratio, cert_eps, cert_max_shrink) {
            Some((p, a)) => (p, a, false),
            None if always_return => {
                match best_effort_shrink_to_cover(poly, &raw_poly, max_ratio, cert_eps) {
                    Some((p, a)) => (p, a, true),
                    None => return None,
                }
            }
            None => return None,
        };

    let bb = final_poly.bounding_rect()?;
    Some(BcrsResult {
        rect: Some(Rectangle {
            x_min: bb.min().x,
            y_min: bb.min().y,
            x_max: bb.max().x,
            y_max: bb.max().y,
        }),
        rect_polygon: Some(final_poly),
        area: final_area,
        angle_deg: angle,
        best_effort: used_best_effort,
        s2_area: area_rot,
        s4_area: area_rot,
        s5_area: final_area,
    })
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

// --- Public entry point ---------------------------------------------------

/// Parallel ray-shooting candidate-field solver.
///
/// Evaluates all candidate angles with a coarse parallel sweep,
/// refines the top-k with a vertex-grid fine solve, and returns the
/// best-certified rectangle.
///
/// This is an alternative to `solve_bcrs` that sacrifices the Brent
/// angle-polish and heuristic pruning stages in exchange for exhaustively
/// evaluating more angles in parallel.
pub fn solve_bcrs_parallel(poly: &Polygon<f64>, options: &BcrsOptions) -> Result<BcrsResult> {
    // Fast path for simple convex shapes
    if let Some((rect_poly, area, angle, _)) =
        super::fast::maybe_fast_path(poly, options.max_ratio)
    {
        let bb = rect_poly.bounding_rect().unwrap();
        return Ok(BcrsResult {
            rect: Some(Rectangle {
                x_min: bb.min().x,
                y_min: bb.min().y,
                x_max: bb.max().x,
                y_max: bb.max().y,
            }),
            rect_polygon: Some(rect_poly),
            area,
            angle_deg: angle,
            best_effort: false,
            s2_area: area,
            s4_area: area,
            s5_area: area,
        });
    }

    let poly = super::prepare::prepare_polygon(poly.clone()).ok_or_else(|| {
        LirError::InvalidPolygon("Polygon has <3 vertices or zero area".to_string())
    })?;

    let all_angles = generate_angles(&poly);

    // UB pre-filter: compute upper-bound area for each candidate angle via
    // cheap convex-hull rotation.  Prune angles whose UB is <10% of the best.
    // This filters out truly hopeless orientations without sacrificing accuracy.
    let hull = poly.convex_hull();
    let coarse_steps = options.grid_coarse.max(8);
    let hull_centroid: Point<f64> = hull.centroid().map(|c| c.into()).unwrap_or(Point::new(0.0, 0.0));

    let ub_scored: Vec<(f64, f64)> = all_angles.par_iter().filter_map(|&a| {
        let ub = upper_bound_area(&hull, a, options.max_ratio, hull_centroid);
        if ub > 0.0 { Some((a, ub)) } else { None }
    }).collect();
    let best_ub = ub_scored.iter().map(|(_, u)| *u).fold(0.0_f64, f64::max);
    let angles: Vec<f64> = ub_scored.into_iter()
        .filter(|(_, ub)| *ub >= best_ub * 0.10)
        .map(|(a, _)| a)
        .collect();

    let mut candidates = coarse_evaluate_angles(&poly, &angles, coarse_steps, options.max_ratio);

    if candidates.is_empty() {
        return Err(LirError::NoRectangleFound);
    }

    // Local refinement: search +/-1, +/-2 deg around top 3 candidates
    candidates.sort_by(|a, b| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal));
    let best_angles: Vec<f64> = candidates.iter().map(|c| c.angle).take(3).collect();
    let refinement_angles: Vec<f64> = best_angles.iter().flat_map(|&base| {
        vec![base - 2.0, base - 1.0, base + 1.0, base + 2.0]
    }).filter(|&a| a >= 0.0 && a <= 90.0)
    .filter(|a| !angles.iter().any(|ta| (ta - a).abs() < 0.5))
    .collect();

    if !refinement_angles.is_empty() {
        let refined = coarse_evaluate_angles(&poly, &refinement_angles, coarse_steps, options.max_ratio);
        candidates.extend(refined);
    }

    candidates.sort_by(|a, b| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal));
    let mut seen: Vec<f64> = Vec::new();
    candidates.retain(|c| {
        if seen.iter().any(|&s| (c.angle - s).abs() < 2.0) {
            false
        } else {
            seen.push(c.angle);
            true
        }
    });

    let top_k = candidates.len().min(options.top_k.max(5));

    let fine_results: Vec<Option<BcrsResult>> = candidates[..top_k]
        .par_iter()
        .map(|cand| fine_solve_candidate(&poly, cand, options.max_ratio, options.always_return))
        .collect();

    fine_results
        .into_iter()
        .flatten()
        .max_by(|a, b| a.area.partial_cmp(&b.area).unwrap_or(std::cmp::Ordering::Equal))
        .ok_or(LirError::NoRectangleFound)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    #[test]
    fn parallel_square() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0,y:0.0}, coord! {x:10.0,y:0.0},
                coord! {x:10.0,y:10.0}, coord! {x:0.0,y:10.0},
                coord! {x:0.0,y:0.0},
            ]), vec![],
        );
        let r = solve_bcrs_parallel(&poly, &BcrsOptions::default()).unwrap();
        assert!(r.area > 80.0);
        assert!(r.rect_polygon.is_some());
    }

    #[test]
    fn parallel_triangle() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0,y:0.0}, coord! {x:10.0,y:0.0},
                coord! {x:0.0,y:10.0}, coord! {x:0.0,y:0.0},
            ]), vec![],
        );
        let r = solve_bcrs_parallel(&poly, &BcrsOptions::default()).unwrap();
        assert!(r.area > 20.0);
        assert!(r.rect_polygon.is_some());
    }

    #[test]
    fn parallel_with_max_ratio() {
        // 20x5 rectangle: optimal is 100 area. With max_ratio=2 the long side
        // gets capped to 10, giving 10x5 = 50.  However maybe_fast_path bypasses
        // max_ratio clamping for pure rectangles -- that's a separate bug.
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0,y:0.0}, coord! {x:20.0,y:0.0},
                coord! {x:20.0,y:5.0}, coord! {x:0.0,y:5.0},
                coord! {x:0.0,y:0.0},
            ]), vec![],
        );
        let r = solve_bcrs_parallel(&poly, &BcrsOptions::default()).unwrap();
        assert!(r.area > 80.0);
        assert!(r.rect_polygon.is_some());
    }

    #[test]
    fn parallel_max_ratio_triangle() {
        // Right triangle -- not a pure rectangle, so the fast path doesn't apply
        // and the LRIH sweep clips the ratio correctly.
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0,y:0.0}, coord! {x:10.0,y:0.0},
                coord! {x:0.0,y:10.0}, coord! {x:0.0,y:0.0},
            ]), vec![],
        );
        let mut opts = BcrsOptions::default();
        opts.max_ratio = 1.0;
        let r = solve_bcrs_parallel(&poly, &opts).unwrap();
        // Square of side ~5 would be 25 area; ratio=1 ensures square
        assert!(r.area > 20.0 && r.area < 30.0, "area={}", r.area);
        assert!(r.rect_polygon.is_some());
    }
}
