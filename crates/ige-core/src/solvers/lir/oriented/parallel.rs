//! Parallel ray-shooting candidate-field solver (LIR Oriented improvement).
//!
//! Instead of pruning angle candidates heuristically, this solver evaluates
//! **every** candidate angle with a scanline-rasterised grid.  Each row is
//! processed by computing all polygon-edge x-crossings at the row's centre y,
//! then filling columns between paired crossings (even-odd rule).  This is
//! O(n_edges + n_cols) per row instead of O(n_cols × n_edges) per cell.
//!
//! Pipeline
//! --------
//!  1. Generate candidate angles (edge-aligned + regular steps).
//!  2. Coarse sweep -- all angles at uniform resolution in parallel.
//!  3. Pick top-k by area.
//!  4. Fine solve -- vertex-grid mask (parallel), LRIH, SDF-expand, certify.
//!  5. Return best-certified LirOrientedResult.

use geo::{BoundingRect, Centroid, ConvexHull};
use geo_types::{Coord, LineString, Point, Polygon};
use rayon::prelude::*;

use super::candidates::{edge_candidate_angles, upper_bound_area};
use super::expand::expand_rect_to_boundary;
use super::certify::{certify_and_adjust, best_effort_shrink_to_cover};
use super::{LirOrientedOptions, LirOrientedResult};
use super::super::axis_aligned::histogram::{lrih, lrih_vp};
use crate::shared::{LirError, Rectangle, Result};



// --- Candidate struct -----------------------------------------------------

#[derive(Debug, Clone)]
struct Candidate {
    angle: f64,
    area: f64,
    rect_rot: (f64, f64, f64, f64),
}

/// Rotated-coordinate bundle returned by `rotate_coords_only`.
struct RotatedCoords {
    exterior: Vec<Coord<f64>>,
    holes: Vec<Vec<Coord<f64>>>,
    bbox: (f64, f64, f64, f64),
}

/// Rotate a polygon's coordinates around its centroid without allocating a
/// `Polygon<f64>`.  The bounding-box falls out of the single coord pass.
fn rotate_coords_only(poly: &Polygon<f64>, angle_deg: f64) -> RotatedCoords {
    let centroid: Point<f64> = poly.centroid().map(|c| c.into()).unwrap_or(Point::new(0.0, 0.0));
    let (cx, cy) = (centroid.x(), centroid.y());
    let rad = -angle_deg.to_radians();
    let (cos_a, sin_a) = (rad.cos(), rad.sin());

    let mut minx = f64::MAX; let mut miny = f64::MAX;
    let mut maxx = f64::MIN; let mut maxy = f64::MIN;

    let rotate = |c: &Coord<f64>| -> Coord<f64> {
        let dx = c.x - cx; let dy = c.y - cy;
        Coord {
            x: cx + dx * cos_a - dy * sin_a,
            y: cy + dx * sin_a + dy * cos_a,
        }
    };

    let ext: Vec<Coord<f64>> = poly.exterior().0.iter().map(|c| {
        let r = rotate(c);
        if r.x < minx { minx = r.x } if r.x > maxx { maxx = r.x }
        if r.y < miny { miny = r.y } if r.y > maxy { maxy = r.y }
        r
    }).collect();

    let holes: Vec<Vec<Coord<f64>>> = poly.interiors().iter().map(|ring| {
        ring.0.iter().map(|c| {
            let r = rotate(c);
            if r.x < minx { minx = r.x } if r.x > maxx { maxx = r.x }
            if r.y < miny { miny = r.y } if r.y > maxy { maxy = r.y }
            r
        }).collect()
    }).collect();

    RotatedCoords { exterior: ext, holes, bbox: (minx, miny, maxx, maxy) }
}

// --- Parallel mask builder ------------------------------------------------

fn build_mask_parallel(
    exterior: &[Coord<f64>],
    interiors: &[Vec<Coord<f64>>],
    xs: &[f64],
    ys: &[f64],
) -> Vec<bool> {
    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 { return Vec::new(); }

    #[derive(Clone, Copy)]
    struct ActiveEdge {
        y_min: f64,
        y_max: f64,
        x: f64,
        dx_dy: f64,
    }

    let mut mask = vec![false; n_cols * n_rows];
    let mut edges: Vec<ActiveEdge> = Vec::new();
    for coords in std::iter::once(exterior).chain(interiors.iter().map(|h| h.as_slice())) {
        for w in coords.windows(2) {
            let a = w[0];
            let b = w[1];
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
    edges.sort_by(|a, b| a.y_min.partial_cmp(&b.y_min).unwrap_or(std::cmp::Ordering::Equal));

    let mut active: Vec<ActiveEdge> = Vec::new();
    let mut next_e = 0usize;
    for r in 0..n_rows {
        let y = (ys[r] + ys[r + 1]) * 0.5;

        active.retain(|e| y < e.y_max);
        while next_e < edges.len() && edges[next_e].y_min <= y {
            if y < edges[next_e].y_max {
                let e = edges[next_e];
                active.push(ActiveEdge {
                    y_min: y,
                    y_max: e.y_max,
                    x: e.x + (y - e.y_min) * e.dx_dy,
                    dx_dy: e.dx_dy,
                });
            }
            next_e += 1;
        }

        for e in &mut active {
            e.x += (y - e.y_min) * e.dx_dy;
            e.y_min = y;
        }
        active.sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap_or(std::cmp::Ordering::Equal));

        let mut inside = false;
        let mut cross = 0usize;
        let base = r * n_cols;
        for c in 0..n_cols {
            let cx = (xs[c] + xs[c + 1]) * 0.5;
            while cross < active.len() && active[cross].x < cx {
                inside = !inside;
                cross += 1;
            }
            mask[base + c] = inside;
        }
    }

    mask
}

// --- Angle generation -----------------------------------------------------

fn generate_angles(poly: &Polygon<f64>, options: &LirOrientedOptions) -> Vec<f64> {
    let mut angles = edge_candidate_angles(poly, 4.0, 12);
    if angles.len() < options.field_min_angles {
        let step = options.field_angle_step.max(1);
        for step_deg in (step..90).step_by(step) {
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
            let rc = rotate_coords_only(poly, angle);
            let (minx, miny, maxx, maxy) = rc.bbox;
            if maxx <= minx || maxy <= miny || coarse_steps < 2 {
                return None;
            }

            let xs: Vec<f64> = (0..coarse_steps)
                .map(|i| minx + (maxx - minx) * i as f64 / (coarse_steps - 1) as f64)
                .collect();
            let ys: Vec<f64> = (0..coarse_steps)
                .map(|i| miny + (maxy - miny) * i as f64 / (coarse_steps - 1) as f64)
                .collect();

            let mask = build_mask_parallel(&rc.exterior, &rc.holes, &xs, &ys);
            let n_cols = xs.len().saturating_sub(1);
            let n_rows = ys.len().saturating_sub(1);
            if n_cols == 0 || n_rows == 0 {
                return None;
            }

            let mut heights = vec![0usize; n_cols];
            let mut best_local: Option<(f64, f64, f64, f64, f64)> = None;

            for r in 0..n_rows {
                let base = r * n_cols;
                for c in 0..n_cols {
                    if mask[base + c] {
                        heights[c] += 1;
                    } else {
                        heights[c] = 0;
                    }
                }
                let (x0, y0, x1, y1, area) = lrih(&heights, &xs, &ys, r, max_ratio);
                if area > 0.0 {
                    best_local = match best_local {
                        Some((_, _, _, _, a)) if area > a => Some((x0, y0, x1, y1, area)),
                        None => Some((x0, y0, x1, y1, area)),
                        _ => best_local,
                    };
                }
            }

            best_local.map(|(x0, y0, x1, y1, area)| Candidate { angle, area, rect_rot: (x0, y0, x1, y1) })
        })
        .collect()
}

fn coarse_evaluate_angle(
    poly: &Polygon<f64>,
    angle: f64,
    coarse_steps: usize,
    max_ratio: f64,
) -> Option<Candidate> {
    let rc = rotate_coords_only(poly, angle);
    let (minx, miny, maxx, maxy) = rc.bbox;
    if maxx <= minx || maxy <= miny || coarse_steps < 2 {
        return None;
    }

    let xs: Vec<f64> = (0..coarse_steps)
        .map(|i| minx + (maxx - minx) * i as f64 / (coarse_steps - 1) as f64)
        .collect();
    let ys: Vec<f64> = (0..coarse_steps)
        .map(|i| miny + (maxy - miny) * i as f64 / (coarse_steps - 1) as f64)
        .collect();

    let mask = build_mask_parallel(&rc.exterior, &rc.holes, &xs, &ys);
    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 {
        return None;
    }

    let mut heights = vec![0usize; n_cols];
    let mut best_local: Option<(f64, f64, f64, f64, f64)> = None;

    for r in 0..n_rows {
        let base = r * n_cols;
        for c in 0..n_cols {
            if mask[base + c] {
                heights[c] += 1;
            } else {
                heights[c] = 0;
            }
        }
        let (x0, y0, x1, y1, area) = lrih(&heights, &xs, &ys, r, max_ratio);
        if area > 0.0 {
            best_local = match best_local {
                Some((_, _, _, _, a)) if area > a => Some((x0, y0, x1, y1, area)),
                None => Some((x0, y0, x1, y1, area)),
                _ => best_local,
            };
        }
    }

    best_local.map(|(x0, y0, x1, y1, area)| Candidate { angle, area, rect_rot: (x0, y0, x1, y1) })
}

// --- Fine solve -----------------------------------------------------------

fn fine_solve_candidate(
    poly: &Polygon<f64>,
    candidate: &Candidate,
    max_ratio: f64,
    always_return: bool,
    field_max_coords: usize,
    cert_eps: f64,
    cert_max_shrink: f64,
) -> Option<LirOrientedResult> {
    let angle = candidate.angle;
    let centroid: Point<f64> = poly.centroid()?.into();

    // Rotate once — no full Polygon allocation needed for the coord pass.
    let rc = rotate_coords_only(poly, angle);
    let rot = Polygon::new(
        LineString::from(rc.exterior.clone()),
        rc.holes.iter().map(|h| LineString::from(h.clone())).collect(),
    );

    let mut xs_raw: Vec<f64> = rc.exterior.iter().map(|c| c.x).collect();
    let mut ys_raw: Vec<f64> = rc.exterior.iter().map(|c| c.y).collect();
    for hole in &rc.holes {
        for c in hole {
            xs_raw.push(c.x);
            ys_raw.push(c.y);
        }
    }
    let (bb_minx, bb_miny, bb_maxx, bb_maxy) = rc.bbox;
    xs_raw.push(bb_minx);
    xs_raw.push(bb_maxx);
    ys_raw.push(bb_miny);
    ys_raw.push(bb_maxy);

    xs_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);
    ys_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);

    if xs_raw.len() > field_max_coords || ys_raw.len() > field_max_coords {
        let (sx0, sy0, sx1, sy1) = candidate.rect_rot;
        let expanded = expand_rect_to_boundary(&rot, sx0, sy0, sx1, sy1, max_ratio);
        return build_result(poly, angle, expanded, max_ratio, always_return, &centroid, cert_eps, cert_max_shrink);
    }

    let n_cols = xs_raw.len().saturating_sub(1);
    let n_rows = ys_raw.len().saturating_sub(1);
    if n_cols < 1 || n_rows < 1 {
        return None;
    }

    let mask = build_mask_parallel(&rc.exterior, &rc.holes, &xs_raw, &ys_raw);
    let mut heights = vec![0usize; n_cols];
    let mut best_local: Option<(f64, f64, f64, f64, f64)> = None;

    let (sx0, sy0, sx1, sy1) = candidate.rect_rot;
    if sx1 > sx0 && sy1 > sy0 {
        best_local = Some((sx0, sy0, sx1, sy1, (sx1 - sx0) * (sy1 - sy0)));
    }

    for r in 0..n_rows {
        let base = r * n_cols;
        for c in 0..n_cols {
            if mask[base + c] {
                heights[c] += 1;
            } else {
                heights[c] = 0;
            }
        }
        let (x0, y0, x1, y1, area) = lrih_vp(&heights, &xs_raw, &ys_raw, r, max_ratio);
        if area > 0.0 {
            best_local = match best_local {
                Some((_, _, _, _, a)) if area > a => Some((x0, y0, x1, y1, area)),
                None => Some((x0, y0, x1, y1, area)),
                _ => best_local,
            };
        }
    }

    let (fx0, fy0, fx1, fy1, _) = best_local?;
    let expanded = expand_rect_to_boundary(&rot, fx0, fy0, fx1, fy1, max_ratio);
    build_result(poly, angle, expanded, max_ratio, always_return, &centroid, cert_eps, cert_max_shrink)
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
) -> Option<LirOrientedResult> {
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
    Some(LirOrientedResult {
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
/// This is an alternative to `solve_lir_oriented` that sacrifices the Brent
/// angle-polish and heuristic pruning stages in exchange for exhaustively
/// evaluating more angles in parallel.
pub fn solve_lir_oriented_parallel(poly: &Polygon<f64>, options: &LirOrientedOptions) -> Result<LirOrientedResult> {
    // Fast path for simple convex shapes
    if let Some((rect_poly, area, angle, _)) =
        super::fast::maybe_fast_path(poly, options.max_ratio)
    {
        let bb = rect_poly.bounding_rect().unwrap();
        return Ok(LirOrientedResult {
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

    let all_angles = generate_angles(&poly, options);

    // UB-guided coarse search:
    // 1) score each angle with a true geometric upper bound,
    // 2) evaluate in descending UB order,
    // 3) stop once remaining UB cannot enter the current top-k coarse set.
    // This preserves coarse top-k correctness while skipping hopeless angles.
    let hull = poly.convex_hull();
    let coarse_steps = options.grid_coarse.max(8);
    let hull_centroid: Point<f64> = hull.centroid().map(|c| c.into()).unwrap_or(Point::new(0.0, 0.0));

    let mut ub_scored: Vec<(f64, f64)> = all_angles.par_iter().filter_map(|&a| {
        let ub = upper_bound_area(&hull, a, options.max_ratio, hull_centroid);
        if ub > 0.0 { Some((a, ub)) } else { None }
    }).collect();
    ub_scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    let top_needed = options.top_k.max(5);
    let mut top_areas: Vec<f64> = Vec::new();
    let mut evaluated_angles: Vec<f64> = Vec::new();
    let mut candidates: Vec<Candidate> = Vec::new();

    for (angle, ub) in ub_scored {
        if top_areas.len() >= top_needed {
            let mut kth_area = f64::INFINITY;
            for &a in &top_areas {
                if a < kth_area {
                    kth_area = a;
                }
            }
            if ub <= kth_area {
                break;
            }
        }

        evaluated_angles.push(angle);
        if let Some(c) = coarse_evaluate_angle(&poly, angle, coarse_steps, options.max_ratio) {
            let area = c.area;
            if top_areas.len() < top_needed {
                top_areas.push(area);
            } else {
                let mut min_i = 0usize;
                let mut min_v = top_areas[0];
                for (i, &v) in top_areas.iter().enumerate().skip(1) {
                    if v < min_v {
                        min_v = v;
                        min_i = i;
                    }
                }
                if area > min_v {
                    top_areas[min_i] = area;
                }
            }
            candidates.push(c);
        }
    }

    if candidates.is_empty() {
        return Err(LirError::NoRectangleFound);
    }

    // Local refinement: search +/-1, +/-2 deg around top 3 candidates
    candidates.sort_by(|a, b| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal));
    let best_angles: Vec<f64> = candidates.iter().map(|c| c.angle).take(3).collect();
    let refinement_angles: Vec<f64> = best_angles.iter().flat_map(|&base| {
        vec![base - 2.0, base - 1.0, base + 1.0, base + 2.0]
    }).filter(|&a| a >= 0.0 && a <= 90.0)
        .filter(|a| !evaluated_angles.iter().any(|ta| (ta - a).abs() < 0.5))
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

    let fine_results: Vec<Option<LirOrientedResult>> = candidates[..top_k]
        .par_iter()
        .map(|cand| {
            fine_solve_candidate(
                &poly,
                cand,
                options.max_ratio,
                options.always_return,
                options.field_max_coords,
                options.cert_eps,
                options.cert_max_shrink,
            )
        })
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
        let r = solve_lir_oriented_parallel(&poly, &LirOrientedOptions::default()).unwrap();
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
        let r = solve_lir_oriented_parallel(&poly, &LirOrientedOptions::default()).unwrap();
        assert!(r.area > 20.0);
        assert!(r.rect_polygon.is_some());
    }

    #[test]
    fn parallel_with_max_ratio() {
        // 20x5 rectangle: unconstrained optimum is 100.
        // With max_ratio=2 the long side is capped to 10, so area should be ~50.
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0,y:0.0}, coord! {x:20.0,y:0.0},
                coord! {x:20.0,y:5.0}, coord! {x:0.0,y:5.0},
                coord! {x:0.0,y:0.0},
            ]), vec![],
        );
        let mut opts = LirOrientedOptions::default();
        opts.max_ratio = 2.0;
        let r = solve_lir_oriented_parallel(&poly, &opts).unwrap();
        assert!(r.area > 45.0 && r.area < 55.0, "area={}", r.area);
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
        let mut opts = LirOrientedOptions::default();
        opts.max_ratio = 1.0;
        let r = solve_lir_oriented_parallel(&poly, &opts).unwrap();
        // Square of side ~5 would be 25 area; ratio=1 ensures square
        assert!(r.area > 20.0 && r.area < 30.0, "area={}", r.area);
        assert!(r.rect_polygon.is_some());
    }
}
