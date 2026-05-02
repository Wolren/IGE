//! Edge-direction angle candidate generation.
//!
//! Port of `_edge_candidate_angles`, `_upper_bound_area`
//! and `heuristic_candidates` from `bcrs_fast_worker.py`.


use geo::{BoundingRect, Centroid, ConvexHull, Rotate};
use geo_types::{Coord, LineString, Point, Polygon};

use crate::axis_aligned::solve_axis_rect_grid;
use crate::bcrs::AngleCandidate;
use crate::bcrs::prepare::simplify_for_solve;
use crate::geometry::rotate_polygon;
use rayon::prelude::*;

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

/// Extract edge-direction angle candidates in `[0deg, 90deg)` weighted by edge length.
///
/// Candidates are smoothed with a Gaussian kernel and peak-picked with a
/// minimum angular separation.  Angles 0deg and 45deg are always included.
///
/// # Arguments
/// * `poly`            -- input polygon (exterior + holes used)
/// * `min_sep_deg`     -- minimum angular distance between two peaks
/// * `max_candidates`  -- maximum number of peaks to return (before inserting 0/45)
pub fn edge_candidate_angles(
    poly: &Polygon<f64>,
    min_sep_deg: f64,
    max_candidates: usize,
) -> Vec<f64> {
    // --- Accumulate edge-length-weighted angle histogram -----------------
    let mut bins = vec![0.0_f64; 91]; // index = angle in [0,90]

    let mut accumulate_ring = |coords: &[geo_types::Coord<f64>]| {
        for i in 0..coords.len().saturating_sub(1) {
            let dx = coords[i + 1].x - coords[i].x;
            let dy = coords[i + 1].y - coords[i].y;
            let len = dx.hypot(dy);
            if len < 1e-12 {
                continue;
            }
            let angle_deg = dy.abs().atan2(dx.abs()).to_degrees();
            let bucket = ((angle_deg % 90.0).round() as usize).min(90);
            bins[bucket] += len;
        }
    };

    accumulate_ring(poly.exterior().0.as_slice());
    for ring in poly.interiors() {
        accumulate_ring(ring.0.as_slice());
    }

    // --- Gaussian smooth (kernel [0.1, 0.2, 0.4, 0.2, 0.1]) -------------
    let kernel = [0.1_f64, 0.2, 0.4, 0.2, 0.1];
    let mut smoothed = vec![0.0_f64; 91];
    for i in 0..91usize {
        let mut acc = 0.0;
        for (ki, &kv) in kernel.iter().enumerate() {
            let offset = ki as isize - 2;
            let idx = i as isize + offset;
            if (0..=90).contains(&idx) {
                acc += kv * bins[idx as usize];
            }
        }
        smoothed[i] = acc;
    }

    // --- Peak-pick with minimum separation --------------------------------
    let sep = (min_sep_deg.max(1.0) as usize).max(1);
    let mut order: Vec<usize> = (0..91).collect();
    order.sort_unstable_by(|&a, &b| smoothed[b].partial_cmp(&smoothed[a]).unwrap());

    let mut peaks: Vec<(usize, f64)> = Vec::new();
    for &idx in &order {
        if peaks
            .iter()
            .all(|&(p, _): &(usize, f64)| (idx as isize - p as isize).unsigned_abs() >= sep)
        {
            // Quadratic interpolation for sub-bin accuracy (no area loss)
            let refined = if idx > 0 && idx < 90 {
                let y0 = smoothed[idx - 1];
                let y1 = smoothed[idx];
                let y2 = smoothed[idx + 1];
                let d = (y2 - y0) / (2.0 * (2.0 * y1 - y2 - y0));
                let clamped = (idx as f64 + d).clamp(0.0, 90.0);
                if d.is_finite() && d.abs() < 1.0 { clamped } else { idx as f64 }
            } else {
                idx as f64
            };
            peaks.push((idx, refined));
        }
        if peaks.len() >= max_candidates {
            break;
        }
    }

    let mut result: Vec<f64> = peaks.into_iter().map(|(_, a)| a).collect();

    // Always include 0deg and 45deg
    if !result.iter().any(|&a| a.abs() < 0.5) {
        result.push(0.0);
    }
    if !result.iter().any(|&a| (a - 45.0).abs() < 0.5) {
        result.push(45.0);
    }

    result.sort_by(|a, b| a.partial_cmp(b).unwrap());
    result.dedup_by(|a, b| (*a - *b).abs() < 0.1);
    result
}

/// Upper-bound on the largest inscribed rectangle area at a given angle.
///
/// Rotates the convex hull by `-angle_deg`, takes its bounding box, and returns
/// half the box area (a known conservative upper bound for the inscribed problem).
pub fn upper_bound_area(
    hull: &Polygon<f64>,
    angle_deg: f64,
    max_ratio: f64,
    centroid: Point<f64>,
) -> f64 {
    let rotated = hull.rotate_around_point(-angle_deg, centroid);
    let bb = match rotated.bounding_rect() {
        Some(b) => b,
        None => return 0.0,
    };
    let bw = bb.max().x - bb.min().x;
    let bh = bb.max().y - bb.min().y;

    if bw <= 0.0 || bh <= 0.0 {
        return 0.0;
    }

    if max_ratio > 0.0 {
        let ls = bw.max(bh);
        let ss = bw.min(bh);
        if ss > 0.0 && ls / ss > max_ratio {
            let trimmed = ss * max_ratio;
            return trimmed * ss * 0.5;
        }
    }
    bw * bh * 0.5
}

/// Filter candidate angles: keep only those whose upper-bound area exceeds
/// a threshold.  Sorts by descending upper bound (most promising first).
pub fn filter_by_upper_bound(
    hull: &Polygon<f64>,
    angles: Vec<f64>,
    min_area_threshold: f64,
    max_ratio: f64,
    centroid: Point<f64>,
) -> Vec<f64> {
    let mut scored: Vec<(f64, f64)> = angles
        .into_iter()
        .map(|a| (a, upper_bound_area(hull, a, max_ratio, centroid)))
        .collect();
    scored.sort_unstable_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    scored
        .into_iter()
        .filter(|(_, ub)| *ub >= min_area_threshold)
        .map(|(a, _)| a)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    #[test]
    fn always_includes_zero_and_45() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0,y:0.0},
                coord! {x:3.0,y:1.0},
                coord! {x:4.0,y:5.0},
                coord! {x:0.0,y:0.0},
            ]),
            vec![],
        );
        let angles = edge_candidate_angles(&poly, 4.0, 12);
        assert!(angles.iter().any(|&a| a.abs() < 0.5), "must include 0deg");
        assert!(angles.iter().any(|&a| (a - 45.0).abs() < 0.5), "must include 45deg");
    }

    #[test]
    fn axis_aligned_square_prefers_zero() {
        // All edges at 0deg and 90deg; after folding into [0,90), dominant peak = 0deg
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0,y:0.0},
                coord! {x:10.0,y:0.0},
                coord! {x:10.0,y:10.0},
                coord! {x:0.0,y:10.0},
                coord! {x:0.0,y:0.0},
            ]),
            vec![],
        );
        let angles = edge_candidate_angles(&poly, 4.0, 12);
        assert!(angles.iter().any(|&a| a.abs() < 1.0));
    }

    #[test]
    fn upper_bound_is_positive() {
        let hull = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0,y:0.0},
                coord! {x:10.0,y:0.0},
                coord! {x:10.0,y:10.0},
                coord! {x:0.0,y:0.0},
            ]),
            vec![],
        );
        let ub = upper_bound_area(&hull, 0.0, 0.0, Point::new(5.0, 3.33));
        assert!(ub > 0.0);
    }
}
/// Principal axis angle of the convex hull in [0, 90).
///
/// Computed from the eigen-direction of the covariance matrix of hull vertices.
/// Captures the dominant orientation of elongated shapes, complementing edge
/// directions which can be misleading for non-convex or multi-lobed polygons.
fn principal_axis_angle(hull: &Polygon<f64>) -> f64 {
    let pts = &hull.exterior().0;
    let n = pts.len();
    if n < 4 {
        return 0.0;
    }
    let n = n - 1; // last is a copy of first

    let mut cx = 0.0_f64;
    let mut cy = 0.0_f64;
    for p in pts.iter().take(n) {
        cx += p.x;
        cy += p.y;
    }
    cx /= n as f64;
    cy /= n as f64;

    let mut xx = 0.0_f64;
    let mut xy = 0.0_f64;
    let mut yy = 0.0_f64;
    for p in pts.iter().take(n) {
        let dx = p.x - cx;
        let dy = p.y - cy;
        xx += dx * dx;
        xy += dx * dy;
        yy += dy * dy;
    }

    let angle = 0.5 * (2.0 * xy).atan2(xx - yy);
    let deg = angle.to_degrees() % 90.0;
    if deg < 0.0 { deg + 90.0 } else { deg }
}

/// Minimum bounding rectangle (MBR) angle of the convex hull in [0, 90).
///
/// The MBR of a convex polygon always has one side collinear with a hull edge.
/// This tries all hull edges (O(m²) with m ≪ 50) and returns the angle of the
/// edge that minimises the rotated bounding-box area.
fn mbr_angle(hull: &Polygon<f64>) -> f64 {
    let pts = &hull.exterior().0;
    let m = pts.len();
    if m < 4 {
        return 0.0;
    }
    let m = m - 1; // last is a copy of first

    let mut best_area = f64::MAX;
    let mut best_angle = 0.0_f64;

    for i in 0..m {
        let p1 = pts[i];
        let p2 = pts[(i + 1) % m];
        let rad = -(p2.y - p1.y).atan2(p2.x - p1.x);
        let (cos_a, sin_a) = (rad.cos(), rad.sin());

        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;

        for p in pts.iter().take(m) {
            let rx = p.x * cos_a - p.y * sin_a;
            let ry = p.x * sin_a + p.y * cos_a;
            min_x = min_x.min(rx);
            min_y = min_y.min(ry);
            max_x = max_x.max(rx);
            max_y = max_y.max(ry);
        }

        let area = (max_x - min_x) * (max_y - min_y);
        if area < best_area {
            best_area = area;
            best_angle = (p2.y - p1.y).atan2(p2.x - p1.x).to_degrees();
        }
    }

    let deg = best_angle % 90.0;
    if deg < 0.0 { deg + 90.0 } else { deg }
}

pub(crate) fn heuristic_candidates(
    poly: &Polygon<f64>,
    angle_step: usize,
    grid_coarse: usize,
    max_ratio: f64,
    top_k: usize,
) -> Vec<AngleCandidate> {
    let cent = match poly.centroid() {
        Some(c) => c,
        None => return Vec::new(),
    };
    let centroid = Point::new(cent.x(), cent.y());
    let hull = poly.convex_hull();
    let (simplified, _) = simplify_for_solve(poly);

    let solve_coarse = |angle_f: f64| -> Option<(f64, f64, f64, f64, f64)> {
        let rot_s = rotate_polygon(&simplified, -angle_f);
        solve_axis_rect_grid(&rot_s, grid_coarse, max_ratio)
    };

    let edge_angles = edge_candidate_angles(poly, 4.0, 12);

    // Supplement edge-direction peaks with principal-axis and MBR angles
    let mut all_angles = edge_angles;
    {
        let pa = principal_axis_angle(&hull);
        if !all_angles.iter().any(|&a| (a - pa).abs() < 2.0) {
            all_angles.push(pa);
        }
        let ma = mbr_angle(&hull);
        if !all_angles.iter().any(|&a| (a - ma).abs() < 2.0) {
            all_angles.push(ma);
        }
    }

    // Parallel: evaluate all candidate angles independently
    let mut raw: Vec<(f64, f64, (f64, f64, f64, f64))> = all_angles
        .par_iter()
        .filter_map(|&a| {
            solve_coarse(a).map(|(x0, y0, x1, y1, area)| (area, a, (x0, y0, x1, y1)))
        })
        .collect();

    // Parallel fallback: fill with regular angles if too few candidates
    if raw.len() < 3 {
        let best_area = raw.iter().map(|r| r.0).fold(0.0_f64, f64::max);
        let step_angles: Vec<f64> = (0..90).step_by(angle_step).map(|a| a as f64).collect();

        let fallback: Vec<(f64, f64, (f64, f64, f64, f64))> = step_angles
            .par_iter()
            .filter_map(|&a| {
                if raw.iter().any(|&(_, angle, _)| (angle - a).abs() < 2.0) {
                    return None;
                }
                let ub = upper_bound_area(&hull, a, max_ratio, centroid);
                if ub <= best_area * crate::tuning::PRUNE_MARGIN {
                    return None;
                }
                solve_coarse(a).and_then(|(x0, y0, x1, y1, area)| {
                    if area > 0.0 { Some((area, a, (x0, y0, x1, y1))) } else { None }
                })
            })
            .collect();
        raw.extend(fallback);
    }

    raw.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut kept: Vec<AngleCandidate> = Vec::new();
    let mut seen: Vec<f64> = Vec::new();

    for (area, angle, (x0, y0, x1, y1)) in raw {
        if seen.iter().any(|&s| (angle - s).abs() < 2.0) {
            continue;
        }
        seen.push(angle);

        // Rotate rect to world frame
        let r_world = Polygon::new(
            LineString::from(vec![
                rotate_point(x0, y0, angle, &centroid),
                rotate_point(x1, y0, angle, &centroid),
                rotate_point(x1, y1, angle, &centroid),
                rotate_point(x0, y1, angle, &centroid),
                rotate_point(x0, y0, angle, &centroid),
            ]),
            vec![],
        );
        let wbb = r_world.bounding_rect().unwrap();
        let wb = (wbb.min().x, wbb.min().y, wbb.max().x, wbb.max().y);

        kept.push(AngleCandidate {
            angle,
            area,
            rect_rot: (x0, y0, x1, y1),
            rect_world_bounds: wb,
            center: centroid,
        });

        if kept.len() >= top_k {
            break;
        }
    }

    kept
}

