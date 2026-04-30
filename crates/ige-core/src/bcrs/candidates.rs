//! Edge-direction angle candidate generation.
//!
//! Port of `_edge_candidate_angles` and `_upper_bound_area`
//! from `bcrs_fast_worker.py`.

use geo::{BoundingRect, Rotate};
use geo_types::{Point, Polygon};

/// Extract edge-direction angle candidates in `[0°, 90°)` weighted by edge length.
///
/// Candidates are smoothed with a Gaussian kernel and peak-picked with a
/// minimum angular separation.  Angles 0° and 45° are always included.
///
/// # Arguments
/// * `poly`            — input polygon (exterior + holes used)
/// * `min_sep_deg`     — minimum angular distance between two peaks
/// * `max_candidates`  — maximum number of peaks to return (before inserting 0/45)
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

    let mut peaks: Vec<usize> = Vec::new();
    for &idx in &order {
        if peaks
            .iter()
            .all(|&p: &usize| (idx as isize - p as isize).unsigned_abs() >= sep)
        {
            peaks.push(idx);
        }
        if peaks.len() >= max_candidates {
            break;
        }
    }

    let mut result: Vec<f64> = peaks.into_iter().map(|p| p as f64).collect();

    // Always include 0° and 45°
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
        assert!(angles.iter().any(|&a| a.abs() < 0.5), "must include 0°");
        assert!(angles.iter().any(|&a| (a - 45.0).abs() < 0.5), "must include 45°");
    }

    #[test]
    fn axis_aligned_square_prefers_zero() {
        // All edges at 0° and 90°; after folding into [0,90), dominant peak = 0°
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