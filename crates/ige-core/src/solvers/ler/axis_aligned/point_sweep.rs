//! O(n log n) plane-sweep for Largest Empty Rectangle amidst point obstacles.
//!
//! Uses sorted `Vec<f64>` + parallel left-barrier array for cache-friendly
//! access. Sweeps x left-to-right, maintaining active gaps between consecutive
//! y-coordinates. Each gap tracks the leftmost x that can serve as the left
//! wall of a rectangle in that y-interval.
//!
//! Keeps all historical gap records so the final bounding-box-edge pass
//! catches rectangles bounded by removed points' y-coordinates.

use geo::BoundingRect;
use geo_types::{Coord, Polygon, Rect};
use crate::shared::{LirError, Rectangle, Result};
use super::{LerOptions, LerResult};

const EPS: f64 = 1e-9;

/// Solve LER for point obstacles using O(n log n) plane-sweep.
pub fn solve_ler_points_sweep(
    poly: &Polygon<f64>,
    points: &[Coord<f64>],
    options: &LerOptions,
) -> Result<LerResult> {
    let bb = poly.bounding_rect().ok_or_else(|| LirError::InvalidPolygon("degenerate".into()))?;
    let (bx0, by0, bx1, by1) = (bb.min().x, bb.min().y, bb.max().x, bb.max().y);

    if bx1 - bx0 < EPS || by1 - by0 < EPS {
        return Ok(LerResult::empty());
    }

    if points.is_empty() {
        if aspect_ok(bx1 - bx0, by1 - by0, options) {
            let r = Rectangle { x_min: bx0, y_min: by0, x_max: bx1, y_max: by1 };
            return Ok(LerResult {
                area: r.area(), rect: Some(r),
                rect_polygon: Some(Rect::new(Coord { x: bx0, y: by0 }, Coord { x: bx1, y: by1 }).to_polygon()),
                angle_deg: 0.0, best_effort: false,
            });
        }
        return Ok(LerResult::empty());
    }

    // Group points by x-coordinate, skip points outside bbox
    let mut x_groups: Vec<(f64, Vec<f64>)> = Vec::new();
    let mut sorted: Vec<(f64, f64)> = points.iter().map(|c| (c.x, c.y)).collect();
    sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.partial_cmp(&b.1).unwrap()));
    for (x, y) in sorted {
        if x <= bx0 + EPS || x >= bx1 - EPS || y <= by0 + EPS || y >= by1 - EPS { continue; }
        if let Some(last) = x_groups.last_mut() {
            if (last.0 - x).abs() < EPS { last.1.push(y); continue; }
        }
        x_groups.push((x, vec![y]));
    }

    // Sorted y-array (contains by0, by1, and all obstacle y's).
    // left[i] = left-barrier x for gap (ys[i], ys[i+1]).
    let mut ys: Vec<f64> = Vec::with_capacity(x_groups.iter().map(|g| g.1.len()).sum::<usize>() + 2);
    ys.push(by0);
    for (_, gys) in &x_groups {
        ys.extend_from_slice(gys);
    }
    ys.push(by1);
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());

    // Left-barrier array, parallel to gaps (ys.len() - 1 gaps).
    let mut left: Vec<f64> = vec![bx0; ys.len() - 1];
    // Store all historical gaps for the final bounding-box-edge pass.
    let mut all_left: Vec<(f64, f64, f64)> = Vec::new();

    fn snapshot(ys: &[f64], left: &[f64], all: &mut Vec<(f64, f64, f64)>) {
        for i in 0..ys.len() - 1 {
            all.push((ys[i], ys[i+1], left[i]));
        }
    }

    snapshot(&ys, &left, &mut all_left);

    let mut best_area = 0.0;
    let mut best_rect: Option<Rectangle> = None;

    #[inline]
    fn record(rect: &Rectangle, area: f64, best_area: &mut f64, best_rect: &mut Option<Rectangle>, opts: &LerOptions) {
        if area > *best_area + EPS {
            let w = rect.x_max - rect.x_min;
            let h = rect.y_max - rect.y_min;
            if w > EPS && h > EPS {
                let (s, l) = (w.min(h), w.max(h));
                let r = l / s;
                let ratio_ok = opts.max_ratio <= 0.0 || r <= opts.max_ratio * 1.000001;
                let min_ok = opts.min_ratio <= 0.0 || r >= opts.min_ratio * 0.999999;
                if ratio_ok && min_ok {
                    *best_area = area;
                    *best_rect = Some(rect.clone());
                }
            }
        }
    }

    // Sweep left to right
    for &(cur_x, ref gys) in &x_groups {
        // Evaluate candidate rectangles from current gaps (right wall = cur_x)
        for i in 0..ys.len() - 1 {
            if cur_x > left[i] + EPS {
                let rect = Rectangle { x_min: left[i], y_min: ys[i], x_max: cur_x, y_max: ys[i + 1] };
                record(&rect, rect.area(), &mut best_area, &mut best_rect, options);
            }
        }

        // Remove points at this x from the y-array, merge adjacent gaps.
        for &y in gys {
            match ys.binary_search_by(|a| a.partial_cmp(&y).unwrap()) {
                Ok(idx) => {
                    if idx == 0 || idx == ys.len() - 1 { continue; }
                    let lo = idx - 1;
                    let new_left = left[lo].max(left[idx]).max(cur_x);
                    ys.remove(idx);
                    left.remove(idx);
                    left[lo] = new_left;
                    all_left.push((ys[lo], ys[lo + 1], new_left));
                }
                Err(pos) => {
                    // Duplicate y already removed; advance barrier for the gap.
                    if pos > 0 && pos < ys.len() && pos - 1 < left.len() {
                        left[pos - 1] = left[pos - 1].max(cur_x);
                    }
                }
            }
        }
    }

    // Final pass: right wall = bx1. Check ALL gaps ever recorded.
    for &(y_lo, y_hi, left_x) in &all_left {
        if bx1 > left_x + EPS {
            let rect = Rectangle { x_min: left_x, y_min: y_lo, x_max: bx1, y_max: y_hi };
            record(&rect, rect.area(), &mut best_area, &mut best_rect, options);
        }
    }

    match best_rect {
        Some(r) => {
            let area = r.area();
            Ok(LerResult {
                area, rect: Some(r.clone()),
                rect_polygon: Some(Rect::new(Coord { x: r.x_min, y: r.y_min }, Coord { x: r.x_max, y: r.y_max }).to_polygon()),
                angle_deg: 0.0, best_effort: false,
            })
        }
        None => Ok(LerResult::empty()),
    }
}

fn aspect_ok(w: f64, h: f64, opts: &LerOptions) -> bool {
    if w < EPS || h < EPS { return false; }
    let (s, l) = (w.min(h), w.max(h));
    let r = l / s;
    if opts.max_ratio > 0.0 && r > opts.max_ratio * 1.000001 { return false; }
    if opts.min_ratio > 0.0 && r < opts.min_ratio * 0.999999 { return false; }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    fn rp(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
        Polygon::new(LineString::from(vec![
            coord! { x: x0, y: y0 }, coord! { x: x1, y: y0 },
            coord! { x: x1, y: y1 }, coord! { x: x0, y: y1 },
            coord! { x: x0, y: y0 },
        ]), vec![])
    }
    fn opts() -> LerOptions { LerOptions::default() }

    #[test] fn no_points_fills_box() { let poly = rp(0.,0.,10.,10.); let r = solve_ler_points_sweep(&poly, &[], &opts()).unwrap(); assert!(r.area > 99.0); }
    #[test] fn single_center_point() { let poly = rp(0.,0.,10.,10.); let pts = vec![coord! { x: 5., y: 5. }]; let r = solve_ler_points_sweep(&poly, &pts, &opts()).unwrap(); assert!(r.area > 20.0 && r.area < 80.0); }
    #[test] fn four_corner_points() { let poly = rp(0.,0.,10.,10.); let pts = vec![coord! { x: 2., y: 2. }, coord! { x: 8., y: 2. }, coord! { x: 2., y: 8. }, coord! { x: 8., y: 8. }]; let r = solve_ler_points_sweep(&poly, &pts, &opts()).unwrap(); assert!(r.area > 0.0); }
    #[test] fn vertical_line_of_points() { let poly = rp(0.,0.,10.,10.); let pts: Vec<_> = (1..10).map(|i| coord! { x: 5., y: i as f64 }).collect(); let r = solve_ler_points_sweep(&poly, &pts, &opts()).unwrap(); assert!(r.area > 0.0); }
    #[test] fn many_random_points() { let poly = rp(0.,0.,100.,100.); let pts: Vec<_> = (0..300).map(|i| coord! { x: ((i * 157) % 99 + 1) as f64, y: ((i * 271) % 99 + 1) as f64 }).collect(); let r = solve_ler_points_sweep(&poly, &pts, &opts()).unwrap(); assert!(r.area > 0.0); }

    #[test]
    fn matches_sweep_line_on_simple() {
        use super::super::solve_ler_axis_aligned_exact;
        let poly = rp(0.,0.,10.,10.);
        let pts = vec![coord! { x: 5., y: 5. }];
        let obs: Vec<Polygon<f64>> = pts.iter().map(|c| {
            Polygon::new(LineString::from(vec![
                coord! { x: c.x - 0.01, y: c.y - 0.01 },
                coord! { x: c.x + 0.01, y: c.y - 0.01 },
                coord! { x: c.x + 0.01, y: c.y + 0.01 },
                coord! { x: c.x - 0.01, y: c.y + 0.01 },
                coord! { x: c.x - 0.01, y: c.y - 0.01 },
            ]), vec![])
        }).collect();
        let r_old = solve_ler_axis_aligned_exact(&poly, &obs, &opts()).unwrap();
        let r_new = solve_ler_points_sweep(&poly, &pts, &opts()).unwrap();
        assert!((r_old.area - r_new.area).abs() < 5.0,
            "old={:.2} new={:.2} differ too much", r_old.area, r_new.area);
    }
}
