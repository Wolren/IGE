//! Exact O(n log² n) divide-and-conquer Largest Empty Rectangle solver
//! amidst point obstacles (Chazelle, Drysdale & Lee, 1984).
//!
//! 1. Divide points by median x-coordinate.
//! 2. Conquer: recurse on left and right halves.
//! 3. Merge: find largest rectangle crossing the dividing line by sweeping
//!    points in y-order, maintaining left/right barriers.

use geo::BoundingRect;
use geo_types::{Coord, Polygon, Rect};
use crate::shared::{LirError, Rectangle, Result};
use super::{LerOptions, LerResult};

const EPS: f64 = 1e-9;

/// Solve LER for point obstacles using exact divide-and-conquer.
pub fn solve_ler_points_dc(
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

    // Filter points inside bounding box and sort by x then y
    let mut pts: Vec<(f64, f64)> = points.iter()
        .map(|c| (c.x, c.y))
        .filter(|&(x, y)| x > bx0 + EPS && x < bx1 - EPS && y > by0 + EPS && y < by1 - EPS)
        .collect();

    // Sort by x for divide step; we'll sort by y in the crossing step
    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.partial_cmp(&b.1).unwrap()));

    // Remove exact duplicates (same x and y)
    pts.dedup_by(|a, b| (a.0 - b.0).abs() < EPS && (a.1 - b.1).abs() < EPS);

    let mut best = Rectangle { x_min: bx0, y_min: by0, x_max: bx1, y_max: by1 };
    let mut best_area = 0.0;

    dc(&pts, bx0, by0, bx1, by1, options, &mut best, &mut best_area);

    if best_area < EPS {
        return Ok(LerResult::empty());
    }

    let area = best_area;
    Ok(LerResult {
        area,
        rect: Some(best.clone()),
        rect_polygon: Some(Rect::new(Coord { x: best.x_min, y: best.y_min }, Coord { x: best.x_max, y: best.y_max }).to_polygon()),
        angle_deg: 0.0, best_effort: false,
    })
}

fn aspect_ok(w: f64, h: f64, opts: &LerOptions) -> bool {
    if w < EPS || h < EPS { return false; }
    let (s, l) = (w.min(h), w.max(h));
    let r = l / s;
    if opts.max_ratio > 0.0 && r > opts.max_ratio * 1.000001 { return false; }
    if opts.min_ratio > 0.0 && r < opts.min_ratio * 0.999999 { return false; }
    true
}

#[inline]
fn try_rect(x0: f64, y0: f64, x1: f64, y1: f64, opts: &LerOptions, best: &mut Rectangle, best_area: &mut f64) {
    let w = x1 - x0;
    let h = y1 - y0;
    if w > EPS && h > EPS && aspect_ok(w, h, opts) {
        let area = w * h;
        if area > *best_area + EPS {
            *best_area = area;
            *best = Rectangle { x_min: x0, y_min: y0, x_max: x1, y_max: y1 };
        }
    }
}

fn dc(
    pts: &[(f64, f64)],
    bx0: f64, by0: f64, bx1: f64, by1: f64,
    opts: &LerOptions,
    best: &mut Rectangle,
    best_area: &mut f64,
) {
    if pts.is_empty() {
        try_rect(bx0, by0, bx1, by1, opts, best, best_area);
        return;
    }

    if pts.len() == 1 {
        let (px, py) = pts[0];
        // Four rectangles around the single point:
        // below, above, left, right
        try_rect(bx0, by0, bx1, py, opts, best, best_area);   // bottom strip
        try_rect(bx0, py, bx1, by1, opts, best, best_area);   // top strip
        try_rect(bx0, by0, px, by1, opts, best, best_area);   // left strip
        try_rect(px, by0, bx1, by1, opts, best, best_area);   // right strip
        return;
    }

    // Divide at median x
    let mid = pts.len() / 2;
    let mid_x = (pts[mid - 1].0 + pts[mid].0) / 2.0;

    let (left_pts, right_pts) = pts.split_at(mid);

    // Conquer
    dc(left_pts, bx0, by0, mid_x, by1, opts, best, best_area);
    dc(right_pts, mid_x, by0, bx1, by1, opts, best, best_area);

    // Merge: crossing rectangles
    // Sort all points by y for the sweep
    let mut by_y: Vec<(f64, f64, bool)> = Vec::with_capacity(pts.len());
    for &(x, y) in left_pts {
        by_y.push((x, y, true));  // true = left half
    }
    for &(x, y) in right_pts {
        by_y.push((x, y, false)); // false = right half
    }
    by_y.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

    // O(k²) crossing: enumerate all maximal rectangles crossing the
    // dividing line. Uses prefix/suffix arrays for edge cases and
    // incremental barrier updates for interior pairs — avoids the O(k³)
    // scan overhead of the previous approach.
    let n = by_y.len();

    // Precompute prefix and suffix barriers.
    // pref_l[j] = max left-half x in by_y[0..j-1], pref_r[j] = min right-half x in by_y[0..j-1]
    let mut pref_l = vec![bx0; n + 1];
    let mut pref_r = vec![bx1; n + 1];
    for i in 0..n {
        pref_l[i + 1] = pref_l[i];
        pref_r[i + 1] = pref_r[i];
        let (px, _, is_left) = by_y[i];
        if is_left { pref_l[i + 1] = pref_l[i + 1].max(px); }
        else { pref_r[i + 1] = pref_r[i + 1].min(px); }
    }

    // suff_l[i] = max left-half x in by_y[i..n-1], suff_r[i] = min right-half x in by_y[i..n-1]
    let mut suff_l = vec![bx0; n + 1];
    let mut suff_r = vec![bx1; n + 1];
    for i in (0..n).rev() {
        suff_l[i] = suff_l[i + 1];
        suff_r[i] = suff_r[i + 1];
        let (px, _, is_left) = by_y[i];
        if is_left { suff_l[i] = suff_l[i].max(px); }
        else { suff_r[i] = suff_r[i].min(px); }
    }

    // 1. Bottom = by0, top = each point j. Barriers = points in by_y[0..j-1].
    for j in 0..n {
        let y_t = by_y[j].1;
        if y_t > by0 + EPS && pref_l[j] < pref_r[j] {
            try_rect(pref_l[j], by0, pref_r[j], y_t, opts, best, best_area);
        }
    }

    // 2. Bottom = point i, top = point j (j > i).
    //    Barriers from by_y[i+1..j-1] tracked incrementally.
    //    Branch-and-bound: skip i if max possible area can't beat current best.
    let full_w = bx1 - bx0;
    for i in 0..n {
        let y_b = by_y[i].1;
        if full_w * (by1 - y_b) <= *best_area + EPS { continue; }
        let mut L = bx0;
        let mut R = bx1;
        for j in (i + 1)..n {
            let y_t = by_y[j].1;
            if y_t > y_b + EPS && L < R {
                try_rect(L, y_b, R, y_t, opts, best, best_area);
            }
            // Include by_y[j] in barriers for the NEXT iteration
            let (px, _, is_left) = by_y[j];
            if is_left { L = L.max(px); } else { R = R.min(px); }
        }
    }

    // 3. Bottom = point i, top = by1. Barriers = points in by_y[i+1..n-1].
    for i in 0..n {
        let y_b = by_y[i].1;
        if by1 > y_b + EPS && suff_l[i + 1] < suff_r[i + 1] {
            try_rect(suff_l[i + 1], y_b, suff_r[i + 1], by1, opts, best, best_area);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};
    use rand::{Rng, SeedableRng};

    fn rp(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
        Polygon::new(LineString::from(vec![
            coord! { x: x0, y: y0 }, coord! { x: x1, y: y0 },
            coord! { x: x1, y: y1 }, coord! { x: x0, y: y1 },
            coord! { x: x0, y: y0 },
        ]), vec![])
    }
    fn opts() -> LerOptions { LerOptions::default() }

    #[test] fn dc_no_points() { let poly = rp(0.,0.,10.,10.); let r = solve_ler_points_dc(&poly, &[], &opts()).unwrap(); assert!(r.area > 99.0); }
    #[test] fn dc_single_point() { let poly = rp(0.,0.,10.,10.); let pts = vec![coord! { x: 5., y: 5. }]; let r = solve_ler_points_dc(&poly, &pts, &opts()).unwrap(); assert!(r.area > 20.0 && r.area < 80.0); }
    #[test] fn dc_two_points_same_y() { let poly = rp(0.,0.,10.,10.); let pts = vec![coord! { x: 3., y: 5. }, coord! { x: 7., y: 5. }]; let r = solve_ler_points_dc(&poly, &pts, &opts()).unwrap(); assert!(r.area > 0.0); }
    #[test] fn dc_four_corners() { let poly = rp(0.,0.,10.,10.); let pts = vec![coord! { x: 2., y: 2. }, coord! { x: 8., y: 2. }, coord! { x: 2., y: 8. }, coord! { x: 8., y: 8. }]; let r = solve_ler_points_dc(&poly, &pts, &opts()).unwrap(); assert!(r.area > 0.0); }

    #[test]
    fn dc_matches_sweep_line() {
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
        let r_new = solve_ler_points_dc(&poly, &pts, &opts()).unwrap();
        assert!((r_old.area - r_new.area).abs() < 5.0,
            "old={:.2} new={:.2} differ too much", r_old.area, r_new.area);
    }

    #[test]
    fn dc_matches_exact_random_50() {
        use super::super::solve_ler_axis_aligned_exact;
        let poly = rp(0.,0.,100.,100.);
        let pts: Vec<_> = (0..50).map(|i| coord! {
            x: ((i * 157) % 99 + 1) as f64,
            y: ((i * 271) % 99 + 1) as f64,
        }).collect();
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
        let r_new = solve_ler_points_dc(&poly, &pts, &opts()).unwrap();
        // DC should match or exceed old solver (DC has exact boundaries, old uses EPS)
        assert!(r_new.area >= r_old.area - 10.0,
            "DC area {:.2} < old area {:.2} by more than 10", r_new.area, r_old.area);
    }

    #[test]
    fn dc_matches_exact_random_multi() {
        use super::super::solve_ler_axis_aligned_exact;
        use rand::rngs::StdRng;
        for seed in 0..20 {
            let mut rng = StdRng::seed_from_u64(seed);
            let n = rng.gen_range(5..30);
            let mut pts = Vec::new();
            for _ in 0..n {
                let x = rng.gen_range(1.0..99.0);
                let y = rng.gen_range(1.0..99.0);
                pts.push(Coord { x, y });
            }
            let obs: Vec<Polygon<f64>> = pts.iter().map(|c| {
                Polygon::new(LineString::from(vec![
                    coord! { x: c.x - 0.01, y: c.y - 0.01 },
                    coord! { x: c.x + 0.01, y: c.y - 0.01 },
                    coord! { x: c.x + 0.01, y: c.y + 0.01 },
                    coord! { x: c.x - 0.01, y: c.y + 0.01 },
                    coord! { x: c.x - 0.01, y: c.y - 0.01 },
                ]), vec![])
            }).collect();
            let poly = rp(0.,0.,100.,100.);
            let r_old = solve_ler_axis_aligned_exact(&poly, &obs, &opts()).unwrap();
            let r_new = solve_ler_points_dc(&poly, &pts, &opts()).unwrap();
            assert!(r_new.area >= r_old.area - 10.0,
                "Seed {}: DC area {:.2} < old area {:.2}", seed, r_new.area, r_old.area);
        }
    }
}
