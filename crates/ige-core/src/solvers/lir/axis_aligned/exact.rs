//! Continuous axis-aligned rectangle solver with deterministic support search.
//!
//! O(nx² × ny × log ny) binary-search support search over unique vertex x/y,
//! parallelized over the outer x-pair loop. Grid is capped at 250 points per
//! axis (subsampled) to prevent blowup on large coordinate sets.

use geo::{BoundingRect, Centroid};
use geo_types::Polygon;
use ordered_float::OrderedFloat;
use rayon::prelude::*;
use std::collections::HashSet;

pub use super::AxisAlignedOptions;
pub use super::containment::rect_fully_contained;
use crate::shared::Rectangle;

use crate::tuning::{AA_EXACT_BINARY_ITERS, AA_EXACT_REFINE_ITERS, AA_EXACT_TOP_SEEDS, AA_EXACT_GRID_CAP, AA_EPS};

const EPS: f64 = crate::tuning::AA_EPS;

fn capped_coords(coords: &mut Vec<f64>) {
    let max_grid = crate::tuning::AA_EXACT_GRID_CAP;
    if coords.len() <= max_grid { return; }
    let step = coords.len() / max_grid;
    *coords = coords.iter().step_by(step.max(1)).take(max_grid).copied().collect();
}

fn collect_unique_coords(poly: &Polygon<f64>) -> (Vec<f64>, Vec<f64>) {
    let mut xs = HashSet::new();
    let mut ys = HashSet::new();

    for c in poly.exterior().0.iter() {
        xs.insert(OrderedFloat(c.x));
        ys.insert(OrderedFloat(c.y));
    }
    for hole in poly.interiors() {
        for c in hole.0.iter() {
            xs.insert(OrderedFloat(c.x));
            ys.insert(OrderedFloat(c.y));
        }
    }
    let mut xv: Vec<f64> = xs.into_iter().map(|v| v.into_inner()).collect();
    let mut yv: Vec<f64> = ys.into_iter().map(|v| v.into_inner()).collect();
    xv.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    yv.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    // Midpoints: provide intermediate search positions for sparse grids
    // (e.g. triangles with only 2 unique coords).  MAX_GRID caps total size.
    let xm: Vec<f64> = xv.windows(2).map(|w| (w[0] + w[1]) * 0.5).collect();
    let ym: Vec<f64> = yv.windows(2).map(|w| (w[0] + w[1]) * 0.5).collect();
    xv.extend(xm); yv.extend(ym);
    xv.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    yv.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    capped_coords(&mut xv);
    capped_coords(&mut yv);
    (xv, yv)
}

fn aspect_ok(width: f64, height: f64, max_ratio: f64) -> bool {
    if width <= EPS || height <= EPS { return false; }
    if max_ratio <= 0.0 { return true; }
    let ls = width.max(height);
    let ss = width.min(height);
    ss > EPS && ls / ss <= max_ratio + 1e-12
}

fn refine_side_min_x(
    poly: &Polygon<f64>, x0: f64, y0: f64, x1: f64, y1: f64,
    x_min_bound: f64, max_ratio: f64,
) -> f64 {
    let mut lo = x_min_bound; let mut hi = x0;
    for _ in 0..AA_EXACT_BINARY_ITERS {
        let mid = (lo + hi) * 0.5;
        if rect_fully_contained(poly, mid, y0, x1, y1) && aspect_ok(x1 - mid, y1 - y0, max_ratio) {
            hi = mid;
        } else { lo = mid; }
    }
    hi
}

fn refine_side_max_x(
    poly: &Polygon<f64>, x0: f64, y0: f64, x1: f64, y1: f64,
    x_max_bound: f64, max_ratio: f64,
) -> f64 {
    let mut lo = x1; let mut hi = x_max_bound;
    for _ in 0..AA_EXACT_BINARY_ITERS {
        let mid = (lo + hi) * 0.5;
        if rect_fully_contained(poly, x0, y0, mid, y1) && aspect_ok(mid - x0, y1 - y0, max_ratio) {
            lo = mid;
        } else { hi = mid; }
    }
    lo
}

fn refine_side_min_y(
    poly: &Polygon<f64>, x0: f64, y0: f64, x1: f64, y1: f64,
    y_min_bound: f64, max_ratio: f64,
) -> f64 {
    let mut lo = y_min_bound; let mut hi = y0;
    for _ in 0..AA_EXACT_BINARY_ITERS {
        let mid = (lo + hi) * 0.5;
        if rect_fully_contained(poly, x0, mid, x1, y1) && aspect_ok(x1 - x0, y1 - mid, max_ratio) {
            hi = mid;
        } else { lo = mid; }
    }
    hi
}

fn refine_side_max_y(
    poly: &Polygon<f64>, x0: f64, y0: f64, x1: f64, y1: f64,
    y_max_bound: f64, max_ratio: f64,
) -> f64 {
    let mut lo = y1; let mut hi = y_max_bound;
    for _ in 0..AA_EXACT_BINARY_ITERS {
        let mid = (lo + hi) * 0.5;
        if rect_fully_contained(poly, x0, y0, x1, mid) && aspect_ok(x1 - x0, mid - y0, max_ratio) {
            lo = mid;
        } else { hi = mid; }
    }
    lo
}

fn refine_continuous(poly: &Polygon<f64>, seed: Rectangle, options: &AxisAlignedOptions) -> Option<Rectangle> {
    let bb = poly.bounding_rect()?;
    let (mut x0, mut y0, mut x1, mut y1) = (seed.x_min, seed.y_min, seed.x_max, seed.y_max);
    if !rect_fully_contained(poly, x0, y0, x1, y1) || !aspect_ok(x1 - x0, y1 - y0, options.max_ratio) {
        return None;
    }
    for _ in 0..AA_EXACT_REFINE_ITERS {
        let p = (x0, y0, x1, y1);
        x0 = refine_side_min_x(poly, x0, y0, x1, y1, bb.min().x, options.max_ratio);
        x1 = refine_side_max_x(poly, x0, y0, x1, y1, bb.max().x, options.max_ratio);
        y0 = refine_side_min_y(poly, x0, y0, x1, y1, bb.min().y, options.max_ratio);
        y1 = refine_side_max_y(poly, x0, y0, x1, y1, bb.max().y, options.max_ratio);
        if (x0 - p.0).abs() + (y0 - p.1).abs() + (x1 - p.2).abs() + (y1 - p.3).abs() < 1e-10 { break; }
    }
    if !rect_fully_contained(poly, x0, y0, x1, y1) || !aspect_ok(x1 - x0, y1 - y0, options.max_ratio) {
        return None;
    }
    Some(Rectangle { x_min: x0, y_min: y0, x_max: x1, y_max: y1 })
}

fn push_top_seed(seeds: &mut Vec<Rectangle>, rect: &Rectangle) {
    let area = rect.area();
    if area <= EPS { return; }
    if seeds.len() < AA_EXACT_TOP_SEEDS { seeds.push(rect.clone()); return; }
    let mut min_idx = 0; let mut min_area = seeds[0].area();
    for (i, r) in seeds.iter().enumerate().skip(1) { let a = r.area(); if a < min_area { min_area = a; min_idx = i; } }
    if area > min_area { seeds[min_idx] = rect.clone(); }
}

/// Outer loop over x-pairs (parallel), inner loop over y0 with binary search for y1.
fn best_discrete_support_rect(poly: &Polygon<f64>, options: &AxisAlignedOptions, xs: &[f64], ys: &[f64]) -> Option<(Rectangle, Vec<Rectangle>)> {
    let y_min = ys[0]; let y_max = ys[ys.len() - 1];
    let partials: Vec<(Option<Rectangle>, f64, Vec<Rectangle>)> = (0..xs.len() - 1)
        .into_par_iter()
        .map(|i| {
            let mut best = None; let mut best_a = 0.0; let mut seeds: Vec<Rectangle> = Vec::new();
            let x0 = xs[i];
            for j in (i + 1)..xs.len() {
                let x1 = xs[j]; let w = x1 - x0;
                if w <= EPS { continue; }
                let max_h = if options.max_ratio > 0.0 { w * options.max_ratio } else { f64::INFINITY };
                if w * (y_max - y_min).min(max_h) <= best_a + EPS { continue; }
                for y0_idx in 0..ys.len() - 1 {
                    let y0 = ys[y0_idx];
                    let mh = (y_max - y0).min(max_h);
                    if w * mh <= best_a + EPS || mh <= EPS { continue; }
                    let yhv = y0 + mh;
                    let mut hi_idx = ys.len() - 1;
                    while hi_idx > y0_idx + 1 && ys[hi_idx] > yhv + EPS { hi_idx -= 1; }
                    if hi_idx <= y0_idx { continue; }
                    let (mut lo, mut hi) = (y0_idx + 1, hi_idx);
                    let mut found = None;
                    while lo <= hi {
                        let mid = (lo + hi) / 2;
                        if rect_fully_contained(poly, x0, y0, x1, ys[mid]) {
                            found = Some(mid); lo = mid + 1;
                        } else { if mid == 0 { break; } hi = mid - 1; }
                    }
                    if let Some(idx) = found {
                        let y1 = ys[idx]; let hh = y1 - y0;
                        if !aspect_ok(w, hh, options.max_ratio) { continue; }
                        let r = Rectangle { x_min: x0, y_min: y0, x_max: x1, y_max: y1 };
                        let a = r.area();
                        push_top_seed(&mut seeds, &r);
                        if a > best_a { best_a = a; best = Some(r); }
                    }
                }
            }
            (best, best_a, seeds)
        })
        .collect();
    let mut best = None; let mut best_a = 0.0; let mut seeds: Vec<Rectangle> = Vec::new();
    for (c, a, s) in partials {
        if a > best_a { best_a = a; best = c; }
        for r in s { push_top_seed(&mut seeds, &r); }
    }
    best.map(|r| (r, seeds))
}

/// Deterministic exhaustive axis-aligned rectangle solver.
pub fn solve_axis_exact(poly: &Polygon<f64>, options: &AxisAlignedOptions) -> Option<Rectangle> {
    let bb = poly.bounding_rect()?;
    if bb.max().x - bb.min().x <= EPS || bb.max().y - bb.min().y <= EPS { return None; }
    let (xs, ys) = collect_unique_coords(poly);
    if xs.len() < 2 || ys.len() < 2 { return None; }
    let (best_disc, mut seeds) = best_discrete_support_rect(poly, options, &xs, &ys)
        .unwrap_or_else(|| {
            // Fallback: seed from centroid if no discrete rect found (triangle case).
            let mut s = Vec::new();
            if let Some(c) = poly.centroid() {
                let e = 1e-6;
                s.push(Rectangle { x_min: c.x() - e, y_min: c.y() - e, x_max: c.x() + e, y_max: c.y() + e });
            }
            (Rectangle { x_min: 0., y_min: 0., x_max: 0., y_max: 0. }, s)
        });
    push_top_seed(&mut seeds, &best_disc);
    let mut best = best_disc; let mut best_a = best.area();
    for seed in seeds {
        if let Some(r) = refine_continuous(poly, seed, options) {
            let a = r.area();
            if a > best_a { best_a = a; best = r; }
        }
    }
    Some(best)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString, Polygon};

    #[test]
    fn exact_solver_square() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0}, coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:10.0}, coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]), vec![],
        );
        let r = solve_axis_exact(&poly, &AxisAlignedOptions::default()).expect("no rect");
        assert!((r.area() - 100.0).abs() < 1e-9);
    }

    #[test]
    fn exact_solver_right_triangle() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0}, coord! {x:10.0, y:0.0},
                coord! {x:0.0, y:10.0}, coord! {x:0.0, y:0.0},
            ]), vec![],
        );
        let r = solve_axis_exact(&poly, &AxisAlignedOptions::default()).expect("no rect");
        assert!(r.area() >= 24.9 && r.area() <= 25.1, "area={}", r.area());
    }
}
