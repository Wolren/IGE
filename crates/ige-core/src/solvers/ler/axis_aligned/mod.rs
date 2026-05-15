//! Axis-aligned Largest Empty Rectangle solver.
//! Uses O(m² × k) sweep-line approach where m = x-candidates, k = obstacles.
//!
//! Modular structure:
//! - `point` — single-point obstacles (exact y-block)
//! - `line`  — line segment obstacles (precise y-overlap via interpolation)
//! - `rect`  — axis-aligned rectangle obstacles (full bbox block)
//! - `mod`   — `ObstacleInput` enum, core sweep algorithm, public API
//!
//! Each module exposes `build`, `collect_x_candidates`, and `y_intervals`
//! so the core solver can remain obstacle-type-agnostic.

pub mod point;
pub mod line;
pub mod rect;
pub mod point_sweep;
pub mod point_dc;

use geo::BoundingRect;
use geo_types::{Polygon, Rect, Coord, LineString};

use crate::shared::{LirError, Rectangle, Result};
use super::{LerOptions, LerResult};

const EPS: f64 = 1e-9;
const MAX_CANDIDATES: usize = 300;
const MAX_OBSTACLES: usize = 300;

/// Public obstacle input — the solver auto-detects the variant.
#[derive(Clone, Debug)]
pub enum ObstacleInput {
    Point(Coord<f64>),
    Line(LineString<f64>),
    Polygon(Polygon<f64>),
}

// ── helpers ──────────────────────────────────────────────────────────

fn poly_bbox(poly: &Polygon<f64>) -> Option<(f64, f64, f64, f64)> {
    let bb = poly.bounding_rect()?;
    Some((bb.min().x, bb.min().y, bb.max().x, bb.max().y))
}

fn aspect_ok(w: f64, h: f64, opts: &LerOptions) -> bool {
    if w < EPS || h < EPS { return false; }
    let (s, l) = (w.min(h), w.max(h));
    let r = l / s;
    if opts.max_ratio > 0.0 && r > opts.max_ratio * 1.000001 { return false; }
    if opts.min_ratio > 0.0 && r < opts.min_ratio * 0.999999 { return false; }
    true
}

/// Merge overlapping y-intervals and return the largest gap within [by0, by1].
fn find_largest_y_gap(intervals: &mut Vec<(f64, f64)>, by0: f64, by1: f64) -> Option<(f64, f64)> {
    if intervals.is_empty() { return Some((by0, by1)); }

    intervals.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let sorted = intervals.as_slice();

    let mut merged = Vec::new();
    merged.push(sorted[0]);
    for iv in sorted.iter().skip(1) {
        if iv.0 <= merged.last().unwrap().1 + EPS {
            let last = merged.pop().unwrap();
            merged.push((last.0, last.1.max(iv.1)));
        } else {
            merged.push(*iv);
        }
    }

    let mut gaps = Vec::new();
    let mut cur = merged[0];
    if cur.0 > by0 + EPS { gaps.push((by0, cur.0)); }
    for iv in merged.iter().skip(1) {
        gaps.push((cur.1, iv.0));
        cur = *iv;
    }
    if cur.1 < by1 - EPS { gaps.push((cur.1, by1)); }

    gaps.into_iter()
        .max_by(|a, b| (a.1 - a.0).partial_cmp(&(b.1 - b.0)).unwrap())
}

// ── obstacle grouping ─────────────────────────────────────────────────

struct GroupedObs {
    points: Vec<point::PointObs>,
    lines:  Vec<line::LineObs>,
    rects:  Vec<rect::RectObs>,
}

fn group_inputs(inputs: &[ObstacleInput]) -> GroupedObs {
    let mut points = Vec::new();
    let mut lines = Vec::new();
    let mut rects = Vec::new();

    for inp in inputs.iter().take(MAX_OBSTACLES) {
        match inp {
            ObstacleInput::Point(c) => {
                points.push(point::PointObs { x: c.x, y: c.y });
            }
            ObstacleInput::Line(ls) => {
                let coords: Vec<Coord<f64>> = ls.coords().copied().collect();
                if coords.len() >= 2 {
                    let (ax, ay) = (coords[0].x, coords[0].y);
                    let (bx, by) = (coords[1].x, coords[1].y);
                    lines.push(line::LineObs { ax, ay, bx, by });
                }
            }
            ObstacleInput::Polygon(p) => {
                if let Some(bb) = p.bounding_rect() {
                    rects.push(rect::RectObs {
                        x0: bb.min().x, x1: bb.max().x,
                        y0: bb.min().y, y1: bb.max().y,
                    });
                }
            }
        }
    }

    GroupedObs { points, lines, rects }
}

/// Collect x-candidates from all obstacle groups plus the polygon.
fn collect_all_x_candidates(poly: &Polygon<f64>, g: &GroupedObs) -> Vec<f64> {
    let mut xs: Vec<f64> = Vec::new();

    for c in poly.exterior().coords() { xs.push(c.x); }
    for ring in poly.interiors() { for c in ring.coords() { xs.push(c.x); } }

    xs.extend(point::collect_x_candidates(&g.points));
    xs.extend(line::collect_x_candidates(&g.lines));
    xs.extend(rect::collect_x_candidates(&g.rects));

    if let Some((x0, _, x1, _)) = poly_bbox(poly) {
        xs.push(x0);
        xs.push(x1);
    }

    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs.dedup();
    xs.into_iter().take(MAX_CANDIDATES).collect()
}

/// Gather y-intervals from all obstacle groups for a given x-span.
fn collect_all_y_intervals(g: &GroupedObs, x0: f64, x1: f64) -> Vec<(f64, f64)> {
    let mut iv = Vec::new();
    iv.extend(point::y_intervals(&g.points, x0, x1));
    iv.extend(line::y_intervals(&g.lines, x0, x1));
    iv.extend(rect::y_intervals(&g.rects, x0, x1));
    iv
}

// ── core solver ───────────────────────────────────────────────────────

fn solve_core(poly: &Polygon<f64>, g: &GroupedObs, options: &LerOptions) -> Result<LerResult> {
    let (bx0, by0, bx1, by1) = poly_bbox(poly).ok_or_else(|| LirError::InvalidPolygon("degenerate".into()))?;
    if bx1 - bx0 < EPS || by1 - by0 < EPS { return Ok(LerResult::empty()); }

    let has_any = !g.points.is_empty() || !g.lines.is_empty() || !g.rects.is_empty();
    if !has_any {
        if aspect_ok(bx1 - bx0, by1 - by0, options) {
            let r = Rectangle { x_min: bx0, y_min: by0, x_max: bx1, y_max: by1 };
            return Ok(LerResult { area: r.area(), rect: Some(r), rect_polygon: Some(Rect::new(Coord { x: bx0, y: by0 }, Coord { x: bx1, y: by1 }).to_polygon()), angle_deg: 0.0, best_effort: false });
        }
        return Ok(LerResult::empty());
    }

    let xs = collect_all_x_candidates(poly, g);
    if xs.len() < 2 { return Ok(LerResult::empty()); }

    let mut best: Option<(f64, f64, f64, f64, f64)> = None;
    let mut best_area = 0.0;

    for i in 0..xs.len() {
        for j in (i + 1)..xs.len() {
            let x0 = xs[i];
            let x1 = xs[j];
            if x1 <= x0 + EPS { continue; }

            let mut intervals = collect_all_y_intervals(g, x0, x1);
            let Some((y0, y1)) = find_largest_y_gap(&mut intervals, by0, by1) else { continue; };
            if y1 <= y0 + EPS { continue; }

            let w = x1 - x0;
            let h = y1 - y0;
            if !aspect_ok(w, h, options) { continue; }

            let area = w * h;
            if area > best_area + EPS {
                best_area = area;
                best = Some((x0, y0, x1, y1, area));
            }
        }
    }

    match best {
        Some((x0, y0, x1, y1, _)) => {
            let r = Rectangle { x_min: x0, y_min: y0, x_max: x1, y_max: y1 };
            let area = r.area();
            Ok(LerResult { area, rect: Some(r.clone()), rect_polygon: Some(Rect::new(Coord { x: r.x_min, y: r.y_min }, Coord { x: r.x_max, y: r.y_max }).to_polygon()), angle_deg: 0.0, best_effort: false })
        }
        None => Ok(LerResult::empty()),
    }
}

// ── public API ────────────────────────────────────────────────────────

/// Polygon obstacles only (backward-compatible).
pub fn solve_ler_axis_aligned_exact(poly: &Polygon<f64>, obstacles: &[Polygon<f64>], options: &LerOptions) -> Result<LerResult> {
    let g = GroupedObs { points: vec![], lines: vec![], rects: rect::build(obstacles) };
    solve_core(poly, &g, options)
}

pub fn solve_ler_axis_aligned_grid(poly: &Polygon<f64>, obstacles: &[Polygon<f64>], options: &LerOptions) -> Result<LerResult> {
    solve_ler_axis_aligned_exact(poly, obstacles, options)
}

/// Line obstacles with thickness (same thick‑rect behaviour as before).
pub fn solve_ler_axis_aligned_with_lines(
    poly: &Polygon<f64>,
    polygon_obstacles: &[Polygon<f64>],
    line_obstacles: &[LineString<f64>],
    line_thickness: f64,
    options: &LerOptions,
) -> Result<LerResult> {
    let mut inputs: Vec<ObstacleInput> = Vec::new();
    for p in polygon_obstacles { inputs.push(ObstacleInput::Polygon(p.clone())); }
    let half = line_thickness / 2.0;
    for ls in line_obstacles {
        let coords: Vec<Coord<f64>> = ls.coords().copied().collect();
        if coords.len() >= 2 {
            let (ax, ay) = (coords[0].x, coords[0].y);
            let (bx, by) = (coords[1].x, coords[1].y);
            let x0 = ax.min(bx) - half;
            let x1 = ax.max(bx) + half;
            let y0 = ay.min(by) - half;
            let y1 = ay.max(by) + half;
            let rect_poly = Polygon::new(
                LineString::from(vec![
                    Coord { x: x0, y: y0 }, Coord { x: x1, y: y0 },
                    Coord { x: x1, y: y1 }, Coord { x: x0, y: y1 },
                    Coord { x: x0, y: y0 },
                ]),
                vec![],
            );
            inputs.push(ObstacleInput::Polygon(rect_poly));
        }
    }
    solve_ler_axis_aligned_mixed(poly, &inputs, options)
}

/// Unified solver with automatic detection.
pub fn solve_ler_axis_aligned_mixed(
    poly: &Polygon<f64>,
    obstacles: &[ObstacleInput],
    options: &LerOptions,
) -> Result<LerResult> {
    let g = group_inputs(obstacles);
    solve_core(poly, &g, options)
}

// ── tests ─────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use geo::Contains;
    use geo_types::{coord, LineString, Point};

    fn rp(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
        Polygon::new(LineString::from(vec![
            coord! { x: x0, y: y0 }, coord! { x: x1, y: y0 },
            coord! { x: x1, y: y1 }, coord! { x: x0, y: y1 },
            coord! { x: x0, y: y0 },
        ]), vec![])
    }
    fn opts() -> LerOptions { LerOptions::default() }

    // ── backward‑compatible rect tests (13 existing) ──
    #[test] fn no_obstacles_fills_box() { let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,10.), &[], &opts()).unwrap(); assert!(r.area > 99.0); }
    #[test] fn thin_vertical_wall_centre() { let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,10.), &[rp(4.95,0.,5.05,10.)], &opts()).unwrap(); assert!(r.area > 45.0); }
    #[test] fn vertical_wall_off_centre() { let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,10.), &[rp(2.95,0.,3.05,10.)], &opts()).unwrap(); assert!(r.area > 60.0); }
    #[test] fn horizontal_wall() { let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,10.), &[rp(0.,3.95,10.,4.05)], &opts()).unwrap(); assert!(r.area > 55.0); }
    #[test] fn two_vertical_walls() { let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,10.), &[rp(2.95,0.,3.05,10.), rp(6.95,0.,7.05,10.)], &opts()).unwrap(); assert!(r.area > 35.0); }
    #[test] fn small_centre_obstacle() { let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,10.), &[rp(4.5,4.5,5.5,5.5)], &opts()).unwrap(); assert!(r.area > 40.0); }
    #[test] fn obstacle_covers_entire_box() { let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,10.), &[rp(0.,0.,10.,10.)], &opts()).unwrap(); assert!(r.area < 1.0); }
    #[test] fn four_corner_obstacles() { let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,10.), &[rp(0.,0.,3.,3.), rp(7.,0.,10.,3.), rp(0.,7.,3.,10.), rp(7.,7.,10.,10.)], &opts()).unwrap(); assert!(r.area > 25.0); }
    #[test] fn result_inside_polygon() { let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,10.), &[rp(3.,3.,7.,7.)], &opts()).unwrap(); if let Some(rect) = &r.rect { assert!(rp(0.,0.,10.,10.).contains(&Point::new((rect.x_min+rect.x_max)*0.5, (rect.y_min+rect.y_max)*0.5))); } }
    #[test] fn degenerate_polygon() { let flat = Polygon::new(LineString::from(vec![coord! { x: 0., y: 0. }, coord! { x: 5., y: 0. }, coord! { x: 0., y: 0. }]), vec![]); let r = solve_ler_axis_aligned_exact(&flat, &[], &opts()); assert!(r.is_ok() && r.unwrap().area == 0.0); }
    #[test] fn obstacle_touching_left_wall() { let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,10.), &[rp(0.,0.,3.,10.)], &opts()).unwrap(); assert!(r.area > 60.0); }
    #[test] fn aspect_max_ratio_square() { let mut o = opts(); o.max_ratio = 1.0; let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,4.), &[], &o).unwrap(); if let Some(rect) = &r.rect { let (w, h) = (rect.x_max-rect.x_min, rect.y_max-rect.y_min); assert!(w.max(h)/w.min(h) <= 1.02); assert!(r.area > 15.0); } }
    #[test] fn aspect_min_ratio_two() { let mut o = opts(); o.min_ratio = 2.0; let r = solve_ler_axis_aligned_exact(&rp(0.,0.,10.,10.), &[], &o).unwrap(); if let Some(rect) = &r.rect { let (w, h) = (rect.x_max-rect.x_min, rect.y_max-rect.y_min); assert!(w.max(h)/w.min(h) >= 1.98); } }

    // ── point obstacle tests ──
    #[test] fn single_point_obstacle() {
        let poly = rp(0.,0.,10.,10.);
        let obstacles = vec![ObstacleInput::Point(coord! { x: 5., y: 5. })];
        let r = solve_ler_axis_aligned_mixed(&poly, &obstacles, &opts()).unwrap();
        assert!(r.area > 20.0 && r.area < 80.0);
        if let Some(rect) = &r.rect {
            let cx = (rect.x_min + rect.x_max) / 2.0;
            let cy = (rect.y_min + rect.y_max) / 2.0;
            assert!((cx - 5.0).abs() > 0.1 || (cy - 5.0).abs() > 0.1);
        }
    }

    #[test] fn points_in_corners() {
        let poly = rp(0.,0.,10.,10.);
        let obstacles = vec![
            ObstacleInput::Point(coord! { x: 2., y: 2. }),
            ObstacleInput::Point(coord! { x: 8., y: 2. }),
            ObstacleInput::Point(coord! { x: 2., y: 8. }),
            ObstacleInput::Point(coord! { x: 8., y: 8. }),
        ];
        let r = solve_ler_axis_aligned_mixed(&poly, &obstacles, &opts()).unwrap();
        assert!(r.area > 0.0);
    }

    // ── line obstacle tests ──
    #[test] fn vertical_line_obstacle() {
        let poly = rp(0.,0.,10.,10.);
        let line = LineString::from(vec![coord! { x: 5., y: 0. }, coord! { x: 5., y: 10. }]);
        let obstacles = vec![ObstacleInput::Line(line)];
        let r = solve_ler_axis_aligned_mixed(&poly, &obstacles, &opts()).unwrap();
        assert!(r.area > 0.0);
        if let Some(rect) = &r.rect {
            assert!(rect.x_max <= 5.0 || rect.x_min >= 5.0);
        }
    }

    #[test] fn horizontal_line_obstacle() {
        let poly = rp(0.,0.,10.,10.);
        let line = LineString::from(vec![coord! { x: 0., y: 5. }, coord! { x: 10., y: 5. }]);
        let obstacles = vec![ObstacleInput::Line(line)];
        let r = solve_ler_axis_aligned_mixed(&poly, &obstacles, &opts()).unwrap();
        assert!(r.area > 0.0);
        if let Some(rect) = &r.rect {
            assert!(rect.y_max <= 5.0 || rect.y_min >= 5.0);
        }
    }

    #[test] fn diagonal_line_leaves_space() {
        let poly = rp(0.,0.,10.,10.);
        let line = LineString::from(vec![coord! { x: 3., y: 3. }, coord! { x: 7., y: 7. }]);
        let obstacles = vec![ObstacleInput::Line(line)];
        let r = solve_ler_axis_aligned_mixed(&poly, &obstacles, &opts()).unwrap();
        assert!(r.area > 0.0);
    }

    #[test] fn perpendicular_diagonals() {
        let poly = rp(0.,0.,10.,10.);
        let obstacles = vec![
            ObstacleInput::Line(LineString::from(vec![coord! { x: 2., y: 2. }, coord! { x: 8., y: 8. }])),
            ObstacleInput::Line(LineString::from(vec![coord! { x: 2., y: 8. }, coord! { x: 8., y: 2. }])),
        ];
        let r = solve_ler_axis_aligned_mixed(&poly, &obstacles, &opts()).unwrap();
        assert!(r.area > 0.0);
    }

    // ── mixed ──
    #[test] fn mixed_obstacles() {
        let poly = rp(0.,0.,10.,10.);
        let obs = vec![
            ObstacleInput::Point(coord! { x: 2., y: 2. }),
            ObstacleInput::Line(LineString::from(vec![coord! { x: 7., y: 0. }, coord! { x: 7., y: 10. }])),
            ObstacleInput::Polygon(rp(4.,4.,5.,5.)),
        ];
        let r = solve_ler_axis_aligned_mixed(&poly, &obs, &opts()).unwrap();
        assert!(r.area > 0.0);
    }

    #[test] fn backward_compat_lines_thickness() {
        let poly = rp(0.,0.,10.,10.);
        let line = LineString::from(vec![coord! { x: 5., y: 0. }, coord! { x: 5., y: 10. }]);
        let r = solve_ler_axis_aligned_with_lines(&poly, &[], &[line], 1.0, &opts()).unwrap();
        assert!(r.area > 0.0);
    }
}
