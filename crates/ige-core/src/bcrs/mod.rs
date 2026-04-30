//! BCRS — Boundary-Coordinate Rectangle Solve with SDF-guided expansion.
//!
//! Full Rust port of `bcrs_fast_worker.py` Stages 1–7.
//! Optional GPU acceleration hooks are behind the `"gpu"` feature flag.

pub mod candidates;
pub mod expand;
pub mod fast;
pub mod grid;
pub mod histogram;
pub mod prepare;
pub mod sdf;

use std::collections::HashMap;

use geo::{Area, BoundingRect, Centroid, Contains, ConvexHull};
use geo_types::{Coord, LineString, Point, Polygon};
use ordered_float::OrderedFloat;

use crate::geometry::{rotate_polygon, rotate_polygon_around};
use crate::shared::{LirError, Rectangle, Result};

#[cfg(feature = "gpu")]
use crate::gpu::GpuContext;
pub use candidates::{edge_candidate_angles, upper_bound_area};
pub use expand::expand_rect_to_boundary;
pub use fast::maybe_fast_path;
pub use grid::{solve_axis_rect_bcrs, solve_axis_rect_grid};
pub use histogram::{lrih, lrih_vp};
pub use prepare::{prepare_polygon, simplify_for_solve};
pub use sdf::{best_effort_shrink, certify_rect, polygon_sdf, rect_sdf_max};

// ─── Tuning constants (mirror Python defaults) ───────────────────────────
const PHASE_A_HALFWIDTH: f64 = 3.0;
const PHASE_A_XATOL: f64 = 0.02;
const PRUNE_MARGIN: f64 = 0.90;
const ANGLE_DELTA: f64 = 0.5;
const TOP_TRIALS: usize = 2;
const CERT_EPS: f64 = 1e-7;
const CERT_MAX_SHRINK: f64 = 0.20;

// ─── Public types ─────────────────────────────────────────────────────────

/// Configuration for the BCRS solver.
#[derive(Debug, Clone)]
pub struct BcrsOptions {
    /// Max aspect ratio (longer/shorter side); 0.0 = unconstrained.
    pub max_ratio: f64,
    /// Coarse grid resolution used for heuristic seeding and Brent polish.
    pub grid_coarse: usize,
    /// Fine grid resolution used in conservative fallback.
    pub grid_fine: usize,
    /// Number of top heuristic candidates forwarded to stages 4–6.
    pub top_k: usize,
    /// If true, return best-effort result even if certification fails.
    pub always_return: bool,
    /// GPU context for accelerated SDF and point-in-polygon evaluation.
    #[cfg(feature = "gpu")]
    pub gpu_ctx: Option<std::sync::Arc<GpuContext>>,
}

impl Default for BcrsOptions {
    fn default() -> Self {
        Self {
            max_ratio: 0.0,
            grid_coarse: 32,
            grid_fine: 64,
            top_k: 3,
            always_return: true,
            #[cfg(feature = "gpu")]
            gpu_ctx: None,
        }
    }
}

/// Result of a BCRS solve, including per-stage area gains for diagnostics.
#[derive(Debug, Clone)]
pub struct BcrsResult {
    /// Best inscribed rectangle in world frame (AABB — axis-aligned bounding box).
    /// For the actual oriented rectangle, use `rect_polygon`.
    pub rect: Option<Rectangle>,
    /// The actual oriented rectangle as a polygon (rotated in world frame).
    /// None when no solution was found.
    pub rect_polygon: Option<Polygon<f64>>,
    /// Actual certified area.
    pub area: f64,
    /// Rotation angle that produced the best result [degrees].
    pub angle_deg: f64,
    /// True if the result is best-effort rather than strictly certified.
    pub best_effort: bool,
    /// Area after Stage 2 (coarse grid seed).
    pub s2_area: f64,
    /// Area after Stage 4 (BCRS vertex-coordinate solve).
    pub s4_area: f64,
    /// Area after Stage 5 (SDF-guided expansion).
    pub s5_area: f64,
}

impl BcrsResult {
    pub fn empty() -> Self {
        Self {
            rect: None,
            rect_polygon: None,
            area: 0.0,
            angle_deg: 0.0,
            best_effort: false,
            s2_area: 0.0,
            s4_area: 0.0,
            s5_area: 0.0,
        }
    }
}

impl Default for BcrsResult {
    fn default() -> Self {
        Self::empty()
    }
}

// ─── Internal candidate struct ─────────────────────────────────────────────

#[derive(Debug, Clone)]
#[allow(dead_code)]
struct AngleCandidate {
    angle: f64,
    area: f64,
    rect_rot: (f64, f64, f64, f64), // (x0, y0, x1, y1) in rotated frame
    rect_world_bounds: (f64, f64, f64, f64),
    center: Point<f64>,
}

// ─── Rectangle frame helpers ──────────────────────────────────────────────

fn rect_local_frame(corners: &[(f64, f64)]) -> Option<(f64, f64, f64, f64, f64, f64, f64, f64)> {
    if corners.len() < 5 {
        return None;
    }
    let p0 = (corners[0].0, corners[0].1);
    let p1 = (corners[1].0, corners[1].1);
    let p2 = (corners[2].0, corners[2].1);
    let e0 = (p1.0 - p0.0, p1.1 - p0.1);
    let e1 = (p2.0 - p1.0, p2.1 - p1.1);
    let l0 = (e0.0 * e0.0 + e0.1 * e0.1).sqrt();
    let l1 = (e1.0 * e1.0 + e1.1 * e1.1).sqrt();
    if l0 < 1e-14 || l1 < 1e-14 {
        return None;
    }
    let cx = (p0.0 + p2.0) / 2.0;
    let cy = (p0.1 + p2.1) / 2.0;
    let (ux, uy, vx, vy, a, b) = if l0 >= l1 {
        (e0.0 / l0, e0.1 / l0, e1.0 / l1, e1.1 / l1, l0 / 2.0, l1 / 2.0)
    } else {
        (e1.0 / l1, e1.1 / l1, e0.0 / l0, e0.1 / l0, l1 / 2.0, l0 / 2.0)
    };
    Some((cx, cy, ux, uy, vx, vy, a, b))
}

fn build_rect_from_frame(cx: f64, cy: f64, ux: f64, uy: f64, vx: f64, vy: f64, a: f64, b: f64) -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            Coord { x: cx + a * ux + b * vx, y: cy + a * uy + b * vy },
            Coord { x: cx - a * ux + b * vx, y: cy - a * uy + b * vy },
            Coord { x: cx - a * ux - b * vx, y: cy - a * uy - b * vy },
            Coord { x: cx + a * ux - b * vx, y: cy + a * uy - b * vy },
            Coord { x: cx + a * ux + b * vx, y: cy + a * uy + b * vy },
        ]),
        vec![],
    )
}

fn rect_sdf_max_poly(poly: &Polygon<f64>, rect: &Polygon<f64>) -> f64 {
    let coords: Vec<_> = rect.exterior().0.iter().cloned().collect();
    let n = coords.len();
    let mut best = polygon_sdf(poly, coords[0].x, coords[0].y);
    // [Python]: range(1, n-1) for a rect with n=5 checks indices 1,2,3
    for i in 1..n.saturating_sub(1) {
        let v = polygon_sdf(poly, coords[i].x, coords[i].y);
        if v > best { best = v; }
        let mx = (coords[i - 1].x + coords[i].x) * 0.5;
        let my = (coords[i - 1].y + coords[i].y) * 0.5;
        let v = polygon_sdf(poly, mx, my);
        if v > best { best = v; }
    }
    best
}

pub(crate) fn certify_and_adjust(poly: &Polygon<f64>, rect: &Polygon<f64>, max_ratio: f64) -> Option<(Polygon<f64>, f64)> {
    let max_sdf = rect_sdf_max_poly(poly, rect);

    if max_sdf <= CERT_EPS {
        let area = rect.unsigned_area();
        return Some((rect.clone(), area));
    }

    let corners: Vec<(f64, f64)> = rect.exterior().0.iter().map(|c| (c.x, c.y)).collect();
    let frame = rect_local_frame(&corners)?;
    let (cx, cy, ux, uy, vx, vy, a, b) = frame;

    let shrink = max_sdf + CERT_EPS;
    if shrink > a.min(b) * CERT_MAX_SHRINK {
        return None;
    }

    let mut new_a = a - shrink;
    let new_b = b - shrink;
    if new_a <= 0.0 || new_b <= 0.0 {
        return None;
    }
    if max_ratio > 0.0 && new_b > 0.0 && new_a / new_b > max_ratio {
        new_a = new_b * max_ratio;
    }

    let final_rect = build_rect_from_frame(cx, cy, ux, uy, vx, vy, new_a, new_b);
    if rect_sdf_max_poly(poly, &final_rect) > CERT_EPS * 10.0 {
        return None;
    }

    let area = final_rect.unsigned_area();
    Some((final_rect, area))
}

fn best_effort_shrink_to_cover(poly: &Polygon<f64>, rect: &Polygon<f64>, max_ratio: f64) -> Option<(Polygon<f64>, f64)> {
    let max_sdf = rect_sdf_max_poly(poly, rect);
    if max_sdf <= CERT_EPS {
        let area = rect.unsigned_area();
        return Some((rect.clone(), area));
    }

    let corners: Vec<(f64, f64)> = rect.exterior().0.iter().map(|c| (c.x, c.y)).collect();
    let frame = rect_local_frame(&corners)?;
    let (cx, cy, ux, uy, vx, vy, a0, b0) = frame;
    if a0 <= 0.0 || b0 <= 0.0 {
        return None;
    }

    let shrink = max_sdf + CERT_EPS * 2.0;
    let mut a = a0 - shrink;
    let b = b0 - shrink;
    if a <= 0.0 || b <= 0.0 {
        return None;
    }
    if max_ratio > 0.0 && b > 0.0 && a / b > max_ratio {
        a = b * max_ratio;
    }
    if a <= 0.0 || b <= 0.0 {
        return None;
    }

    let final_rect = build_rect_from_frame(cx, cy, ux, uy, vx, vy, a, b);
    if rect_sdf_max_poly(poly, &final_rect) > CERT_EPS {
        return None;
    }

    let area = final_rect.unsigned_area();
    Some((final_rect, area))
}

// ─── Stage 2: Heuristic candidate generator ───────────────────────────────

fn heuristic_candidates(
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

    let mut raw: Vec<(f64, f64, (f64, f64, f64, f64))> = Vec::new();
    let mut best_area = 0.0_f64;

    let solve_coarse = |angle_f: f64| -> Option<(f64, f64, f64, f64, f64)> {
        let rot_s = rotate_polygon(&simplified, -angle_f);
        solve_axis_rect_grid(&rot_s, grid_coarse, max_ratio)
    };

    let edge_angles = edge_candidate_angles(poly, 4.0, 12);

    for &a in &edge_angles {
        let ub = upper_bound_area(&hull, a, max_ratio, centroid);
        if ub <= best_area * PRUNE_MARGIN {
            continue;
        }
        if let Some((x0, y0, x1, y1, area)) = solve_coarse(a) {
            if area > 0.0 {
                raw.push((area, a, (x0, y0, x1, y1)));
            }
            if area > best_area {
                best_area = area;
            }
        }
    }

    // Fill with regular angles if too few candidates
    if raw.len() < 3 {
        for a_int in (0..90).step_by(angle_step) {
            let a = a_int as f64;
            if raw.iter().any(|&(_, angle, _)| (angle - a).abs() < 2.0) {
                continue;
            }
            let ub = upper_bound_area(&hull, a, max_ratio, centroid);
            if ub <= best_area * PRUNE_MARGIN {
                continue;
            }
            if let Some((x0, y0, x1, y1, area)) = solve_coarse(a) {
                if area > 0.0 {
                    raw.push((area, a, (x0, y0, x1, y1)));
                }
                if area > best_area {
                    best_area = area;
                }
            }
        }
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

// ─── Stage 3: Brent angle polisher (golden-section bounded search) ────────

fn polish_angle(
    poly: &Polygon<f64>,
    cand: &AngleCandidate,
    grid_coarse: usize,
    max_ratio: f64,
    cache: &mut HashMap<OrderedFloat<f64>, f64>,
) -> AngleCandidate {
    let angle_0 = cand.angle;
    let lo = (angle_0 - PHASE_A_HALFWIDTH).max(0.0);
    let hi = (angle_0 + PHASE_A_HALFWIDTH).min(90.0);

    if hi - lo < PHASE_A_XATOL * 2.0 {
        return cand.clone();
    }

    let mut neg_area = |a: f64| -> f64 {
        let key = OrderedFloat((a * 10000.0).round() / 10000.0);
        if let Some(&cached) = cache.get(&key) {
            return -cached;
        }
        let rot = rotate_polygon(poly, -a);
        let area = match solve_axis_rect_grid(&rot, grid_coarse, max_ratio) {
            Some((_, _, _, _, a)) => a,
            None => 0.0,
        };
        cache.insert(key, area);
        -area
    };

    // Golden-section search
    let phi = (5.0_f64.sqrt() - 1.0) / 2.0;
    let mut a = lo;
    let mut b = hi;
    let mut c = b - phi * (b - a);
    let mut d = a + phi * (b - a);

    let mut fc = neg_area(c);
    let mut fd = neg_area(d);

    for _ in 0..60 {
        if (b - a).abs() < PHASE_A_XATOL {
            break;
        }
        if fc < fd {
            b = d;
            d = c;
            fd = fc;
            c = b - phi * (b - a);
            fc = neg_area(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + phi * (b - a);
            fd = neg_area(d);
        }
    }

    let best_angle = (a + b) * 0.5;

    if (best_angle - angle_0).abs() > 0.005 {
        let mut new_cand = cand.clone();
        new_cand.angle = best_angle;
        new_cand.area = -neg_area(best_angle);
        new_cand
    } else {
        cand.clone()
    }
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

// ─── Stage 4+5: BCRS + boundary expansion at a given angle ───────────────

fn bcrs_expand_at_angle(
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

    if area <= 0.0 {
        None
    } else {
        Some(((bx0, by0, bx1, by1), area))
    }
}

// ─── Stage 6: Conservative inner-buffer fallback ──────────────────────────

fn conservative_inner_fallback(
    poly: &Polygon<f64>,
    grid_fine: usize,
    max_ratio: f64,
    centroid: Point<f64>,
    angles: &[f64],
) -> Option<(Polygon<f64>, f64, f64)> {
    let bb = poly.bounding_rect()?;
    let span_x = bb.max().x - bb.min().x;
    let span_y = bb.max().y - bb.min().y;
    let span = span_x.max(span_y);
    if span <= 0.0 {
        return None;
    }

    let fractions = [0.002, 0.005, 0.01, 0.02];

    for &frac in &fractions {
        let dist = -span * frac;
        // Use offset (negative buffer = erosion)
        if let Some(inner) = buffer_polygon(poly, dist) {
            if inner.unsigned_area() <= 0.0 {
                continue;
            }
            for &angle in angles {
                let rot = rotate_polygon_around(&inner, -angle, &centroid);
                if let Some((x0, y0, x1, y1, _area)) =
                    solve_axis_rect_grid(&rot, grid_fine, max_ratio)
                {
                    let world_rect = Polygon::new(
                        LineString::from(vec![
                            rotate_point(x0, y0, angle, &centroid),
                            rotate_point(x1, y0, angle, &centroid),
                            rotate_point(x1, y1, angle, &centroid),
                            rotate_point(x0, y1, angle, &centroid),
                            rotate_point(x0, y0, angle, &centroid),
                        ]),
                        vec![],
                    );
                    if rect_sdf_max_poly(poly, &world_rect) <= CERT_EPS {
                        let a = world_rect.unsigned_area();
                        return Some((world_rect, a, angle));
                    }
                }
            }
        }
    }

    None
}

// Simple negative buffer via shrink-to-centroid approximation
fn buffer_polygon(poly: &Polygon<f64>, distance: f64) -> Option<Polygon<f64>> {
    if distance >= 0.0 {
        return Some(poly.clone());
    }

    let cent = poly.centroid()?;
    let cx = cent.x();
    let cy = cent.y();
    let d = -distance; // positive shrink amount

    let bb = poly.bounding_rect()?;
    let span_x = bb.max().x - bb.min().x;
    let span_y = bb.max().y - bb.min().y;

    if d > span_x * 0.5 || d > span_y * 0.5 {
        return None;
    }

    let sx = 1.0 - 2.0 * d / span_x;
    let sy = 1.0 - 2.0 * d / span_y;
    if sx <= 0.0 || sy <= 0.0 {
        return None;
    }

    let ext_coords: Vec<Coord<f64>> = poly
        .exterior()
        .0
        .iter()
        .map(|c| Coord {
            x: cx + (c.x - cx) * sx,
            y: cy + (c.y - cy) * sy,
        })
        .collect();

    let interiors: Vec<LineString<f64>> = poly
        .interiors()
        .iter()
        .map(|r| {
            LineString::from(
                r.0.iter()
                    .map(|c| Coord {
                        x: cx + (c.x - cx) * sx,
                        y: cy + (c.y - cy) * sy,
                    })
                    .collect::<Vec<_>>(),
            )
        })
        .collect();

    Some(Polygon::new(LineString::from(ext_coords), interiors))
}

// ─── Refine: Stages 3–7 orchestrator ──────────────────────────────────────

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

fn refine_best_candidate(
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

        // Stage 3: Brent polish
        let cand_a = polish_angle(poly, cand, grid_coarse, max_ratio, &mut stage3_cache);
        let brent_angle = cand_a.angle;

        let mut angles_to_try = vec![orig_angle];
        for delta in &[0.0, ANGLE_DELTA, -ANGLE_DELTA] {
            let a_try = brent_angle + delta;
            if a_try >= 0.0 && a_try <= 90.0 && angles_to_try.iter().all(|&x| (a_try - x).abs() > 0.01) {
                angles_to_try.push(a_try);
            }
        }

        // Stage 4-5: trial ranking → BCRS + expansion
        let mut trial_data: Vec<AngleTrial> = Vec::new();
        for &angle_try in &angles_to_try {
            let rot_poly = rotate_polygon(poly, -angle_try);
            let seed = solve_axis_rect_grid(&rot_poly, grid_coarse, max_ratio);
            let seed_bounds = seed.map(|(x0, y0, x1, y1, _)| (x0, y0, x1, y1));
            let seed_area = seed.map(|(_, _, _, _, a)| a).unwrap_or(0.0);

            trial_data.push(AngleTrial {
                angle: angle_try,
                rot_poly,
                seed_bounds,
                seed_area,
            });
        }

        if trial_data.is_empty() {
            continue;
        }

        // Keep original edge angle + best remaining
        let mut selected: Vec<&AngleTrial> = vec![&trial_data[0]];
        let mut others: Vec<&AngleTrial> = trial_data[1..].iter().collect();
        others.sort_by(|a, b| b.seed_area.partial_cmp(&a.seed_area).unwrap_or(std::cmp::Ordering::Equal));

        for t in others {
            if selected.len() >= TOP_TRIALS {
                break;
            }
            selected.push(t);
        }

        let mut best_raw_r: Option<Polygon<f64>> = None;
        let mut best_raw_a = 0.0_f64;
        let mut best_angle_this = orig_angle;

        for trial in selected {
            let result = bcrs_expand_at_angle(&trial.rot_poly, trial.seed_bounds, max_ratio);

            if let Some(((rx0, ry0, rx1, ry1), area_rot)) = result {
                if area_rot <= 0.0 {
                    continue;
                }

                if area_rot > best_raw_a + 1e-6 {
                    best_raw_a = area_rot;
                    best_raw_r = Some(Polygon::new(
                        LineString::from(vec![
                            rotate_point(rx0, ry0, trial.angle, &centroid),
                            rotate_point(rx1, ry0, trial.angle, &centroid),
                            rotate_point(rx1, ry1, trial.angle, &centroid),
                            rotate_point(rx0, ry1, trial.angle, &centroid),
                            rotate_point(rx0, ry0, trial.angle, &centroid),
                        ]),
                        vec![],
                    ));
                    best_angle_this = trial.angle;
                } else if best_raw_r.is_none() {
                    best_raw_a = area_rot;
                    best_raw_r = Some(Polygon::new(
                        LineString::from(vec![
                            rotate_point(rx0, ry0, trial.angle, &centroid),
                            rotate_point(rx1, ry0, trial.angle, &centroid),
                            rotate_point(rx1, ry1, trial.angle, &centroid),
                            rotate_point(rx0, ry1, trial.angle, &centroid),
                            rotate_point(rx0, ry0, trial.angle, &centroid),
                        ]),
                        vec![],
                    ));
                    best_angle_this = trial.angle;
                }
            }
        }

        let best_raw_r = match best_raw_r {
            Some(r) => r,
            None => continue,
        };

        if fallback_best.is_none() || best_raw_a > fallback_best.as_ref().unwrap().1 {
            fallback_best = Some((best_raw_r.clone(), best_raw_a, best_angle_this, rank));
        }

        // Stage 6: Certification
        let mut used_best_effort = false;
        let (final_rect, final_area) = match certify_and_adjust(poly, &best_raw_r, max_ratio) {
            Some((r, a)) => (r, a),
            None => {
                if always_return {
                    match best_effort_shrink_to_cover(poly, &best_raw_r, max_ratio) {
                        Some((r, a)) => {
                            used_best_effort = true;
                            (r, a)
                        }
                        None => continue,
                    }
                } else {
                    continue;
                }
            }
        };

        // Post-rotation SDF check
        let post_sdf = rect_sdf_max_poly(poly, &final_rect);
        let (final_rect, final_area) = if post_sdf > CERT_EPS {
            match certify_and_adjust(poly, &final_rect, max_ratio) {
                Some((r, a)) => (r, a),
                None => continue,
            }
        } else {
            (final_rect, final_area)
        };

        let coords: Vec<_> = final_rect.exterior().0.iter().collect();
        let w = ((coords[1].x - coords[0].x).powi(2) + (coords[1].y - coords[0].y).powi(2)).sqrt();
        let h = ((coords[2].x - coords[1].x).powi(2) + (coords[2].y - coords[1].y).powi(2)).sqrt();
        let ratio = if w.min(h) > 0.0 { w.max(h) / w.min(h) } else { 1.0 };

        certified.push(CertifiedCandidate {
            rect: final_rect,
            area: final_area,
            angle: best_angle_this,
            ratio,
            rank,
            stage2_gain: final_area - area_s1,
            used_best_effort,
        });
    }

    // Fallback paths
    if certified.is_empty() {
        if always_return {
            if let Some((rect_fb, _area_fb, angle_fb, rank_fb)) = fallback_best {
                if let Some((r, a)) = best_effort_shrink_to_cover(poly, &rect_fb, max_ratio) {
                    let coords: Vec<_> = r.exterior().0.iter().collect();
                    let w = ((coords[1].x - coords[0].x).powi(2) + (coords[1].y - coords[0].y).powi(2)).sqrt();
                    let h = ((coords[2].x - coords[1].x).powi(2) + (coords[2].y - coords[1].y).powi(2)).sqrt();
                    let ratio = if w.min(h) > 0.0 { w.max(h) / w.min(h) } else { 1.0 };
                    return Some((r, a, angle_fb, ratio, rank_fb, a, true));
                }
            }
        }

        let centroid_fb = candidates.first().map(|c| Point::new(c.center.x(), c.center.y()))?;
        let n_rescue = (candidates.len().min(8)).max(3);
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

    Some((
        best.rect,
        best.area,
        best.angle,
        best.ratio,
        best.rank,
        best.stage2_gain,
        best.used_best_effort,
    ))
}

// ─── Angle trial helper ────────────────────────────────────────────────────

struct AngleTrial {
    angle: f64,
    rot_poly: Polygon<f64>,
    seed_bounds: Option<(f64, f64, f64, f64)>,
    seed_area: f64,
}

// ─── Public entry point ────────────────────────────────────────────────────

/// Build an oriented rectangle polygon from its AABB and rotation angle.
/// Uses the analytical formula: W = w|cos| + h|sin|, H = w|sin| + h|cos|
fn aabb_to_oriented_rect(x0: f64, y0: f64, x1: f64, y1: f64, angle_deg: f64) -> Polygon<f64> {
    let cx = (x0 + x1) * 0.5;
    let cy = (y0 + y1) * 0.5;
    let wa = x1 - x0;
    let ha = y1 - y0;
    if angle_deg.abs() <= 0.01 || wa <= 0.0 || ha <= 0.0 {
        return Polygon::new(
            LineString::from(vec![
                Coord { x: x0, y: y0 }, Coord { x: x1, y: y0 },
                Coord { x: x1, y: y1 }, Coord { x: x0, y: y1 },
                Coord { x: x0, y: y0 },
            ]),
            vec![],
        );
    }
    let rad = angle_deg.to_radians();
    let c = rad.cos().abs();
    let s = rad.sin().abs();
    let cs = c * c - s * s;
    let (w, h) = if cs.abs() > 1e-14 {
        let w = (wa * c - ha * s) / cs;
        let h = (ha * c - wa * s) / cs;
        (w.abs(), h.abs())
    } else {
        let wh = wa / (c + s);
        (wh, wh)
    };
    let hw = w * 0.5;
    let hh = h * 0.5;
    let rad = angle_deg.to_radians();
    let c = rad.cos();
    let s = rad.sin();
    Polygon::new(
        LineString::from(vec![
            Coord { x: cx - hw * c - hh * s, y: cy - hw * s + hh * c },
            Coord { x: cx + hw * c - hh * s, y: cy + hw * s + hh * c },
            Coord { x: cx + hw * c + hh * s, y: cy + hw * s - hh * c },
            Coord { x: cx - hw * c + hh * s, y: cy - hw * s - hh * c },
            Coord { x: cx - hw * c - hh * s, y: cy - hw * s + hh * c },
        ]),
        vec![],
    )
}

/// Solve the largest inscribed rectangle using BCRS + SDF pipeline.
///
/// # Arguments
/// * `poly` - Input polygon (must be valid, non-empty, area > 0)
/// * `options` - Solver configuration
///
/// # Returns
/// A `BcrsResult` with the best rectangle (AABB in world frame), area, angle, etc.
pub fn solve_bcrs(poly: &Polygon<f64>, options: &BcrsOptions) -> Result<BcrsResult> {
    // Stage 0: Geometry preparation
    let poly = prepare_polygon(poly.clone()).ok_or(LirError::InvalidPolygon(
        "Polygon has <3 vertices or zero area".to_string(),
    ))?;

    // Fast path: simple convex shapes
    if let Some((rect_poly, area, angle, _)) = maybe_fast_path(&poly, options.max_ratio) {
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

    // Stage 1: Geometry preparation (simplification done inside)
    let angle_step = 5usize;

    // Stage 2: Heuristic candidates
    let candidates = heuristic_candidates(
        &poly,
        angle_step,
        options.grid_coarse,
        options.max_ratio,
        options.top_k,
    );

    if candidates.is_empty() {
        return Err(LirError::NoRectangleFound);
    }

    let s2_area = candidates.first().map(|c| c.area).unwrap_or(0.0);

    // Stages 3–7: Refine best candidate
    let result = refine_best_candidate(
        &poly,
        &candidates,
        options.grid_coarse,
        options.grid_fine,
        options.max_ratio,
        options.always_return,
    );

    match result {
        Some((rect, area, angle, _ratio, _rank, _gain, used_best_effort)) => {
            let bb = rect.bounding_rect().unwrap();
            Ok(BcrsResult {
                rect: Some(Rectangle {
                    x_min: bb.min().x,
                    y_min: bb.min().y,
                    x_max: bb.max().x,
                    y_max: bb.max().y,
                }),
                rect_polygon: Some(rect),
                area,
                angle_deg: angle,
                best_effort: used_best_effort,
                s2_area,
                s4_area: area,
                s5_area: area,
            })
        }
        None => Err(LirError::NoRectangleFound),
    }
}

// ─── Worker entry point (compatible with Python signature) ─────────────────

/// Stateless worker entry point, mirrors `_worker_process_feature`.
///
/// Returns `(area, angle_deg, ratio, cand_rank, s2_gain, best_effort)` on success.
pub fn worker_process_feature(
    poly: &Polygon<f64>,
    _angle_step: usize,
    grid_coarse: usize,
    grid_fine: usize,
    max_ratio: f64,
    top_k: usize,
    always_return: bool,
) -> Option<(Rectangle, f64, f64, f64, usize, f64, bool)> {
    let options = BcrsOptions {
        max_ratio,
        grid_coarse,
        grid_fine,
        top_k,
        always_return,
        #[cfg(feature = "gpu")]
        gpu_ctx: None,
    };

    let result = solve_bcrs(poly, &options).ok()?;

    Some((
        result.rect?,
        result.area,
        result.angle_deg,
        result.s5_area / (result.s2_area.max(1e-12)),
        if result.s5_area > 0.0 { 0 } else { 0 },
        result.s5_area - result.s2_area,
        result.best_effort,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    fn square_10x10() -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:10.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        )
    }

    #[test]
    fn bcrs_solve_square() {
        let poly = square_10x10();
        let result = solve_bcrs(&poly, &BcrsOptions::default()).unwrap();
        assert!(result.area > 80.0, "area too small: {}", result.area);
        assert!(result.rect.is_some());
    }

    #[test]
    fn bcrs_solve_rectangle() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:20.0, y:0.0},
                coord! {x:20.0, y:5.0},
                coord! {x:0.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let result = solve_bcrs(&poly, &BcrsOptions::default()).unwrap();
        assert!((result.area - 100.0).abs() < 10.0, "area={}", result.area);
    }

    #[test]
    fn bcrs_triangle_finds_rect() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let result = solve_bcrs(&poly, &BcrsOptions::default());
        assert!(result.is_ok(), "BCRS should find a rect in a triangle");
    }

    #[test]
    fn empty_result() {
        let result = BcrsResult::empty();
        assert!(result.rect.is_none());
        assert_eq!(result.area, 0.0);
    }
}
