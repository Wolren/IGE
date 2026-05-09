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

#[derive(Debug, Clone, Copy)]
struct Candidate {
    angle: f64,
    area: f64,
    rect_rot: (f64, f64, f64, f64),
}

#[derive(Clone, Copy)]
struct TinyRng {
    state: u64,
}

impl TinyRng {
    fn new(seed: u64) -> Self {
        Self { state: seed | 1 }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn uniform(&mut self) -> f64 {
        let v = self.next_u64() >> 11;
        (v as f64) * (1.0 / ((1u64 << 53) as f64))
    }

    fn normal(&mut self) -> f64 {
        let u1 = self.uniform().clamp(1e-12, 1.0 - 1e-12);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
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
        if r.x < minx { minx = r.x }
        if r.x > maxx { maxx = r.x }
        if r.y < miny { miny = r.y }
        if r.y > maxy { maxy = r.y }
        r
    }).collect();

    let holes: Vec<Vec<Coord<f64>>> = poly.interiors().iter().map(|ring| {
        ring.0.iter().map(|c| {
            let r = rotate(c);
            if r.x < minx { minx = r.x }
            if r.x > maxx { maxx = r.x }
            if r.y < miny { miny = r.y }
            if r.y > maxy { maxy = r.y }
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

    // If PCA is enabled, add principal component analysis angles
    if options.use_pca_axes {
        let pca_angles = super::candidates::pca_candidate_angles(poly);
        for pca_angle in pca_angles {
            if !angles.iter().any(|&ea| (ea - pca_angle).abs() < 1.0) {
                angles.push(pca_angle);
            }
        }
    }

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

#[inline]
fn wrap_angle_90(mut angle: f64) -> f64 {
    angle = angle.rem_euclid(90.0);
    if angle >= 90.0 {
        angle -= 90.0;
    }
    angle
}

#[inline]
fn cross2(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    ax * by - ay * bx
}

fn point_in_rotated_polygon(rc: &RotatedCoords, px: f64, py: f64) -> bool {
    let mut inside = false;
    for ring in std::iter::once(rc.exterior.as_slice()).chain(rc.holes.iter().map(|h| h.as_slice())) {
        if ring.len() < 2 {
            continue;
        }
        let mut j = ring.len() - 1usize;
        for i in 0..ring.len() {
            let a = ring[i];
            let b = ring[j];
            let dy = b.y - a.y;
            if ((a.y > py) != (b.y > py)) && (px < (b.x - a.x) * (py - a.y) / (dy + 1e-18) + a.x) {
                inside = !inside;
            }
            j = i;
        }
    }
    inside
}

fn ray_distance_to_boundary(rc: &RotatedCoords, px: f64, py: f64, dx: f64, dy: f64) -> Option<f64> {
    let mut best = f64::INFINITY;
    for ring in std::iter::once(rc.exterior.as_slice()).chain(rc.holes.iter().map(|h| h.as_slice())) {
        for w in ring.windows(2) {
            let a = w[0];
            let b = w[1];
            let ex = b.x - a.x;
            let ey = b.y - a.y;
            let denom = cross2(dx, dy, ex, ey);
            if denom.abs() < 1e-15 {
                continue;
            }
            let relx = a.x - px;
            let rely = a.y - py;
            let t = cross2(relx, rely, ex, ey) / denom;
            let u = cross2(relx, rely, dx, dy) / denom;
            if t > 1e-9 && (-1e-9..=1.0 + 1e-9).contains(&u) && t < best {
                best = t;
            }
        }
    }
    if best.is_finite() { Some(best) } else { None }
}

fn clamp_ratio_about_center(
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    max_ratio: f64,
    min_ratio: f64,
) -> (f64, f64, f64, f64) {
    let w = x1 - x0;
    let h = y1 - y0;
    if w <= 0.0 || h <= 0.0 {
        return (x0, y0, x1, y1);
    }
    let ls = w.max(h);
    let ss = w.min(h);
    let current_ratio = ls / ss;
    if max_ratio > 0.0 && current_ratio > max_ratio {
        let nl = ss * max_ratio;
        let cx = (x0 + x1) * 0.5;
        let cy = (y0 + y1) * 0.5;
        if w >= h {
            (cx - nl * 0.5, y0, cx + nl * 0.5, y1)
        } else {
            (x0, cy - nl * 0.5, x1, cy + nl * 0.5)
        }
    } else if min_ratio > 0.0 && current_ratio < min_ratio {
        let nl = ss * min_ratio;
        let cx = (x0 + x1) * 0.5;
        let cy = (y0 + y1) * 0.5;
        if w >= h {
            (cx - nl * 0.5, y0, cx + nl * 0.5, y1)
        } else {
            (x0, cy - nl * 0.5, x1, cy + nl * 0.5)
        }
    } else {
        (x0, y0, x1, y1)
    }
}

fn cross_ray_clearances(
    rc: &RotatedCoords,
    cx: f64,
    cy: f64,
) -> Option<(f64, f64, f64, f64)> {
    let dpx = ray_distance_to_boundary(rc, cx, cy, 1.0, 0.0)?;
    let dnx = ray_distance_to_boundary(rc, cx, cy, -1.0, 0.0)?;
    let dpy = ray_distance_to_boundary(rc, cx, cy, 0.0, 1.0)?;
    let dny = ray_distance_to_boundary(rc, cx, cy, 0.0, -1.0)?;
    Some((dpx, dnx, dpy, dny))
}

fn candidate_from_clearances(
    angle_deg: f64,
    cx: f64,
    cy: f64,
    (dpx, dnx, dpy, dny): (f64, f64, f64, f64),
    max_ratio: f64,
    min_ratio: f64,
) -> Option<(Candidate, f64)> {
    let mut x0 = cx - dnx;
    let mut x1 = cx + dpx;
    let mut y0 = cy - dny;
    let mut y1 = cy + dpy;
    (x0, y0, x1, y1) = clamp_ratio_about_center(x0, y0, x1, y1, max_ratio, min_ratio);
    let w = (x1 - x0).max(0.0);
    let h = (y1 - y0).max(0.0);
    if w <= 0.0 || h <= 0.0 {
        return None;
    }
    let area = w * h;
    let imbalance = (dpx - dnx).abs() + (dpy - dny).abs();
    let span = (dpx + dnx + dpy + dny).max(1e-9);
    let score = area * (1.0 - 0.35 * (imbalance / span)).max(0.1);
    Some((
        Candidate {
            angle: wrap_angle_90(angle_deg),
            area,
            rect_rot: (x0, y0, x1, y1),
        },
        score,
    ))
}

fn candidate_from_cross_rays_rotated(
    rc: &RotatedCoords,
    cx: f64,
    cy: f64,
    angle_deg: f64,
    max_ratio: f64,
    min_ratio: f64,
) -> Option<(Candidate, f64)> {
    if !point_in_rotated_polygon(rc, cx, cy) {
        return None;
    }
    let clearances = cross_ray_clearances(rc, cx, cy)?;
    candidate_from_clearances(angle_deg, cx, cy, clearances, max_ratio, min_ratio)
}

fn vertex_snapped_valid_candidate(
    rc: &RotatedCoords,
    angle: f64,
    max_ratio: f64,
    min_ratio: f64,
    field_max_coords: usize,
) -> Option<Candidate> {
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
        return None;
    }

    let n_cols = xs_raw.len().saturating_sub(1);
    let n_rows = ys_raw.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 {
        return None;
    }

    let mask = build_mask_parallel(&rc.exterior, &rc.holes, &xs_raw, &ys_raw);
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
        let (x0, y0, x1, y1, area) = lrih_vp(&heights, &xs_raw, &ys_raw, r, max_ratio, min_ratio);
        if area > 0.0 {
            best_local = match best_local {
                Some((_, _, _, _, a)) if area > a => Some((x0, y0, x1, y1, area)),
                None => Some((x0, y0, x1, y1, area)),
                _ => best_local,
            };
        }
    }

    best_local.map(|(x0, y0, x1, y1, area)| Candidate {
        angle,
        area,
        rect_rot: (x0, y0, x1, y1),
    })
}

fn center_seed_from_unconstrained_rect(
    rc: &RotatedCoords,
    angle: f64,
    max_ratio: f64,
    min_ratio: f64,
    current_best_area: f64,
) -> Option<Candidate> {
    let (minx, miny, maxx, maxy) = rc.bbox;
    let cx = (minx + maxx) * 0.5;
    let cy = (miny + maxy) * 0.5;
    if !point_in_rotated_polygon(rc, cx, cy) {
        return None;
    }

    let clearances = cross_ray_clearances(rc, cx, cy)?;
    let clearance_min = clearances
        .0
        .min(clearances.1)
        .min(clearances.2)
        .min(clearances.3);
    let diag = ((maxx - minx).powi(2) + (maxy - miny).powi(2)).sqrt();
    if clearance_min < diag * 0.01 {
        return None;
    }

    let cross_area = (clearances.0 + clearances.1) * (clearances.2 + clearances.3);
    if current_best_area > 0.0 && cross_area < current_best_area * 0.20 {
        return None;
    }

    candidate_from_clearances(angle, cx, cy, clearances, max_ratio, min_ratio).map(|(c, _)| c)
}

/// Fast-path detector for near-rectangular shapes.
/// DISABLED: Area calculation was incorrect, causing false triggers on 
/// non-rectangular polygons (33-41% fill ratio shapes attempting 45° rotations).
/// To re-enable, need accurate signed area computation and higher threshold (>0.90).
#[allow(dead_code)]
fn try_fast_path_rectangle(
    _poly: &Polygon<f64>,
    _max_ratio: f64,
) -> Option<Candidate> {
    None  // Disabled
}

fn run_simulated_annealing_candidates(
    poly: &Polygon<f64>,
    seeds: &[Candidate],
    max_ratio: f64,
    min_ratio: f64,
    coarse_steps: usize,
) -> Vec<Candidate> {
    if seeds.is_empty() {
        return Vec::new();
    }
    let bb = match poly.bounding_rect() {
        Some(v) => v,
        None => return Vec::new(),
    };
    let _diag = ((bb.max().x - bb.min().x).powi(2) + (bb.max().y - bb.min().y).powi(2)).sqrt();

    let chain_count = seeds.len().min(6);
    let mut out = Vec::with_capacity(chain_count * 2);

    for (si, seed) in seeds.iter().take(chain_count).enumerate() {
        let mut rng = TinyRng::new((seed.angle.to_bits() ^ ((si as u64 + 1) * 0x9E37_79B9_7F4A_7C15)) | 1);

        // Initialize chain with the seed candidate
        let mut current_angle = seed.angle;
        let mut current_cand = seed.clone();
        let mut best_cand = seed.clone();

        let iterations = 48usize;
        for k in 0..iterations {
            let t = 1.0 - (k as f64 / iterations as f64);
            let sigma_a = 9.0 * t + 0.4;
            if rng.uniform() < 0.08 {
                // Occasional large jump
            }

            // Propose new angle
            let proposal_angle = wrap_angle_90(current_angle + rng.normal() * sigma_a);

            // Evaluate proposal using coarse evaluate (actual area landscape)
            let Some(proposal_cand) = coarse_evaluate_angle(poly, proposal_angle, coarse_steps, max_ratio, min_ratio) else {
                continue;
            };

            // Accept/reject based on actual coarse area
            let accept = if proposal_cand.area >= current_cand.area {
                true
            } else {
                let temp = 0.08 + 0.92 * t;
                let denom = (current_cand.area.abs().max(1e-9)) * temp;
                ((proposal_cand.area - current_cand.area) / denom).exp() > rng.uniform()
            };

            if accept {
                current_angle = proposal_angle;
                current_cand = proposal_cand;
                if current_cand.area > best_cand.area {
                    best_cand = current_cand.clone();
                }
            }
        }
        out.push(best_cand);
        out.push(current_cand);
    }
    out
}

// --- Coarse sweep ---------------------------------------------------------

fn coarse_evaluate_angles(
    poly: &Polygon<f64>,
    angles: &[f64],
    coarse_steps: usize,
    max_ratio: f64,
    min_ratio: f64,
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
                let (x0, y0, x1, y1, area) = lrih(&heights, &xs, &ys, r, max_ratio, min_ratio);
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
    min_ratio: f64,
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
        let (x0, y0, x1, y1, area) = lrih(&heights, &xs, &ys, r, max_ratio, min_ratio);
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
    min_ratio: f64,
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
        let expanded = expand_rect_to_boundary(&rot, sx0, sy0, sx1, sy1, max_ratio, min_ratio);
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
        let (x0, y0, x1, y1, area) = lrih_vp(&heights, &xs_raw, &ys_raw, r, max_ratio, min_ratio);
        if area > 0.0 {
            best_local = match best_local {
                Some((_, _, _, _, a)) if area > a => Some((x0, y0, x1, y1, area)),
                None => Some((x0, y0, x1, y1, area)),
                _ => best_local,
            };
        }
    }

    let (fx0, fy0, fx1, fy1, _) = best_local?;
    let expanded = expand_rect_to_boundary(&rot, fx0, fy0, fx1, fy1, max_ratio, min_ratio);
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
        super::fast::maybe_fast_path(poly, options.max_ratio, options.min_ratio)
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

        let c = coarse_evaluate_angle(&poly, angle, coarse_steps, options.max_ratio, options.min_ratio);

        if let Some(c) = c {
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
        let refined = coarse_evaluate_angles(&poly, &refinement_angles, coarse_steps, options.max_ratio, options.min_ratio);
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

    if options.use_simulated_annealing {
        let seed_n = candidates.len().min(options.top_k.max(6));
        let sa_candidates = run_simulated_annealing_candidates(&poly, &candidates[..seed_n], options.max_ratio, options.min_ratio, coarse_steps);
        candidates.extend(sa_candidates);
        candidates.sort_by(|a, b| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal));
        let mut seen_sa: Vec<f64> = Vec::new();
        // Wider dedup tolerance for SA: allow candidates within 2.0° to survive dedup
        candidates.retain(|c| {
            if seen_sa.iter().any(|&s| (c.angle - s).abs() < 2.0) {
                false
            } else {
                seen_sa.push(c.angle);
                true
            }
        });
    }

    let top_k = candidates
        .len()
        .min(options.top_k.max(5) + if options.use_simulated_annealing { 4 } else { 0 });

    let fine_results: Vec<Option<LirOrientedResult>> = candidates[..top_k]
        .par_iter()
        .map(|cand| {
            fine_solve_candidate(
                &poly,
                cand,
                options.max_ratio,
                options.min_ratio,
                options.always_return,
                options.field_max_coords,
                options.cert_eps,
                options.cert_max_shrink,
            )
        })
        .collect();

    let mut best = fine_results
        .into_iter()
        .flatten()
        .max_by(|a, b| a.area.partial_cmp(&b.area).unwrap_or(std::cmp::Ordering::Equal))
        .ok_or(LirError::NoRectangleFound)?;

    // Optional post-refinement around the best angle for improved accuracy.
    // This is intentionally gated so default runs stay fast.
    if options.use_parallel_field {
        let deltas = [-0.75_f64, -0.5, -0.25, 0.25, 0.5, 0.75];
        let polish_angles: Vec<f64> = deltas
            .iter()
            .map(|d| best.angle_deg + d)
            .filter(|a| *a >= 0.0 && *a <= 90.0)
            .collect();
        if !polish_angles.is_empty() {
            let polished = coarse_evaluate_angles(&poly, &polish_angles, coarse_steps.max(16), options.max_ratio, options.min_ratio);
            let polished_results: Vec<Option<LirOrientedResult>> = polished
                .iter()
                .map(|cand| {
                    fine_solve_candidate(
                        &poly,
                        cand,
                        options.max_ratio,
                        options.min_ratio,
                        options.always_return,
                        options.field_max_coords,
                        options.cert_eps,
                        options.cert_max_shrink,
                    )
                })
                .collect();
            if let Some(polished_best) = polished_results
                .into_iter()
                .flatten()
                .max_by(|a, b| a.area.partial_cmp(&b.area).unwrap_or(std::cmp::Ordering::Equal))
            {
                if polished_best.area > best.area {
                    best = polished_best;
                }
            }
        }
    }

    if options.use_bootstrap_seeds {
        let angle = best.angle_deg;
        let rc = rotate_coords_only(&poly, angle);
        let baseline_rect_rot = candidates
            .iter()
            .min_by(|a, b| {
                (a.angle - angle)
                    .abs()
                    .partial_cmp(&(b.angle - angle).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|c| c.rect_rot);
        let snapped = vertex_snapped_valid_candidate(
            &rc,
            angle,
            options.max_ratio,
            options.min_ratio,
            options.field_max_coords,
        );

        let mut seed_rect = baseline_rect_rot;
        let mut continue_to_center_seed = true;

        // 1) Vertex-snapped family first. Proceed only when invalid or too low.
        if let Some(snapped_cand) = snapped {
            seed_rect = Some(snapped_cand.rect_rot);
            if let Some(res) = fine_solve_candidate(
                &poly,
                &snapped_cand,
                options.max_ratio,
                options.min_ratio,
                options.always_return,
                options.field_max_coords,
                options.cert_eps,
                options.cert_max_shrink,
            ) {
                if res.area > best.area {
                    best = res;
                    continue_to_center_seed = false;
                }
            }
        }

        // 2) Same-angle center candidate from seed rectangle.
        if continue_to_center_seed {
            if let Some((x0, y0, x1, y1)) = seed_rect {
                let cx = (x0 + x1) * 0.5;
                let cy = (y0 + y1) * 0.5;
                if let Some((center_seed, _)) = candidate_from_cross_rays_rotated(
                    &rc,
                    cx,
                    cy,
                    angle,
                    options.max_ratio,
                    options.min_ratio,
                ) {
                    if let Some(res) = fine_solve_candidate(
                        &poly,
                        &center_seed,
                        options.max_ratio,
                        options.min_ratio,
                        options.always_return,
                        options.field_max_coords,
                        options.cert_eps,
                        options.cert_max_shrink,
                    ) {
                        if res.area > best.area {
                            best = res;
                        }
                    }
                }
            }
        }

        // 3) Separate last check: unconstrained-center proposal.
        if let Some(unconstrained_seed) = center_seed_from_unconstrained_rect(
            &rc,
            angle,
            options.max_ratio,
            options.min_ratio,
            best.area,
        ) {
            if let Some(res) = fine_solve_candidate(
                &poly,
                &unconstrained_seed,
                options.max_ratio,
                options.min_ratio,
                options.always_return,
                options.field_max_coords,
                options.cert_eps,
                options.cert_max_shrink,
            ) {
                if res.area > best.area {
                    best = res;
                }
            }
        }
    }

    if options.use_edge_anchored {
        let mut test_angles: Vec<f64> = Vec::new();

        let edge_angles = edge_candidate_angles(&poly, 8.0, 6);
        test_angles.extend(edge_angles.clone());

        let current_angle = best.angle_deg;
        for delta in &[-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0] {
            let a = current_angle + delta;
            if a >= 0.0 && a <= 90.0 && !test_angles.iter().any(|&x| (x - a).abs() < 1.0) {
                test_angles.push(a);
            }
        }

        let edge_results: Vec<Option<LirOrientedResult>> = test_angles
            .par_iter()
            .map(|&test_angle| {
                let edge_candidates = super::edge_anchor::generate_edge_anchored_candidates(
                    &poly,
                    test_angle,
                    options,
                    best.area,
                );

                let mut local_best: Option<LirOrientedResult> = None;
                for edge_cand in edge_candidates {
                    if let Some(res) = fine_solve_candidate(
                        &poly,
                        &Candidate {
                            angle: edge_cand.angle,
                            area: edge_cand.area,
                            rect_rot: edge_cand.rect_rot,
                        },
                        options.max_ratio,
                        options.min_ratio,
                        options.always_return,
                        options.field_max_coords,
                        options.cert_eps,
                        options.cert_max_shrink,
                    ) {
                        if local_best.is_none() || res.area > local_best.as_ref().unwrap().area {
                            local_best = Some(res);
                        }
                    }
                }
                local_best
            })
            .collect();

        for res_opt in edge_results {
            if let Some(res) = res_opt {
                if res.area > best.area {
                    best = res;
                }
            }
        }
    }

    Ok(best)
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

    #[test]
    fn parallel_with_sa_rescue() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0,y:0.0}, coord! {x:10.0,y:0.0},
                coord! {x:8.0,y:2.0}, coord! {x:6.0,y:8.0},
                coord! {x:2.0,y:9.0}, coord! {x:0.0,y:0.0},
            ]),
            vec![],
        );
        let mut opts = LirOrientedOptions::default();
        opts.use_simulated_annealing = true;
        let r = solve_lir_oriented_parallel(&poly, &opts).unwrap();
        assert!(r.area > 10.0, "area={}", r.area);
        assert!(r.rect_polygon.is_some());
    }

    #[test]
    fn parallel_with_bootstrap_seeds() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0,y:0.0},
                coord! {x:14.0,y:0.0},
                coord! {x:14.0,y:3.0},
                coord! {x:6.0,y:3.0},
                coord! {x:6.0,y:10.0},
                coord! {x:0.0,y:10.0},
                coord! {x:0.0,y:0.0},
            ]),
            vec![],
        );
        let mut opts = LirOrientedOptions::default();
        opts.use_bootstrap_seeds = true;
        let r = solve_lir_oriented_parallel(&poly, &opts).unwrap();
        assert!(r.area > 30.0, "area={}", r.area);
        assert!(r.rect_polygon.is_some());
    }
}
