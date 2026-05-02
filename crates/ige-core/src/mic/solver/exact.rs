use geo::Centroid;
use geo_types::Point;
use rustc_hash::FxHashSet;

use crate::mic::certify::certify_candidate;
use crate::mic::input::SegmentIndex;
use crate::mic::workspace::{MicCandidate, MicWorkspace};
use crate::mic::{MicError, MicOptions, MicResult, MicUsedEngine, RobustMode};

const CANDIDATE_QUANTIZE: f64 = 1e9;
const MAX_TRIPLE_VERTICES: usize = 48;
const MAX_SEG_TRIPLES: usize = 100;
const MAX_SS_VERTICES: usize = 64;

fn quantize_origin(host: &crate::mic::input::HostPolygon) -> (f64, f64) {
    let Some((min_x, min_y, max_x, _max_y)) = host.bounds() else {
        return (0.0, 0.0);
    };
    let span_x = (max_x - min_x).max(1.0);
    (min_x - span_x * 0.1, min_y - span_x * 0.1)
}

pub fn solve_exact(
    workspace: &mut MicWorkspace,
    opts: &MicOptions,
) -> std::result::Result<MicResult, MicError> {
    workspace.clear_candidates();
    let mut seen = FxHashSet::<(i64, i64)>::default();
    let q_origin = quantize_origin(&workspace.host);

    if let Some(c) = workspace.host.polygon.centroid() {
        insert_candidate(workspace, &mut seen, c.x(), c.y(), q_origin);
    }

    let vertices = workspace.host.unique_vertices();
    for v in &vertices {
        insert_candidate(workspace, &mut seen, v[0], v[1], q_origin);
    }

    for seg_idx in 0..workspace.seg_index.len() {
        let (mx, my) = workspace.seg_index.midpoint(seg_idx);
        insert_candidate(workspace, &mut seen, mx, my, q_origin);
    }

    let seg_ref = &workspace.seg_index;
    let cand_buf = &mut workspace.candidate_buf;
    generate_segment_triple_candidates(seg_ref, &mut seen, cand_buf, q_origin);
    generate_ear_candidates(&workspace.host, &mut seen, cand_buf, q_origin);

    if matches!(opts.robust_mode, RobustMode::Filtered) {
        let vertices_ref = workspace.host.unique_vertices();
        generate_seg_seg_vertex_candidates(seg_ref, &vertices_ref, &mut seen, cand_buf, q_origin);

        let sampled = sample_vertices(&vertices, MAX_TRIPLE_VERTICES);
        for i in 0..sampled.len() {
            for j in i + 1..sampled.len() {
                for k in j + 1..sampled.len() {
                    if let Some((cx, cy)) = circumcenter(sampled[i], sampled[j], sampled[k]) {
                        insert_candidate(workspace, &mut seen, cx, cy, q_origin);
                    }
                }
            }
        }
    }

    if workspace.candidate_buf.is_empty() {
        return Err(MicError::NoCircleFound);
    }

    let candidate_count = workspace.candidate_buf.len();
    let mut best_any: Option<MicCandidate> = None;

    let pip_index = &workspace.pip_index;
    let nb_index = &workspace.nb_index;
    let candidate_buf = &mut workspace.candidate_buf;

    for cand in candidate_buf.iter_mut() {
        if !pip_index.contains_strict_xy(cand.x, cand.y) {
            continue;
        }

        let Some((radius_sq, _nearest_idx)) = nb_index.nearest_distance_sq(cand.x, cand.y) else {
            continue;
        };
        if !radius_sq.is_finite() || radius_sq <= 0.0 {
            continue;
        }

        cand.radius_sq = radius_sq;

        if !certify_candidate(pip_index, nb_index, cand.x, cand.y, cand.radius_sq) {
            continue;
        }

        if best_any
            .as_ref()
            .map(|b| cand.radius_sq > b.radius_sq)
            .unwrap_or(true)
        {
            best_any = Some(cand.clone());
        }
    }

    let best = best_any.ok_or(MicError::NoCircleFound)?;
    let center = Point::new(best.x, best.y);
    let support_eps = best.radius_sq.max(1.0) * 1e-10;
    let support_segments =
        nb_index.supporting_segments(best.x, best.y, best.radius_sq, support_eps);

    Ok(MicResult {
        center,
        radius: best.radius_sq.sqrt(),
        radius_sq: best.radius_sq,
        support_segments,
        candidate_count,
        used_engine: MicUsedEngine::Exact,
        component_index: None,
    })
}

fn insert_candidate(
    workspace: &mut MicWorkspace,
    seen: &mut FxHashSet<(i64, i64)>,
    x: f64,
    y: f64,
    q_origin: (f64, f64),
) {
    push_candidate(&mut workspace.candidate_buf, seen, x, y, q_origin);
}

fn push_candidate(
    buf: &mut Vec<MicCandidate>,
    seen: &mut FxHashSet<(i64, i64)>,
    x: f64,
    y: f64,
    q_origin: (f64, f64),
) {
    if !x.is_finite() || !y.is_finite() {
        return;
    }
    let qx = quantize(x - q_origin.0);
    let qy = quantize(y - q_origin.1);
    if !seen.insert((qx, qy)) {
        return;
    }
    buf.push(MicCandidate { x, y, radius_sq: 0.0 });
}

fn quantize(v: f64) -> i64 {
    (v * CANDIDATE_QUANTIZE).round() as i64
}

fn sample_vertices(vertices: &[[f64; 2]], max_vertices: usize) -> Vec<[f64; 2]> {
    if vertices.len() <= max_vertices {
        return vertices.to_vec();
    }
    let step = ((vertices.len() as f64) / (max_vertices as f64)).ceil() as usize;
    vertices.iter().step_by(step.max(1)).copied().collect()
}

fn generate_segment_triple_candidates(
    seg_index: &SegmentIndex,
    seen: &mut FxHashSet<(i64, i64)>,
    candidate_buf: &mut Vec<MicCandidate>,
    q_origin: (f64, f64),
) {
    let n = seg_index.len();
    if n < 3 {
        return;
    }

    let lines = precompute_segment_lines(seg_index);

    let sampled_indices: Vec<usize> = if n <= MAX_SEG_TRIPLES {
        (0..n).collect()
    } else {
        let step = n / MAX_SEG_TRIPLES;
        (0..n).step_by(step.max(1)).take(MAX_SEG_TRIPLES).collect()
    };

    for ii in 0..sampled_indices.len() {
        let i = sampled_indices[ii];
        for jj in ii + 1..sampled_indices.len() {
            let j = sampled_indices[jj];
            for kk in jj + 1..sampled_indices.len() {
                let k = sampled_indices[kk];
                if let Some((cx, cy)) = segment_incenter(&lines, i, j, k) {
                    push_candidate(candidate_buf, seen, cx, cy, q_origin);
                }
            }
        }
    }
}

fn generate_seg_seg_vertex_candidates(
    seg_index: &SegmentIndex,
    vertices: &[[f64; 2]],
    seen: &mut FxHashSet<(i64, i64)>,
    candidate_buf: &mut Vec<MicCandidate>,
    q_origin: (f64, f64),
) {
    if seg_index.len() < 2 || vertices.len() < 1 {
        return;
    }

    let lines = precompute_segment_lines(seg_index);

    let sampled_segs: Vec<usize> = if seg_index.len() <= MAX_SEG_TRIPLES {
        (0..seg_index.len()).collect()
    } else {
        let step = seg_index.len() / MAX_SEG_TRIPLES;
        (0..seg_index.len()).step_by(step.max(1)).take(MAX_SEG_TRIPLES).collect()
    };

    let max_verts = MAX_SS_VERTICES.min(vertices.len());
    let sampled_verts: Vec<[f64; 2]> = if vertices.len() <= max_verts {
        vertices.to_vec()
    } else {
        let step = vertices.len() / max_verts;
        vertices.iter().step_by(step.max(1)).copied().take(max_verts).collect()
    };

    for ii in 0..sampled_segs.len() {
        let i = sampled_segs[ii];
        let li = &lines[i];
        for jj in ii + 1..sampled_segs.len() {
            let j = sampled_segs[jj];
            let lj = &lines[j];

            let nx = li.nx - lj.nx;
            let ny = li.ny - lj.ny;
            let n_len_sq = nx * nx + ny * ny;
            if n_len_sq <= 1e-14 {
                continue;
            }
            let d_ij = li.c - lj.c;
            let inv_n2 = 1.0 / n_len_sq;
            let c0x = nx * d_ij * inv_n2;
            let c0y = ny * d_ij * inv_n2;
            let n_len = n_len_sq.sqrt();
            let dx = -ny / n_len;
            let dy = nx / n_len;

            let dist0 = li.nx * c0x + li.ny * c0y - li.c;
            let nd = li.nx * dx + li.ny * dy;
            let coeff_a = 1.0 - nd * nd;
            if coeff_a.abs() <= 1e-14 {
                continue;
            }
            let inv_2a = 0.5 / coeff_a;

            for v in &sampled_verts {
                let dvx = c0x - v[0];
                let dvy = c0y - v[1];
                let delta_sq = dvx * dvx + dvy * dvy;
                let delta_dot_d = dvx * dx + dvy * dy;

                let coeff_b = 2.0 * (delta_dot_d - dist0 * nd);
                let coeff_c = delta_sq - dist0 * dist0;
                let disc = coeff_b * coeff_b - 4.0 * coeff_a * coeff_c;
                if disc < 0.0 {
                    continue;
                }
                let sqrt_disc = disc.sqrt();

                for t in [(-coeff_b + sqrt_disc) * inv_2a, (-coeff_b - sqrt_disc) * inv_2a] {
                    let cx = c0x + t * dx;
                    let cy = c0y + t * dy;
                    if !cx.is_finite() || !cy.is_finite() {
                        continue;
                    }
                    let d_i = li.nx * cx + li.ny * cy - li.c;
                    if d_i <= 0.0 {
                        continue;
                    }
                    let d_j = lj.nx * cx + lj.ny * cy - lj.c;
                    if d_j <= 0.0 {
                        continue;
                    }
                    push_candidate(candidate_buf, seen, cx, cy, q_origin);
                }
            }
        }
    }
}

fn generate_ear_candidates(
    host: &crate::mic::input::HostPolygon,
    seen: &mut FxHashSet<(i64, i64)>,
    candidate_buf: &mut Vec<MicCandidate>,
    q_origin: (f64, f64),
) {
    let outer = host.outer_ring();
    let n = outer.len();
    if n < 4 {
        return;
    }
    let verts: &[[f64; 2]] = &outer[..n - 1];
    let m = verts.len();
    if m < 3 {
        return;
    }
    for i in 0..m {
        let prev = if i == 0 { m - 1 } else { i - 1 };
        let next = if i + 1 >= m { 0 } else { i + 1 };
        let a = verts[prev];
        let b = verts[i];
        let c = verts[next];
        let cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
        if cross > 1e-14 {
            if let Some((cx, cy)) = circumcenter(a, b, c) {
                push_candidate(candidate_buf, seen, cx, cy, q_origin);
            }
        }
    }
}

struct SegmentLine {
    nx: f64,
    ny: f64,
    c: f64,
}

fn precompute_segment_lines(seg_index: &SegmentIndex) -> Vec<SegmentLine> {
    let mut lines = Vec::with_capacity(seg_index.len());
    for idx in 0..seg_index.len() {
        let dx = seg_index.dir_x[idx];
        let dy = seg_index.dir_y[idx];
        let len = seg_index.len_sq[idx].sqrt();
        let inv_len = 1.0 / len;
        let is_hole = seg_index.is_hole_edge[idx];

        let (nx, ny) = if !is_hole {
            (-dy * inv_len, dx * inv_len)
        } else {
            (dy * inv_len, -dx * inv_len)
        };

        let c = nx * seg_index.ax[idx] + ny * seg_index.ay[idx];
        lines.push(SegmentLine { nx, ny, c });
    }
    lines
}

fn segment_incenter(lines: &[SegmentLine], i: usize, j: usize, k: usize) -> Option<(f64, f64)> {
    let li = &lines[i];
    let lj = &lines[j];
    let lk = &lines[k];

    let a_x = li.nx - lj.nx;
    let a_y = li.ny - lj.ny;
    let d_ij = li.c - lj.c;

    let b_x = li.nx - lk.nx;
    let b_y = li.ny - lk.ny;
    let d_ik = li.c - lk.c;

    let det = a_x * b_y - a_y * b_x;
    if det.abs() <= 1e-14 {
        return None;
    }

    let inv_det = 1.0 / det;
    let x = (d_ij * b_y - d_ik * a_y) * inv_det;
    let y = (a_x * d_ik - b_x * d_ij) * inv_det;

    if !x.is_finite() || !y.is_finite() {
        return None;
    }

    let d_i = li.nx * x + li.ny * y - li.c;
    if d_i <= 0.0 {
        return None;
    }

    Some((x, y))
}

fn circumcenter(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Option<(f64, f64)> {
    let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    if d.abs() <= 1e-14 {
        return None;
    }

    let a2 = a[0] * a[0] + a[1] * a[1];
    let b2 = b[0] * b[0] + b[1] * b[1];
    let c2 = c[0] * c[0] + c[1] * c[1];

    let ux = (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d;
    let uy = (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d;
    if ux.is_finite() && uy.is_finite() {
        Some((ux, uy))
    } else {
        None
    }
}
