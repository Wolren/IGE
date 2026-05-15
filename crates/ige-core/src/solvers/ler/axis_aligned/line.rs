const EPS: f64 = 1e-9;

#[derive(Clone, Copy, Debug)]
pub struct LineObs {
    pub ax: f64, pub ay: f64,
    pub bx: f64, pub by: f64,
}

/// Compute the y-range of the segment that overlaps x-span [x0, x1].
fn y_overlap(seg: &LineObs, x0: f64, x1: f64) -> Option<(f64, f64)> {
    let sx0 = seg.ax.min(seg.bx);
    let sx1 = seg.ax.max(seg.bx);
    if sx1 < x0 + EPS || sx0 > x1 - EPS { return None; }

    if (seg.ax - seg.bx).abs() < EPS {
        return Some((seg.ay.min(seg.by), seg.ay.max(seg.by)));
    }

    let clamp_x0 = x0.max(sx0);
    let clamp_x1 = x1.min(sx1);
    if clamp_x1 <= clamp_x0 + EPS { return None; }

    let slope = (seg.by - seg.ay) / (seg.bx - seg.ax);
    let y_at_l = seg.ay + slope * (clamp_x0 - seg.ax);
    let y_at_r = seg.ay + slope * (clamp_x1 - seg.ax);
    let lo = y_at_l.min(y_at_r);
    let hi = y_at_l.max(y_at_r);
    Some((lo, hi))
}

pub fn build(inputs: &[(f64, f64, f64, f64)]) -> Vec<LineObs> {
    inputs.iter().map(|&(ax, ay, bx, by)| LineObs { ax, ay, bx, by }).collect()
}

pub fn collect_x_candidates(obs: &[LineObs]) -> Vec<f64> {
    let mut xs = Vec::new();
    for seg in obs {
        xs.push(seg.ax);
        xs.push(seg.bx);
    }
    xs
}

pub fn y_intervals(obs: &[LineObs], x0: f64, x1: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for seg in obs {
        if let Some((ly0, ly1)) = y_overlap(seg, x0, x1) {
            out.push((ly0 - EPS, ly1 + EPS));
        }
    }
    out
}
