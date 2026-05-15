const EPS: f64 = 1e-9;

#[derive(Clone, Copy, Debug)]
pub struct PointObs {
    pub x: f64,
    pub y: f64,
}

pub fn build(inputs: &[(f64, f64)]) -> Vec<PointObs> {
    let mut out: Vec<PointObs> = inputs.iter().map(|&(x, y)| PointObs { x, y }).collect();
    out.sort_by(|a, b| a.y.partial_cmp(&b.y).unwrap());
    out
}

pub fn collect_x_candidates(obs: &[PointObs]) -> Vec<f64> {
    obs.iter().map(|p| p.x).collect()
}

pub fn y_intervals(obs: &[PointObs], x0: f64, x1: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for p in obs {
        if p.x > x0 + EPS && p.x < x1 - EPS {
            out.push((p.y - EPS, p.y + EPS));
        }
    }
    out
}
