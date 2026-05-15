use geo::BoundingRect;
use geo_types::Polygon;

const EPS: f64 = 1e-9;

#[derive(Clone, Copy, Debug)]
pub struct RectObs {
    pub x0: f64, pub x1: f64,
    pub y0: f64, pub y1: f64,
}

/// Merge overlapping rects: if two rects overlap in both x and y, their
/// combined bounding box is equivalent for the sweep-line algorithm.
/// This reduces k (obstacle count) without any accuracy loss.
fn merge(obs: &[RectObs]) -> Vec<RectObs> {
    if obs.len() <= 1 { return obs.to_vec(); }

    let mut sorted: Vec<RectObs> = obs.to_vec();
    sorted.sort_by(|a, b| a.x0.partial_cmp(&b.x0).unwrap().then(a.y0.partial_cmp(&b.y0).unwrap()));

    let mut out: Vec<RectObs> = Vec::new();
    for r in sorted {
        if let Some(last) = out.last_mut() {
            // Overlap test: x-ranges overlap AND y-ranges overlap
            if r.x0 < last.x1 + EPS && r.y0 < last.y1 + EPS {
                last.x1 = last.x1.max(r.x1);
                last.y0 = last.y0.min(r.y0);
                last.y1 = last.y1.max(r.y1);
                continue;
            }
        }
        out.push(r);
    }
    out
}

pub fn build(inputs: &[Polygon<f64>]) -> Vec<RectObs> {
    let mut rects: Vec<RectObs> = inputs.iter()
        .filter_map(|p| {
            let bb = p.bounding_rect()?;
            Some(RectObs { x0: bb.min().x, x1: bb.max().x, y0: bb.min().y, y1: bb.max().y })
        })
        .collect();
    let merged = merge(&rects);
    // Sort by y0 so y_intervals returns pre-sorted intervals
    let mut sorted = merged;
    sorted.sort_by(|a, b| a.y0.partial_cmp(&b.y0).unwrap());
    sorted
}

pub fn collect_x_candidates(obs: &[RectObs]) -> Vec<f64> {
    let mut xs = Vec::new();
    for r in obs {
        xs.push(r.x0);
        xs.push(r.x1);
    }
    xs
}

pub fn y_intervals(obs: &[RectObs], x0: f64, x1: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for r in obs {
        if r.x1 > x0 + EPS && r.x0 < x1 - EPS {
            out.push((r.y0, r.y1));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn merge_overlapping() {
        let obs = vec![
            RectObs { x0: 0., x1: 3., y0: 0., y1: 3. },
            RectObs { x0: 2., x1: 5., y0: 2., y1: 5. },
        ];
        let m = merge(&obs);
        assert_eq!(m.len(), 1);
        assert!((m[0].x0 - 0.).abs() < EPS);
        assert!((m[0].x1 - 5.).abs() < EPS);
        assert!((m[0].y0 - 0.).abs() < EPS);
        assert!((m[0].y1 - 5.).abs() < EPS);
    }

    #[test]
    fn merge_non_overlapping() {
        let obs = vec![
            RectObs { x0: 0., x1: 3., y0: 0., y1: 3. },
            RectObs { x0: 5., x1: 8., y0: 5., y1: 8. },
        ];
        let m = merge(&obs);
        assert_eq!(m.len(), 2);
    }

    #[test]
    fn merge_chain() {
        let obs = vec![
            RectObs { x0: 0., x1: 2., y0: 0., y1: 2. },
            RectObs { x0: 1., x1: 3., y0: 1., y1: 3. },
            RectObs { x0: 2., x1: 4., y0: 2., y1: 4. },
        ];
        let m = merge(&obs);
        assert_eq!(m.len(), 1);
        assert!((m[0].x0 - 0.).abs() < EPS);
        assert!((m[0].x1 - 4.).abs() < EPS);
        assert!((m[0].y0 - 0.).abs() < EPS);
        assert!((m[0].y1 - 4.).abs() < EPS);
    }
}
