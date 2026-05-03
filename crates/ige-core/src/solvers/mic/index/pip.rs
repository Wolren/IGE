use super::super::input::{HostPolygon, RingMeta};

/// Point-in-polygon check using winding number over ring data.
#[derive(Debug, Clone)]
pub struct PipIndex {
    coords: Vec<[f64; 2]>,
    rings: Vec<RingMeta>,
}

impl PipIndex {
    pub fn new(host: &HostPolygon) -> Self {
        Self {
            coords: host.coords.clone(),
            rings: host.rings.clone(),
        }
    }

    pub fn contains_strict_xy(&self, x: f64, y: f64) -> bool {
        for meta in &self.rings {
            let ring = &self.coords[meta.start..meta.end];
            let inside = point_in_ring(x, y, ring);
            if meta.is_hole {
                if inside {
                    return false;
                }
            } else {
                if !inside {
                    return false;
                }
            }
        }
        true
    }
}

fn point_in_ring(x: f64, y: f64, ring: &[[f64; 2]]) -> bool {
    let mut inside = false;
    let mut j = ring.len() - 1;
    for i in 0..ring.len() {
        let ai = ring[i];
        let aj = ring[j];
        let (ax, ay) = (ai[0], ai[1]);
        let (bx, by) = (aj[0], aj[1]);

        let crosses = (ay > y) != (by > y);
        if crosses {
            let x_intersect = (bx - ax) * (y - ay) / (by - ay) + ax;
            if x < x_intersect {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}
