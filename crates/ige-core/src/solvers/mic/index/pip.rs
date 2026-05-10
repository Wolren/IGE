use super::super::input::{HostPolygon, RingMeta};

/// Bounding box for a single ring.
#[derive(Debug, Clone, Copy)]
struct RingBbox {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

/// Point-in-polygon check using winding number over ring data.
/// Optimized with per-ring bounding box prefilter to skip ray-casting when point is outside ring AABB.
#[derive(Debug, Clone)]
pub struct PipIndex {
    coords: Vec<[f64; 2]>,
    rings: Vec<RingMeta>,
    ring_bboxes: Vec<RingBbox>,
}

impl PipIndex {
    pub fn new(host: &HostPolygon) -> Self {
        let coords = host.coords.clone();
        let rings = host.rings.clone();

        // Precompute per-ring bounding boxes
        let ring_bboxes = rings.iter()
            .map(|meta| {
                let ring = &coords[meta.start..meta.end];
                let mut min_x = f64::INFINITY;
                let mut min_y = f64::INFINITY;
                let mut max_x = f64::NEG_INFINITY;
                let mut max_y = f64::NEG_INFINITY;
                for pt in ring {
                    min_x = min_x.min(pt[0]);
                    min_y = min_y.min(pt[1]);
                    max_x = max_x.max(pt[0]);
                    max_y = max_y.max(pt[1]);
                }
                RingBbox { min_x, min_y, max_x, max_y }
            })
            .collect();

        Self { coords, rings, ring_bboxes }
    }

    pub fn contains_strict_xy(&self, x: f64, y: f64) -> bool {
        for (ring_idx, meta) in self.rings.iter().enumerate() {
            let bbox = self.ring_bboxes[ring_idx];
            // Quick bbox reject: if point outside ring's AABB, it cannot be inside the ring
            if x < bbox.min_x || x > bbox.max_x || y < bbox.min_y || y > bbox.max_y {
                if meta.is_hole {
                    // Outside hole bbox → inside hole → outside polygon (for exterior)
                    // But if point is outside hole bbox, it's outside the hole → OK so far, continue to next ring
                    // Actually: for a hole, we need point NOT inside hole.
                    // If point is outside hole's bbox, it's definitely not inside hole → passes this ring
                } else {
                    // Outside exterior bbox → definitely not inside polygon
                    return false;
                }
            } else {
                // Point inside ring bbox, need full ray-casting
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
