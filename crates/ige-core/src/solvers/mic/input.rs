use geo::Area;
use geo_types::{Coord, LineString, Polygon};

use super::MicError;

const NORMALIZE_EPS: f64 = 1e-12;

/// Metadata describing a ring inside the flat coordinate buffer.
#[derive(Debug, Clone)]
pub struct RingMeta {
    pub start: usize,
    pub end: usize,
    pub is_hole: bool,
}

/// Normalized polygon input used by MIC solvers.
#[derive(Debug, Clone)]
pub struct HostPolygon {
    /// Flat coordinate storage for all rings.
    pub coords: Vec<[f64; 2]>,
    /// Ring offsets into `coords`.
    pub rings: Vec<RingMeta>,
    /// Canonicalized geometry used for predicates.
    pub polygon: Polygon<f64>,
}

impl HostPolygon {
    pub fn from_polygon(poly: &Polygon<f64>) -> Result<Self, MicError> {
        let outer = normalize_ring(poly.exterior(), false)?;
        let mut holes = Vec::with_capacity(poly.interiors().len());
        for hole in poly.interiors() {
            holes.push(normalize_ring(hole, true)?);
        }

        let mut coords = Vec::new();
        let mut rings = Vec::with_capacity(1 + holes.len());

        let push_ring = |coords: &mut Vec<[f64; 2]>,
                         rings: &mut Vec<RingMeta>,
                         ring: &[[f64; 2]],
                         is_hole: bool| {
            let start = coords.len();
            coords.extend_from_slice(ring);
            let end = coords.len();
            rings.push(RingMeta { start, end, is_hole });
        };

        push_ring(&mut coords, &mut rings, &outer, false);
        for hole in &holes {
            push_ring(&mut coords, &mut rings, hole, true);
        }

        let exterior = ring_to_linestring(&outer);
        let interior_rings: Vec<LineString<f64>> = holes.iter().map(|ring| ring_to_linestring(ring)).collect();
        let normalized_polygon = Polygon::new(exterior, interior_rings);
        if normalized_polygon.unsigned_area() <= NORMALIZE_EPS {
            return Err(MicError::InvalidInput(
                "polygon area is zero after normalization".to_string(),
            ));
        }

        Ok(Self {
            coords,
            rings,
            polygon: normalized_polygon,
        })
    }

    pub fn ring_coords(&self, ring_id: usize) -> &[[f64; 2]] {
        let meta = &self.rings[ring_id];
        &self.coords[meta.start..meta.end]
    }

    pub fn outer_ring(&self) -> &[[f64; 2]] {
        self.ring_coords(0)
    }

    pub fn ring_count(&self) -> usize {
        self.rings.len()
    }

    pub fn unique_vertices(&self) -> Vec<[f64; 2]> {
        let mut out = Vec::new();
        for ring_id in 0..self.rings.len() {
            let ring = self.ring_coords(ring_id);
            if ring.len() < 2 {
                continue;
            }
            for p in &ring[..ring.len() - 1] {
                out.push(*p);
            }
        }
        out
    }

    pub fn bounds(&self) -> Option<(f64, f64, f64, f64)> {
        if self.coords.is_empty() {
            return None;
        }
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;

        for p in &self.coords {
            min_x = min_x.min(p[0]);
            min_y = min_y.min(p[1]);
            max_x = max_x.max(p[0]);
            max_y = max_y.max(p[1]);
        }

        Some((min_x, min_y, max_x, max_y))
    }
}

/// Struct-of-arrays segment table over all ring edges.
#[derive(Debug, Clone)]
pub struct SegmentIndex {
    pub ax: Vec<f64>,
    pub ay: Vec<f64>,
    pub bx: Vec<f64>,
    pub by: Vec<f64>,
    pub ring_id: Vec<usize>,
    pub edge_id: Vec<usize>,
    pub is_hole_edge: Vec<bool>,
    pub bbox_minx: Vec<f64>,
    pub bbox_maxx: Vec<f64>,
    pub bbox_miny: Vec<f64>,
    pub bbox_maxy: Vec<f64>,
    pub dir_x: Vec<f64>,
    pub dir_y: Vec<f64>,
    pub len_sq: Vec<f64>,
}

impl SegmentIndex {
    pub fn from_host(host: &HostPolygon) -> Self {
        let mut index = Self {
            ax: Vec::new(),
            ay: Vec::new(),
            bx: Vec::new(),
            by: Vec::new(),
            ring_id: Vec::new(),
            edge_id: Vec::new(),
            is_hole_edge: Vec::new(),
            bbox_minx: Vec::new(),
            bbox_maxx: Vec::new(),
            bbox_miny: Vec::new(),
            bbox_maxy: Vec::new(),
            dir_x: Vec::new(),
            dir_y: Vec::new(),
            len_sq: Vec::new(),
        };

        for (rid, meta) in host.rings.iter().enumerate() {
            let ring = host.ring_coords(rid);
            if ring.len() < 2 {
                continue;
            }
            for eid in 0..ring.len() - 1 {
                let a = ring[eid];
                let b = ring[eid + 1];
                let dx = b[0] - a[0];
                let dy = b[1] - a[1];
                let len_sq = dx * dx + dy * dy;
                if len_sq <= NORMALIZE_EPS {
                    continue;
                }

                index.ax.push(a[0]);
                index.ay.push(a[1]);
                index.bx.push(b[0]);
                index.by.push(b[1]);
                index.ring_id.push(rid);
                index.edge_id.push(eid);
                index.is_hole_edge.push(meta.is_hole);
                index.bbox_minx.push(a[0].min(b[0]));
                index.bbox_maxx.push(a[0].max(b[0]));
                index.bbox_miny.push(a[1].min(b[1]));
                index.bbox_maxy.push(a[1].max(b[1]));
                index.dir_x.push(dx);
                index.dir_y.push(dy);
                index.len_sq.push(len_sq);
            }
        }

        index
    }

    pub fn len(&self) -> usize {
        self.ax.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn midpoint(&self, seg_idx: usize) -> (f64, f64) {
        (
            (self.ax[seg_idx] + self.bx[seg_idx]) * 0.5,
            (self.ay[seg_idx] + self.by[seg_idx]) * 0.5,
        )
    }

    pub fn point_segment_distance_sq(&self, seg_idx: usize, x: f64, y: f64) -> f64 {
        let ax = self.ax[seg_idx];
        let ay = self.ay[seg_idx];
        let dx = self.dir_x[seg_idx];
        let dy = self.dir_y[seg_idx];
        let len_sq = self.len_sq[seg_idx];

        let t = (((x - ax) * dx + (y - ay) * dy) / len_sq).clamp(0.0, 1.0);
        let px = ax + t * dx;
        let py = ay + t * dy;
        let ex = x - px;
        let ey = y - py;
        ex * ex + ey * ey
    }
}

fn normalize_ring(ring: &LineString<f64>, is_hole: bool) -> Result<Vec<[f64; 2]>, MicError> {
    let mut pts = Vec::<[f64; 2]>::new();
    for c in &ring.0 {
        if !c.x.is_finite() || !c.y.is_finite() {
            return Err(MicError::InvalidInput(
                "ring contains non-finite coordinates".to_string(),
            ));
        }
        let p = [c.x, c.y];
        if pts
            .last()
            .map(|last| approx_same(*last, p))
            .unwrap_or(false)
        {
            continue;
        }
        pts.push(p);
    }

    if pts.len() < 3 {
        return Err(MicError::InvalidInput(
            "ring has fewer than 3 distinct vertices".to_string(),
        ));
    }

    if approx_same(*pts.first().expect("ring has first"), *pts.last().expect("ring has last")) {
        pts.pop();
    }

    if pts.len() < 3 {
        return Err(MicError::InvalidInput(
            "ring collapsed after closure normalization".to_string(),
        ));
    }

    let signed_area = ring_signed_area_open(&pts);
    if signed_area.abs() <= NORMALIZE_EPS {
        return Err(MicError::InvalidInput(
            "ring area is zero after normalization".to_string(),
        ));
    }

    let should_be_ccw = !is_hole;
    let is_ccw = signed_area > 0.0;
    if should_be_ccw != is_ccw {
        pts.reverse();
    }

    pts.push(*pts.first().expect("normalized ring has first"));
    Ok(pts)
}

fn ring_to_linestring(ring: &[[f64; 2]]) -> LineString<f64> {
    let coords = ring
        .iter()
        .map(|p| Coord { x: p[0], y: p[1] })
        .collect::<Vec<_>>();
    LineString::from(coords)
}

fn approx_same(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).abs() <= NORMALIZE_EPS && (a[1] - b[1]).abs() <= NORMALIZE_EPS
}

fn ring_signed_area_open(open_ring: &[[f64; 2]]) -> f64 {
    let n = open_ring.len();
    let mut sum = 0.0;
    for i in 0..n {
        let a = open_ring[i];
        let b = open_ring[(i + 1) % n];
        sum += a[0] * b[1] - b[0] * a[1];
    }
    sum * 0.5
}
