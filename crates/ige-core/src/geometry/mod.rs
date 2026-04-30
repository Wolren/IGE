//! Geometry utilities for LIRiAP

use geo::Centroid;
use geo_types::{Coord, LineString, Point, Polygon};

/// Rotate polygon around its centroid.
pub fn rotate_polygon(poly: &Polygon<f64>, angle_deg: f64) -> Polygon<f64> {
    if angle_deg.abs() < 1e-9 {
        return poly.clone();
    }
    match poly.centroid() {
        Some(centroid) => rotate_polygon_around(poly, angle_deg, &centroid),
        None => poly.clone(),
    }
}

/// Rotate polygon around a given point.
pub fn rotate_polygon_around(poly: &Polygon<f64>, angle_deg: f64, center: &Point<f64>) -> Polygon<f64> {
    let rad = angle_deg.to_radians();
    let cos_a = rad.cos();
    let sin_a = rad.sin();
    let cx = center.x();
    let cy = center.y();

    let rotate = |c: &Coord<f64>| Coord {
        x: cx + (c.x - cx) * cos_a - (c.y - cy) * sin_a,
        y: cy + (c.x - cx) * sin_a + (c.y - cy) * cos_a,
    };

    let ext = LineString::from(poly.exterior().0.iter().map(&rotate).collect::<Vec<_>>());
    let interiors: Vec<LineString<f64>> = poly.interiors().iter()
        .map(|r| LineString::from(r.0.iter().map(&rotate).collect::<Vec<_>>()))
        .collect();

    Polygon::new(ext, interiors)
}