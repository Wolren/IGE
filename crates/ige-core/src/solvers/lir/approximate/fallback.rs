//! Conservative fallback and buffer-based erosion.
//!
//! OpenEvolve target: ``--target lir_approximate_oriented/fallback.rs --mode balanced``
//!
//! Stage 6: when no candidate passes certification, erode the polygon
//! by fractional distances and retry.

use geo::{Area, BoundingRect, Centroid};
use geo_types::{Coord, LineString, Point, Polygon};

use super::super::axis_aligned::solve_axis_rect_grid;
use crate::geometry::{rotate_polygon_around};
use super::super::approximate::certify::rect_sdf_max_poly;

/// Erode the polygon and try a fine-grid solve at each rescue angle.
/// Returns ``(rect, area, angle)`` on success.
pub(crate) fn conservative_inner_fallback(
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
    if span <= 0.0 { return None; }

    let fractions = [0.002, 0.005, 0.01, 0.02];

    for &frac in &fractions {
        let dist = -span * frac;
        if let Some(inner) = buffer_polygon(poly, dist) {
            if inner.unsigned_area() <= 0.0 { continue; }
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
                    if rect_sdf_max_poly(poly, &world_rect) <= crate::tuning::CERT_EPS {
                        let a = world_rect.unsigned_area();
                        return Some((world_rect, a, angle));
                    }
                }
            }
        }
    }
    None
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

/// Simple negative buffer: shrink polygon toward centroid.
fn buffer_polygon(poly: &Polygon<f64>, distance: f64) -> Option<Polygon<f64>> {
    if distance >= 0.0 { return Some(poly.clone()); }

    let cent = poly.centroid()?;
    let cx = cent.x();
    let cy = cent.y();
    let d = -distance;

    let bb = poly.bounding_rect()?;
    let span_x = bb.max().x - bb.min().x;
    let span_y = bb.max().y - bb.min().y;

    if d > span_x * 0.5 || d > span_y * 0.5 { return None; }

    let sx = 1.0 - 2.0 * d / span_x;
    let sy = 1.0 - 2.0 * d / span_y;
    if sx <= 0.0 || sy <= 0.0 { return None; }

    let ext_coords: Vec<Coord<f64>> = poly.exterior().0.iter()
        .map(|c| Coord { x: cx + (c.x - cx) * sx, y: cy + (c.y - cy) * sy }).collect();
    let interiors: Vec<LineString<f64>> = poly.interiors().iter()
        .map(|r| LineString::from(r.0.iter().map(|c| Coord { x: cx + (c.x - cx) * sx, y: cy + (c.y - cy) * sy }).collect::<Vec<_>>())).collect();

    Some(Polygon::new(LineString::from(ext_coords), interiors))
}
