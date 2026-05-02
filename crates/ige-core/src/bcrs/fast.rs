//! Fast-path solver for simple convex polygons.
//!
//! Port of `_maybe_fast_path` from `bcrs_fast_worker.py`.
//! For rectangles and simple convex shapes (<=8 vertices, no holes),
//! the optimal inscribed rectangle is edge-aligned, skipping the full BCRS pipeline.

use geo::{Area, BoundingRect, Centroid, ConvexHull};
use geo_types::{Coord, Point, Polygon};

use crate::bcrs::expand::expand_rect_to_boundary;
use crate::axis_aligned::solve_axis_rect_grid;
use crate::geometry::rotate_polygon;

/// Try the convex fast path. Returns `(certified_polygon, area, angle_deg, ratio)` or `None`.
pub fn maybe_fast_path(poly: &Polygon<f64>, max_ratio: f64) -> Option<(Polygon<f64>, f64, f64, f64)> {
    let coords: Vec<Coord<f64>> = poly.exterior().0.iter().cloned().collect();
    let nv = if coords.len() > 1 { coords.len() - 1 } else { 0 };
    let has_holes = !poly.interiors().is_empty();

    // Rectangle (identity) -- 4 vertices, no holes
    if nv == 4 && !has_holes {
        for i in 0..4 {
            let p0 = coords[i];
            let p1 = coords[(i + 1) % 4];
            let p2 = coords[(i + 2) % 4];
            let v1 = (p1.x - p0.x, p1.y - p0.y);
            let v2 = (p2.x - p1.x, p2.y - p1.y);
            let n1 = (v1.0 * v1.0 + v1.1 * v1.1).sqrt();
            let n2 = (v2.0 * v2.0 + v2.1 * v2.1).sqrt();
            if n1 > 0.0 && n2 > 0.0 {
                let dot = v1.0 * v2.0 + v1.1 * v2.1;
                if (dot / (n1 * n2)).abs() > 1e-6 {
                    break;
                }
            }
            if i == 3 {
                let cp = &coords;
                let wp = ((cp[1].x - cp[0].x).powi(2) + (cp[1].y - cp[0].y).powi(2)).sqrt();
                let hp = ((cp[2].x - cp[1].x).powi(2) + (cp[2].y - cp[1].y).powi(2)).sqrt();

                let e0 = (cp[1].x - cp[0].x, cp[1].y - cp[0].y);
                let e1 = (cp[2].x - cp[1].x, cp[2].y - cp[1].y);
                let ang = if wp >= hp {
                    e0.1.atan2(e0.0).to_degrees() % 90.0
                } else {
                    e1.1.atan2(e1.0).to_degrees() % 90.0
                };

                let rect_poly = Polygon::new(
                    geo_types::LineString::from(vec![cp[0], cp[1], cp[2], cp[3], cp[0]]),
                    vec![],
                );
                if let Some((cert_poly, cert_area)) = super::certify_and_adjust(
                    poly,
                    &rect_poly,
                    max_ratio,
                    crate::tuning::CERT_EPS,
                    crate::tuning::CERT_MAX_SHRINK,
                ) {
                    let corners: Vec<_> = cert_poly.exterior().0.iter().collect();
                    let w = ((corners[1].x - corners[0].x).powi(2) + (corners[1].y - corners[0].y).powi(2)).sqrt();
                    let h = ((corners[2].x - corners[1].x).powi(2) + (corners[2].y - corners[1].y).powi(2)).sqrt();
                    let rat = if w.min(h) > 0.0 { w.max(h) / w.min(h) } else { 1.0 };
                    return Some((cert_poly, cert_area, ang, rat));
                }
                return None;
            }
        }
    }

    // Simple convex: <=8 vertices, no holes, near-convex
    if has_holes || nv < 3 || nv > 8 {
        return None;
    }

    let hull = poly.convex_hull();
    let hull_area = hull.unsigned_area();
    let poly_area = poly.unsigned_area();
    if poly_area <= 0.0 || hull_area / poly_area > 1.005 {
        return None;
    }

    // Edge-aligned angles from hull
    let mut raw_angles: Vec<f64> = Vec::new();
    let hull_coords: Vec<_> = hull.exterior().0.iter().cloned().collect();
    for i in 0..hull_coords.len().saturating_sub(1) {
        let dx = hull_coords[i + 1].x - hull_coords[i].x;
        let dy = hull_coords[i + 1].y - hull_coords[i].y;
        if dx.abs() > 1e-12 || dy.abs() > 1e-12 {
            let a = dy.atan2(dx).to_degrees() % 90.0;
            if !raw_angles.iter().any(|&ra| (a - ra).abs() < 1.0) {
                raw_angles.push(a);
            }
        }
    }
    for fixed in &[0.0, 45.0] {
        if !raw_angles.iter().any(|&ra| (fixed - ra).abs() < 1.0) {
            raw_angles.push(*fixed);
        }
    }
    raw_angles.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let cent = poly.centroid()?;
    let centroid = Point::new(cent.x(), cent.y());
    let mut best: Option<(Polygon<f64>, f64, f64, f64)> = None;

    for &a in &raw_angles {
        let rot = rotate_polygon(poly, -a);
        let seed = solve_axis_rect_grid(&rot, 60, max_ratio);
        let (sx0, sy0, sx1, sy1) = match seed {
            Some((x0, y0, x1, y1, _)) => (x0, y0, x1, y1),
            None => continue,
        };

        let (bx0, by0, bx1, by1) = expand_rect_to_boundary(&rot, sx0, sy0, sx1, sy1, max_ratio);
        let area_r = (bx1 - bx0) * (by1 - by0);
        if area_r <= 0.0 {
            continue;
        }

        let world_rect = Polygon::new(
            geo_types::LineString::from(vec![
                rotate_point(bx0, by0, a, &centroid),
                rotate_point(bx1, by0, a, &centroid),
                rotate_point(bx1, by1, a, &centroid),
                rotate_point(bx0, by1, a, &centroid),
                rotate_point(bx0, by0, a, &centroid),
            ]),
            vec![],
        );

        // Certify the actual oriented rect (not its AABB)
        if let Some((cert_poly, cert_area)) = super::certify_and_adjust(poly, &world_rect, max_ratio, crate::tuning::CERT_EPS, crate::tuning::CERT_MAX_SHRINK) {
            if cert_area > 0.0 {
                let corners: Vec<_> = cert_poly.exterior().0.iter().collect();
                let w = ((corners[1].x - corners[0].x).powi(2) + (corners[1].y - corners[0].y).powi(2)).sqrt();
                let h = ((corners[2].x - corners[1].x).powi(2) + (corners[2].y - corners[1].y).powi(2)).sqrt();
                let rat = if w.min(h) > 0.0 { w.max(h) / w.min(h) } else { 1.0 };
                let candidate = (cert_poly, cert_area, a.rem_euclid(90.0), rat);

                if let Some((_, ref cur_best_area, _, _)) = best {
                    if cert_area > *cur_best_area {
                        best = Some(candidate);
                    }
                } else {
                    best = Some(candidate);
                }
            }
        }
    }

    best
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

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    #[test]
    fn rectangle_is_fast_path() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:5.0},
                coord! {x:0.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let result = maybe_fast_path(&poly, 0.0);
        assert!(result.is_some());
        let (_, area, _, _) = result.unwrap();
        assert!((area - 50.0).abs() < 1.0);
    }

    #[test]
    fn rectangle_fast_path_respects_max_ratio() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:20.0, y:0.0},
                coord! {x:20.0, y:5.0},
                coord! {x:0.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let (_, area, _, ratio) = maybe_fast_path(&poly, 2.0).unwrap();
        assert!(area > 45.0 && area < 55.0, "area={area}");
        assert!(ratio <= 2.0 + 1e-9, "ratio={ratio}");
    }

    #[test]
    fn complex_shape_not_fast_path() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:3.0},
                coord! {x:5.0, y:5.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        assert!(maybe_fast_path(&poly, 0.0).is_none());
    }
}
