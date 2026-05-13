//! Geometry preparation for BCRS pipeline.
//!
//! Port of `_prepare_polygon` and `_simplify_for_solve` from `bcrs_fast_worker.py`.

use geo::{Area, BoundingRect, Simplify};
use geo_types::Polygon;

const SIMPLIFY_THRESHOLD: usize = 300;
const SIMPLIFY_TOL_FRAC: f64 = 0.001;

pub fn prepare_polygon(poly: &Polygon<f64>) -> Option<()> {
    let n_unique = poly.exterior().0.windows(2).filter(|w| w[0] != w[1]).count()
        + if poly.exterior().0.first() != poly.exterior().0.last() { 1 } else { 0 };
    if n_unique < 3 {
        return None;
    }
    if poly.unsigned_area() <= 0.0 {
        return None;
    }
    Some(())
}

pub fn simplify_for_solve(poly: &Polygon<f64>) -> (Polygon<f64>, bool) {
    let mut n_verts = poly.exterior().0.len();
    for interior in poly.interiors() {
        n_verts += interior.0.len();
    }
    if n_verts <= SIMPLIFY_THRESHOLD {
        return (poly.clone(), false);
    }

    let bb = match poly.bounding_rect() {
        Some(b) => b,
        None => return (poly.clone(), false),
    };
    let span = (bb.max().x - bb.min().x).min(bb.max().y - bb.min().y);
    let tol = span * SIMPLIFY_TOL_FRAC;
    if tol <= 0.0 {
        return (poly.clone(), false);
    }

    let simplified = poly.simplify(&tol);
    if simplified.exterior().0.len() < 4 || simplified.unsigned_area() <= 0.0 {
        return (poly.clone(), false);
    }

    (simplified, true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    #[test]
    fn valid_polygon_ok() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:10.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        assert!(prepare_polygon(&poly).is_some());
    }

    #[test]
    fn triangle_accepted() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        assert!(prepare_polygon(&poly).is_some());
    }
}
