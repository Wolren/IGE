//! Built-in test shapes used for algorithm demonstrations.

use geo_types::{Coord, LineString, Polygon};

/// Create an L-shaped polygon centered at (cx, cy).
pub fn make_l_shape(cx: f64, cy: f64, size: f64) -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            Coord { x: cx - size, y: cy - size },
            Coord { x: cx + size, y: cy - size },
            Coord { x: cx + size, y: cy - size * 0.3 },
            Coord { x: cx + size * 0.3, y: cy - size * 0.3 },
            Coord { x: cx + size * 0.3, y: cy + size },
            Coord { x: cx - size, y: cy + size },
            Coord { x: cx - size, y: cy - size },
        ]),
        vec![],
    )
}

/// Create a U-shaped polygon centered at (cx, cy).
pub fn make_u_shape(cx: f64, cy: f64, size: f64) -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            Coord { x: cx - size, y: cy - size },
            Coord { x: cx + size, y: cy - size },
            Coord { x: cx + size, y: cy + size },
            Coord { x: cx + size * 0.4, y: cy + size },
            Coord { x: cx + size * 0.4, y: cy },
            Coord { x: cx - size * 0.4, y: cy },
            Coord { x: cx - size * 0.4, y: cy + size },
            Coord { x: cx - size, y: cy + size },
            Coord { x: cx - size, y: cy - size },
        ]),
        vec![],
    )
}

/// Create a zigzag-shaped polygon centered at (cx, cy).
pub fn make_zigzag(cx: f64, cy: f64, size: f64) -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            Coord { x: cx - size, y: cy - size },
            Coord { x: cx - size * 0.6, y: cy - size },
            Coord { x: cx - size * 0.2, y: cy },
            Coord { x: cx + size * 0.2, y: cy },
            Coord { x: cx + size * 0.6, y: cy - size },
            Coord { x: cx + size, y: cy - size },
            Coord { x: cx + size, y: cy + size },
            Coord { x: cx + size * 0.6, y: cy + size },
            Coord { x: cx + size * 0.2, y: cy },
            Coord { x: cx - size * 0.2, y: cy },
            Coord { x: cx - size * 0.6, y: cy + size },
            Coord { x: cx - size, y: cy + size },
            Coord { x: cx - size, y: cy - size },
        ]),
        vec![],
    )
}
