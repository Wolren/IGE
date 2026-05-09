//! GeoJSON loading and polygon parsing utilities.

use geo_types::{Coord, LineString, Polygon};
use serde_json::Value;
use std::fs;

/// Parse a coordinate ring (linear ring) from a GeoJSON value.
fn parse_ring(value: &Value) -> Option<Vec<Coord<f64>>> {
    let ring = value.as_array()?;
    let mut coords = Vec::new();
    for point in ring {
        let pt = point.as_array()?;
        if pt.len() >= 2 {
            let x = pt[0].as_f64()?;
            let y = pt[1].as_f64()?;
            coords.push(Coord { x, y });
        }
    }
    Some(coords)
}

/// Parse a single Polygon from a GeoJSON geometry object.
pub fn parse_polygon(geom: &Value) -> Option<Polygon<f64>> {
    let arr = geom.get("coordinates")?.as_array()?;
    parse_polygon_coords(arr)
}

/// Parse polygon coordinates array (supports both Polygon and MultiPolygon rings).
pub fn parse_polygon_coords(arr: &[Value]) -> Option<Polygon<f64>> {
    let ext_ring = arr.get(0)?;
    let exterior = parse_ring(ext_ring)?;
    if exterior.len() < 3 {
        return None;
    }
    let exterior_ls = LineString::from(exterior);
    let holes: Vec<LineString<f64>> = arr[1..]
        .iter()
        .filter_map(|ring| parse_ring(ring))
        .filter(|ls| ls.len() >= 3)
        .map(LineString::from)
        .collect();
    if holes.is_empty() {
        Some(Polygon::new(exterior_ls, vec![]))
    } else {
        Some(Polygon::new(exterior_ls, holes))
    }
}

/// Extract all polygons from a GeoJSON geometry (Polygon or MultiPolygon).
pub fn parse_feature_polygons(geom: &Value) -> Vec<Polygon<f64>> {
    let Some(geom_type) = geom.get("type").and_then(|v| v.as_str()) else {
        return Vec::new();
    };
    match geom_type {
        "Polygon" => parse_polygon(geom).into_iter().collect(),
        "MultiPolygon" => {
            let Some(all_polys) = geom.get("coordinates").and_then(|v| v.as_array()) else {
                return Vec::new();
            };
            all_polys
                .iter()
                .filter_map(|poly_coords| poly_coords.as_array())
                .filter_map(|poly_arr| parse_polygon_coords(poly_arr))
                .collect()
        }
        _ => Vec::new(),
    }
}

/// Load polygons from a GeoJSON file or use the default test dataset.
pub fn load_polygons_from(path: Option<&str>) -> Vec<(String, Polygon<f64>)> {
    let content = match path {
        Some(p) => fs::read_to_string(p).expect("Failed to read file"),
        None => include_str!("../../tests/real_world_data/realworld_290.geojson").to_string(),
    };
    let json: Value = serde_json::from_str(&content).expect("Failed to parse GeoJSON");
    let features = json.get("features").expect("No features");
    let arr = features.as_array().expect("Features is not array");
    let mut out = Vec::new();
    for (feature_idx, f) in arr.iter().enumerate() {
        let Some(geom) = f.get("geometry") else {
            continue;
        };
        let polys = parse_feature_polygons(geom);
        if polys.is_empty() {
            continue;
        }
        let fid = f
            .get("properties")
            .and_then(|p| p.get("fid"))
            .and_then(|v| v.as_u64())
            .unwrap_or((feature_idx + 1) as u64);
        let multi = polys.len() > 1;
        for (poly_idx, poly) in polys.into_iter().enumerate() {
            let id = if multi {
                format!("Real #{fid} [{}]", poly_idx + 1)
            } else {
                format!("Real #{fid}")
            };
            out.push((id, poly));
        }
    }
    out
}
