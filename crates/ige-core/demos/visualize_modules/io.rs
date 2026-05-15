//! GeoJSON loading and polygon parsing utilities.

use geo_types::{Coord, LineString, Polygon};
use serde_json::Value;
use std::fs;

/// Parse a LineString from a GeoJSON geometry object.
pub fn parse_linestring(geom: &Value) -> Option<LineString<f64>> {
    let coords = geom.get("coordinates")?.as_array()?;
    let mut points = Vec::new();
    for pt in coords {
        let arr = pt.as_array()?;
        if arr.len() >= 2 {
            let x = arr[0].as_f64()?;
            let y = arr[1].as_f64()?;
            points.push(Coord { x, y });
        }
    }
    if points.len() >= 2 {
        Some(LineString::from(points))
    } else {
        None
    }
}

/// Parse feature lines from a GeoJSON.
pub fn parse_feature_linestrings(geom: &Value) -> Vec<LineString<f64>> {
    let Some(geom_type) = geom.get("type").and_then(|v| v.as_str()) else {
        return Vec::new();
    };
    match geom_type {
        "LineString" => parse_linestring(geom).into_iter().collect(),
        "MultiLineString" => {
            let Some(all_lines) = geom.get("coordinates").and_then(|v| v.as_array()) else {
                return Vec::new();
            };
            all_lines
                .iter()
                .filter_map(|line_coords| {
                    let arr = line_coords.as_array()?;
                    let mut points = Vec::new();
                    for pt in arr {
                        let pt_arr = pt.as_array()?;
                        if pt_arr.len() >= 2 {
                            let x = pt_arr[0].as_f64()?;
                            let y = pt_arr[1].as_f64()?;
                            points.push(Coord { x, y });
                        }
                    }
                    if points.len() >= 2 {
                        Some(LineString::from(points))
                    } else {
                        None
                    }
                })
                .collect()
        }
        _ => Vec::new(),
    }
}

/// Load line features from a GeoJSON file.
pub fn load_linestrings_from(path: Option<&str>) -> Vec<LineString<f64>> {
    let content = match path {
        Some(p) => fs::read_to_string(p).expect("Failed to read lines file"),
        None => return Vec::new(),
    };
    let json: Value = serde_json::from_str(&content).expect("Failed to parse GeoJSON");
    let features = match json.get("features") {
        Some(f) => f.as_array().expect("Features is not array"),
        None => return Vec::new(),
    };
    let mut out = Vec::new();
    for f in features.iter() {
        let Some(geom) = f.get("geometry") else {
            continue;
        };
        let lines = parse_feature_linestrings(geom);
        out.extend(lines);
    }
    out
}

/// Load line features grouped by cluster_id.
/// Returns Vec<(cluster_id, Vec<LineString>)>.
pub fn load_linestrings_clustered(path: Option<&str>) -> Vec<(usize, Vec<LineString<f64>>)> {
    let content = match path {
        Some(p) => fs::read_to_string(p).expect("Failed to read lines file"),
        None => return Vec::new(),
    };
    let json: Value = serde_json::from_str(&content).expect("Failed to parse GeoJSON");
    let features = match json.get("features") {
        Some(f) => f.as_array().expect("Features is not array"),
        None => return Vec::new(),
    };
    let mut cluster_map: std::collections::HashMap<usize, Vec<LineString<f64>>> = std::collections::HashMap::new();
    for f in features.iter() {
        let cluster_id = f
            .get("properties")
            .and_then(|p| p.get("cluster_id"))
            .and_then(|v| v.as_u64())
            .unwrap_or(0) as usize;
        let Some(geom) = f.get("geometry") else {
            continue;
        };
        let lines = parse_feature_linestrings(geom);
        for ls in lines {
            cluster_map.entry(cluster_id).or_default().push(ls);
        }
    }
    let mut out: Vec<(usize, Vec<LineString<f64>>)> = cluster_map.into_iter().collect();
    out.sort_by_key(|(id, _)| *id);
    out
}

/// Compute bounding box of a set of line strings.
pub fn line_cluster_bbox(lines: &[LineString<f64>]) -> Option<(f64, f64, f64, f64)> {
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;
    for ls in lines {
        for c in ls.coords() {
            min_x = min_x.min(c.x);
            min_y = min_y.min(c.y);
            max_x = max_x.max(c.x);
            max_y = max_y.max(c.y);
        }
    }
    if min_x.is_finite() && min_y.is_finite() {
        Some((min_x, min_y, max_x, max_y))
    } else {
        None
    }
}

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

/// Load polygon features grouped by cluster_id.
/// Returns Vec<(cluster_id, Vec<Polygon>)>.
pub fn load_polygons_clustered(path: Option<&str>) -> Vec<(usize, Vec<Polygon<f64>>)> {
    let content = match path {
        Some(p) => fs::read_to_string(p).expect("Failed to read file"),
        None => return Vec::new(),
    };
    let json: Value = serde_json::from_str(&content).expect("Failed to parse GeoJSON");
    let features = match json.get("features") {
        Some(f) => f.as_array().expect("Features is not array"),
        None => return Vec::new(),
    };
    let mut cluster_map: std::collections::HashMap<usize, Vec<Polygon<f64>>> = std::collections::HashMap::new();
    for f in features.iter() {
        let cluster_id = f
            .get("properties")
            .and_then(|p| p.get("cluster_id"))
            .and_then(|v| v.as_u64())
            .unwrap_or(0) as usize;
        let Some(geom) = f.get("geometry") else { continue; };
        let polys = parse_feature_polygons(geom);
        for p in polys {
            cluster_map.entry(cluster_id).or_default().push(p);
        }
    }
    let mut out: Vec<(usize, Vec<Polygon<f64>>)> = cluster_map.into_iter().collect();
    out.sort_by_key(|(id, _)| *id);
    out
}

/// Compute bounding box of a set of polygons.
pub fn polygon_cluster_bbox(polys: &[Polygon<f64>]) -> Option<(f64, f64, f64, f64)> {
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;
    for p in polys {
        for c in p.exterior().coords() {
            min_x = min_x.min(c.x); min_y = min_y.min(c.y);
            max_x = max_x.max(c.x); max_y = max_y.max(c.y);
        }
        for hole in p.interiors() {
            for c in hole.coords() {
                min_x = min_x.min(c.x); min_y = min_y.min(c.y);
                max_x = max_x.max(c.x); max_y = max_y.max(c.y);
            }
        }
    }
    if min_x.is_finite() && min_y.is_finite() {
        Some((min_x, min_y, max_x, max_y))
    } else {
        None
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
