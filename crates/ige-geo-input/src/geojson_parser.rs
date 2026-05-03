//! GeoJSON parsing utilities

use crate::error::GeoInputError;
use geo_types::{Coord, LineString, Polygon};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct GeoJson {
    #[serde(rename = "type")]
    pub geo_type: String,
    #[serde(rename = "features")]
    pub features: Option<Vec<Feature>>,
    #[serde(rename = "geometry")]
    pub geometry: Option<Geometry>,
    #[serde(rename = "coordinates")]
    pub coordinates: Option<GeoJsonValue>,
}

#[derive(Debug, Deserialize)]
pub struct Feature {
    pub geometry: Option<Geometry>,
}

#[derive(Debug, Deserialize)]
pub struct Geometry {
    #[serde(rename = "type")]
    pub geo_type: String,
    #[serde(rename = "coordinates")]
    pub coordinates: Option<GeoJsonValue>,
}

#[derive(Debug, Deserialize)]
#[serde(untagged)]
pub enum GeoJsonValue {
    Point([f64; 2]),
    LineString(Vec<[f64; 2]>),
    Polygon(Vec<Vec<[f64; 2]>>),
    MultiPolygon(Vec<Vec<Vec<[f64; 2]>>>),
}

fn coords_to_ring(coords: &[[f64; 2]]) -> LineString<f64> {
    LineString::from(
        coords.iter()
            .map(|c| Coord { x: c[0], y: c[1] })
            .collect::<Vec<_>>()
    )
}

fn rings_to_polygon(rings: &Vec<Vec<[f64; 2]>>) -> Result<Polygon<f64>, GeoInputError> {
    if rings.is_empty() {
        return Err(GeoInputError::NoPolygon);
    }
    
    let exterior = coords_to_ring(&rings[0]);
    let interiors: Vec<LineString<f64>> = rings[1..]
        .iter()
        .map(|ring| coords_to_ring(ring))
        .collect();
    
    Ok(Polygon::new(exterior, interiors))
}

/// Parse a single polygon from GeoJSON string.
pub fn parse_geojson(json: &str) -> Result<Polygon<f64>, GeoInputError> {
    let geo: GeoJson = serde_json::from_str(json)
        .map_err(|e| GeoInputError::ParseError(e.to_string()))?;

    // Try FeatureCollection first
    if let Some(features) = geo.features {
        for feature in features {
            if let Some(geometry) = feature.geometry {
                if let Some(poly) = geometry_to_polygon(&geometry) {
                    return Ok(poly);
                }
            }
        }
        return Err(GeoInputError::NoPolygon);
    }

    // Try direct geometry
    if let Some(geometry) = geo.geometry {
        return geometry_to_polygon(&geometry)
            .ok_or_else(|| GeoInputError::ParseError("No valid polygon found".to_string()));
    }

    Err(GeoInputError::ParseError("No valid geometry found".to_string()))
}

/// Parse all polygons from GeoJSON string.
pub fn parse_all_geometries(json: &str) -> Result<Vec<Polygon<f64>>, GeoInputError> {
    let geo: GeoJson = serde_json::from_str(json)
        .map_err(|e| GeoInputError::ParseError(e.to_string()))?;

    let mut polygons = Vec::new();

    // FeatureCollection
    if let Some(features) = geo.features {
        for feature in features {
            if let Some(geometry) = feature.geometry {
                if let Some(poly) = geometry_to_polygon(&geometry) {
                    polygons.push(poly);
                }
            }
        }
    }

    // Direct geometry
    if let Some(geometry) = geo.geometry {
        if let Some(poly) = geometry_to_polygon(&geometry) {
            polygons.push(poly);
        }
    }

    if polygons.is_empty() {
        Err(GeoInputError::NoPolygon)
    } else {
        Ok(polygons)
    }
}

fn geometry_to_polygon(geometry: &Geometry) -> Option<Polygon<f64>> {
    match geometry.geo_type.as_str() {
        "Polygon" => {
            if let Some(GeoJsonValue::Polygon(rings)) = &geometry.coordinates {
                rings_to_polygon(rings).ok()
            } else {
                None
            }
        }
        "MultiPolygon" => {
            if let Some(GeoJsonValue::MultiPolygon(multi)) = &geometry.coordinates {
                let first = multi.first()?;
                rings_to_polygon(first).ok()
            } else {
                None
            }
        }
        _ => None,
    }
}

/// Convert a polygon to GeoJSON string.
pub fn polygon_to_geojson(polygon: &Polygon<f64>) -> Result<String, GeoInputError> {
    let exterior: Vec<Vec<f64>> = polygon.exterior().0.iter()
        .map(|c| vec![c.x, c.y])
        .collect();
    
    let mut rings = vec![exterior];
    
    for interior in polygon.interiors() {
        let ring: Vec<Vec<f64>> = interior.0.iter()
            .map(|c| vec![c.x, c.y])
            .collect();
        rings.push(ring);
    }
    
    let feature = serde_json::json!({
        "type": "Feature",
        "geometry": {
            "type": "Polygon",
            "coordinates": rings
        },
        "properties": {}
    });
    
    serde_json::to_string_pretty(&feature)
        .map_err(|e| GeoInputError::WriteError(e.to_string()))
}