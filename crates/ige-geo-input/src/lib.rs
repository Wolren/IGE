//! IGE Geo-Input - GIS file format parsing
//!
//! Load polygons from various GIS file formats.
//!
//! # Usage
//!
//! ```rust,ignore
//! use ige_geo_input::load_polygon;
//!
//! match load_polygon("data.geojson") {
//!     Ok(poly) => println!("Loaded polygon with {} points", poly.exterior().0.len()),
//!     Err(e) => eprintln!("Failed: {}", e),
//! }
//! ```

pub mod error;
pub mod geojson_parser;

use std::path::Path;
use crate::error::GeoInputError;
use geo_types::Polygon;

/// Supported file formats based on extension.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileFormat {
    GeoJson,
    Unknown,
}

impl FileFormat {
    pub fn from_path<P: AsRef<Path>>(path: P) -> Self {
        let ext = path.as_ref()
            .extension()
            .and_then(|e| e.to_str())
            .map(|s| s.to_lowercase())
            .unwrap_or_default();

        match ext.as_str() {
            "geojson" | "json" => FileFormat::GeoJson,
            _ => FileFormat::Unknown,
        }
    }
}

/// Load a single polygon from a GeoJSON file.
pub fn load_polygon<P: AsRef<Path>>(path: P) -> Result<Polygon<f64>, GeoInputError> {
    let format = FileFormat::from_path(&path);
    
    match format {
        FileFormat::GeoJson => {
            let content = std::fs::read_to_string(path.as_ref())?;
            geojson_parser::parse_geojson(&content)
        }
        FileFormat::Unknown => {
            let content = std::fs::read_to_string(path.as_ref())?;
            geojson_parser::parse_geojson(&content)
        }
    }
}

/// Load all polygons from a GeoJSON file.
pub fn load_polygons<P: AsRef<Path>>(path: P) -> Result<Vec<Polygon<f64>>, GeoInputError> {
    let content = std::fs::read_to_string(path.as_ref())?;
    geojson_parser::parse_all_geometries(&content)
}

/// Write a polygon to a GeoJSON file.
pub fn write_geojson<P: AsRef<Path>>(path: P, polygon: &Polygon<f64>) -> Result<(), GeoInputError> {
    let json = geojson_parser::polygon_to_geojson(polygon)?;
    std::fs::write(path.as_ref(), json)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unknown_format() {
        // Unknown extension falls back to GeoJSON parsing
        let result = load_polygon("data.xyz");
        assert!(result.is_err());
    }

    #[test]
    fn test_file_format_detection() {
        assert_eq!(FileFormat::from_path("file.geojson"), FileFormat::GeoJson);
        assert_eq!(FileFormat::from_path("file.json"), FileFormat::GeoJson);
        assert_eq!(FileFormat::from_path("file.unknown"), FileFormat::Unknown);
    }
}