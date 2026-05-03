//! Error types for geo-input

use thiserror::Error;

#[derive(Debug, Error)]
pub enum GeoInputError {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Failed to parse: {0}")]
    ParseError(String),

    #[error("Failed to write: {0}")]
    WriteError(String),

    #[error("No polygon found in file")]
    NoPolygon,

    #[error("Unknown file format")]
    UnknownFormat,

    #[error("Feature not enabled: {0}. Enable with corresponding feature flag.")]
    FeatureNotEnabled(String),
}