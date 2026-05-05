//! Shared types and utilities for LIR algorithms.
//!
//! These types are used across all solver implementations.

use geo::Centroid;
use geo_types::{Coord, LineString, Point, Polygon};
use thiserror::Error;

#[derive(Debug, Clone)]
pub struct Rectangle {
    pub x_min: f64,
    pub y_min: f64,
    pub x_max: f64,
    pub y_max: f64,
}

impl Rectangle {
    pub fn area(&self) -> f64 {
        (self.x_max - self.x_min) * (self.y_max - self.y_min)
    }

    pub fn to_polygon(&self) -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                Coord {
                    x: self.x_min,
                    y: self.y_min,
                },
                Coord {
                    x: self.x_max,
                    y: self.y_min,
                },
                Coord {
                    x: self.x_max,
                    y: self.y_max,
                },
                Coord {
                    x: self.x_min,
                    y: self.y_max,
                },
                Coord {
                    x: self.x_min,
                    y: self.y_min,
                },
            ]),
            vec![],
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolygonType {
    ConvexNoHoles,
    ConvexWithHoles,
    ConcaveNoHoles,
    ConcaveWithHoles,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolverBackend {
    Cpu,
    #[cfg(feature = "gpu")]
    Gpu,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlgorithmCategory {
    AxisAligned,
    Oriented,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlgorithmPrecision {
    Exact,
    Approx,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlgorithmSpeed {
    Standard,
    Fast,
}

#[derive(Debug, Error)]
pub enum LirError {
    #[error("Invalid polygon: {0}")]
    InvalidPolygon(String),
    #[error("No rectangle found")]
    NoRectangleFound,
    #[error("GPU error: {0}")]
    GpuError(String),
    #[error("Algorithm not supported: {0}")]
    NotSupported(String),
    #[error("Internal error: {0}")]
    Internal(String),
}

pub type Result<T> = std::result::Result<T, LirError>;

#[derive(Debug, Clone)]
pub struct SolverOptions {
    pub rotation_degrees: f64,
    pub prefer_gpu: bool,
    pub force_cpu: bool,
    pub max_aspect_ratio: f64,
    pub gpu_threshold: usize,
}

impl Default for SolverOptions {
    fn default() -> Self {
        Self {
            rotation_degrees: 0.0,
            prefer_gpu: false,
            force_cpu: true,
            max_aspect_ratio: 0.0,
            gpu_threshold: 1000,
        }
    }
}

pub fn rotate_polygon(poly: &Polygon<f64>, angle_deg: f64) -> Polygon<f64> {
    if angle_deg.abs() < 1e-9 {
        return poly.clone();
    }
    match poly.centroid() {
        Some(centroid) => rotate_polygon_around(poly, angle_deg, &centroid),
        None => poly.clone(),
    }
}

pub fn rotate_polygon_around(
    poly: &Polygon<f64>,
    angle_deg: f64,
    center: &Point<f64>,
) -> Polygon<f64> {
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
    let interiors: Vec<LineString<f64>> = poly
        .interiors()
        .iter()
        .map(|r| LineString::from(r.0.iter().map(&rotate).collect::<Vec<_>>()))
        .collect();

    Polygon::new(ext, interiors)
}
