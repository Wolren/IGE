use ige_core::bcrs::{solve_bcrs, BcrsOptions};
use ige_core::{solve_axis_aligned, AxisAlignedOptions, Rectangle, rotate_polygon};
use geo::BoundingRect;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use geo_types::{Polygon, LineString, Coord};

// ─── Oriented solver ──────────────────────────────────────────────────────

#[pyclass]
#[derive(Clone)]
pub struct PyOrientedLirResult {
    #[pyo3(get)]
    pub x_min: f64,
    #[pyo3(get)]
    pub y_min: f64,
    #[pyo3(get)]
    pub x_max: f64,
    #[pyo3(get)]
    pub y_max: f64,
    #[pyo3(get)]
    pub area: f64,
}

#[pyfunction(signature = (exterior, rotation_degrees=None))]
pub fn solve_oriented_lir_py(
    exterior: Vec<(f64, f64)>,
    rotation_degrees: Option<f64>,
) -> PyResult<PyOrientedLirResult> {
    if exterior.len() < 3 {
        return Err(PyValueError::new_err("polygon exterior must contain at least 3 points"));
    }

    let coords: Vec<Coord<f64>> = exterior
        .into_iter()
        .map(|(x, y)| Coord { x, y })
        .collect();
    let exterior_ls = LineString::from(coords);
    let polygon = Polygon::new(exterior_ls, vec![]);
    let rotation = rotation_degrees.unwrap_or(0.0);
    let working_polygon = if rotation.abs() > 1e-12 {
        rotate_polygon(&polygon, rotation)
    } else {
        polygon.clone()
    };

    let result = solve_bcrs(&working_polygon, &BcrsOptions::default())
        .map_err(|e| PyValueError::new_err(format!("solve failed: {e}")))?;
    let mut rect_poly = result
        .rect_polygon
        .ok_or_else(|| PyValueError::new_err("solve failed: empty result polygon"))?;
    if rotation.abs() > 1e-12 {
        rect_poly = rotate_polygon(&rect_poly, -rotation);
    }
    let bb = rect_poly
        .bounding_rect()
        .ok_or_else(|| PyValueError::new_err("solve failed: invalid result bounds"))?;
    let result = Rectangle {
        x_min: bb.min().x,
        y_min: bb.min().y,
        x_max: bb.max().x,
        y_max: bb.max().y,
    };

    Ok(PyOrientedLirResult {
        x_min: result.x_min,
        y_min: result.y_min,
        x_max: result.x_max,
        y_max: result.y_max,
        area: result.area(),
    })
}

#[pyfunction]
fn oriented_lir_demo() -> PyResult<String> {
    let result = solve_oriented_lir_py(
        vec![(0.0, 0.0), (8.0, 1.0), (7.0, 7.0), (2.0, 8.0), (-1.0, 4.0)],
        Some(0.0),
    )?;
    Ok(format!(
        "area={:.3}, bounds=({:.3}, {:.3}, {:.3}, {:.3})",
        result.area, result.x_min, result.y_min, result.x_max, result.y_max
    ))
}

// ─── Axis-aligned solver ──────────────────────────────────────────────────

#[pyclass]
#[derive(Clone)]
pub struct PyAxisAlignedResult {
    #[pyo3(get)]
    pub x_min: f64,
    #[pyo3(get)]
    pub y_min: f64,
    #[pyo3(get)]
    pub x_max: f64,
    #[pyo3(get)]
    pub y_max: f64,
    #[pyo3(get)]
    pub area: f64,
}

#[pyfunction(signature = (exterior, max_aspect_ratio=None))]
pub fn solve_axis_aligned_py(
    exterior: Vec<(f64, f64)>,
    max_aspect_ratio: Option<f64>,
) -> PyResult<PyAxisAlignedResult> {
    if exterior.len() < 3 {
        return Err(PyValueError::new_err("polygon exterior must contain at least 3 points"));
    }

    let coords: Vec<Coord<f64>> = exterior
        .into_iter()
        .map(|(x, y)| Coord { x, y })
        .collect();
    let exterior_ls = LineString::from(coords);
    let polygon = Polygon::new(exterior_ls, vec![]);

    let mut opts = AxisAlignedOptions::default();
    if let Some(ratio) = max_aspect_ratio {
        opts.max_ratio = ratio;
    }

    let result = solve_axis_aligned(&polygon, &opts)
        .ok_or_else(|| PyValueError::new_err("solve failed"))?;

    Ok(PyAxisAlignedResult {
        x_min: result.x_min,
        y_min: result.y_min,
        x_max: result.x_max,
        y_max: result.y_max,
        area: result.area(),
    })
}

#[pyfunction]
fn axis_aligned_demo() -> PyResult<String> {
    let result = solve_axis_aligned_py(
        vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)],
        None,
    )?;
    Ok(format!(
        "area={:.3}, bounds=({:.3}, {:.3}, {:.3}, {:.3})",
        result.area, result.x_min, result.y_min, result.x_max, result.y_max
    ))
}

// ─── BCRS solver (oriented, with parallel option) ─────────────────────────

#[pyclass]
#[derive(Clone)]
pub struct PyBcrsResult {
    #[pyo3(get)]
    pub x_min: f64,
    #[pyo3(get)]
    pub y_min: f64,
    #[pyo3(get)]
    pub x_max: f64,
    #[pyo3(get)]
    pub y_max: f64,
    #[pyo3(get)]
    pub area: f64,
    #[pyo3(get)]
    pub angle_deg: f64,
}

#[pyfunction(signature = (exterior, max_aspect_ratio=None, use_parallel_field=false))]
pub fn solve_bcrs_py(
    exterior: Vec<(f64, f64)>,
    max_aspect_ratio: Option<f64>,
    use_parallel_field: bool,
) -> PyResult<PyBcrsResult> {
    if exterior.len() < 3 {
        return Err(PyValueError::new_err("polygon exterior must contain at least 3 points"));
    }

    let coords: Vec<Coord<f64>> = exterior
        .into_iter()
        .map(|(x, y)| Coord { x, y })
        .collect();
    let exterior_ls = LineString::from(coords);
    let polygon = Polygon::new(exterior_ls, vec![]);

    let mut opts = BcrsOptions::default();
    if let Some(ratio) = max_aspect_ratio {
        opts.max_ratio = ratio;
    }
    opts.use_parallel_field = use_parallel_field;

    let result = solve_bcrs(&polygon, &opts)
        .map_err(|e| PyValueError::new_err(format!("solve failed: {e}")))?;

    let rect = result.rect.unwrap_or(ige_core::Rectangle {
        x_min: 0.0, y_min: 0.0, x_max: 0.0, y_max: 0.0,
    });

    Ok(PyBcrsResult {
        x_min: rect.x_min,
        y_min: rect.y_min,
        x_max: rect.x_max,
        y_max: rect.y_max,
        area: result.area,
        angle_deg: result.angle_deg,
    })
}

// ─── Module registration ──────────────────────────────────────────────────

#[pymodule]
fn _native(_py: Python, m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<PyOrientedLirResult>()?;
    m.add_function(wrap_pyfunction!(solve_oriented_lir_py, m)?)?;
    m.add_function(wrap_pyfunction!(oriented_lir_demo, m)?)?;

    m.add_class::<PyAxisAlignedResult>()?;
    m.add_function(wrap_pyfunction!(solve_axis_aligned_py, m)?)?;
    m.add_function(wrap_pyfunction!(axis_aligned_demo, m)?)?;

    m.add_class::<PyBcrsResult>()?;
    m.add_function(wrap_pyfunction!(solve_bcrs_py, m)?)?;

    Ok(())
}
