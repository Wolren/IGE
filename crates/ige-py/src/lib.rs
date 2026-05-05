use ige_core::solvers::lir::oriented::{solve_lir_oriented, LirOrientedOptions};
use ige_core::{solve_axis_aligned, AxisAlignedOptions, Rectangle, rotate_polygon};
use ige_core::solvers::mic::{maximum_inscribed_circle, MicEngine, MicOptions, MicUsedEngine, RobustMode};
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

    let result = solve_lir_oriented(&working_polygon, &LirOrientedOptions::default())
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

// ─── LIR Approximate Oriented solver ───────────────────────────────────────

#[pyclass]
#[derive(Clone)]
pub struct PyLirOrientedResult {
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
pub fn solve_lir_oriented_py(
    exterior: Vec<(f64, f64)>,
    max_aspect_ratio: Option<f64>,
    use_parallel_field: bool,
) -> PyResult<PyLirOrientedResult> {
    if exterior.len() < 3 {
        return Err(PyValueError::new_err("polygon exterior must contain at least 3 points"));
    }

    let coords: Vec<Coord<f64>> = exterior
        .into_iter()
        .map(|(x, y)| Coord { x, y })
        .collect();
    let exterior_ls = LineString::from(coords);
    let polygon = Polygon::new(exterior_ls, vec![]);

    let mut opts = LirOrientedOptions::default();
    if let Some(ratio) = max_aspect_ratio {
        opts.max_ratio = ratio;
    }
    opts.use_parallel_field = use_parallel_field;

    let result = solve_lir_oriented(&polygon, &opts)
        .map_err(|e| PyValueError::new_err(format!("solve failed: {e}")))?;

    let rect = result.rect.unwrap_or(ige_core::Rectangle {
        x_min: 0.0, y_min: 0.0, x_max: 0.0, y_max: 0.0,
    });

    Ok(PyLirOrientedResult {
        x_min: rect.x_min,
        y_min: rect.y_min,
        x_max: rect.x_max,
        y_max: rect.y_max,
        area: result.area,
        angle_deg: result.angle_deg,
    })
}

// ─── MIC solver ─────────────────────────────────────────────────────────────

#[pyclass]
#[derive(Clone)]
pub struct PyMicResult {
    #[pyo3(get)]
    pub center_x: f64,
    #[pyo3(get)]
    pub center_y: f64,
    #[pyo3(get)]
    pub radius: f64,
    #[pyo3(get)]
    pub radius_sq: f64,
    #[pyo3(get)]
    pub used_engine: String,
    #[pyo3(get)]
    pub candidate_count: usize,
}

fn parse_mic_engine(value: Option<&str>) -> PyResult<MicEngine> {
    match value.unwrap_or("exact_then_geos") {
        "exact_only" => Ok(MicEngine::ExactOnly),
        "fallback_only" => Ok(MicEngine::FallbackOnly),
        "exact_then_geos" => Ok(MicEngine::ExactThenGeos),
        other => Err(PyValueError::new_err(format!(
            "invalid engine '{other}'; expected exact_only|fallback_only|exact_then_geos"
        ))),
    }
}

fn parse_robust_mode(value: Option<&str>) -> PyResult<RobustMode> {
    match value.unwrap_or("filtered") {
        "fast_f64" => Ok(RobustMode::FastF64),
        "filtered" => Ok(RobustMode::Filtered),
        other => Err(PyValueError::new_err(format!(
            "invalid robust_mode '{other}'; expected fast_f64|filtered"
        ))),
    }
}

fn mic_used_engine_name(value: MicUsedEngine) -> &'static str {
    match value {
        MicUsedEngine::Exact => "exact",
        MicUsedEngine::GeosFallback => "geos_fallback",
    }
}

#[pyfunction(signature = (exterior, engine=None, robust_mode=None))]
pub fn solve_mic_py(
    exterior: Vec<(f64, f64)>,
    engine: Option<&str>,
    robust_mode: Option<&str>,
) -> PyResult<PyMicResult> {
    if exterior.len() < 3 {
        return Err(PyValueError::new_err("polygon exterior must contain at least 3 points"));
    }

    let coords: Vec<Coord<f64>> = exterior
        .into_iter()
        .map(|(x, y)| Coord { x, y })
        .collect();
    let exterior_ls = LineString::from(coords);
    let polygon = Polygon::new(exterior_ls, vec![]);

    let opts = MicOptions {
        engine: parse_mic_engine(engine)?,
        robust_mode: parse_robust_mode(robust_mode)?,
    };

    let result = maximum_inscribed_circle(&polygon, &opts)
        .map_err(|e| PyValueError::new_err(format!("solve failed: {e}")))?;

    Ok(PyMicResult {
        center_x: result.center.x(),
        center_y: result.center.y(),
        radius: result.radius,
        radius_sq: result.radius_sq,
        used_engine: mic_used_engine_name(result.used_engine).to_string(),
        candidate_count: result.candidate_count,
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

    m.add_class::<PyLirOrientedResult>()?;
    m.add_function(wrap_pyfunction!(solve_lir_oriented_py, m)?)?;

    m.add_class::<PyMicResult>()?;
    m.add_function(wrap_pyfunction!(solve_mic_py, m)?)?;

    Ok(())
}
