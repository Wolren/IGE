use geo::BoundingRect;
use geo_types::{Coord, LineString, Polygon};
use ige_core::solvers::lir::oriented::{solve_lir_oriented, LirOrientedOptions};
use ige_core::solvers::mic::{maximum_inscribed_circle, MicEngine, MicOptions, MicUsedEngine, RobustMode};
use ige_core::{rotate_polygon, solve_axis_aligned, AxisAlignedOptions};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

// ─── Oriented solver ──────────────────────────────────────────────────────

#[pyclass]
#[derive(Clone)]
pub struct PyOrientedLirResult {
    #[pyo3(get)]
    pub center_x: f64,
    #[pyo3(get)]
    pub center_y: f64,
    #[pyo3(get)]
    pub width: f64,
    #[pyo3(get)]
    pub height: f64,
    #[pyo3(get)]
    pub angle_deg: f64,
    #[pyo3(get)]
    pub area: f64,
    #[pyo3(get)]
    pub aspect_ratio: f64,
    #[pyo3(get)]
    pub best_effort: bool,
    #[pyo3(get)]
    pub polygon_wkt: String,
}

#[pyfunction(signature = (
    exterior,
    holes=None,
    rotation_degrees=None,
    max_aspect_ratio=None,
    min_aspect_ratio=None,
    grid_coarse=None,
    grid_fine=None,
    top_k=None,
    always_return=true,
    use_parallel_field=false,
    use_simulated_annealing=false,
    use_bootstrap_seeds=false,
    use_pca_axes=false,
    use_edge_anchored=false,
))]
pub fn solve_oriented_lir_py(
    exterior: Vec<(f64, f64)>,
    holes: Option<Vec<Vec<(f64, f64)>>>,
    rotation_degrees: Option<f64>,
    max_aspect_ratio: Option<f64>,
    min_aspect_ratio: Option<f64>,
    grid_coarse: Option<usize>,
    grid_fine: Option<usize>,
    top_k: Option<usize>,
    always_return: bool,
    use_parallel_field: bool,
    use_simulated_annealing: bool,
    use_bootstrap_seeds: bool,
    use_pca_axes: bool,
    use_edge_anchored: bool,
) -> PyResult<PyOrientedLirResult> {
    if exterior.len() < 3 {
        return Err(PyValueError::new_err("polygon exterior must contain at least 3 points"));
    }

    let coords: Vec<Coord<f64>> = exterior
        .into_iter()
        .map(|(x, y)| Coord { x, y })
        .collect();
    let exterior_ls = LineString::from(coords);

    let interiors: Vec<LineString<f64>> = holes
        .unwrap_or_default()
        .into_iter()
        .map(|ring| {
            let ring_coords: Vec<Coord<f64>> = ring
                .into_iter()
                .map(|(x, y)| Coord { x, y })
                .collect();
            LineString::from(ring_coords)
        })
        .collect();

    let polygon = Polygon::new(exterior_ls, interiors);
    let rotation = rotation_degrees.unwrap_or(0.0);
    let working_polygon = if rotation.abs() > 1e-12 {
        rotate_polygon(&polygon, rotation)
    } else {
        polygon.clone()
    };

    let mut opts = LirOrientedOptions::default();
    if let Some(ratio) = max_aspect_ratio {
        opts.max_ratio = ratio;
    }
    if let Some(ratio) = min_aspect_ratio {
        opts.min_ratio = ratio;
    }
    if let Some(v) = grid_coarse {
        opts.grid_coarse = v;
    }
    if let Some(v) = grid_fine {
        opts.grid_fine = v;
    }
    if let Some(v) = top_k {
        opts.top_k = v;
    }
    opts.always_return = always_return;
    opts.use_parallel_field = use_parallel_field;
    opts.use_simulated_annealing = use_simulated_annealing;
    opts.use_bootstrap_seeds = use_bootstrap_seeds;
    opts.use_pca_axes = use_pca_axes;
    opts.use_edge_anchored = use_edge_anchored;

    let result = solve_lir_oriented(&working_polygon, &opts)
        .map_err(|e| PyValueError::new_err(format!("solve failed: {e}")))?;
    let mut rect_poly = result
        .rect_polygon
        .ok_or_else(|| PyValueError::new_err("solve failed: empty result polygon"))?;
    if rotation.abs() > 1e-12 {
        rect_poly = rotate_polygon(&rect_poly, -rotation);
    }

    let ext = rect_poly.exterior();
    let coords: Vec<(f64, f64)> = ext.0.iter().map(|c| (c.x, c.y)).collect();

    let (cx, cy) = if coords.len() >= 4 {
        let c0 = &coords[0];
        let c2 = &coords[2];
        ((c0.0 + c2.0) / 2.0, (c0.1 + c2.1) / 2.0)
    } else {
        (0.0, 0.0)
    };

    let bb = rect_poly
        .bounding_rect()
        .ok_or_else(|| PyValueError::new_err("solve failed: invalid result bounds"))?;
    let width = bb.max().x - bb.min().x;
    let height = bb.max().y - bb.min().y;
    let aspect = if height > 0.0 { width / height } else { 0.0 };

    let polygon_wkt = format!(
        "POLYGON(({:.6} {:.6}, {:.6} {:.6}, {:.6} {:.6}, {:.6} {:.6}, {:.6} {:.6}))",
        coords[0].0, coords[0].1,
        coords[1].0, coords[1].1,
        coords[2].0, coords[2].1,
        coords[3].0, coords[3].1,
        coords[0].0, coords[0].1
    );

    Ok(PyOrientedLirResult {
        center_x: cx,
        center_y: cy,
        width,
        height,
        angle_deg: result.angle_deg,
        area: result.area,
        aspect_ratio: aspect,
        best_effort: result.best_effort,
        polygon_wkt,
    })
}

#[pyfunction]
fn oriented_lir_demo() -> PyResult<String> {
    let result = solve_oriented_lir_py(
        vec![(0.0, 0.0), (8.0, 1.0), (7.0, 7.0), (2.0, 8.0), (-1.0, 4.0)],
        None,
        Some(0.0),
        None,
        None,
        None,
        None,
        None,
        true,
        false,
        false,
        false,
        false,
        false,
    )?;
    Ok(format!(
        "area={:.3}, center=({:.3},{:.3}), size={:.3}x{:.3}, angle={:.1}",
        result.area, result.center_x, result.center_y, result.width, result.height, result.angle_deg
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

#[pyfunction(signature = (exterior, holes=None, max_aspect_ratio=None, min_aspect_ratio=None, max_grid=None))]
pub fn solve_axis_aligned_py(
    exterior: Vec<(f64, f64)>,
    holes: Option<Vec<Vec<(f64, f64)>>>,
    max_aspect_ratio: Option<f64>,
    min_aspect_ratio: Option<f64>,
    max_grid: Option<usize>,
) -> PyResult<PyAxisAlignedResult> {
    if exterior.len() < 3 {
        return Err(PyValueError::new_err("polygon exterior must contain at least 3 points"));
    }

    let coords: Vec<Coord<f64>> = exterior
        .into_iter()
        .map(|(x, y)| Coord { x, y })
        .collect();
    let exterior_ls = LineString::from(coords);

    let interiors: Vec<LineString<f64>> = holes
        .unwrap_or_default()
        .into_iter()
        .map(|ring| {
            let ring_coords: Vec<Coord<f64>> = ring
                .into_iter()
                .map(|(x, y)| Coord { x, y })
                .collect();
            LineString::from(ring_coords)
        })
        .collect();

    let polygon = Polygon::new(exterior_ls, interiors);

    let mut opts = AxisAlignedOptions::default();
    if let Some(ratio) = max_aspect_ratio {
        opts.max_ratio = ratio;
    }
    if let Some(ratio) = min_aspect_ratio {
        opts.min_ratio = ratio;
    }
    if let Some(grid) = max_grid {
        opts.max_grid = grid;
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
        None,
        None,
        None,
    )?;
    Ok(format!(
        "area={:.3}, bounds=({:.3}, {:.3}, {:.3}, {:.3})",
        result.area, result.x_min, result.y_min, result.x_max, result.y_max
    ))
}

// ─── LIR Approximate Oriented solver ───────────────────────────────────────

#[pyfunction(signature = (
    exterior,
    holes=None,
    max_aspect_ratio=None,
    min_aspect_ratio=None,
    grid_coarse=None,
    grid_fine=None,
    top_k=None,
    always_return=true,
    use_parallel_field=false,
    use_simulated_annealing=false,
    use_bootstrap_seeds=false,
    use_pca_axes=false,
    use_edge_anchored=false,
    polish_halwidth_deg=None,
    polish_xatol_deg=None,
    cert_eps=None,
))]
pub fn solve_lir_oriented_py(
    exterior: Vec<(f64, f64)>,
    holes: Option<Vec<Vec<(f64, f64)>>>,
    max_aspect_ratio: Option<f64>,
    min_aspect_ratio: Option<f64>,
    grid_coarse: Option<usize>,
    grid_fine: Option<usize>,
    top_k: Option<usize>,
    always_return: bool,
    use_parallel_field: bool,
    use_simulated_annealing: bool,
    use_bootstrap_seeds: bool,
    use_pca_axes: bool,
    use_edge_anchored: bool,
    polish_halwidth_deg: Option<f64>,
    polish_xatol_deg: Option<f64>,
    cert_eps: Option<f64>,
) -> PyResult<PyOrientedLirResult> {
    if exterior.len() < 3 {
        return Err(PyValueError::new_err("polygon exterior must contain at least 3 points"));
    }

    let coords: Vec<Coord<f64>> = exterior
        .into_iter()
        .map(|(x, y)| Coord { x, y })
        .collect();
    let exterior_ls = LineString::from(coords);

    let interiors: Vec<LineString<f64>> = holes
        .unwrap_or_default()
        .into_iter()
        .map(|ring| {
            let ring_coords: Vec<Coord<f64>> = ring
                .into_iter()
                .map(|(x, y)| Coord { x, y })
                .collect();
            LineString::from(ring_coords)
        })
        .collect();

    let polygon = Polygon::new(exterior_ls, interiors);

    let mut opts = LirOrientedOptions::default();
    if let Some(ratio) = max_aspect_ratio {
        opts.max_ratio = ratio;
    }
    if let Some(ratio) = min_aspect_ratio {
        opts.min_ratio = ratio;
    }
    if let Some(v) = grid_coarse {
        opts.grid_coarse = v;
    }
    if let Some(v) = grid_fine {
        opts.grid_fine = v;
    }
    if let Some(v) = top_k {
        opts.top_k = v;
    }
    opts.always_return = always_return;
    opts.use_parallel_field = use_parallel_field;
    opts.use_simulated_annealing = use_simulated_annealing;
    opts.use_bootstrap_seeds = use_bootstrap_seeds;
    opts.use_pca_axes = use_pca_axes;
    opts.use_edge_anchored = use_edge_anchored;
    if let Some(v) = polish_halwidth_deg {
        opts.polish_halwidth_deg = v;
    }
    if let Some(v) = polish_xatol_deg {
        opts.polish_xatol_deg = v;
    }
    if let Some(v) = cert_eps {
        opts.cert_eps = v;
    }

    let result = solve_lir_oriented(&polygon, &opts)
        .map_err(|e| PyValueError::new_err(format!("solve failed: {e}")))?;

    let rect_poly = result
        .rect_polygon
        .ok_or_else(|| PyValueError::new_err("solve failed: empty result polygon"))?;

    let ext = rect_poly.exterior();
    let coords: Vec<(f64, f64)> = ext.0.iter().map(|c| (c.x, c.y)).collect();

    let (cx, cy) = if coords.len() >= 4 {
        let c0 = &coords[0];
        let c2 = &coords[2];
        ((c0.0 + c2.0) / 2.0, (c0.1 + c2.1) / 2.0)
    } else {
        (0.0, 0.0)
    };

    let bb = rect_poly
        .bounding_rect()
        .ok_or_else(|| PyValueError::new_err("solve failed: invalid result bounds"))?;
    let width = bb.max().x - bb.min().x;
    let height = bb.max().y - bb.min().y;
    let aspect = if height > 0.0 { width / height } else { 0.0 };

    let polygon_wkt = format!(
        "POLYGON(({:.6} {:.6}, {:.6} {:.6}, {:.6} {:.6}, {:.6} {:.6}, {:.6} {:.6}))",
        coords[0].0, coords[0].1,
        coords[1].0, coords[1].1,
        coords[2].0, coords[2].1,
        coords[3].0, coords[3].1,
        coords[0].0, coords[0].1
    );

    Ok(PyOrientedLirResult {
        center_x: cx,
        center_y: cy,
        width,
        height,
        angle_deg: result.angle_deg,
        area: result.area,
        aspect_ratio: aspect,
        best_effort: result.best_effort,
        polygon_wkt,
    })
}

#[pyfunction(signature = (
    exterior,
    holes=None,
    max_aspect_ratio=None,
    min_aspect_ratio=None,
    grid_coarse=None,
    grid_fine=None,
    top_k=None,
    always_return=true,
    use_parallel_field=false,
    use_simulated_annealing=false,
    use_bootstrap_seeds=false,
    use_pca_axes=false,
    use_edge_anchored=false,
    polish_halwidth_deg=None,
    polish_xatol_deg=None,
    cert_eps=None,
))]
pub fn solve_bcrs_py(
    exterior: Vec<(f64, f64)>,
    holes: Option<Vec<Vec<(f64, f64)>>>,
    max_aspect_ratio: Option<f64>,
    min_aspect_ratio: Option<f64>,
    grid_coarse: Option<usize>,
    grid_fine: Option<usize>,
    top_k: Option<usize>,
    always_return: bool,
    use_parallel_field: bool,
    use_simulated_annealing: bool,
    use_bootstrap_seeds: bool,
    use_pca_axes: bool,
    use_edge_anchored: bool,
    polish_halwidth_deg: Option<f64>,
    polish_xatol_deg: Option<f64>,
    cert_eps: Option<f64>,
) -> PyResult<PyOrientedLirResult> {
    solve_lir_oriented_py(
        exterior,
        holes,
        max_aspect_ratio,
        min_aspect_ratio,
        grid_coarse,
        grid_fine,
        top_k,
        always_return,
        use_parallel_field,
        use_simulated_annealing,
        use_bootstrap_seeds,
        use_pca_axes,
        use_edge_anchored,
        polish_halwidth_deg,
        polish_xatol_deg,
        cert_eps,
    )
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
        MicUsedEngine::Grid => "grid",
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

// ─── LER solver (placeholder) ─────────────────────────────────────────────

#[pyfunction]
pub fn solve_ler_axis_aligned_py(
    _exterior: Vec<(f64, f64)>,
    _obstacles: Option<Vec<Vec<(f64, f64)>>>,
) -> PyResult<PyAxisAlignedResult> {
    Err(PyValueError::new_err("LER solver not yet implemented"))
}

#[pyfunction]
pub fn solve_ler_oriented_py(
    _exterior: Vec<(f64, f64)>,
    _obstacles: Option<Vec<Vec<(f64, f64)>>>,
) -> PyResult<PyOrientedLirResult> {
    Err(PyValueError::new_err("LER oriented solver not yet implemented"))
}

// ─── Nesting solver (placeholder) ─────────────────────────────────────────

#[pyclass]
#[derive(Clone)]
pub struct PyNestingResult {
    #[pyo3(get)]
    pub area: f64,
    #[pyo3(get)]
    pub fill_ratio: f64,
}

#[pyfunction]
pub fn solve_nesting_py(
    _exterior: Vec<(f64, f64)>,
) -> PyResult<PyNestingResult> {
    Err(PyValueError::new_err("Nesting solver not yet implemented"))
}

// ─── LER + LIR combined solver (placeholder) ─────────────────────────────

#[pyclass]
#[derive(Clone)]
pub struct PyLerLirResult {
    #[pyo3(get)]
    pub lir_area: f64,
    #[pyo3(get)]
    pub ler_area: f64,
}

#[pyfunction]
pub fn solve_ler_lir_py(
    _exterior: Vec<(f64, f64)>,
    _obstacles: Option<Vec<Vec<(f64, f64)>>>,
) -> PyResult<PyLerLirResult> {
    Err(PyValueError::new_err("LER+LIR solver not yet implemented"))
}

// ─── OBB solver (placeholder) ─────────────────────────────────────────────

#[pyclass]
#[derive(Clone)]
pub struct PyObbResult {
    #[pyo3(get)]
    pub area: f64,
    #[pyo3(get)]
    pub angle_deg: f64,
    #[pyo3(get)]
    pub width: f64,
    #[pyo3(get)]
    pub height: f64,
    #[pyo3(get)]
    pub aspect_ratio: f64,
    #[pyo3(get)]
    pub fill_ratio: f64,
}

#[pyfunction]
pub fn solve_obb_py(
    _exterior: Vec<(f64, f64)>,
) -> PyResult<PyObbResult> {
    Err(PyValueError::new_err("OBB solver not yet implemented"))
}

// ─── Module registration ──────────────────────────────────────────────────

#[pymodule]
fn ige(_py: Python, m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<PyOrientedLirResult>()?;
    m.add_function(wrap_pyfunction!(solve_oriented_lir_py, m)?)?;
    m.add_function(wrap_pyfunction!(oriented_lir_demo, m)?)?;

    m.add_class::<PyAxisAlignedResult>()?;
    m.add_function(wrap_pyfunction!(solve_axis_aligned_py, m)?)?;
    m.add_function(wrap_pyfunction!(axis_aligned_demo, m)?)?;

    m.add_class::<PyOrientedLirResult>()?;
    m.add_function(wrap_pyfunction!(solve_lir_oriented_py, m)?)?;
    m.add_function(wrap_pyfunction!(solve_bcrs_py, m)?)?;

    m.add_class::<PyMicResult>()?;
    m.add_function(wrap_pyfunction!(solve_mic_py, m)?)?;

    // LER solvers
    m.add_function(wrap_pyfunction!(solve_ler_axis_aligned_py, m)?)?;
    m.add_function(wrap_pyfunction!(solve_ler_oriented_py, m)?)?;

    // Nesting solver
    m.add_class::<PyNestingResult>()?;
    m.add_function(wrap_pyfunction!(solve_nesting_py, m)?)?;

    // LER + LIR solver
    m.add_class::<PyLerLirResult>()?;
    m.add_function(wrap_pyfunction!(solve_ler_lir_py, m)?)?;

    // OBB solver
    m.add_class::<PyObbResult>()?;
    m.add_function(wrap_pyfunction!(solve_obb_py, m)?)?;

    Ok(())
}
