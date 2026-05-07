//! IGE Visual Preview Tool — LIR Oriented + baseline comparison.
//!
//! Output: `target/ige_output/index.html`
//!
//! Usage:
//!   cargo run --package ige-core --example visualize
//!   cargo run --package ige-core --example visualize -- --baseline
//!   cargo run --package ige-core --example visualize -- --baseline --axis-solver vertex
//!   cargo run --package ige-core --example visualize -- --baseline --axis-solver exact
//!   cargo run --package ige-core --example visualize -- --baseline --axis-solver grid --mask-backend cpu
//!   cargo run --package ige-core --example visualize --features gpu -- --baseline --axis-solver grid --mask-backend gpu-grid
//!   cargo run --package ige-core --example visualize -- --parallel   # oriented with extra local angle polish
//!   cargo run --package ige-core --example visualize -- --sa         # oriented with simulated annealing rescue
//!   cargo run --package ige-core --example visualize -- --limit 50
//!   cargo run --package ige-core --example visualize --features geos -- --mic-compare --real-only --file crates/ige-core/tests/real_world_data/realworld.geojson

use geo::Area;
use geo_types::{Coord, LineString, Polygon};
use ige_core::shared::Rectangle;
use ige_core::solvers::lir::axis_aligned::{solve_axis_exact, solve_vertex_grid};
use ige_core::solvers::lir::oriented::{solve_lir_oriented, LirOrientedOptions};
use ige_core::solvers::mic::{maximum_inscribed_circle, MicEngine, MicOptions, MicResult, RobustMode};
use ige_core::{solve_axis_rect_grid_with_backend, AxisAlignedOptions, MaskBackend};
use rayon::prelude::*;
use serde_json::Value;
use std::fs;

#[cfg(feature = "gpu")]
fn parse_mask_backend(args: &[String]) -> MaskBackend {
    let value = args
        .iter()
        .position(|a| a == "--mask-backend")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("auto");
    match value {
        "cpu" => MaskBackend::Cpu,
        "gpu-sdf" => MaskBackend::GpuSdf,
        "gpu-grid" => MaskBackend::GpuGridBatch,
        "auto" => MaskBackend::Auto,
        _ => {
            eprintln!("Unknown --mask-backend '{value}', using auto");
            MaskBackend::Auto
        }
    }
}

#[cfg(not(feature = "gpu"))]
fn parse_mask_backend(args: &[String]) -> MaskBackend {
    if let Some(value) = args
        .iter()
        .position(|a| a == "--mask-backend")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
    {
        if value != "cpu" && value != "auto" {
            eprintln!("--mask-backend={value} requires --features gpu; using cpu");
        }
    }
    MaskBackend::Cpu
}

#[cfg(feature = "gpu")]
fn mask_backend_name(backend: MaskBackend) -> &'static str {
    match backend {
        MaskBackend::Cpu => "cpu",
        MaskBackend::GpuSdf => "gpu-sdf",
        MaskBackend::GpuGridBatch => "gpu-grid",
        MaskBackend::Auto => "auto",
    }
}

#[cfg(not(feature = "gpu"))]
fn mask_backend_name(_backend: MaskBackend) -> &'static str {
    "cpu"
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AxisCliSolver {
    Vertex,
    Exact,
    Grid,
}

fn parse_axis_solver(args: &[String]) -> AxisCliSolver {
    // Backward compatibility: old flag implied grid baseline mode.
    if args.contains(&"--baseline-grid".to_string()) {
        return AxisCliSolver::Grid;
    }

    let value = args
        .iter()
        .position(|a| a == "--axis-solver")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("vertex");

    match value {
        "vertex" | "vertex_grid" => AxisCliSolver::Vertex,
        "exact" => AxisCliSolver::Exact,
        "grid" => AxisCliSolver::Grid,
        _ => {
            eprintln!("Unknown --axis-solver '{value}', using vertex");
            AxisCliSolver::Vertex
        }
    }
}

fn axis_solver_name(solver: AxisCliSolver) -> &'static str {
    match solver {
        AxisCliSolver::Vertex => "vertex",
        AxisCliSolver::Exact => "exact",
        AxisCliSolver::Grid => "grid",
    }
}

fn rect_to_polygon(rect: Rectangle) -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            Coord { x: rect.x_min, y: rect.y_min },
            Coord { x: rect.x_max, y: rect.y_min },
            Coord { x: rect.x_max, y: rect.y_max },
            Coord { x: rect.x_min, y: rect.y_max },
            Coord { x: rect.x_min, y: rect.y_min },
        ]),
        vec![],
    )
}

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

fn parse_polygon(geom: &Value) -> Option<Polygon<f64>> {
    let arr = geom.get("coordinates")?.as_array()?;
    parse_polygon_coords(arr)
}

fn parse_polygon_coords(arr: &[Value]) -> Option<Polygon<f64>> {
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

fn parse_feature_polygons(geom: &Value) -> Vec<Polygon<f64>> {
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

fn load_polygons_from(path: Option<&str>) -> Vec<(String, Polygon<f64>)> {
    let content = match path {
        Some(p) => fs::read_to_string(p).expect("Failed to read file"),
        None => include_str!("../tests/real_world_data/realworld_290.geojson").to_string(),
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

fn make_l_shape(cx: f64, cy: f64, size: f64) -> Polygon<f64> {
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

fn make_u_shape(cx: f64, cy: f64, size: f64) -> Polygon<f64> {
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

fn make_zigzag(cx: f64, cy: f64, size: f64) -> Polygon<f64> {
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

fn get_polygon_bounds(poly: &Polygon<f64>) -> (f64, f64, f64, f64) {
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;
    for coord in poly.exterior().0.iter() {
        min_x = min_x.min(coord.x);
        min_y = min_y.min(coord.y);
        max_x = max_x.max(coord.x);
        max_y = max_y.max(coord.y);
    }
    (min_x, min_y, max_x, max_y)
}

fn gen_svg_card(
    id: &str,
    poly: &Polygon<f64>,
    rect_polygon: Option<&Polygon<f64>>,
    rect_area: f64,
    angle_deg: f64,
    best_effort: bool,
    time_ms: f64,
) -> String {
    let (min_x, min_y, max_x, max_y) = get_polygon_bounds(poly);
    let poly_area = poly.unsigned_area();

    let svg_size = 200.0;
    let pad = 10.0;
    let draw_size = svg_size - 2.0 * pad;
    let span_x = max_x - min_x;
    let span_y = max_y - min_y;
    let scale = if span_x > 0.0 && span_y > 0.0 {
        (draw_size / span_x).min(draw_size / span_y)
    } else {
        draw_size
    };
    let ox = pad + (draw_size - span_x * scale) * 0.5;
    let oy = pad + (draw_size - span_y * scale) * 0.5;

    let to_svg = |x: f64, y: f64| (ox + (x - min_x) * scale, oy + (y - min_y) * scale);

    let ext_pts: String = poly.exterior().0.iter()
        .map(|c| { let (sx, sy) = to_svg(c.x, c.y); format!("{:.1},{:.1}", sx, sy) })
        .collect::<Vec<_>>()
        .join(" ");

    let holes_svg: String = poly.interiors().iter()
        .map(|hole| {
            let pts: String = hole.0.iter()
                .map(|c| { let (sx, sy) = to_svg(c.x, c.y); format!("{:.1},{:.1}", sx, sy) })
                .collect::<Vec<_>>()
                .join(" ");
            format!(r#"<polygon class="hole" points="{}"/>"#, pts)
        })
        .collect();

    let (rect_svg, extra) = match rect_polygon {
        Some(rp) => {
            let rpts: String = rp.exterior().0.iter()
                .map(|c| { let (sx, sy) = to_svg(c.x, c.y); format!("{:.1},{:.1}", sx, sy) })
                .collect::<Vec<_>>()
                .join(" ");
            let cls = if best_effort { "rect best-effort" } else { "rect" };
            let ang_s = if angle_deg.abs() > 0.01 { format!("Angle: {:.1}°<br/>", angle_deg) } else { String::new() };
            let be_s = if best_effort { "best-effort<br/>" } else { "" };
            (format!(r#"<polygon class="{}" points="{}"/>"#, cls, rpts), format!("{}{}", ang_s, be_s))
        }
        None => (String::new(), String::new()),
    };

    let ratio = if poly_area > 0.0 { rect_area / poly_area * 100.0 } else { 0.0 };

    format!(
        r#"<div class="card">
            <svg viewBox="0 0 {s:.0} {s:.0}">
                <polygon class="polygon" points="{p}"/>
                {h}
                {r}
            </svg>
            <div class="info">
                <strong>{id}</strong><br/>
                Polygon: {pa:.1}<br/>
                Rectangle: {ra:.1}<br/>
                Fill: {fr:.1}%<br/>
                {ex}Time: {t:.2}ms
            </div>
        </div>"#,
        s = svg_size,
        p = ext_pts,
        h = holes_svg,
        r = rect_svg,
        id = id,
        pa = poly_area,
        ra = rect_area,
        fr = ratio,
        ex = extra,
        t = time_ms,
    )
}

fn gen_mic_card(
    id: &str,
    poly: &Polygon<f64>,
    exact: Option<&MicResult>,
    geos: Option<&MicResult>,
    exact_err: Option<&str>,
    geos_err: Option<&str>,
    exact_ms: f64,
    geos_ms: f64,
) -> String {
    let (min_x, min_y, max_x, max_y) = get_polygon_bounds(poly);
    let poly_area = poly.unsigned_area();

    let svg_size = 220.0;
    let pad = 10.0;
    let draw_size = svg_size - 2.0 * pad;
    let span_x = max_x - min_x;
    let span_y = max_y - min_y;
    let scale = if span_x > 0.0 && span_y > 0.0 {
        (draw_size / span_x).min(draw_size / span_y)
    } else {
        draw_size
    };
    let ox = pad + (draw_size - span_x * scale) * 0.5;
    let oy = pad + (draw_size - span_y * scale) * 0.5;

    let to_svg = |x: f64, y: f64| (ox + (x - min_x) * scale, oy + (y - min_y) * scale);

    let ext_pts: String = poly.exterior().0.iter()
        .map(|c| { let (sx, sy) = to_svg(c.x, c.y); format!("{sx:.1},{sy:.1}") })
        .collect::<Vec<_>>()
        .join(" ");

    let holes_svg: String = poly.interiors().iter()
        .map(|hole| {
            let pts: String = hole.0.iter()
                .map(|c| { let (sx, sy) = to_svg(c.x, c.y); format!("{sx:.1},{sy:.1}") })
                .collect::<Vec<_>>()
                .join(" ");
            format!(r#"<polygon class="hole" points="{pts}"/>"#)
        })
        .collect();

    let exact_svg = exact.map(|mic| {
        let (cx, cy) = to_svg(mic.center.x(), mic.center.y());
        let r = (mic.radius * scale).max(0.5);
        format!(
            r#"<circle class="mic-exact" cx="{cx:.2}" cy="{cy:.2}" r="{r:.2}"/><circle class="pt-exact" cx="{cx:.2}" cy="{cy:.2}" r="2.2"/>"#
        )
    }).unwrap_or_default();

    let geos_svg = geos.map(|mic| {
        let (cx, cy) = to_svg(mic.center.x(), mic.center.y());
        let r = (mic.radius * scale).max(0.5);
        format!(
            r#"<circle class="mic-geos" cx="{cx:.2}" cy="{cy:.2}" r="{r:.2}"/><circle class="pt-geos" cx="{cx:.2}" cy="{cy:.2}" r="2.2"/>"#
        )
    }).unwrap_or_default();

    let exact_info = match exact {
        Some(m) => format!("Exact radius: {:.4}<br/>", m.radius),
        None => format!("Exact: error ({})<br/>", exact_err.unwrap_or("unknown")),
    };
    let geos_info = match geos {
        Some(m) => format!("GEOS radius: {:.4}<br/>", m.radius),
        None => format!("GEOS: error ({})<br/>", geos_err.unwrap_or("unknown")),
    };

    let speed_line = if exact_ms > 0.0 && geos_ms > 0.0 {
        let ratio = exact_ms / geos_ms;
        let label = if ratio <= 1.0 {
            format!("{:.1}x faster", 1.0 / ratio.max(1e-12))
        } else {
            format!("{:.1}x slower", ratio)
        };
        format!(
            r#"<span class="speed-exact">Exact: {:.2}ms</span> &nbsp; <span class="speed-geos">GEOS: {:.2}ms</span> &nbsp; <span class="speed-label">{}</span><br/>"#,
            exact_ms, geos_ms, label
        )
    } else {
        format!("Exact: {:.2}ms &nbsp; GEOS: {:.2}ms<br/>", exact_ms, geos_ms)
    };

    let rel_info = match (exact, geos) {
        (Some(e), Some(g)) if g.radius > 0.0 => {
            let rel_err = (e.radius - g.radius).abs() / g.radius;
            format!("Rel err vs GEOS: {:.3}%<br/>", rel_err * 100.0)
        }
        _ => String::new(),
    };

    format!(
        r#"<div class="card">
            <svg viewBox="0 0 {s:.0} {s:.0}">
                <polygon class="polygon" points="{p}"/>
                {h}
                {ex}
                {ge}
            </svg>
            <div class="info">
                <strong>{id}</strong><br/>
                Polygon area: {pa:.1}<br/>
                {exact_info}{geos_info}{rel_info}{speed}
            </div>
        </div>"#,
        s = svg_size,
        p = ext_pts,
        h = holes_svg,
        ex = exact_svg,
        ge = geos_svg,
        id = id,
        pa = poly_area,
        exact_info = exact_info,
        geos_info = geos_info,
        rel_info = rel_info,
        speed = speed_line,
    )
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let use_mic_compare = args.contains(&"--mic-compare".to_string());
    let real_only = args.contains(&"--real-only".to_string());
    let use_parallel = args.contains(&"--parallel".to_string());
    let use_sa = args.contains(&"--sa".to_string());
    let use_bootstrap_seeds = args.contains(&"--bootstrap-seeds".to_string());
    let use_pca_axes = args.contains(&"--pca-axes".to_string());
    let use_multi_center = args.contains(&"--multi-center".to_string());
    let use_early_stopping = args.contains(&"--early-stop".to_string());
    let use_edge_anchored = args.contains(&"--edge-anchored".to_string());
    let use_approx_oriented = !args.contains(&"--baseline".to_string());
    let use_json = args.contains(&"--json".to_string());
    let limit = args.iter()
        .position(|a| a == "--limit")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse::<usize>().ok());
    let file_path = args.iter()
        .position(|a| a == "--file")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str());
    let axis_solver = parse_axis_solver(&args);
    let mask_backend = parse_mask_backend(&args);
    if !use_approx_oriented
        && !use_mic_compare
        && axis_solver != AxisCliSolver::Grid
        && args.iter().any(|a| a == "--mask-backend")
    {
        eprintln!("--mask-backend is ignored unless --axis-solver grid is selected");
    }
    if !use_approx_oriented && use_sa {
        eprintln!("--sa is ignored in baseline axis-aligned mode");
    }

    let algo_name = if use_mic_compare {
        "MIC exact vs GEOS".to_string()
    } else if use_sa && use_parallel {
        "LIR Approx Oriented + SA + local angle polish".to_string()
    } else if use_sa {
        "LIR Approx Oriented + SA rescue".to_string()
    } else if use_bootstrap_seeds && use_edge_anchored {
        "LIR Approx Oriented + bootstrap seeds + edge-anchored".to_string()
    } else if use_bootstrap_seeds && use_parallel {
        "LIR Approx Oriented + bootstrap seeds + local angle polish".to_string()
    } else if use_bootstrap_seeds {
        "LIR Approx Oriented + bootstrap seeds".to_string()
    } else if use_edge_anchored {
        "LIR Approx Oriented + edge-anchored".to_string()
    } else if use_parallel {
        "LIR Approx Oriented + local angle polish".to_string()
    } else if use_approx_oriented {
        "LIR Approx Oriented".to_string()
    } else {
        match axis_solver {
            AxisCliSolver::Grid => {
                format!(
                    "axis-aligned (solver={}, mask={})",
                    axis_solver_name(axis_solver),
                    mask_backend_name(mask_backend)
                )
            }
            _ => format!("axis-aligned (solver={})", axis_solver_name(axis_solver)),
        }
    };

    let real = load_polygons_from(file_path);
    eprintln!("Loaded {} polygons from geojson", real.len());

    let mut all_polygons: Vec<(String, Polygon<f64>)> = if real_only || use_mic_compare {
        real
    } else {
        let mut data = vec![
            ("Square 10x10".into(), Polygon::new(
                LineString::from(vec![
                    Coord { x: 0.0, y: 0.0 }, Coord { x: 10.0, y: 0.0 },
                    Coord { x: 10.0, y: 10.0 }, Coord { x: 0.0, y: 10.0 },
                    Coord { x: 0.0, y: 0.0 },
                ]), vec![],
            )),
            ("Rectangle 20x5".into(), Polygon::new(
                LineString::from(vec![
                    Coord { x: 0.0, y: 0.0 }, Coord { x: 20.0, y: 0.0 },
                    Coord { x: 20.0, y: 5.0 }, Coord { x: 0.0, y: 5.0 },
                    Coord { x: 0.0, y: 0.0 },
                ]), vec![],
            )),
            ("Triangle".into(), Polygon::new(
                LineString::from(vec![
                    Coord { x: 0.0, y: 0.0 }, Coord { x: 10.0, y: 0.0 },
                    Coord { x: 5.0, y: 10.0 }, Coord { x: 0.0, y: 0.0 },
                ]), vec![],
            )),
            ("L-Shape".into(), make_l_shape(5.0, 5.0, 5.0)),
            ("U-Shape".into(), make_u_shape(5.0, 5.0, 5.0)),
            ("Zigzag".into(), make_zigzag(5.0, 5.0, 5.0)),
        ];
        data.extend(real);
        data
    };
    if let Some(n) = limit {
        all_polygons.truncate(n);
    }
    eprintln!("Algorithm: {algo_name}");
    eprintln!("Total shapes: {}", all_polygons.len());

    let out_dir = std::env::current_dir().unwrap().join("target").join("ige_output");
    fs::create_dir_all(&out_dir).unwrap();

    if use_mic_compare {
        let exact_opts = MicOptions {
            engine: MicEngine::ExactOnly,
            robust_mode: RobustMode::Filtered,
        };
        let geos_opts = MicOptions {
            engine: MicEngine::FallbackOnly,
            robust_mode: RobustMode::Filtered,
        };

        let wall_start = std::time::Instant::now();
        let mut results: Vec<(usize, String, Option<f64>, Option<f64>, f64, f64)> = all_polygons
            .par_iter()
            .enumerate()
            .map(|(idx, (id, poly))| {
                let t0 = std::time::Instant::now();
                let exact = maximum_inscribed_circle(poly, &exact_opts);
                let exact_ms = t0.elapsed().as_secs_f64() * 1000.0;

                let t1 = std::time::Instant::now();
                let geos = maximum_inscribed_circle(poly, &geos_opts);
                let geos_ms = t1.elapsed().as_secs_f64() * 1000.0;

                let exact_err = exact.as_ref().err().map(|e| e.to_string());
                let geos_err = geos.as_ref().err().map(|e| e.to_string());
                let exact_radius = exact.as_ref().ok().map(|r| r.radius);
                let geos_radius = geos.as_ref().ok().map(|r| r.radius);
                let card = gen_mic_card(
                    id,
                    poly,
                    exact.as_ref().ok(),
                    geos.as_ref().ok(),
                    exact_err.as_deref(),
                    geos_err.as_deref(),
                    exact_ms,
                    geos_ms,
                );
                (idx, card, exact_radius, geos_radius, exact_ms, geos_ms)
            })
            .collect();
        let wall_total_ms = wall_start.elapsed().as_secs_f64() * 1000.0;
        results.sort_by_key(|(idx, ..)| *idx);

        let mut cards = String::new();
        let mut exact_ok = 0usize;
        let mut geos_ok = 0usize;
        let mut both_ok = 0usize;
        let mut rel_errs = Vec::new();
        let mut exact_ms_acc = 0.0;
        let mut geos_ms_acc = 0.0;
        // Track error direction: positive = exact larger, negative = GEOS larger
        // Store (id, exact_r, geos_r, abs_err_pct, direction)
        let mut per_polygon_errors: Vec<(String, f64, f64, f64, &str)> = Vec::new();
        for (idx, card, exact_r, geos_r, exact_ms, geos_ms) in &results {
            cards.push_str(card);
            exact_ms_acc += *exact_ms;
            geos_ms_acc += *geos_ms;
            if exact_r.is_some() { exact_ok += 1; }
            if geos_r.is_some() { geos_ok += 1; }
            if let (Some(e), Some(g)) = (*exact_r, *geos_r) {
                if g > 0.0 {
                    both_ok += 1;
                    let abs_pct = (e - g).abs() / g * 100.0;
                    rel_errs.push(abs_pct);
                    let dir = if e > g { "exact_larger" } else { "exact_smaller" };
                    per_polygon_errors.push((all_polygons[*idx].0.clone(), e, g, abs_pct, dir));
                }
            }
        }
        rel_errs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let rel_mean = if rel_errs.is_empty() {
            0.0
        } else {
            rel_errs.iter().sum::<f64>() / rel_errs.len() as f64
        };
        let rel_median = if rel_errs.is_empty() {
            0.0
        } else {
            rel_errs[rel_errs.len() / 2]
        };
        let n_results = results.len();
        let avg_exact_ms = if n_results > 0 { exact_ms_acc / n_results as f64 } else { 0.0 };
        let avg_geos_ms = if n_results > 0 { geos_ms_acc / n_results as f64 } else { 0.0 };

        // Collect top 10 errors (by absolute error)
        per_polygon_errors.sort_by(|a, b| b.3.partial_cmp(&a.3).unwrap_or(std::cmp::Ordering::Equal));
        let top_errors: Vec<serde_json::Value> = per_polygon_errors.iter().take(10).map(|(id, e, g, pct, dir)| {
            serde_json::json!({
                "id": id, "exact_radius": e, "geos_radius": g,
                "err_pct": pct, "direction": dir,
            })
        }).collect();

        if use_json {
            let json = serde_json::json!({
                "mode": "mic_compare",
                "total": all_polygons.len(),
                "exact_ok": exact_ok,
                "geos_ok": geos_ok,
                "both_ok": both_ok,
                "rel_err_pct": { "median": rel_median, "mean": rel_mean },
                "exact_larger_count": per_polygon_errors.iter().filter(|e| e.4 == "exact_larger").count(),
                "exact_smaller_count": per_polygon_errors.iter().filter(|e| e.4 == "exact_smaller").count(),
                "top_errors": top_errors,
                "avg_exact_ms": avg_exact_ms,
                "avg_geos_ms": avg_geos_ms,
                "speed_ratio": if avg_geos_ms > 0.0 { avg_exact_ms / avg_geos_ms } else { 0.0 },
                "wall_ms": wall_total_ms,
            });
            println!("{}", serde_json::to_string(&json).unwrap());
            return;
        }

        let speed_ratio: f64 = if avg_geos_ms > 0.0 { avg_exact_ms / avg_geos_ms } else { 0.0 };
        let speed_label = if speed_ratio <= 1.0 {
            format!("{:.1}x <span class=speed-label>faster</span>", 1.0 / speed_ratio.max(1e-12))
        } else {
            format!("{:.1}x <span class=speed-label>slower</span>", speed_ratio)
        };

        let path = out_dir.join("index.html");
        let html = format!(r#"<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>IGE Visual Preview — MIC compare</title>
<style>
body{{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Arial,sans-serif;margin:20px;background:#1a1a2e;color:#eee}}
h1{{color:#eee;margin-bottom:10px}}
.grid{{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:15px}}
.card{{background:#16213e;border-radius:8px;padding:10px;box-shadow:0 2px 8px rgba(0,0,0,.3)}}
svg{{width:100%;height:220px;background:#0f0f23;border-radius:4px}}
.polygon{{fill:#e9456033;stroke:#ff6b6b;stroke-width:1}}
.hole{{fill:none;stroke:#666;stroke-width:1;stroke-dasharray:3}}
.mic-exact{{fill:none;stroke:#60a5fa;stroke-width:2}}
.pt-exact{{fill:#60a5fa;stroke:none}}
.mic-geos{{fill:none;stroke:#22c55e;stroke-width:2;stroke-dasharray:4 2}}
.pt-geos{{fill:#22c55e;stroke:none}}
.info{{margin-top:8px;font-size:11px;color:#aaa;line-height:1.4}}
.speed-exact{{color:#60a5fa}}
.speed-geos{{color:#22c55e}}
.speed-label{{color:#fbbf24;font-weight:bold}}
.stats{{background:#16213e;padding:20px;border-radius:8px;margin-bottom:20px;box-shadow:0 2px 8px rgba(0,0,0,.3)}}
.stats p{{margin:5px 0;color:#ccc}}
.stats strong{{color:#fff}}
</style></head><body>
<h1>IGE — MIC exact vs GEOS</h1>
<p style="color:#aaa;">Blue = exact solver, Green dashed = GEOS fallback &nbsp;|&nbsp; <span class="speed-exact">Exact time</span> vs <span class="speed-geos">GEOS time</span></p>
<div class="stats">
<p><strong>Exact success:</strong> {exact_ok}/{n} &nbsp; <strong>GEOS success:</strong> {geos_ok}/{n} &nbsp; <strong>Both:</strong> {both_ok}</p>
<p><strong>Rel err vs GEOS:</strong> median {rel_median:.3}% &nbsp; mean {rel_mean:.3}%</p>
<p><strong><span class=speed-exact>Exact avg:</span></strong> {avg_exact_ms:.3}ms/shape &nbsp; <strong><span class=speed-geos>GEOS avg:</span></strong> {avg_geos_ms:.3}ms/shape &nbsp; <strong>Exact is {sit}</strong></p>
<p><strong>Total time:</strong> {wall_total_ms:.1}ms</p>
</div>
<div class="grid">{cards}</div>
</body></html>"#,
                           n = all_polygons.len(),
                           exact_ok = exact_ok,
                           geos_ok = geos_ok,
                           both_ok = both_ok,
                           rel_median = rel_median,
                           rel_mean = rel_mean,
                           wall_total_ms = wall_total_ms,
                           avg_exact_ms = avg_exact_ms,
                           avg_geos_ms = avg_geos_ms,
                           sit = speed_label,
                           cards = cards,
        );
        fs::write(&path, &html).unwrap();
        println!("Generated: {}  ({algo_name})", path.display());
        return;
    }

    // Wall-clock timer for the parallel section
    let wall_start = std::time::Instant::now();

    // Process all polygons in parallel with rayon
    let mut results: Vec<(usize, String, f64, f64, bool, f64, f64)> = all_polygons
        .par_iter()
        .enumerate()
        .map(|(idx, (id, poly))| {
            let start = std::time::Instant::now();
            let poly_area = poly.unsigned_area();

            let (rp, ra, ang, be) = if use_parallel || use_sa || use_bootstrap_seeds || use_pca_axes || use_multi_center || use_early_stopping || use_edge_anchored {
                let mut opts = LirOrientedOptions::default();
                opts.use_parallel_field = use_parallel;
                opts.use_simulated_annealing = use_sa;
                opts.use_bootstrap_seeds = use_bootstrap_seeds;
                opts.use_pca_axes = use_pca_axes;
                opts.use_multi_center = use_multi_center;
                opts.use_early_stopping = use_early_stopping;
                opts.use_edge_anchored = use_edge_anchored;
                match solve_lir_oriented(poly, &opts) {
                    Ok(r) => (r.rect_polygon, r.area, r.angle_deg, r.best_effort),
                    Err(_) => (None, 0.0, 0.0, false),
                }
            } else if use_approx_oriented {
                match solve_lir_oriented(poly, &LirOrientedOptions::default()) {
                    Ok(r) => (r.rect_polygon, r.area, r.angle_deg, r.best_effort),
                    Err(_) => (None, 0.0, 0.0, false),
                }
            } else {
                let opts = AxisAlignedOptions::default();
                match axis_solver {
                    AxisCliSolver::Vertex => {
                        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| solve_vertex_grid(poly, &opts))) {
                            Ok(Some(rect)) => {
                                let area = rect.area();
                                (Some(rect_to_polygon(rect)), area, 0.0, false)
                            }
                            _ => (None, 0.0, 0.0, false),
                        }
                    }
                    AxisCliSolver::Exact => {
                        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| solve_axis_exact(poly, &opts))) {
                            Ok(Some(rect)) => {
                                let area = rect.area();
                                (Some(rect_to_polygon(rect)), area, 0.0, false)
                            }
                            _ => (None, 0.0, 0.0, false),
                        }
                    }
                    AxisCliSolver::Grid => {
                        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            solve_axis_rect_grid_with_backend(poly, opts.max_grid, opts.max_ratio, opts.min_ratio, mask_backend)
                        })) {
                            Ok(Some((x0, y0, x1, y1, _))) => {
                                let rp = Polygon::new(LineString::from(vec![
                                    Coord { x: x0, y: y0 },
                                    Coord { x: x1, y: y0 },
                                    Coord { x: x1, y: y1 },
                                    Coord { x: x0, y: y1 },
                                    Coord { x: x0, y: y0 },
                                ]), vec![]);
                                let area = (x1 - x0) * (y1 - y0);
                                (Some(rp), area, 0.0, false)
                            }
                            _ => (None, 0.0, 0.0, false),
                        }
                    }
                }
            };
            let ms = start.elapsed().as_secs_f64() * 1000.0;
            let fill_pct = if poly_area > 0.0 { ra / poly_area * 100.0 } else { 0.0 };
            let card = gen_svg_card(id, poly, rp.as_ref(), ra, ang, be, ms);
            (idx, card, ra, ang, be, ms, fill_pct)
        })
        .collect();

    let wall_total_ms = wall_start.elapsed().as_secs_f64() * 1000.0;

    results.sort_by_key(|(idx, ..)| *idx);

    let mut cards = String::new();
    let mut success = 0usize;
    let mut failed = 0usize;
    let mut total_rect_area = 0.0;
    let mut total_poly_area = 0.0;
    let mut per_shape_pcts: Vec<f64> = Vec::new();

    for (idx, card, ra, _ang, _be, _ms, fill_pct) in &results {
        let (_, poly) = &all_polygons[*idx];
        total_poly_area += poly.unsigned_area();
        total_rect_area += ra;
        per_shape_pcts.push(*fill_pct);
        if *ra > 0.0 { success += 1; } else { failed += 1; }
        cards.push_str(card);
    }

    let fill = if total_poly_area > 0.0 { total_rect_area / total_poly_area * 100.0 } else { 0.0 };
    let avg = wall_total_ms / all_polygons.len() as f64;

    // Per-shape fill-rate distribution
    per_shape_pcts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = per_shape_pcts.len();
    let median_pct = if n > 0 { per_shape_pcts[n / 2] } else { 0.0 };
    let mean_pct = if n > 0 { per_shape_pcts.iter().sum::<f64>() / n as f64 } else { 0.0 };

    if use_json {
        let json = serde_json::json!({
            "success": success,
            "total": all_polygons.len(),
            "fill_rate": fill / 100.0,
            "avg_ms": avg,
            "wall_ms": wall_total_ms,
            "per_shape_pct": {
                "median": median_pct,
                "mean": mean_pct,
            },
        });
        println!("{}", serde_json::to_string(&json).unwrap());
    } else {
        let path = out_dir.join("index.html");
        let html = format!(r#"<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>IGE Visual Preview — {algo}</title>
<style>
body{{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Arial,sans-serif;margin:20px;background:#1a1a2e;color:#eee}}
h1{{color:#eee;margin-bottom:10px}}
.grid{{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:15px}}
.card{{background:#16213e;border-radius:8px;padding:10px;box-shadow:0 2px 8px rgba(0,0,0,.3)}}
svg{{width:100%;height:200px;background:#0f0f23;border-radius:4px}}
.polygon{{fill:#e94560;stroke:#ff6b6b;stroke-width:1}}
.rect{{fill:rgba(66,133,244,.4);stroke:#4285f4;stroke-width:2}}
.best-effort{{fill:rgba(255,193,7,.3);stroke:#ffc107}}
.hole{{fill:none;stroke:#666;stroke-width:1;stroke-dasharray:3}}
.info{{margin-top:8px;font-size:11px;color:#aaa;line-height:1.4}}
.stats{{background:#16213e;padding:20px;border-radius:8px;margin-bottom:20px;box-shadow:0 2px 8px rgba(0,0,0,.3)}}
.stats p{{margin:5px 0;color:#ccc}}
.stats strong{{color:#fff}}
</style></head><body>
<h1>IGE — Largest Inscribed Rectangle Preview</h1>
<p style="color:#aaa;">Algorithm: <strong style="color:#4285f4;">{algo}</strong> &mdash; full dataset ({n} shapes)</p>
<div class="stats">
<p><strong>Success:</strong> {ok}/{n} ({pct:.1}%) &nbsp; <strong>Failed:</strong> {fail}</p>
<p><strong>Polygon area:</strong> {pa:.0} &nbsp; <strong>Inscribed area:</strong> {ra:.0} ({fill:.1}%)</p>
<p><strong>Per-shape fill rate:</strong> median {median_pct:.1}% &nbsp; mean {mean_pct:.1}%</p>
<p><strong>Total time:</strong> {t:.1}ms &nbsp; <strong>Avg:</strong> {avg:.2}ms/shape</p>
</div>
<div class="grid">{cards}</div>
</body></html>"#,
                           algo = algo_name,
                           n = all_polygons.len(),
                           ok = success,
                           pct = success as f64 / all_polygons.len() as f64 * 100.0,
                           fail = failed,
                           pa = total_poly_area,
                           ra = total_rect_area,
                           fill = fill,
                           median_pct = median_pct,
                           mean_pct = mean_pct,
                           t = wall_total_ms,
                           avg = avg,
                           cards = cards,
        );
        fs::write(&path, &html).unwrap();
        println!("Generated: {}  ({algo_name})", path.display());
    }
}
