//! SVG and HTML card generation for polygon visualization.

use geo::algorithm::area::Area;
use geo_types::{Coord, LineString, Polygon};
use ige_core::solvers::mic::{MicResult, MicUsedEngine};

/// Convert a Rectangle to a Polygon for SVG rendering.
pub fn rect_to_polygon(rect: ige_core::shared::Rectangle) -> Polygon<f64> {
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

/// Compute bounding box of a polygon (min_x, min_y, max_x, max_y).
pub fn get_polygon_bounds(poly: &Polygon<f64>) -> (f64, f64, f64, f64) {
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

/// Polygon complexity metrics for diagnostics.
#[derive(Debug, Clone)]
pub struct ComplexityMetrics {
    pub exterior_vertices: usize,
    pub hole_count: usize,
    pub total_hole_vertices: usize,
    pub total_segments: usize,
    pub reflex_vertices: Option<usize>,  // None if not computed
}

/// Compute basic complexity metrics for a polygon.
pub fn compute_complexity(poly: &Polygon<f64>) -> ComplexityMetrics {
    let exterior = poly.exterior();
    let exterior_vertices = exterior.0.len().saturating_sub(1); // exclude closing vertex

    let hole_count = poly.interiors().len();
    let mut total_hole_vertices = 0;
    for hole in poly.interiors() {
        total_hole_vertices += hole.0.len().saturating_sub(1);
    }

    let total_segments = exterior_vertices + total_hole_vertices;

    ComplexityMetrics {
        exterior_vertices,
        hole_count,
        total_hole_vertices,
        total_segments,
        reflex_vertices: None,
    }
}

/// Format complexity metrics as a compact string for display.
pub fn format_complexity(metrics: &ComplexityMetrics) -> String {
    let mut parts = vec![
        format!("{} verts", metrics.exterior_vertices),
        format!("{} segs", metrics.total_segments),
    ];
    if metrics.hole_count > 0 {
        parts.push(format!("{} holes", metrics.hole_count));
    }
    if let Some(reflex) = metrics.reflex_vertices {
        if reflex > 0 {
            parts.push(format!("{} reflex", reflex));
        }
    }
    parts.join(" · ")
}

/// Generate an SVG visualization card for a polygon with its inscribed rectangle.
pub fn gen_svg_card(
    id: &str,
    poly: &Polygon<f64>,
    rect_polygon: Option<&Polygon<f64>>,
    rect_area: f64,
    angle_deg: f64,
    best_effort: bool,
    time_ms: f64,
    use_ler: bool,
) -> String {
    let (mut min_x, mut min_y, mut max_x, mut max_y) = get_polygon_bounds(poly);
    
    if let Some(rp) = rect_polygon {
        for c in rp.exterior().coords() {
            min_x = min_x.min(c.x);
            min_y = min_y.min(c.y);
            max_x = max_x.max(c.x);
            max_y = max_y.max(c.y);
        }
    }
    
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

    // Draw obstacle points for LER mode
    let obstacle_points: String = if use_ler {
        let mut pts = vec![];
        for c in poly.exterior().coords() {
            let (sx, sy) = to_svg(c.x, c.y);
            pts.push(format!(r#"<circle class="obstacle" cx="{:.1}" cy="{:.1}" r="2.5"/>"#, sx, sy));
        }
        for hole in poly.interiors() {
            for c in hole.coords() {
                let (sx, sy) = to_svg(c.x, c.y);
                pts.push(format!(r#"<circle class="obstacle" cx="{:.1}" cy="{:.1}" r="2.5"/>"#, sx, sy));
            }
        }
        pts.join("")
    } else {
        String::new()
    };

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
                {ob}
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
        ob = obstacle_points,
        id = id,
        pa = poly_area,
        ra = rect_area,
        fr = ratio,
        ex = extra,
        t = time_ms,
    )
}

/// Generate an SVG visualization card for MIC (Maximum Inscribed Circle) comparison.
pub fn gen_mic_card(
    id: &str,
    poly: &Polygon<f64>,
    exact: Option<&MicResult>,
    geos: Option<&MicResult>,
    exact_err: Option<&str>,
    geos_err: Option<&str>,
    exact_ms: f64,
    geos_ms: f64,
    candidate_count: usize,
    used_engine: MicUsedEngine,
    complexity: &ComplexityMetrics,
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

    let engine_label = match used_engine {
        MicUsedEngine::Exact => "Exact",
        MicUsedEngine::Grid => "Grid",
        MicUsedEngine::GeosFallback => "Geos",
    };
    let exact_info = match exact {
        Some(m) => format!("Exact radius: {:.4} [{}]<br/>", m.radius, engine_label),
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

    let complexity_str = format_complexity(complexity);
    let candidates_info = if candidate_count > 0 {
        format!("Candidates: {} · {}", candidate_count, complexity_str)
    } else {
        complexity_str
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
                <span class="diagnostics">{diag}</span>
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
        diag = candidates_info,
    )
}

/// CSS styles for both modes (LIR and MIC).
pub fn styles() -> &'static str {
    r#"body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Arial,sans-serif;margin:20px;background:#1a1a2e;color:#eee}
h1{color:#eee;margin-bottom:10px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:15px}
.card{background:#16213e;border-radius:8px;padding:10px;box-shadow:0 2px 8px rgba(0,0,0,.3)}
svg{width:100%;height:200px;background:#0f0f23;border-radius:4px}
.polygon{fill:#e94560;stroke:#ff6b6b;stroke-width:1}
.rect{fill:rgba(66,133,244,.4);stroke:#4285f4;stroke-width:2}
.obstacle{fill:#ff4444;stroke:none}
.best-effort{fill:rgba(255,193,7,.3);stroke:#ffc107}
.hole{fill:none;stroke:#666;stroke-width:1;stroke-dasharray:3}
.info{margin-top:8px;font-size:11px;color:#aaa;line-height:1.4}
.stats{background:#16213e;padding:20px;border-radius:8px;margin-bottom:20px;box-shadow:0 2px 8px rgba(0,0,0,.3)}
.stats p{margin:5px 0;color:#ccc}
.stats strong{color:#fff}"#
}

pub fn mic_styles() -> &'static str {
    r#"body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Arial,sans-serif;margin:20px;background:#1a1a2e;color:#eee}
h1{color:#eee;margin-bottom:10px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:15px}
.card{background:#16213e;border-radius:8px;padding:10px;box-shadow:0 2px 8px rgba(0,0,0,.3)}
svg{width:100%;height:220px;background:#0f0f23;border-radius:4px}
.polygon{fill:#e9456033;stroke:#ff6b6b;stroke-width:1}
.hole{fill:none;stroke:#666;stroke-width:1;stroke-dasharray:3}
.mic-exact{fill:none;stroke:#60a5fa;stroke-width:2}
.pt-exact{fill:#60a5fa;stroke:none}
.mic-geos{fill:none;stroke:#22c55e;stroke-width:2;stroke-dasharray:4 2}
.pt-geos{fill:#22c55e;stroke:none}
.info{margin-top:8px;font-size:11px;color:#aaa;line-height:1.4}
.speed-exact{color:#60a5fa}
.speed-geos{color:#22c55e}
.speed-label{color:#fbbf24;font-weight:bold}
.diagnostics{color:#888;font-size:10px;display:block;margin-top:4px}
.stats{background:#16213e;padding:20px;border-radius:8px;margin-bottom:20px;box-shadow:0 2px 8px rgba(0,0,0,.3)}
.stats p{margin:5px 0;color:#ccc}
.stats strong{color:#fff}"#
}
