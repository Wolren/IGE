//! IGE Visual Preview Tool — BCRS + baseline comparison.
//!
//! Output: `target/ige_output/index.html`
//!
//! Usage:
//!   cargo run --package ige-core --example visualize
//!   cargo run --package ige-core --example visualize -- --baseline
//!   cargo run --package ige-core --example visualize -- --limit 50

use geo::Area;
use geo_types::{Coord, LineString, Polygon};
use ige_core::bcrs::{solve_bcrs, BcrsOptions};
use ige_core::solve_oriented_lir;
use rayon::prelude::*;
use serde_json::Value;
use std::fs;

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
    let coords = geom.get("coordinates")?;
    let arr = coords.as_array()?;
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

fn load_polygons_from(path: Option<&str>) -> Vec<(usize, Polygon<f64>)> {
    let content = match path {
        Some(p) => fs::read_to_string(p).expect("Failed to read file"),
        None => include_str!("../tests/real_world_data/realworld.geojson").to_string(),
    };
    let json: Value = serde_json::from_str(&content).expect("Failed to parse GeoJSON");
    let features = json.get("features").expect("No features");
    let arr = features.as_array().expect("Features is not array");
    arr.iter()
        .filter_map(|f| {
            let id = f.get("properties")?.get("fid")?.as_u64()? as usize;
            let geom = f.get("geometry")?;
            let poly = parse_polygon(geom)?;
            Some((id, poly))
        })
        .collect()
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

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let use_bcrs = !args.contains(&"--baseline".to_string());
    let limit = args.iter()
        .position(|a| a == "--limit")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse::<usize>().ok());
    let file_path = args.iter()
        .position(|a| a == "--file")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str());

    let algo_name = if use_bcrs { "BCRS" } else { "Baseline (vertex grid)" };

    let real = load_polygons_from(file_path);
    eprintln!("Loaded {} polygons from geojson", real.len());

    let mut all_polygons: Vec<(String, Polygon<f64>)> = vec![
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
    for (id, poly) in real {
        let vc = poly.exterior().0.len() - 1;
        all_polygons.push((format!("Real #{} ({}v)", id, vc), poly));
        if let Some(n) = limit {
            if all_polygons.len() >= n { break; }
        }
    }
    eprintln!("Algorithm: {algo_name}");
    eprintln!("Total shapes: {}", all_polygons.len());

    let out_dir = std::env::current_dir().unwrap().join("target").join("ige_output");
    fs::create_dir_all(&out_dir).unwrap();

    // Wall-clock timer for the parallel section
    let wall_start = std::time::Instant::now();

    // Process all polygons in parallel with rayon, collecting (index, card, area, angle, best_effort, ms)
    let mut results: Vec<(usize, String, f64, f64, bool, f64)> = all_polygons
        .par_iter()
        .enumerate()
        .map(|(idx, (id, poly))| {
            let start = std::time::Instant::now();

            let (rp, ra, ang, be) = if use_bcrs {
                match solve_bcrs(poly, &BcrsOptions::default()) {
                    Ok(r) => (r.rect_polygon, r.area, r.angle_deg, r.best_effort),
                    Err(_) => (None, 0.0, 0.0, false),
                }
            } else {
                match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| solve_oriented_lir(poly))) {
                    Ok(Some(rect)) => {
                        let rp = Polygon::new(LineString::from(vec![
                            Coord { x: rect.x_min, y: rect.y_min },
                            Coord { x: rect.x_max, y: rect.y_min },
                            Coord { x: rect.x_max, y: rect.y_max },
                            Coord { x: rect.x_min, y: rect.y_max },
                            Coord { x: rect.x_min, y: rect.y_min },
                        ]), vec![]);
                        (Some(rp), rect.area(), 0.0, false)
                    }
                    _ => (None, 0.0, 0.0, false),
                }
            };
            let ms = start.elapsed().as_secs_f64() * 1000.0;
            let card = gen_svg_card(id, poly, rp.as_ref(), ra, ang, be, ms);
            (idx, card, ra, ang, be, ms)
        })
        .collect();

    let wall_total_ms = wall_start.elapsed().as_secs_f64() * 1000.0;

    results.sort_by_key(|(idx, ..)| *idx);

    let mut cards = String::new();
    let mut success = 0usize;
    let mut failed = 0usize;
    let mut total_rect_area = 0.0;
    let mut total_poly_area = 0.0;

    for (idx, card, ra, _ang, _be, _ms) in &results {
        let (_, poly) = &all_polygons[*idx];
        total_poly_area += poly.unsigned_area();
        total_rect_area += ra;
        if *ra > 0.0 { success += 1; } else { failed += 1; }
        cards.push_str(card);
    }

    let fill = if total_poly_area > 0.0 { total_rect_area / total_poly_area * 100.0 } else { 0.0 };
    let avg = wall_total_ms / all_polygons.len() as f64;

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
        t = wall_total_ms,
        avg = avg,
        cards = cards,
    );

    let path = out_dir.join("index.html");
    fs::write(&path, &html).unwrap();
    println!("Generated: {}  ({algo_name})", path.display());
}
