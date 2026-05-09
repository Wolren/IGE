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
//!   cargo run --package ige-core --example visualize --features simd   # compile with SIMD (shows "+ SIMD" in title)

mod visualize_modules;

use rayon::prelude::*;
use std::fs;

use crate::visualize_modules::cli::{AxisCliSolver, CliConfig, algo_name, simd_status};
use crate::visualize_modules::io::load_polygons_from;
use crate::visualize_modules::render::{
    gen_mic_card, gen_svg_card,
};
use crate::visualize_modules::render::ComplexityMetrics;
use crate::visualize_modules::shapes::{make_l_shape, make_u_shape, make_zigzag};
use ige_core::solvers::ler::axis_aligned::solve_ler_axis_aligned_exact;
use ige_core::solvers::ler::LerOptions;
use ige_core::solvers::lir::axis_aligned::{solve_axis_exact, solve_vertex_grid};
use ige_core::solvers::lir::oriented::{solve_lir_oriented, LirOrientedOptions};
use ige_core::solvers::mic::{maximum_inscribed_circle, MicEngine, MicOptions, MicUsedEngine, RobustMode};
use ige_core::{solve_axis_rect_grid_with_backend, AxisAlignedOptions, MaskBackend};
use geo::algorithm::area::Area;

/// Build standard test shapes and merge them with real polygons if needed.
fn build_polygons(real: Vec<(String, geo_types::Polygon<f64>)>, config: &CliConfig) -> Vec<(String, geo_types::Polygon<f64>)> {
    if config.real_only || config.use_mic_compare {
        return real;
    }

    let mut data = vec![
        ("Square 10x10".into(), geo_types::Polygon::new(
            geo_types::LineString::from(vec![
                geo_types::Coord { x: 0.0, y: 0.0 }, geo_types::Coord { x: 10.0, y: 0.0 },
                geo_types::Coord { x: 10.0, y: 10.0 }, geo_types::Coord { x: 0.0, y: 10.0 },
                geo_types::Coord { x: 0.0, y: 0.0 },
            ]), vec![],
        )),
        ("Rectangle 20x5".into(), geo_types::Polygon::new(
            geo_types::LineString::from(vec![
                geo_types::Coord { x: 0.0, y: 0.0 }, geo_types::Coord { x: 20.0, y: 0.0 },
                geo_types::Coord { x: 20.0, y: 5.0 }, geo_types::Coord { x: 0.0, y: 5.0 },
                geo_types::Coord { x: 0.0, y: 0.0 },
            ]), vec![],
        )),
        ("Triangle".into(), geo_types::Polygon::new(
            geo_types::LineString::from(vec![
                geo_types::Coord { x: 0.0, y: 0.0 }, geo_types::Coord { x: 10.0, y: 0.0 },
                geo_types::Coord { x: 5.0, y: 10.0 }, geo_types::Coord { x: 0.0, y: 0.0 },
            ]), vec![],
        )),
        ("L-Shape".into(), make_l_shape(5.0, 5.0, 5.0)),
        ("U-Shape".into(), make_u_shape(5.0, 5.0, 5.0)),
        ("Zigzag".into(), make_zigzag(5.0, 5.0, 5.0)),
    ];
    data.extend(real);
    data
}

/// Solve for a single polygon according to the active solver mode.
fn solve_polygon(
    poly: &geo_types::Polygon<f64>,
    config: &CliConfig,
    mask_backend: MaskBackend,
) -> (Option<geo_types::Polygon<f64>>, f64, f64, bool) {
    if config.use_ler {
        let mut obstacles = vec![];
        let obs_size = 0.15;
        for c in poly.exterior().coords() {
            let ox0 = c.x - obs_size;
            let oy0 = c.y - obs_size;
            let ox1 = c.x + obs_size;
            let oy1 = c.y + obs_size;
            let obs = geo_types::Polygon::new(
                geo_types::LineString::from(vec![
                    geo_types::Coord { x: ox0, y: oy0 },
                    geo_types::Coord { x: ox1, y: oy0 },
                    geo_types::Coord { x: ox1, y: oy1 },
                    geo_types::Coord { x: ox0, y: oy1 },
                    geo_types::Coord { x: ox0, y: oy0 },
                ]),
                vec![],
            );
            obstacles.push(obs);
        }

        for hole in poly.interiors() {
            for c in hole.coords() {
                let ox0 = c.x - obs_size;
                let oy0 = c.y - obs_size;
                let ox1 = c.x + obs_size;
                let oy1 = c.y + obs_size;
                let obs = geo_types::Polygon::new(
                    geo_types::LineString::from(vec![
                        geo_types::Coord { x: ox0, y: oy0 },
                        geo_types::Coord { x: ox1, y: oy0 },
                        geo_types::Coord { x: ox1, y: oy1 },
                        geo_types::Coord { x: ox0, y: oy1 },
                        geo_types::Coord { x: ox0, y: oy0 },
                    ]),
                    vec![],
                );
                obstacles.push(obs);
            }
        }
        match solve_ler_axis_aligned_exact(poly, &obstacles, &LerOptions::default()) {
            Ok(r) => (r.rect_polygon, r.area, r.angle_deg, r.best_effort),
            Err(_) => (None, 0.0, 0.0, false),
        }
    } else if config.use_parallel || config.use_sa || config.use_bootstrap_seeds || config.use_pca_axes || config.use_early_stopping || config.use_edge_anchored {
        let mut opts = LirOrientedOptions::default();
        opts.use_parallel_field = config.use_parallel;
        opts.use_simulated_annealing = config.use_sa;
        opts.use_bootstrap_seeds = config.use_bootstrap_seeds;
        opts.use_pca_axes = config.use_pca_axes;
        opts.use_edge_anchored = config.use_edge_anchored;
        match solve_lir_oriented(poly, &opts) {
            Ok(r) => (r.rect_polygon, r.area, r.angle_deg, r.best_effort),
            Err(_) => (None, 0.0, 0.0, false),
        }
    } else if config.use_approx_oriented {
        match solve_lir_oriented(poly, &LirOrientedOptions::default()) {
            Ok(r) => (r.rect_polygon, r.area, r.angle_deg, r.best_effort),
            Err(_) => (None, 0.0, 0.0, false),
        }
    } else {
        let opts = AxisAlignedOptions::default();
        match config.axis_solver {
            AxisCliSolver::Vertex => {
                match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| solve_vertex_grid(poly, &opts))) {
                    Ok(Some(rect)) => {
                        let area = rect.area();
                        (Some(crate::visualize_modules::render::rect_to_polygon(rect)), area, 0.0, false)
                    }
                    _ => (None, 0.0, 0.0, false),
                }
            }
            AxisCliSolver::Exact => {
                match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| solve_axis_exact(poly, &opts))) {
                    Ok(Some(rect)) => {
                        let area = rect.area();
                        (Some(crate::visualize_modules::render::rect_to_polygon(rect)), area, 0.0, false)
                    }
                    _ => (None, 0.0, 0.0, false),
                }
            }
            AxisCliSolver::Grid => {
                match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    solve_axis_rect_grid_with_backend(poly, opts.max_grid, opts.max_ratio, opts.min_ratio, mask_backend)
                })) {
                    Ok(Some((x0, y0, x1, y1, _))) => {
                        let rp = geo_types::Polygon::new(geo_types::LineString::from(vec![
                            geo_types::Coord { x: x0, y: y0 },
                            geo_types::Coord { x: x1, y: y0 },
                            geo_types::Coord { x: x1, y: y1 },
                            geo_types::Coord { x: x0, y: y1 },
                            geo_types::Coord { x: x0, y: y0 },
                        ]), vec![]);
                        let area = (x1 - x0) * (y1 - y0);
                        (Some(rp), area, 0.0, false)
                    }
                    _ => (None, 0.0, 0.0, false),
                }
            }
        }
    }
}

fn build_html_lir(
    all_polygons: &[(String, geo_types::Polygon<f64>)],
    results: &[(usize, String, f64, f64, bool, f64, f64)],
    config: &CliConfig,
    wall_total_ms: f64,
) -> String {
    let success = results.iter().filter(|(_, _, ra, _, _, _, _)| *ra > 0.0).count();
    let failed = results.len() - success;
    let total_poly_area: f64 = results.iter()
        .map(|(idx, _card, _ra, _ang, _be, _ms, _fill_pct)| all_polygons[*idx].1.unsigned_area())
        .sum();
    let total_rect_area: f64 = results.iter().map(|(_, _, ra, _, _, _, _)| ra).sum();

    let perf = crate::visualize_modules::stats::LirStats::from_iter(
        results.iter()
            .map(|(idx, _card, ra, _ang, _be, _ms, _fill_pct)| (*ra, all_polygons[*idx].1.unsigned_area()))
    );
    let fill = perf.overall_fill_pct();
    let avg = wall_total_ms / all_polygons.len() as f64;

    let cards: String = results.iter()
        .map(|(_, card, _, _, _, _, _)| card.as_str())
        .collect();

    let title = if config.use_ler { "Largest Empty Rectangle" } else { "Largest Inscribed Rectangle" };

    let area_label = if config.use_ler { "Empty area" } else { "Inscribed area" };

    format!(
        r#"<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>IGE Visual Preview — {title}</title>
<style>
{style}
</style></head><body>
<h1>IGE — {title}</h1>
<p style="color:#aaa;">Algorithm: <strong style="color:#4285f4;">{algo}</strong> &mdash; full dataset ({n} shapes)</p>
<div class="stats">
<p><strong>Success:</strong> {ok}/{n} ({pct:.1}%) &nbsp; <strong>Failed:</strong> {fail}</p>
<p><strong>Polygon area:</strong> {pa:.0} &nbsp; <strong>{area_label}:</strong> {ra:.0} ({fill:.1}%)</p>
<p><strong>Per-shape fill rate:</strong> median {median_pct:.1}% &nbsp; mean {mean_pct:.1}%</p>
<p><strong>Total time:</strong> {t:.1}ms &nbsp; <strong>Avg:</strong> {avg:.2}ms/shape</p>
</div>
<div class="grid">{cards}</div>
</body></html>"#,
         style = crate::visualize_modules::render::styles(),
        title = title,
        algo = algo_name(config),
        n = all_polygons.len(),
        ok = success,
        pct = success as f64 / all_polygons.len() as f64 * 100.0,
        fail = failed,
        pa = total_poly_area,
        ra = total_rect_area,
        fill = fill,
        median_pct = perf.median_pct(),
        mean_pct = perf.mean_pct(),
        t = wall_total_ms,
        avg = avg,
        cards = cards,
        area_label = area_label,
    )
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let config = CliConfig::from_args(&args);

    // Preserve helpful warnings from the original script.
    if !config.use_approx_oriented
        && !config.use_mic_compare
        && config.axis_solver != AxisCliSolver::Grid
        && args.iter().any(|a| a == "--mask-backend")
    {
        eprintln!("--mask-backend is ignored unless --axis-solver grid is selected");
    }
    if !config.use_approx_oriented && config.use_sa {
        eprintln!("--sa is ignored in baseline axis-aligned mode");
    }

    let real = load_polygons_from(config.file_path.as_deref());
    eprintln!("Loaded {} polygons from geojson", real.len());

    let mut all_polygons = build_polygons(real, &config);
    if let Some(n) = config.limit {
        all_polygons.truncate(n);
    }

    let algo = algo_name(&config);
    eprintln!("Algorithm: {algo}");
    eprintln!("Total shapes: {}", all_polygons.len());

    let out_dir = std::env::current_dir().unwrap().join("target").join("ige_output");
    std::fs::create_dir_all(&out_dir).unwrap();

    if config.use_mic_compare {
        run_mic_mode(&all_polygons, &config, &out_dir);
        return;
    }

    let wall_start = std::time::Instant::now();

    let mut results: Vec<(usize, String, f64, f64, bool, f64, f64)> = all_polygons
        .par_iter()
        .enumerate()
        .map(|(idx, (id, poly))| {
            let start = std::time::Instant::now();
            let (rp, ra, ang, be) = solve_polygon(poly, &config, config.mask_backend);
            let ms = start.elapsed().as_secs_f64() * 1000.0;
            let poly_area = poly.unsigned_area();
            let fill_pct = if poly_area > 0.0 { ra / poly_area * 100.0 } else { 0.0 };
            let card = gen_svg_card(id, poly, rp.as_ref(), ra, ang, be, ms, config.use_ler);
            (idx, card, ra, ang, be, ms, fill_pct)
        })
        .collect();

    let wall_total_ms = wall_start.elapsed().as_secs_f64() * 1000.0;
    results.sort_by_key(|(idx, ..)| *idx);

    if config.use_json {
        let perf = crate::visualize_modules::stats::LirStats::from_iter(
            results.iter()
                .map(|(idx, _card, ra, _ang, _be, _ms, _fill_pct)| (*ra, all_polygons[*idx].1.unsigned_area()))
        );
        let json = serde_json::json!({
            "success": results.iter().filter(|(_, _, ra, _, _, _, _)| *ra > 0.0).count(),
            "total": all_polygons.len(),
            "fill_rate": perf.overall_fill_pct() / 100.0,
            "avg_ms": wall_total_ms / all_polygons.len() as f64,
            "wall_ms": wall_total_ms,
            "per_shape_pct": {
                "median": perf.median_pct(),
                "mean": perf.mean_pct(),
            },
        });
        println!("JSON: {}", serde_json::to_string_pretty(&json).unwrap());
    } else {
        let path = out_dir.join("index.html");
        let html = build_html_lir(&all_polygons, &results, &config, wall_total_ms);
        fs::write(&path, &html).unwrap();
        println!("Generated: {}  ({algo})", path.display());
    }
}

/// Run MIC comparison mode.
fn run_mic_mode(polygons: &[(String, geo_types::Polygon<f64>)], config: &CliConfig, out_dir: &std::path::Path) {
    let exact_opts = MicOptions {
        engine: MicEngine::ExactOnly,
        robust_mode: RobustMode::Filtered,
    };
    let geos_opts = MicOptions {
        engine: MicEngine::FallbackOnly,
        robust_mode: RobustMode::Filtered,
    };

    let wall_start = std::time::Instant::now();

    let mut results: Vec<(usize, String, Option<f64>, Option<f64>, f64, f64, usize, ComplexityMetrics, MicUsedEngine)> = polygons
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
            let candidate_count = exact.as_ref().map(|r| r.candidate_count).unwrap_or(0);
            let used_engine = exact.as_ref().map(|r| r.used_engine).unwrap_or(MicUsedEngine::Exact);
            let complexity = crate::visualize_modules::render::compute_complexity(poly);

            let card = gen_mic_card(
                id,
                poly,
                exact.as_ref().ok(),
                geos.as_ref().ok(),
                exact_err.as_deref(),
                geos_err.as_deref(),
                exact_ms,
                geos_ms,
                candidate_count,
                used_engine,
                &complexity,
            );
            (idx, card, exact_radius, geos_radius, exact_ms, geos_ms, candidate_count, complexity, used_engine)
        })
        .collect();

    let wall_total_ms = wall_start.elapsed().as_secs_f64() * 1000.0;
    results.sort_by_key(|(idx, ..)| *idx);

    let polygon_ids: Vec<String> = polygons.iter().map(|(id, _)| id.clone()).collect();

    if config.use_json {
        let mut stats = crate::visualize_modules::stats::MicStats::new();
        for (idx, _card, exact_r, geos_r, exact_ms, geos_ms, _candidate_count, _complexity, _used_engine) in &results {
            stats.update(*idx, "", *exact_r, *geos_r, *exact_ms, *geos_ms, &polygon_ids);
        }
        let summary = stats.finalize(results.len());
        let json = serde_json::json!({
            "mode": "mic_compare",
            "total": polygons.len(),
            "exact_ok": summary.exact_ok,
            "geos_ok": summary.geos_ok,
            "both_ok": summary.both_ok,
            "rel_err_pct": { "median": summary.rel_median, "mean": summary.rel_mean },
            "exact_larger_count": summary.exact_larger_count,
            "exact_smaller_count": summary.exact_smaller_count,
            "top_errors": summary.top_errors,
            "avg_exact_ms": summary.avg_exact_ms,
            "avg_geos_ms": summary.avg_geos_ms,
            "speed_ratio": summary.speed_ratio(),
            "wall_ms": wall_total_ms,
        });
        println!("{}", serde_json::to_string(&json).unwrap());
        return;
    }

    let mut stats = crate::visualize_modules::stats::MicStats::new();
    let mut cards = String::new();

    for (idx, card, exact_r, geos_r, exact_ms, geos_ms, _candidate_count, _complexity, _used_engine) in &results {
        cards.push_str(card);
        stats.update(*idx, card, *exact_r, *geos_r, *exact_ms, *geos_ms, &polygon_ids);
    }

    let summary = stats.finalize(results.len());
    let speed_ratio = summary.speed_ratio();
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
{style}
</style></head><body>
<h1>IGE — MIC exact vs GEOS{simd}</h1>
<p style="color:#aaa;">Blue = exact solver, Green dashed = GEOS fallback &nbsp;|&nbsp; <span class="speed-exact">Exact time</span> vs <span class="speed-geos">GEOS time</span></p>
<div class="stats">
<p><strong>Exact success:</strong> {exact_ok}/{n} &nbsp; <strong>GEOS success:</strong> {geos_ok}/{n} &nbsp; <strong>Both:</strong> {both_ok}</p>
<p><strong>Rel err vs GEOS:</strong> median {rel_median:.3}% &nbsp; mean {rel_mean:.3}%</p>
<p><strong><span class=speed-exact>Exact avg:</span></strong> {avg_exact_ms:.3}ms/shape &nbsp; <strong><span class=speed-geos>GEOS avg:</span></strong> {avg_geos_ms:.3}ms/shape &nbsp; <strong>Exact is {sit}</strong></p>
<p><strong>Total time:</strong> {wall_total_ms:.1}ms</p>
</div>
<div class="grid">{cards}</div>
</body></html>"#,
        style = crate::visualize_modules::render::mic_styles(),
        n = polygons.len(),
        exact_ok = summary.exact_ok,
        geos_ok = summary.geos_ok,
        both_ok = summary.both_ok,
        rel_median = summary.rel_median,
        rel_mean = summary.rel_mean,
        avg_exact_ms = summary.avg_exact_ms,
        avg_geos_ms = summary.avg_geos_ms,
        sit = speed_label,
        wall_total_ms = wall_total_ms,
        cards = cards,
        simd = simd_status(),
    );
    fs::write(&path, &html).unwrap();
    println!("Generated: {}  (MIC exact vs GEOS{})", path.display(), simd_status());
}
