#![cfg(feature = "geos")]

//! Tests comparing MIC solver against GEOS implementation
//!
//! Run: `cargo test --test mic_geos_comparison_tests`
//! Run all: `cargo test --workspace`
//! Requires: `geos` feature enabled

use std::time::Instant;

use geo_types::{Coord, LineString, Polygon};
use ige_core::mic::{maximum_inscribed_circle, MicEngine, MicOptions, RobustMode};

fn make_polygon(exterior: &[(f64, f64)], holes: &[&[(f64, f64)]]) -> Polygon<f64> {
    let ext = LineString::from(
        exterior
            .iter()
            .map(|(x, y)| Coord { x: *x, y: *y })
            .collect::<Vec<_>>(),
    );
    let interiors = holes
        .iter()
        .map(|ring| {
            LineString::from(
                ring.iter()
                    .map(|(x, y)| Coord { x: *x, y: *y })
                    .collect::<Vec<_>>(),
            )
        })
        .collect::<Vec<_>>();
    Polygon::new(ext, interiors)
}

fn fixtures() -> Vec<(&'static str, Polygon<f64>)> {
    vec![
        ("20x20 square",
            make_polygon(&[(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0), (0.0, 0.0)], &[])),
        ("C-shape concave",
            make_polygon(&[
                (0.0, 0.0), (14.0, 0.0), (14.0, 12.0), (9.0, 12.0),
                (9.0, 4.0), (5.0, 4.0), (5.0, 12.0), (0.0, 12.0), (0.0, 0.0),
            ], &[])),
        ("rect with hole",
            make_polygon(&[(0.0, 0.0), (30.0, 0.0), (30.0, 22.0), (0.0, 22.0), (0.0, 0.0)],
                &[&[(9.0, 7.0), (21.0, 7.0), (21.0, 15.0), (9.0, 15.0), (9.0, 7.0)]])),
        ("40x4 thin rect",
            make_polygon(&[(0.0, 0.0), (40.0, 0.0), (40.0, 4.0), (0.0, 4.0), (0.0, 0.0)], &[])),
        ("regular hexagon",
            make_polygon(&[
                (10.0, 0.0), (5.0, 8.66), (-5.0, 8.66), (-10.0, 0.0),
                (-5.0, -8.66), (5.0, -8.66), (10.0, 0.0),
            ], &[])),
        ("right triangle",
            make_polygon(&[(0.0, 0.0), (12.0, 0.0), (0.0, 9.0), (0.0, 0.0)], &[])),
    ]
}

fn exact_options() -> MicOptions {
    MicOptions {
        engine: MicEngine::ExactOnly,
        robust_mode: RobustMode::Filtered,
    }
}

fn geos_options() -> MicOptions {
    MicOptions {
        engine: MicEngine::FallbackOnly,
        robust_mode: RobustMode::Filtered,
    }
}

#[test]
fn mic_accuracy_against_geos_radius() {
    let polys = fixtures();
    let exact_opts = exact_options();
    let geos_opts = geos_options();
    let mut worst_err = 0.0f64;
    let mut worst_name = "";

    for (name, poly) in &polys {
        let exact = maximum_inscribed_circle(poly, &exact_opts).expect("exact solve failed");
        let geos = maximum_inscribed_circle(poly, &geos_opts).expect("GEOS solve failed");
        assert!(geos.radius > 0.0);
        let rel_err = (exact.radius - geos.radius).abs() / geos.radius.max(1e-12);
        if rel_err > worst_err {
            worst_err = rel_err;
            worst_name = name;
        }
        assert!(
            rel_err <= 0.20,
            "MIC radius differs too much vs GEOS: {} exact={} geos={} rel_err={}",
            name, exact.radius, geos.radius, rel_err
        );
    }

    println!(
        "\n  Accuracy vs GEOS: worst rel_err = {:.6} on '{}'",
        worst_err, worst_name
    );
}

#[test]
fn mic_speed_comparison() {
    let polys = fixtures();
    let exact_opts = exact_options();
    let geos_opts = geos_options();
    let iters = 500usize;

    println!("\n  ┌──────────────────────────────┬────────────┬────────────┬───────────┬──────────┐");
    println!("  │ Fixture                      │ Exact (μs) │ GEOS (μs)  │ Ratio     │ Speedup  │");
    println!("  ├──────────────────────────────┼────────────┬────────────┼───────────┬──────────┤");

    for (name, poly) in &polys {
        let t0 = Instant::now();
        for _ in 0..iters {
            let _ = maximum_inscribed_circle(poly, &exact_opts).expect("exact solve failed");
        }
        let exact_per = t0.elapsed().as_secs_f64() / iters as f64 * 1e6;

        let t1 = Instant::now();
        for _ in 0..iters {
            let _ = maximum_inscribed_circle(poly, &geos_opts).expect("GEOS solve failed");
        }
        let geos_per = t1.elapsed().as_secs_f64() / iters as f64 * 1e6;

        let ratio = if geos_per > 0.0 { exact_per / geos_per } else { 0.0 };
        let speedup = if ratio <= 1.0 {
            format!("{:5.2}x faster", 1.0 / ratio.max(1e-12))
        } else {
            format!("{:5.2}x slower", ratio)
        };

        let padded = format!(" {:<28}", name);
        println!(
            "  │{}│ {:>8.1} │ {:>8.1} │ {:>7.3} │ {:>8} │",
            padded, exact_per, geos_per, ratio, speedup
        );
    }

    println!("  └──────────────────────────────┴────────────┴────────────┴───────────┴──────────┘\n");
}

#[test]
fn mic_speed_against_geos() {
    let polys: Vec<Polygon<f64>> = fixtures().into_iter().map(|(_, p)| p).collect();
    let exact_opts = exact_options();
    let geos_opts = geos_options();
    let iters = 200usize;

    let t0 = Instant::now();
    for _ in 0..iters {
        for poly in &polys {
            let _ = maximum_inscribed_circle(poly, &exact_opts).expect("exact solve failed");
        }
    }
    let exact_elapsed = t0.elapsed();

    let t1 = Instant::now();
    for _ in 0..iters {
        for poly in &polys {
            let _ = maximum_inscribed_circle(poly, &geos_opts).expect("GEOS solve failed");
        }
    }
    let geos_elapsed = t1.elapsed();

    assert!(
        exact_elapsed <= geos_elapsed.mul_f64(5.0),
        "exact solver is unexpectedly slower than GEOS: exact={:?}, geos={:?}",
        exact_elapsed,
        geos_elapsed
    );
}
