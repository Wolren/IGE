//! Tests for LIR Approximate Oriented solver
//!
//! Run: `cargo test --test lir_approximate_oriented_tests`
//! Run all: `cargo test --workspace`

use geo::Area;
use geo_types::{coord, LineString, Polygon};
use ige_core::solvers::lir::approximate::{solve_lir_approximate_oriented, LirApproxOrientedOptions};

fn main() {
    let poly = Polygon::new(
        LineString::from(vec![
            coord! { x: 0.0, y: 0.0 },
            coord! { x: 10.0, y: 0.0 },
            coord! { x: 10.0, y: 10.0 },
            coord! { x: 0.0, y: 10.0 },
            coord! { x: 0.0, y: 0.0 },
        ]),
        vec![],
    );

    let std_opts = LirApproxOrientedOptions::default();
    let par_opts = LirApproxOrientedOptions {
        use_parallel_field: true,
        ..Default::default()
    };

    let std_res = solve_lir_approximate_oriented(&poly, &std_opts).expect("standard lir approx oriented solve failed");
    let par_res = solve_lir_approximate_oriented(&poly, &par_opts).expect("parallel lir approx oriented solve failed");

    println!("Standard: area={}", std_res.area);
    println!("Parallel: area={}", par_res.area);
}

fn square_with_hole() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            coord! {x:0.0, y:0.0},
            coord! {x:12.0, y:0.0},
            coord! {x:12.0, y:12.0},
            coord! {x:0.0, y:12.0},
            coord! {x:0.0, y:0.0},
        ]),
        vec![LineString::from(vec![
            coord! {x:4.0, y:4.0},
            coord! {x:8.0, y:4.0},
            coord! {x:8.0, y:8.0},
            coord! {x:4.0, y:8.0},
            coord! {x:4.0, y:4.0},
        ])],
    )
}

#[test]
fn parallel_quality_on_representative_shapes() {
    let shapes = vec![
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:20.0, y:0.0},
                coord! {x:20.0, y:5.0},
                coord! {x:0.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        ),
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:8.0, y:0.0},
                coord! {x:8.0, y:2.0},
                coord! {x:3.0, y:2.0},
                coord! {x:3.0, y:8.0},
                coord! {x:0.0, y:8.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        ),
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:1.0},
                coord! {x:11.0, y:8.0},
                coord! {x:4.0, y:11.0},
                coord! {x:-1.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        ),
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:10.0},
                coord! {x:6.0, y:10.0},
                coord! {x:6.0, y:3.0},
                coord! {x:4.0, y:3.0},
                coord! {x:4.0, y:10.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        ),
        square_with_hole(),
    ];

    for poly in &shapes {
        assert!(poly.unsigned_area() > 0.0);
        compare_parallel_quality(poly, 0.75);
    }
}

fn compare_parallel_quality(poly: &Polygon<f64>, min_ratio: f64) {
    let std_opts = LirApproxOrientedOptions::default();
    let par_opts = LirApproxOrientedOptions {
        use_parallel_field: true,
        ..Default::default()
    };

    let std_res = solve_lir_approximate_oriented(poly, &std_opts).expect("standard solve failed");
    let par_res = solve_lir_approximate_oriented(poly, &par_opts).expect("parallel solve failed");

    let ratio = std_res.area.min(par_res.area) / std_res.area.max(1e-10);
    assert!(ratio >= min_ratio, "parallel quality too low: {} < {}", ratio, min_ratio);
}
