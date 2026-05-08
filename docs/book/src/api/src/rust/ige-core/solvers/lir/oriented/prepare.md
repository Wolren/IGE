# ige-core::solvers::lir::oriented::prepare <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Geometry preparation for BCRS pipeline.

Port of `_prepare_polygon` and `_simplify_for_solve` from `bcrs_fast_worker.py`.

## Functions

### `ige-core::solvers::lir::oriented::prepare::prepare_polygon`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn prepare_polygon (poly : Polygon < f64 >) -> Option < Polygon < f64 > >
```

<details>
<summary>Source</summary>

```rust
pub fn prepare_polygon(poly: Polygon<f64>) -> Option<Polygon<f64>> {
    let n_unique = poly.exterior().0.windows(2).filter(|w| w[0] != w[1]).count()
        + if poly.exterior().0.first() != poly.exterior().0.last() { 1 } else { 0 };
    if n_unique < 3 {
        return None;
    }
    if poly.unsigned_area() <= 0.0 {
        return None;
    }
    Some(poly)
}
```

</details>



### `ige-core::solvers::lir::oriented::prepare::simplify_for_solve`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn simplify_for_solve (poly : & Polygon < f64 >) -> (Polygon < f64 > , bool)
```

<details>
<summary>Source</summary>

```rust
pub fn simplify_for_solve(poly: &Polygon<f64>) -> (Polygon<f64>, bool) {
    let mut n_verts = poly.exterior().0.len();
    for interior in poly.interiors() {
        n_verts += interior.0.len();
    }
    if n_verts <= SIMPLIFY_THRESHOLD {
        return (poly.clone(), false);
    }

    let bb = match poly.bounding_rect() {
        Some(b) => b,
        None => return (poly.clone(), false),
    };
    let span = (bb.max().x - bb.min().x).min(bb.max().y - bb.min().y);
    let tol = span * SIMPLIFY_TOL_FRAC;
    if tol <= 0.0 {
        return (poly.clone(), false);
    }

    let simplified = poly.simplify(&tol);
    if simplified.exterior().0.len() < 4 || simplified.unsigned_area() <= 0.0 {
        return (poly.clone(), false);
    }

    (simplified, true)
}
```

</details>



