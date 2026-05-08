# ige-core::solvers::obb::solver <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


OBB solver implementation.

Implements various algorithms for finding the minimal oriented bounding box.

## Functions

### `ige-core::solvers::obb::solver::solve_obb_rotating_calipers`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_obb_rotating_calipers (_poly : & Polygon < f64 > , _options : & ObbOptions ,) -> Result < ObbResult >
```

Solve OBB using rotating calipers approach.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_obb_rotating_calipers(
    _poly: &Polygon<f64>,
    _options: &ObbOptions,
) -> Result<ObbResult> {
    Err(crate::shared::LirError::NotSupported("OBB rotating calipers not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::obb::solver::solve_obb_angle_sweep`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_obb_angle_sweep (_poly : & Polygon < f64 > , _options : & ObbOptions ,) -> Result < ObbResult >
```

Solve OBB using angle sweep with refinement.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_obb_angle_sweep(
    _poly: &Polygon<f64>,
    _options: &ObbOptions,
) -> Result<ObbResult> {
    Err(crate::shared::LirError::NotSupported("OBB angle sweep not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::obb::solver::solve_obb_pca`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_obb_pca (_poly : & Polygon < f64 > , _options : & ObbOptions ,) -> Result < ObbResult >
```

Solve OBB using PCA (Principal Component Analysis) approach.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_obb_pca(
    _poly: &Polygon<f64>,
    _options: &ObbOptions,
) -> Result<ObbResult> {
    Err(crate::shared::LirError::NotSupported("OBB PCA not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::obb::solver::solve_obb_constrained`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_obb_constrained (_poly : & Polygon < f64 > , _options : & ObbOptions ,) -> Result < ObbResult >
```

Solve OBB with aspect ratio constraints.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_obb_constrained(
    _poly: &Polygon<f64>,
    _options: &ObbOptions,
) -> Result<ObbResult> {
    Err(crate::shared::LirError::NotSupported("OBB constrained not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::obb::solver::compute_obb_metrics`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn compute_obb_metrics (poly : & Polygon < f64 > , angle_deg : f64 ,) -> ObbResult
```

Compute OBB metrics from a candidate box.

Helper function to calculate area, perimeter, aspect ratio, etc.

<details>
<summary>Source</summary>

```rust
pub fn compute_obb_metrics(
    poly: &Polygon<f64>,
    angle_deg: f64,
) -> ObbResult {
    let area = poly.unsigned_area();
    let perimeter = poly.exterior().0.windows(2).map(|w| {
        let dx = w[1].x - w[0].x;
        let dy = w[1].y - w[0].y;
        (dx * dx + dy * dy).sqrt()
    }).sum::<f64>();

    let bb = match poly.bounding_rect() {
        Some(b) => b,
        None => return ObbResult::empty(),
    };

    let width = bb.max().x - bb.min().x;
    let height = bb.max().y - bb.min().y;
    let aspect_ratio = if height > 0.0 { width / height } else { 1.0 };
    let aspect_ratio = aspect_ratio.max(1.0 / aspect_ratio.max(1e-10));

    let fill_ratio = if width * height > 0.0 {
        area / (width * height)
    } else {
        0.0
    };

    ObbResult {
        polygon: Some(poly.clone()),
        area: width * height,
        perimeter,
        angle_deg,
        width,
        height,
        centroid: poly.centroid(),
        aspect_ratio,
        fill_ratio,
    }
}
```

</details>



