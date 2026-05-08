# ige-core::solvers::lir::oriented <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


LIR Oriented — Largest Inscribed Rectangle Solver with free orientation (rotation).

Parallel solver that evaluates multiple candidate angles in parallel for better quality results.
Optional GPU acceleration hooks are behind the `"gpu"` feature flag.

## Structs

### `ige-core::solvers::lir::oriented::LirOrientedOptions`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Configuration for the LIR approximate oriented solver.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `max_ratio` | `f64` | Max aspect ratio (longer/shorter side); 0.0 = unconstrained. |
| `min_ratio` | `f64` | Min aspect ratio (longer/shorter side); 0.0 = unconstrained.
Use this to require rectangles to be at least this elongated. |
| `grid_coarse` | `usize` | Coarse grid resolution used for heuristic seeding and Brent polish. |
| `grid_fine` | `usize` | Fine grid resolution used in conservative fallback. |
| `top_k` | `usize` | Number of top heuristic candidates forwarded to stages 4–6. |
| `always_return` | `bool` | If true, return best-effort result even if certification fails. |
| `use_parallel_field` | `bool` | If true, enable an additional local angle-polish pass (higher accuracy, slightly slower). |
| `use_simulated_annealing` | `bool` | If true, run an experimental simulated-annealing basin escape over (center, angle). |
| `use_bootstrap_seeds` | `bool` | If true, use deterministic bootstrap seeds (vertex-snapped + center seeds) per angle. |
| `use_pca_axes` | `bool` | If true, use Principal Component Analysis to guide initial angle candidates. |
| `use_edge_anchored` | `bool` | If true, generate edge-anchored candidates from boundary support relationships. |
| `polish_halwidth_deg` | `f64` | Half-width (degrees) for the Brent golden-section polish. |
| `polish_xatol_deg` | `f64` | Convergence tolerance for Brent polish (degrees). |
| `prune_margin` | `f64` | Prune margin for angle upper-bound pruning. |
| `angle_delta_deg` | `f64` | Angle offset tried around each polished angle (degrees). |
| `top_trials` | `usize` | Number of angle variants to keep per candidate. |
| `cert_eps` | `f64` | SDF certification epsilon. |
| `cert_max_shrink` | `f64` | Max shrink fraction during certification. |
| `field_min_angles` | `usize` | Minimum angles to pad (parallel field). |
| `field_angle_step` | `usize` | Step size for regular angle padding (parallel field). |
| `field_max_coords` | `usize` | Max vertex coords per axis before uniform fallback. |
| `gpu_ctx` | `Option < std :: sync :: Arc < GpuContext > >` | GPU context for accelerated SDF evaluation. |



### `ige-core::solvers::lir::oriented::LirOrientedResult`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Result of a LIR approximate oriented solve, including per-stage area gains for diagnostics.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `rect` | `Option < Rectangle >` | Best inscribed rectangle in world frame (AABB — axis-aligned bounding box).
For the actual oriented rectangle, use `rect_polygon`. |
| `rect_polygon` | `Option < Polygon < f64 > >` | The actual oriented rectangle as a polygon (rotated in world frame).
None when no solution was found. |
| `area` | `f64` | Actual certified area. |
| `angle_deg` | `f64` | Rotation angle that produced the best result [degrees]. |
| `best_effort` | `bool` | True if the result is best-effort rather than strictly certified. |
| `s2_area` | `f64` | Area after Stage 2 (coarse grid seed). |
| `s4_area` | `f64` | Area after Stage 4 (BCRS vertex-coordinate solve). |
| `s5_area` | `f64` | Area after Stage 5 (SDF-guided expansion). |

#### Methods

##### `empty` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn empty () -> Self
```

<details>
<summary>Source</summary>

```rust
    pub fn empty() -> Self {
        Self {
            rect: None,
            rect_polygon: None,
            area: 0.0,
            angle_deg: 0.0,
            best_effort: false,
            s2_area: 0.0,
            s4_area: 0.0,
            s5_area: 0.0,
        }
    }
```

</details>





### `ige-core::solvers::lir::oriented::AngleCandidate`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">pub(crate)</span>


**Derives:** `Debug`, `Clone`

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `angle` | `f64` |  |
| `area` | `f64` |  |
| `rect_rot` | `(f64 , f64 , f64 , f64)` |  |
| `rect_world_bounds` | `(f64 , f64 , f64 , f64)` |  |
| `center` | `Point < f64 >` |  |



## Functions

### `ige-core::solvers::lir::oriented::solve_lir_oriented`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_lir_oriented (poly : & Polygon < f64 > , options : & LirOrientedOptions) -> Result < LirOrientedResult >
```

Solve the largest inscribed rectangle using BCRS + SDF pipeline.

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `poly` | `-` | Input polygon (must be valid, non-empty, area > 0) |
| `options` | `-` | Solver configuration |


**Returns:**

A `LirOrientedResult` with the best rectangle (AABB in world frame), area, angle, etc.

<details>
<summary>Source</summary>

```rust
pub fn solve_lir_oriented(poly: &Polygon<f64>, options: &LirOrientedOptions) -> Result<LirOrientedResult> {
    solve_lir_oriented_parallel(poly, options)
}
```

</details>



### `ige-core::solvers::lir::oriented::worker_process_feature`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn worker_process_feature (poly : & Polygon < f64 > , _angle_step : usize , grid_coarse : usize , grid_fine : usize , max_ratio : f64 , min_ratio : f64 , top_k : usize , always_return : bool ,) -> Option < (Rectangle , f64 , f64 , f64 , usize , f64 , bool) >
```

Stateless worker entry point, mirrors `_worker_process_feature`.

Returns `(area, angle_deg, ratio, cand_rank, s2_gain, best_effort)` on success.

<details>
<summary>Source</summary>

```rust
pub fn worker_process_feature(
    poly: &Polygon<f64>,
    _angle_step: usize,
    grid_coarse: usize,
    grid_fine: usize,
    max_ratio: f64,
    min_ratio: f64,
    top_k: usize,
    always_return: bool,
) -> Option<(Rectangle, f64, f64, f64, usize, f64, bool)> {
    let options = LirOrientedOptions {
        max_ratio,
        min_ratio,
        grid_coarse,
        grid_fine,
        top_k,
        always_return,
        use_parallel_field: false,
        use_simulated_annealing: false,
        use_bootstrap_seeds: false,
        use_pca_axes: false,
        use_edge_anchored: false,
        polish_halwidth_deg: crate::tuning::POLISH_HALFWIDTH,
        polish_xatol_deg: crate::tuning::POLISH_XATOL,
        prune_margin: crate::tuning::PRUNE_MARGIN,
        angle_delta_deg: crate::tuning::ANGLE_DELTA,
        top_trials: crate::tuning::TOP_TRIALS,
        cert_eps: crate::tuning::CERT_EPS,
        cert_max_shrink: crate::tuning::CERT_MAX_SHRINK,
        field_min_angles: crate::tuning::FIELD_MIN_ANGLES,
        field_angle_step: crate::tuning::FIELD_ANGLE_STEP,
        field_max_coords: crate::tuning::FIELD_MAX_COORDS,
        #[cfg(feature = "gpu")]
        gpu_ctx: None,
    };

    let result = solve_lir_oriented(poly, &options).ok()?;

    Some((
        result.rect?,
        result.area,
        result.angle_deg,
        result.s5_area / (result.s2_area.max(1e-12)),
        if result.s5_area > 0.0 { 0 } else { 0 },
        result.s5_area - result.s2_area,
        result.best_effort,
    ))
}
```

</details>



