# ige-core::solvers::mic <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Maximum Inscribed Circle (MIC) solvers for polygonal inputs.

## Structs

### `ige-core::solvers::mic::MicOptions`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`, `Copy`, `PartialEq`, `Eq`

Solver configuration.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `engine` | `MicEngine` |  |
| `robust_mode` | `RobustMode` |  |



### `ige-core::solvers::mic::MicResult`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

MIC solve result.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `center` | `Point < f64 >` |  |
| `radius` | `f64` |  |
| `radius_sq` | `f64` |  |
| `support_segments` | `Vec < usize >` |  |
| `candidate_count` | `usize` |  |
| `used_engine` | `MicUsedEngine` |  |
| `component_index` | `Option < usize >` |  |



## Enums

### `ige-core::solvers::mic::MicEngine` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


Engine selection for MIC solving.

#### Variants

- **`ExactOnly`** - Use the native Rust polygon-specialized solver only.
- **`FallbackOnly`** - Use GEOS fallback only.
- **`ExactThenGeos`** - Try exact first, then GEOS fallback if exact fails.



### `ige-core::solvers::mic::RobustMode` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


Numeric robustness mode for the exact engine.

#### Variants

- **`FastF64`** - Fast finite-precision mode.
- **`Filtered`** - Extra candidate filtering and certification.



### `ige-core::solvers::mic::MicUsedEngine` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


Engine that produced the final result.

#### Variants

- **`Exact`**
- **`GeosFallback`**



### `ige-core::solvers::mic::MicError` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


MIC solver error.

#### Variants

- **`InvalidInput`**
- **`NoCircleFound`**
- **`ExactFailed`**
- **`GeosFeatureDisabled`**
- **`GeosFailed`**
- **`UnsupportedGeosOutput`**



## Functions

### `ige-core::solvers::mic::maximum_inscribed_circle`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn maximum_inscribed_circle (poly : & Polygon < f64 > , opts : & MicOptions ,) -> Result < MicResult , MicError >
```

Solve MIC on a single polygon.

<details>
<summary>Source</summary>

```rust
pub fn maximum_inscribed_circle(
    poly: &Polygon<f64>,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    let host = HostPolygon::from_polygon(poly)?;
    solve_on_host_polygon(&host, opts)
}
```

</details>



### `ige-core::solvers::mic::maximum_inscribed_circle_with_workspace`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn maximum_inscribed_circle_with_workspace (workspace : & mut MicWorkspace , opts : & MicOptions ,) -> Result < MicResult , MicError >
```

Solve MIC with a reusable workspace (avoids rebuilding indexes per call).

The workspace is rebuilt only if `host` changes; for repeated calls on
different polygons, create a fresh [`MicWorkspace`] each time.

<details>
<summary>Source</summary>

```rust
pub fn maximum_inscribed_circle_with_workspace(
    workspace: &mut MicWorkspace,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    solve_exact(workspace, opts).map_err(|err| MicError::ExactFailed(err.to_string()))
}
```

</details>



### `ige-core::solvers::mic::maximum_inscribed_circle_multipolygon`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn maximum_inscribed_circle_multipolygon (multi : & MultiPolygon < f64 > , opts : & MicOptions ,) -> Result < MicResult , MicError >
```

Solve MIC on a multipolygon by solving each component and keeping the best.

<details>
<summary>Source</summary>

```rust
pub fn maximum_inscribed_circle_multipolygon(
    multi: &MultiPolygon<f64>,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    if multi.0.is_empty() {
        return Err(MicError::InvalidInput("multipolygon has no components".to_string()));
    }

    let mut best: Option<MicResult> = None;
    let mut last_error: Option<MicError> = None;

    for (idx, poly) in multi.0.iter().enumerate() {
        match maximum_inscribed_circle(poly, opts) {
            Ok(mut result) => {
                result.component_index = Some(idx);
                let replace = best
                    .as_ref()
                    .map(|current| result.radius_sq > current.radius_sq)
                    .unwrap_or(true);
                if replace {
                    best = Some(result);
                }
            }
            Err(err) => {
                last_error = Some(err);
            }
        }
    }

    best.ok_or_else(|| last_error.unwrap_or(MicError::NoCircleFound))
}
```

</details>



### `ige-core::solvers::mic::solve_on_host_polygon`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn solve_on_host_polygon (host : & HostPolygon , opts : & MicOptions ,) -> Result < MicResult , MicError >
```

<details>
<summary>Source</summary>

```rust
fn solve_on_host_polygon(
    host: &HostPolygon,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    // Phase 0: Analytical fast paths — exact O(1), no workspace needed.
    // Verification: compute the true nearest-boundary distance and compare.
    // Reject if the analytical result is not within 1% of the true distance
    // (catches degenerate inputs that match ring-length checks but are not
    // valid triangles/quads — e.g., repeated vertices, near-zero areas).
    if opts.engine == MicEngine::ExactOnly || opts.engine == MicEngine::ExactThenGeos {
        'fast: {
            let result = if let Some(r) = solver::exact::fast_triangle(host) { r }
            else if let Some(r) = solver::exact::fast_convex_quad(host) { r }
            else { break 'fast; };

            // Verify: compute exact nearest-boundary distance via linear scan
            let seg_idx = input::SegmentIndex::from_host(host);
            let mut actual_sq = f64::INFINITY;
            for idx in 0..seg_idx.len() {
                let d = seg_idx.point_segment_distance_sq(idx, result.center.x(), result.center.y());
                if d < actual_sq { actual_sq = d; }
            }
            let actual = actual_sq.sqrt();
            if actual > 0.0 && (result.radius - actual).abs() / actual < 0.01 {
                return Ok(result);
            }
        }
    }

    match opts.engine {
        MicEngine::ExactOnly => run_exact(host, opts),
        MicEngine::FallbackOnly => run_geos(host, None, opts),
        MicEngine::ExactThenGeos => {
            let mut workspace = match MicWorkspace::new(host.clone()) {
                Ok(w) => w,
                Err(e) => {
                    #[cfg(feature = "geos")]
                    { return run_geos(host, None, opts).map_err(|fe| MicError::GeosFailed(format!("workspace failed ({e}); fallback failed ({fe})"))); }
                    #[cfg(not(feature = "geos"))]
                    { return Err(e); }
                }
            };
            match solve_exact(&mut workspace, opts) {
                Ok(result) => Ok(result),
                Err(e) => {
                    #[cfg(feature = "geos")]
                    {
                        let seg_index = workspace.seg_index.clone();
                        run_geos(host, Some(&seg_index), opts).map_err(|fallback_err| {
                            MicError::GeosFailed(format!("exact failed ({e}); fallback failed ({fallback_err})"))
                        })
                    }
                    #[cfg(not(feature = "geos"))]
                    {
                        Err(MicError::ExactFailed(e.to_string()))
                    }
                }
            }
        }
    }
}
```

</details>



### `ige-core::solvers::mic::run_exact`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn run_exact (host : & HostPolygon , opts : & MicOptions ,) -> Result < MicResult , MicError >
```

<details>
<summary>Source</summary>

```rust
fn run_exact(
    host: &HostPolygon,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    let mut workspace = MicWorkspace::new(host.clone())?;
    solve_exact(&mut workspace, opts).map_err(|err| MicError::ExactFailed(err.to_string()))
}
```

</details>



### `ige-core::solvers::mic::run_geos`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn run_geos (host : & HostPolygon , existing_seg_index : Option < & input :: SegmentIndex > , opts : & MicOptions ,) -> Result < MicResult , MicError >
```

<details>
<summary>Source</summary>

```rust
fn run_geos(
    host: &HostPolygon,
    existing_seg_index: Option<&input::SegmentIndex>,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    #[cfg(feature = "geos")]
    {
        self::solver::geos_fallback::solve_with_geos(host, opts, existing_seg_index)
    }
    #[cfg(not(feature = "geos"))]
    {
        let _ = (host, existing_seg_index, opts);
        Err(MicError::GeosFeatureDisabled)
    }
}
```

</details>



