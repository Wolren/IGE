# Solver Parameters

All parameters are tuned through the options structs. Default values come from `crates/ige-core/src/tuning.rs`.

## `LirOrientedOptions`

### Grid Resolution

| Parameter | Type | Default | Range | Effect |
|---|---|---|---|---|
| `grid_coarse` | `usize` | 32 | 4–256 | Scanline resolution for UB scoring and coarse LRIH |
| `grid_fine` | `usize` | 64 | 10–256 | Vertex-grid resolution for fine solve LRIH |
| `top_k` | `usize` | 20 | 1–100 | Candidates forwarded to fine solve |

### Aspect Ratio Constraints

| Parameter | Type | Default | Effect |
|---|---|---|---|
| `max_ratio` | `f64` | 0 (unlimited) | Maximum longer/shorter side ratio. 0 = no constraint. |
| `min_ratio` | `f64` | 0 (unlimited) | Minimum longer/shorter side ratio. 0 = no constraint. |

Ratio clamping is applied in `clamp_ratio_about_center` during LRIH and in `clamp_half_sides_to_ratio` during certification.

### Optional Stages

| Parameter | Type | Default | Description |
|---|---|---|---|
| `always_return` | `bool` | `true` | Return best-effort result if certification fails |
| `use_parallel_field` | `bool` | `false` | Enable local angle polish: ±0.75° around best angle at 16+ coarse resolution |
| `use_simulated_annealing` | `bool` | `false` | Run SA basin escape over top-6 candidates |
| `use_bootstrap_seeds` | `bool` | `false` | Run vertex-snapped + cross-ray + unconstrained centroid seeds at best angle |
| `use_pca_axes` | `bool` | `false` | Add PCA eigendecomposition angles to candidate set |
| `use_edge_anchored` | `bool` | `false` | Generate edge-support candidates at 8 angles around best angle |

### Angle Generation

| Parameter | Type | Default | Description |
|---|---|---|---|
| `field_min_angles` | `usize` | 45 | Minimum candidate angles before grid fill |
| `field_angle_step` | `usize` | 5 | Degrees between grid-fill angles |
| `field_max_coords` | `usize` | 3000 | Max distinct x or y coords before vertex grid falls back to uniform |

### Polish / Certification

| Parameter | Type | Default | Description |
|---|---|---|---|
| `polish_halwidth_deg` | `f64` | 1.0 | Half-width of Brent polish search window (degrees) |
| `polish_xatol_deg` | `f64` | 0.02 | Convergence tolerance for Brent polish |
| `prune_margin` | `f64` | 0.92 | UB pruning safety margin (angles with UB < margin × kth_area are skipped) |
| `angle_delta_deg` | `f64` | 0.5 | Angle offset around each polished angle |
| `top_trials` | `usize` | 2 | Number of angle variants to keep per candidate |
| `cert_eps` | `f64` | 1e-7 | SDF epsilon for certification: rect certified if max_SDF > eps |
| `cert_max_shrink` | `f64` | 0.20 | Maximum shrink fraction during certification (fraction of shorter half-side) |

### GPU (Optional)

| Parameter | Type | Feature | Description |
|---|---|---|---|
| `gpu_ctx` | `Option<Arc<GpuContext>>` | `gpu` | GPU context for accelerated SDF evaluation |

## `LirAxisAlignedOptions`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `max_ratio` | `f64` | 0 | Maximum width/height ratio |
| `min_ratio` | `f64` | 0 | Minimum width/height ratio |

## `MicOptions`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `max_iterations` | `usize` | 1000 | SDF descent iteration limit |
| `convergence_tol` | `f64` | 1e-6 | Gradient descent stops when step size < tol |
| `min_radius` | `f64` | 1e-9 | Minimum valid circle radius |

## Tuning Defaults Source

All defaults are centralized in `crates/ige-core/src/tuning.rs`:

```rust
pub const GRID_COARSE: usize = 32;
pub const GRID_FINE: usize = 64;
pub const TOP_K: usize = 20;
pub const POLISH_HALFWIDTH: f64 = 3.0;
pub const POLISH_XATOL: f64 = 0.02;
pub const PRUNE_MARGIN: f64 = 0.92;
pub const CERT_EPS: f64 = 1e-7;
pub const CERT_MAX_SHRINK: f64 = 0.20;
pub const FIELD_MIN_ANGLES: usize = 45;
pub const FIELD_ANGLE_STEP: usize = 5;
pub const FIELD_MAX_COORDS: usize = 3000;
pub const EXPAND_BINARY_STEPS: usize = 24;
```