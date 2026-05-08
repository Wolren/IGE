# ige-core::solvers::ler_lir <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Combined LER + LIR solver.

Solves both Largest Empty Rectangle and Largest Inscribed Rectangle
in a single pass, which can be more efficient than running them separately.

## Structs

### `ige-core::solvers::ler_lir::LerLirOptions`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Configuration for combined LER + LIR solvers.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `max_ratio` | `f64` | Max aspect ratio for rectangles (longer/shorter side); 0.0 = unconstrained. |
| `min_ratio` | `f64` | Min aspect ratio for rectangles; 0.0 = unconstrained. |
| `grid_coarse` | `usize` | Grid resolution for coarse search. |
| `top_k` | `usize` | Number of top candidates to refine. |
| `always_return` | `bool` | If true, return best-effort result even if certification fails. |
| `axis_aligned_only` | `bool` | If true, solve for axis-aligned rectangles only. |



### `ige-core::solvers::ler_lir::LerLirResult`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Result of a combined LER + LIR solve.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `lir_rect` | `Option < Rectangle >` | LIR: Largest Inscribed Rectangle (axis-aligned bounding box). |
| `lir_polygon` | `Option < Polygon < f64 > >` | LIR: The inscribed rectangle as a polygon. |
| `lir_area` | `f64` | LIR: Area of the inscribed rectangle. |
| `lir_angle_deg` | `f64` | LIR: Rotation angle in degrees. |
| `ler_rect` | `Option < Rectangle >` | LER: Largest Empty Rectangle (axis-aligned bounding box). |
| `ler_polygon` | `Option < Polygon < f64 > >` | LER: The empty rectangle as a polygon. |
| `ler_area` | `f64` | LER: Area of the empty rectangle. |
| `ler_angle_deg` | `f64` | LER: Rotation angle in degrees. |
| `lir_best_effort` | `bool` | True if LIR result is best-effort. |
| `ler_best_effort` | `bool` | True if LER result is best-effort. |

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
            lir_rect: None,
            lir_polygon: None,
            lir_area: 0.0,
            lir_angle_deg: 0.0,
            ler_rect: None,
            ler_polygon: None,
            ler_area: 0.0,
            ler_angle_deg: 0.0,
            lir_best_effort: false,
            ler_best_effort: false,
        }
    }
```

</details>





## Functions

### `ige-core::solvers::ler_lir::solve_ler_lir`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_ler_lir (_poly : & Polygon < f64 > , _obstacles : & [Polygon < f64 >] , _options : & LerLirOptions ,) -> Result < LerLirResult >
```

Solve combined LER + LIR using parallel approach.

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `poly` | `-` | Input polygon |
| `obstacles` | `-` | Optional obstacle polygons for LER |
| `options` | `-` | Solver configuration |


**Returns:**

A `LerLirResult` with both LER and LIR results.

<details>
<summary>Source</summary>

```rust
pub fn solve_ler_lir(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerLirOptions,
) -> Result<LerLirResult> {
    Err(crate::shared::LirError::NotSupported("LER+LIR combined not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::ler_lir::solve_ler_lir_axis_aligned`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_ler_lir_axis_aligned (_poly : & Polygon < f64 > , _obstacles : & [Polygon < f64 >] , _options : & LerLirOptions ,) -> Result < LerLirResult >
```

Solve combined LER + LIR axis-aligned only.

This is simpler and faster than the oriented version.

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `poly` | `-` | Input polygon |
| `obstacles` | `-` | Optional obstacle polygons for LER |
| `options` | `-` | Solver configuration |


**Returns:**

A `LerLirResult` with both LER and LIR results.

<details>
<summary>Source</summary>

```rust
pub fn solve_ler_lir_axis_aligned(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerLirOptions,
) -> Result<LerLirResult> {
    Err(crate::shared::LirError::NotSupported("LER+LIR axis-aligned not yet implemented".to_string()))
}
```

</details>



