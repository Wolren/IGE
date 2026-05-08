# ige-core::solvers::obb <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Oriented Bounding Box (OBB) solver.

Finds the minimal area oriented bounding box that encloses a polygon.
Unlike LIR which finds the largest rectangle INSIDE a polygon,
OBB finds the smallest bounding box that CONTAINS the polygon.
This is useful for collision detection, packing, and shape analysis.

## Structs

### `ige-core::solvers::obb::ObbOptions`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Configuration for OBB solvers.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `max_ratio` | `f64` | Max aspect ratio (longer/shorter side); 0.0 = unconstrained. |
| `min_ratio` | `f64` | Min aspect ratio; 0.0 = unconstrained. |
| `angle_samples` | `usize` | Number of angle samples for search. |
| `use_pca` | `bool` | If true, use PCA for initial angle guess. |
| `use_refinement` | `bool` | If true, enable refinement after initial find. |
| `xatol_deg` | `f64` | Convergence tolerance for refinement (degrees). |



### `ige-core::solvers::obb::ObbResult`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Result of an OBB solve.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `polygon` | `Option < Polygon < f64 > >` | The oriented bounding box as a polygon. |
| `area` | `f64` | Area of the bounding box. |
| `perimeter` | `f64` | Perimeter of the bounding box. |
| `angle_deg` | `f64` | Rotation angle in degrees. |
| `width` | `f64` | Width of the bounding box. |
| `height` | `f64` | Height of the bounding box. |
| `centroid` | `Option < Point < f64 > >` | Centroid of the bounding box. |
| `aspect_ratio` | `f64` | Aspect ratio (width/height or height/width, whichever is larger). |
| `fill_ratio` | `f64` | Fill ratio (polygon area / OBB area). |

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
            polygon: None,
            area: 0.0,
            perimeter: 0.0,
            angle_deg: 0.0,
            width: 0.0,
            height: 0.0,
            centroid: None,
            aspect_ratio: 1.0,
            fill_ratio: 0.0,
        }
    }
```

</details>





## Functions

### `ige-core::solvers::obb::solve_obb`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_obb (_poly : & Polygon < f64 > , _options : & ObbOptions ,) -> Result < ObbResult >
```

Solve for the minimal oriented bounding box.

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `poly` | `-` | Input polygon |
| `options` | `-` | Solver configuration |


**Returns:**

An `ObbResult` with the minimal bounding box.

<details>
<summary>Source</summary>

```rust
pub fn solve_obb(
    _poly: &Polygon<f64>,
    _options: &ObbOptions,
) -> Result<ObbResult> {
    Err(crate::shared::LirError::NotSupported("OBB not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::obb::solve_obb_constrained`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_obb_constrained (_poly : & Polygon < f64 > , _options : & ObbOptions ,) -> Result < ObbResult >
```

Solve for minimal OBB with aspect ratio constraints.

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `poly` | `-` | Input polygon |
| `options` | `-` | Solver configuration (max_ratio and min_ratio will be applied) |


**Returns:**

An `ObbResult` with the constrained bounding box.

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



