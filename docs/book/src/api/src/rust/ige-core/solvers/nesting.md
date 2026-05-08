# ige-core::solvers::nesting <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Nesting - Largest polygon inside polygon.

Finds the largest polygon that can be inscribed within a given polygon.
This is useful for nesting problems in manufacturing and layout.

## Structs

### `ige-core::solvers::nesting::NestingOptions`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Configuration for nesting solvers.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `max_ratio` | `f64` | Max aspect ratio for bounding box (longer/shorter side); 0.0 = unconstrained. |
| `min_ratio` | `f64` | Min aspect ratio for bounding box; 0.0 = unconstrained. |
| `max_vertices` | `usize` | Max vertices in output polygon (simplification). |
| `grid_coarse` | `usize` | Grid resolution for coarse search. |
| `prefer_convex` | `bool` | If true, prefer convex solutions. |



### `ige-core::solvers::nesting::NestingResult`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Result of a nesting solve.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `polygon` | `Option < Polygon < f64 > >` | The largest inscribed polygon. |
| `area` | `f64` | Area of the inscribed polygon. |
| `centroid` | `Option < geo_types :: Point < f64 > >` | Centroid of the inscribed polygon. |
| `fill_ratio` | `f64` | Fill ratio (area of inscribed / area of container). |

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
            centroid: None,
            fill_ratio: 0.0,
        }
    }
```

</details>





## Functions

### `ige-core::solvers::nesting::solve_nesting`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_nesting (_container : & Polygon < f64 > , _options : & NestingOptions ,) -> Result < NestingResult >
```

Solve largest polygon inside polygon (general case).

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `container` | `-` | The containing polygon |
| `options` | `-` | Solver configuration |


**Returns:**

A `NestingResult` with the largest inscribed polygon.

<details>
<summary>Source</summary>

```rust
pub fn solve_nesting(
    _container: &Polygon<f64>,
    _options: &NestingOptions,
) -> Result<NestingResult> {
    Err(crate::shared::LirError::NotSupported("Nesting not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::nesting::solve_nesting_convex`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_nesting_convex (_container : & Polygon < f64 > , _options : & NestingOptions ,) -> Result < NestingResult >
```

Solve largest convex polygon inside convex polygon.

This is a simpler case that can be solved more efficiently.

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `container` | `-` | The containing convex polygon |
| `options` | `-` | Solver configuration |


**Returns:**

A `NestingResult` with the largest inscribed convex polygon.

<details>
<summary>Source</summary>

```rust
pub fn solve_nesting_convex(
    _container: &Polygon<f64>,
    _options: &NestingOptions,
) -> Result<NestingResult> {
    Err(crate::shared::LirError::NotSupported("Nesting convex not yet implemented".to_string()))
}
```

</details>



