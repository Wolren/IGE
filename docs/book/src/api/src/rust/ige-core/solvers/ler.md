# ige-core::solvers::ler <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Largest Empty Rectangle (LER) solvers.

LER finds the largest axis-aligned or oriented rectangle that fits inside
a polygon while remaining completely empty (containing no obstacles).
This is complementary to LIR (Largest Inscribed Rectangle).

## Structs

### `ige-core::solvers::ler::LerOptions`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Configuration for LER solvers.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `max_ratio` | `f64` | Max aspect ratio (longer/shorter side); 0.0 = unconstrained. |
| `min_ratio` | `f64` | Min aspect ratio (longer/shorter side); 0.0 = unconstrained. |
| `grid_coarse` | `usize` | Grid resolution for coarse search. |
| `top_k` | `usize` | Number of top candidates to refine. |
| `always_return` | `bool` | If true, return best-effort result even if certification fails. |



### `ige-core::solvers::ler::LerResult`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Result of an LER solve.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `rect` | `Option < Rectangle >` | The largest empty rectangle (axis-aligned bounding box). |
| `rect_polygon` | `Option < Polygon < f64 > >` | The oriented rectangle as a polygon (if oriented). |
| `area` | `f64` | Area of the empty rectangle. |
| `angle_deg` | `f64` | Rotation angle in degrees (for oriented version). |
| `best_effort` | `bool` | True if result is best-effort rather than certified. |

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
        }
    }
```

</details>





## Functions

### `ige-core::solvers::ler::solve_ler_axis_aligned`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_ler_axis_aligned (poly : & Polygon < f64 > , obstacles : & [Polygon < f64 >] , options : & LerOptions ,) -> Result < LerResult >
```

Solve largest empty rectangle with axis-aligned constraints.

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `poly` | `-` | Input polygon defining the free space |
| `obstacles` | `-` | Optional collection of obstacle polygons to avoid |
| `options` | `-` | Solver configuration |


**Returns:**

A `LerResult` with the largest empty rectangle.

<details>
<summary>Source</summary>

```rust
pub fn solve_ler_axis_aligned(
    poly: &Polygon<f64>,
    obstacles: &[Polygon<f64>],
    options: &LerOptions,
) -> Result<LerResult> {
    super::ler::axis_aligned::solve_ler_axis_aligned_exact(poly, obstacles, options)
}
```

</details>



### `ige-core::solvers::ler::solve_ler_oriented`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_ler_oriented (_poly : & Polygon < f64 > , _obstacles : & [Polygon < f64 >] , _options : & LerOptions ,) -> Result < LerResult >
```

Solve largest empty rectangle with free orientation.

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `poly` | `-` | Input polygon defining the free space |
| `obstacles` | `-` | Optional collection of obstacle polygons to avoid |
| `options` | `-` | Solver configuration |


**Returns:**

A `LerResult` with the largest empty rectangle.

<details>
<summary>Source</summary>

```rust
pub fn solve_ler_oriented(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerOptions,
) -> Result<LerResult> {
    Err(crate::shared::LirError::NotSupported("LER oriented not yet implemented".to_string()))
}
```

</details>



