# ige-core::solvers::ler_lir::combined <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Combined LER + LIR solver implementation.

Finds both largest empty rectangle and largest inscribed rectangle
in a single pass for efficiency.

## Functions

### `ige-core::solvers::ler_lir::combined::solve_ler_lir_unified`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_ler_lir_unified (_poly : & Polygon < f64 > , _obstacles : & [Polygon < f64 >] , _options : & LerLirOptions ,) -> Result < LerLirResult >
```

Solve combined LER + LIR using unified angle sweep.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_ler_lir_unified(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerLirOptions,
) -> Result<LerLirResult> {
    Err(crate::shared::LirError::NotSupported("LER+LIR unified not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::ler_lir::combined::solve_ler_lir_grid`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_ler_lir_grid (_poly : & Polygon < f64 > , _obstacles : & [Polygon < f64 >] , _options : & LerLirOptions ,) -> Result < LerLirResult >
```

Solve combined LER + LIR using grid-based approach.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_ler_lir_grid(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerLirOptions,
) -> Result<LerLirResult> {
    Err(crate::shared::LirError::NotSupported("LER+LIR grid not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::ler_lir::combined::solve_ler_lir_axis_aligned_histogram`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_ler_lir_axis_aligned_histogram (_poly : & Polygon < f64 > , _obstacles : & [Polygon < f64 >] , _options : & LerLirOptions ,) -> Result < LerLirResult >
```

Solve combined LER + LIR axis-aligned using histogram approach.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_ler_lir_axis_aligned_histogram(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerLirOptions,
) -> Result<LerLirResult> {
    Err(crate::shared::LirError::NotSupported("LER+LIR axis-aligned histogram not yet implemented".to_string()))
}
```

</details>



