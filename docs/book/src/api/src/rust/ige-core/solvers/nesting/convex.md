# ige-core::solvers::nesting::convex <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Convex polygon nesting solver.

Solves the largest convex polygon inside a convex container.
This is simpler than the general case.

## Functions

### `ige-core::solvers::nesting::convex::solve_nesting_convex_offset`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_nesting_convex_offset (_container : & Polygon < f64 > , _options : & NestingOptions ,) -> Result < NestingResult >
```

Solve convex nesting using polygon offset approach.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_nesting_convex_offset(
    _container: &Polygon<f64>,
    _options: &NestingOptions,
) -> Result<NestingResult> {
    Err(crate::shared::LirError::NotSupported("Nesting convex not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::nesting::convex::solve_nesting_convex_vertex`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_nesting_convex_vertex (_container : & Polygon < f64 > , _options : & NestingOptions ,) -> Result < NestingResult >
```

Solve convex nesting using vertex insertion approach.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_nesting_convex_vertex(
    _container: &Polygon<f64>,
    _options: &NestingOptions,
) -> Result<NestingResult> {
    Err(crate::shared::LirError::NotSupported("Nesting convex vertex not yet implemented".to_string()))
}
```

</details>



