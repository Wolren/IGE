# ige-core::solvers::nesting::general <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


General polygon nesting solver.

Solves the largest polygon (potentially concave) inside a general polygon.
This is the most general case and can handle containers with holes.

## Functions

### `ige-core::solvers::nesting::general::solve_nesting_general_morphological`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_nesting_general_morphological (_container : & Polygon < f64 > , _options : & NestingOptions ,) -> Result < NestingResult >
```

Solve general nesting using morphological approach.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_nesting_general_morphological(
    _container: &Polygon<f64>,
    _options: &NestingOptions,
) -> Result<NestingResult> {
    Err(crate::shared::LirError::NotSupported("Nesting general not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::nesting::general::solve_nesting_general_subdivision`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_nesting_general_subdivision (_container : & Polygon < f64 > , _options : & NestingOptions ,) -> Result < NestingResult >
```

Solve general nesting using subdivision approach.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_nesting_general_subdivision(
    _container: &Polygon<f64>,
    _options: &NestingOptions,
) -> Result<NestingResult> {
    Err(crate::shared::LirError::NotSupported("Nesting general subdivision not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::nesting::general::solve_nesting_general_skeleton`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_nesting_general_skeleton (_container : & Polygon < f64 > , _options : & NestingOptions ,) -> Result < NestingResult >
```

Solve general nesting using skeleton approach.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_nesting_general_skeleton(
    _container: &Polygon<f64>,
    _options: &NestingOptions,
) -> Result<NestingResult> {
    Err(crate::shared::LirError::NotSupported("Nesting general skeleton not yet implemented".to_string()))
}
```

</details>



