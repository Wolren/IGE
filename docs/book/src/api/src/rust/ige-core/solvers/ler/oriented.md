# ige-core::solvers::ler::oriented <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Oriented Largest Empty Rectangle solver.

Finds the largest rectangle with free orientation that fits in the free space
of a polygon while avoiding obstacles.

## Functions

### `ige-core::solvers::ler::oriented::solve_ler_oriented_parallel`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_ler_oriented_parallel (_poly : & Polygon < f64 > , _obstacles : & [Polygon < f64 >] , _options : & LerOptions ,) -> Result < LerResult >
```

Solve oriented LER using parallel angle sweep.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_ler_oriented_parallel(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerOptions,
) -> Result<LerResult> {
    Err(crate::shared::LirError::NotSupported("LER oriented not yet implemented".to_string()))
}
```

</details>



### `ige-core::solvers::ler::oriented::solve_ler_oriented_refine`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_ler_oriented_refine (_poly : & Polygon < f64 > , _obstacles : & [Polygon < f64 >] , _options : & LerOptions ,) -> Result < LerResult >
```

Solve oriented LER using coarse-to-fine refinement.

This is a placeholder implementation.

<details>
<summary>Source</summary>

```rust
pub fn solve_ler_oriented_refine(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerOptions,
) -> Result<LerResult> {
    Err(crate::shared::LirError::NotSupported("LER oriented refine not yet implemented".to_string()))
}
```

</details>



