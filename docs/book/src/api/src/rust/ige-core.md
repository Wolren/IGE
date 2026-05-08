# ige-core <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Inscribed Geometry Engine (IGE) - Largest Inscribed Rectangle algorithms

## Functions

### `ige-core::solve_oriented_lir`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_oriented_lir (poly : & Polygon < f64 >) -> Option < Rectangle >
```

<details>
<summary>Source</summary>

```rust
pub fn solve_oriented_lir(poly: &Polygon<f64>) -> Option<Rectangle> {
    solve_lir_oriented(poly, &LirOrientedOptions::default())
        .ok()
        .and_then(|r| r.rect)
}
```

</details>



### `ige-core::solve_axis_aligned`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_axis_aligned (poly : & Polygon < f64 > , options : & AxisAlignedOptions) -> Option < Rectangle >
```

<details>
<summary>Source</summary>

```rust
pub fn solve_axis_aligned(poly: &Polygon<f64>, options: &AxisAlignedOptions) -> Option<Rectangle> {
    solve_vertex_grid(poly, options)
}
```

</details>



