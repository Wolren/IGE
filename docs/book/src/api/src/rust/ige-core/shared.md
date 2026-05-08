# ige-core::shared <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Shared types and utilities for LIR algorithms.

These types are used across all solver implementations.

## Structs

### `ige-core::shared::Rectangle`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `x_min` | `f64` |  |
| `y_min` | `f64` |  |
| `x_max` | `f64` |  |
| `y_max` | `f64` |  |

#### Methods

##### `area` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn area (& self) -> f64
```

<details>
<summary>Source</summary>

```rust
    pub fn area(&self) -> f64 {
        (self.x_max - self.x_min) * (self.y_max - self.y_min)
    }
```

</details>



##### `to_polygon` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn to_polygon (& self) -> Polygon < f64 >
```

<details>
<summary>Source</summary>

```rust
    pub fn to_polygon(&self) -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                Coord {
                    x: self.x_min,
                    y: self.y_min,
                },
                Coord {
                    x: self.x_max,
                    y: self.y_min,
                },
                Coord {
                    x: self.x_max,
                    y: self.y_max,
                },
                Coord {
                    x: self.x_min,
                    y: self.y_max,
                },
                Coord {
                    x: self.x_min,
                    y: self.y_min,
                },
            ]),
            vec![],
        )
    }
```

</details>





### `ige-core::shared::SolverOptions`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `rotation_degrees` | `f64` |  |
| `prefer_gpu` | `bool` |  |
| `force_cpu` | `bool` |  |
| `max_aspect_ratio` | `f64` |  |
| `gpu_threshold` | `usize` |  |



## Enums

### `ige-core::shared::PolygonType` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


#### Variants

- **`ConvexNoHoles`**
- **`ConvexWithHoles`**
- **`ConcaveNoHoles`**
- **`ConcaveWithHoles`**



### `ige-core::shared::SolverBackend` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


#### Variants

- **`Cpu`**
- **`Gpu`**



### `ige-core::shared::AlgorithmCategory` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


#### Variants

- **`AxisAligned`**
- **`Oriented`**



### `ige-core::shared::AlgorithmPrecision` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


#### Variants

- **`Exact`**
- **`Approx`**



### `ige-core::shared::AlgorithmSpeed` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


#### Variants

- **`Standard`**
- **`Fast`**



### `ige-core::shared::LirError` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


#### Variants

- **`InvalidPolygon`**
- **`NoRectangleFound`**
- **`GpuError`**
- **`NotSupported`**
- **`Internal`**



## Functions

### `ige-core::shared::rotate_polygon`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn rotate_polygon (poly : & Polygon < f64 > , angle_deg : f64) -> Polygon < f64 >
```

<details>
<summary>Source</summary>

```rust
pub fn rotate_polygon(poly: &Polygon<f64>, angle_deg: f64) -> Polygon<f64> {
    if angle_deg.abs() < 1e-9 {
        return poly.clone();
    }
    match poly.centroid() {
        Some(centroid) => rotate_polygon_around(poly, angle_deg, &centroid),
        None => poly.clone(),
    }
}
```

</details>



### `ige-core::shared::rotate_polygon_around`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn rotate_polygon_around (poly : & Polygon < f64 > , angle_deg : f64 , center : & Point < f64 > ,) -> Polygon < f64 >
```

<details>
<summary>Source</summary>

```rust
pub fn rotate_polygon_around(
    poly: &Polygon<f64>,
    angle_deg: f64,
    center: &Point<f64>,
) -> Polygon<f64> {
    let rad = angle_deg.to_radians();
    let cos_a = rad.cos();
    let sin_a = rad.sin();
    let cx = center.x();
    let cy = center.y();

    let rotate = |c: &Coord<f64>| Coord {
        x: cx + (c.x - cx) * cos_a - (c.y - cy) * sin_a,
        y: cy + (c.x - cx) * sin_a + (c.y - cy) * cos_a,
    };

    let ext = LineString::from(poly.exterior().0.iter().map(&rotate).collect::<Vec<_>>());
    let interiors: Vec<LineString<f64>> = poly
        .interiors()
        .iter()
        .map(|r| LineString::from(r.0.iter().map(&rotate).collect::<Vec<_>>()))
        .collect();

    Polygon::new(ext, interiors)
}
```

</details>



