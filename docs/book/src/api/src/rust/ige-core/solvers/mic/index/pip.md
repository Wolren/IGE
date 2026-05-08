# ige-core::solvers::mic::index::pip <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


## Structs

### `ige-core::solvers::mic::index::pip::PipIndex`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Point-in-polygon check using winding number over ring data.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `coords` | `Vec < [f64 ; 2] >` |  |
| `rings` | `Vec < RingMeta >` |  |

#### Methods

##### `new` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn new (host : & HostPolygon) -> Self
```

<details>
<summary>Source</summary>

```rust
    pub fn new(host: &HostPolygon) -> Self {
        Self {
            coords: host.coords.clone(),
            rings: host.rings.clone(),
        }
    }
```

</details>



##### `contains_strict_xy` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn contains_strict_xy (& self , x : f64 , y : f64) -> bool
```

<details>
<summary>Source</summary>

```rust
    pub fn contains_strict_xy(&self, x: f64, y: f64) -> bool {
        for meta in &self.rings {
            let ring = &self.coords[meta.start..meta.end];
            let inside = point_in_ring(x, y, ring);
            if meta.is_hole {
                if inside {
                    return false;
                }
            } else {
                if !inside {
                    return false;
                }
            }
        }
        true
    }
```

</details>





## Functions

### `ige-core::solvers::mic::index::pip::point_in_ring`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn point_in_ring (x : f64 , y : f64 , ring : & [[f64 ; 2]]) -> bool
```

<details>
<summary>Source</summary>

```rust
fn point_in_ring(x: f64, y: f64, ring: &[[f64; 2]]) -> bool {
    let mut inside = false;
    let mut j = ring.len() - 1;
    for i in 0..ring.len() {
        let ai = ring[i];
        let aj = ring[j];
        let (ax, ay) = (ai[0], ai[1]);
        let (bx, by) = (aj[0], aj[1]);

        let crosses = (ay > y) != (by > y);
        if crosses {
            let x_intersect = (bx - ax) * (y - ay) / (by - ay) + ax;
            if x < x_intersect {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}
```

</details>



