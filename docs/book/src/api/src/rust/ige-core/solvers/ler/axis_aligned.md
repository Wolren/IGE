# ige-core::solvers::ler::axis_aligned <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Axis-aligned Largest Empty Rectangle solver. Uses O(m² × k) sweep-line approach where m = x-candidates, k = obstacles.

## Structs

### `ige-core::solvers::ler::axis_aligned::Obstacle`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


**Derives:** `Clone`, `Copy`, `Debug`

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `x0` | `f64` |  |
| `x1` | `f64` |  |
| `y0` | `f64` |  |
| `y1` | `f64` |  |



## Functions

### `ige-core::solvers::ler::axis_aligned::poly_bbox`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn poly_bbox (poly : & Polygon < f64 >) -> Option < (f64 , f64 , f64 , f64) >
```

<details>
<summary>Source</summary>

```rust
fn poly_bbox(poly: &Polygon<f64>) -> Option<(f64, f64, f64, f64)> {
    let bb = poly.bounding_rect()?;
    Some((bb.min().x, bb.min().y, bb.max().x, bb.max().y))
}
```

</details>



### `ige-core::solvers::ler::axis_aligned::build_obstacles`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn build_obstacles (obstacles : & [Polygon < f64 >]) -> Vec < Obstacle >
```

<details>
<summary>Source</summary>

```rust
fn build_obstacles(obstacles: &[Polygon<f64>]) -> Vec<Obstacle> {
    obstacles.iter()
        .filter_map(|obs| {
            let bb = obs.bounding_rect()?;
            Some(Obstacle { x0: bb.min().x, x1: bb.max().x, y0: bb.min().y, y1: bb.max().y })
        })
        .take(MAX_OBSTACLES)
        .collect()
}
```

</details>



### `ige-core::solvers::ler::axis_aligned::aspect_ok`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn aspect_ok (w : f64 , h : f64 , opts : & LerOptions) -> bool
```

<details>
<summary>Source</summary>

```rust
fn aspect_ok(w: f64, h: f64, opts: &LerOptions) -> bool {
    if w < EPS || h < EPS { return false; }
    let (s, l) = (w.min(h), w.max(h));
    let r = l / s;
    if opts.max_ratio > 0.0 && r > opts.max_ratio * 1.000001 { return false; }
    if opts.min_ratio > 0.0 && r < opts.min_ratio * 0.999999 { return false; }
    true
}
```

</details>



### `ige-core::solvers::ler::axis_aligned::find_largest_y_gap`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn find_largest_y_gap (obs : & [Obstacle] , x0 : f64 , x1 : f64 , by0 : f64 , by1 : f64) -> Option < (f64 , f64) >
```

<details>
<summary>Source</summary>

```rust
fn find_largest_y_gap(obs: &[Obstacle], x0: f64, x1: f64, by0: f64, by1: f64) -> Option<(f64, f64)> {
    let mut intervals: Vec<(f64, f64)> = Vec::new();
    for o in obs {
        if o.x1 > x0 + EPS && o.x0 < x1 - EPS {
            intervals.push((o.y0, o.y1));
        }
    }
    if intervals.is_empty() { return Some((by0, by1)); }

    intervals.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let mut merged = vec![intervals[0]];
    for iv in intervals.iter().skip(1) {
        if iv.0 <= merged.last().unwrap().1 + EPS {
            let last = merged.pop().unwrap();
            merged.push((last.0, last.1.max(iv.1)));
        } else {
            merged.push(*iv);
        }
    }

    let mut gaps = Vec::new();
    let mut cur = merged[0];
    if cur.0 > by0 + EPS { gaps.push((by0, cur.0)); }
    for iv in merged.iter().skip(1) {
        gaps.push((cur.1, iv.0));
        cur = *iv;
    }
    if cur.1 < by1 - EPS { gaps.push((cur.1, by1)); }

    gaps.into_iter()
        .max_by(|a, b| (a.1 - a.0).partial_cmp(&(b.1 - b.0)).unwrap())
}
```

</details>



### `ige-core::solvers::ler::axis_aligned::collect_x_candidates`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn collect_x_candidates (poly : & Polygon < f64 > , obs : & [Obstacle]) -> Vec < f64 >
```

<details>
<summary>Source</summary>

```rust
fn collect_x_candidates(poly: &Polygon<f64>, obs: &[Obstacle]) -> Vec<f64> {
    let mut xs: Vec<f64> = Vec::new();

    for c in poly.exterior().coords() { xs.push(c.x); }
    for ring in poly.interiors() { for c in ring.coords() { xs.push(c.x); } }

    for o in obs {
        xs.push(o.x0);
        xs.push(o.x1);
    }

    if let Some((x0, _, x1, _)) = poly_bbox(poly) {
        xs.push(x0);
        xs.push(x1);
    }

    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs.dedup();
    xs.into_iter().take(MAX_CANDIDATES).collect()
}
```

</details>



### `ige-core::solvers::ler::axis_aligned::solve_ler_axis_aligned_exact`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_ler_axis_aligned_exact (poly : & Polygon < f64 > , obstacles : & [Polygon < f64 >] , options : & LerOptions) -> Result < LerResult >
```

<details>
<summary>Source</summary>

```rust
pub fn solve_ler_axis_aligned_exact(poly: &Polygon<f64>, obstacles: &[Polygon<f64>], options: &LerOptions) -> Result<LerResult> {
    let (bx0, by0, bx1, by1) = poly_bbox(poly).ok_or_else(|| LirError::InvalidPolygon("degenerate".into()))?;
    if bx1 - bx0 < EPS || by1 - by0 < EPS { return Ok(LerResult::empty()); }

    let obs = build_obstacles(obstacles);

    if obs.is_empty() {
        if aspect_ok(bx1 - bx0, by1 - by0, options) {
            let r = Rectangle { x_min: bx0, y_min: by0, x_max: bx1, y_max: by1 };
            return Ok(LerResult { area: r.area(), rect: Some(r), rect_polygon: Some(Rect::new(Coord { x: bx0, y: by0 }, Coord { x: bx1, y: by1 }).to_polygon()), angle_deg: 0.0, best_effort: false });
        }
        return Ok(LerResult::empty());
    }

    let xs = collect_x_candidates(poly, &obs);
    if xs.len() < 2 { return Ok(LerResult::empty()); }

    let mut best: Option<(f64, f64, f64, f64, f64)> = None;
    let mut best_area = 0.0;

    for i in 0..xs.len() {
        for j in (i + 1)..xs.len() {
            let x0 = xs[i];
            let x1 = xs[j];

            if x1 <= x0 + EPS { continue; }

            let Some((y0, y1)) = find_largest_y_gap(&obs, x0, x1, by0, by1) else { continue; };

            if y1 <= y0 + EPS { continue; }

            let w = x1 - x0;
            let h = y1 - y0;

            if !aspect_ok(w, h, options) { continue; }

            let area = w * h;
            if area > best_area + EPS {
                best_area = area;
                best = Some((x0, y0, x1, y1, area));
            }
        }
    }

    match best {
        Some((x0, y0, x1, y1, _)) => {
            let r = Rectangle { x_min: x0, y_min: y0, x_max: x1, y_max: y1 };
            let area = r.area();
            Ok(LerResult { area, rect: Some(r.clone()), rect_polygon: Some(Rect::new(Coord { x: r.x_min, y: r.y_min }, Coord { x: r.x_max, y: r.y_max }).to_polygon()), angle_deg: 0.0, best_effort: false })
        }
        None => Ok(LerResult::empty()),
    }
}
```

</details>



### `ige-core::solvers::ler::axis_aligned::solve_ler_axis_aligned_grid`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_ler_axis_aligned_grid (poly : & Polygon < f64 > , obstacles : & [Polygon < f64 >] , options : & LerOptions) -> Result < LerResult >
```

<details>
<summary>Source</summary>

```rust
pub fn solve_ler_axis_aligned_grid(poly: &Polygon<f64>, obstacles: &[Polygon<f64>], options: &LerOptions) -> Result<LerResult> {
    solve_ler_axis_aligned_exact(poly, obstacles, options)
}
```

</details>



