# ige-core::solvers::lir::oriented::edge_anchor <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Edge-anchored candidate generation for oriented LIR.

This module generates candidate rectangles from boundary support relationships,
complementing the center-driven pipeline for cases where the optimal rectangle
is edge-supported rather than center-driven.
Core idea: Work in the rotated coordinate frame where the candidate rectangle
is axis-aligned, then generate rectangles from three fast support families:
1. Paired vertical supports: left/right sides anchored to boundary events
2. Paired horizontal supports: bottom/top sides anchored to boundary events
3. Single-edge anchored slide-grow: pin one rectangle side to a dominant boundary chain

## Structs

### `ige-core::solvers::lir::oriented::edge_anchor::EdgeCandidate`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


#### Fields

| Name | Type | Description |
|------|------|-------------|
| `rect_rot` | `(f64 , f64 , f64 , f64)` |  |
| `angle` | `f64` |  |
| `area` | `f64` |  |
| `support_score` | `f64` |  |
| `validity_score` | `f64` |  |
| `origin` | `EdgeOrigin` |  |



### `ige-core::solvers::lir::oriented::edge_anchor::RotFrame`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


#### Fields

| Name | Type | Description |
|------|------|-------------|
| `poly` | `Polygon < f64 >` |  |
| `xs` | `Vec < f64 >` |  |
| `ys` | `Vec < f64 >` |  |
| `x_events` | `Vec < f64 >` |  |
| `y_events` | `Vec < f64 >` |  |
| `bbox` | `(f64 , f64 , f64 , f64)` |  |



## Enums

### `ige-core::solvers::lir::oriented::edge_anchor::EdgeOrigin` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


#### Variants

- **`VerticalPair`**
- **`HorizontalPair`**
- **`SingleSideAnchor`**



### `ige-core::solvers::lir::oriented::edge_anchor::EdgeChain` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


#### Variants

- **`Vertical`**
- **`Horizontal`**



## Functions

### `ige-core::solvers::lir::oriented::edge_anchor::rotate_polygon_to_frame`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn rotate_polygon_to_frame (poly : & Polygon < f64 > , angle_deg : f64) -> Polygon < f64 >
```

<details>
<summary>Source</summary>

```rust
fn rotate_polygon_to_frame(poly: &Polygon<f64>, angle_deg: f64) -> Polygon<f64> {
    let centroid: Point<f64> = poly.centroid().map(|c| c.into()).unwrap_or(Point::new(0.0, 0.0));
    let rad = -angle_deg.to_radians();
    let cos_a = rad.cos();
    let sin_a = rad.sin();
    let cx = centroid.x();
    let cy = centroid.y();

    let rotate = |c: &Coord<f64>| -> Coord<f64> {
        let dx = c.x - cx;
        let dy = c.y - cy;
        Coord {
            x: cx + dx * cos_a - dy * sin_a,
            y: cy + dx * sin_a + dy * cos_a,
        }
    };

    let ext_coords: Vec<Coord<f64>> = poly.exterior().0.iter().map(&rotate).collect();
    let interiors: Vec<geo_types::LineString<f64>> = poly
        .interiors()
        .iter()
        .map(|ring| {
            geo_types::LineString::from(ring.0.iter().map(&rotate).collect::<Vec<_>>())
        })
        .collect();

    Polygon::new(geo_types::LineString::from(ext_coords), interiors)
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::build_rot_frame`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn build_rot_frame (poly : & Polygon < f64 > , angle_deg : f64) -> RotFrame
```

<details>
<summary>Source</summary>

```rust
fn build_rot_frame(poly: &Polygon<f64>, angle_deg: f64) -> RotFrame {
    let rot_poly = rotate_polygon_to_frame(poly, angle_deg);

    let mut xs_raw: Vec<f64> = rot_poly.exterior().0.iter().map(|c| c.x).collect();
    let mut ys_raw: Vec<f64> = rot_poly.exterior().0.iter().map(|c| c.y).collect();

    for ring in rot_poly.interiors() {
        for c in &ring.0 {
            xs_raw.push(c.x);
            ys_raw.push(c.y);
        }
    }

    let bbox = rot_poly.bounding_rect().unwrap();
    xs_raw.push(bbox.min().x);
    xs_raw.push(bbox.max().x);
    ys_raw.push(bbox.min().y);
    ys_raw.push(bbox.max().y);

    xs_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());

    xs_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
    ys_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-12);

    let mut x_events = xs_raw.clone();
    let mut y_events = ys_raw.clone();

    for i in 0..xs_raw.len().saturating_sub(1) {
        let mid = (xs_raw[i] + xs_raw[i + 1]) * 0.5;
        x_events.push(mid);
    }
    for i in 0..ys_raw.len().saturating_sub(1) {
        let mid = (ys_raw[i] + ys_raw[i + 1]) * 0.5;
        y_events.push(mid);
    }

    x_events.sort_by(|a, b| a.partial_cmp(b).unwrap());
    y_events.sort_by(|a, b| a.partial_cmp(b).unwrap());

    x_events.dedup_by(|a, b| (*a - *b).abs() < 1e-10);
    y_events.dedup_by(|a, b| (*a - *b).abs() < 1e-10);

    RotFrame {
        poly: rot_poly,
        xs: xs_raw,
        ys: ys_raw,
        x_events,
        y_events,
        bbox: (bbox.min().x, bbox.min().y, bbox.max().x, bbox.max().y),
    }
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::vertical_line_intervals`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn vertical_line_intervals (poly : & Polygon < f64 > , x : f64) -> Vec < (f64 , f64) >
```

<details>
<summary>Source</summary>

```rust
fn vertical_line_intervals(poly: &Polygon<f64>, x: f64) -> Vec<(f64, f64)> {
    let bbox = match poly.bounding_rect() {
        Some(b) => b,
        None => return Vec::new(),
    };

    let y_min = bbox.min().y;
    let y_max = bbox.max().y;
    if y_max <= y_min {
        return Vec::new();
    }

    let mut inside_points: Vec<f64> = Vec::new();
    let n_samples = 100;
    let step = (y_max - y_min) / (n_samples as f64);

    for i in 0..=n_samples {
        let y = y_min + step * (i as f64);
        let sdf = polygon_sdf(poly, x, y);
        if sdf < 0.0 {
            inside_points.push(y);
        }
    }

    if inside_points.is_empty() {
        return Vec::new();
    }

    inside_points.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let mut intervals = Vec::new();
    let mut start: Option<f64> = None;
    let mut prev: f64 = inside_points[0];

    for &y in &inside_points[1..] {
        if (y - prev) > step * 1.5 {
            if let Some(s) = start {
                if prev > s + 1e-10 {
                    intervals.push((s, prev));
                }
            }
            start = Some(y);
        } else {
            start = start.or(Some(prev));
        }
        prev = y;
    }

    if let Some(s) = start {
        if prev > s + 1e-10 {
            intervals.push((s, prev));
        }
    }

    intervals
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::horizontal_line_intervals`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn horizontal_line_intervals (poly : & Polygon < f64 > , y : f64) -> Vec < (f64 , f64) >
```

<details>
<summary>Source</summary>

```rust
fn horizontal_line_intervals(poly: &Polygon<f64>, y: f64) -> Vec<(f64, f64)> {
    let bbox = match poly.bounding_rect() {
        Some(b) => b,
        None => return Vec::new(),
    };

    let x_min = bbox.min().x;
    let x_max = bbox.max().x;
    if x_max <= x_min {
        return Vec::new();
    }

    let mut inside_points: Vec<f64> = Vec::new();
    let n_samples = 100;
    let step = (x_max - x_min) / (n_samples as f64);

    for i in 0..=n_samples {
        let x = x_min + step * (i as f64);
        let sdf = polygon_sdf(poly, x, y);
        if sdf < 0.0 {
            inside_points.push(x);
        }
    }

    if inside_points.is_empty() {
        return Vec::new();
    }

    inside_points.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let mut intervals = Vec::new();
    let mut start: Option<f64> = None;
    let mut prev: f64 = inside_points[0];

    for &x in &inside_points[1..] {
        if (x - prev) > step * 1.5 {
            if let Some(s) = start {
                if prev > s + 1e-10 {
                    intervals.push((s, prev));
                }
            }
            start = Some(x);
        } else {
            start = start.or(Some(prev));
        }
        prev = x;
    }

    if let Some(s) = start {
        if prev > s + 1e-10 {
            intervals.push((s, prev));
        }
    }

    intervals
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::common_y_interval_for_x_span`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn common_y_interval_for_x_span (poly : & Polygon < f64 > , x0 : f64 , x1 : f64 , probes : usize ,) -> Option < (f64 , f64) >
```

<details>
<summary>Source</summary>

```rust
fn common_y_interval_for_x_span(
    poly: &Polygon<f64>,
    x0: f64,
    x1: f64,
    probes: usize,
) -> Option<(f64, f64)> {
    if x1 <= x0 {
        return None;
    }

    let xs: Vec<f64> = if probes == 3 {
        vec![x0, (x0 + x1) * 0.5, x1]
    } else {
        let step = (x1 - x0) / (probes - 1) as f64;
        (0..probes).map(|i| x0 + step * i as f64).collect()
    };

    let mut common_intervals: Vec<(f64, f64)> = Vec::new();

    for &x in &xs {
        let intervals = vertical_line_intervals(poly, x);
        if intervals.is_empty() {
            return None;
        }
        if common_intervals.is_empty() {
            common_intervals = intervals;
        } else {
            let mut new_common = Vec::new();
            for &(a_lo, a_hi) in &common_intervals {
                for &(b_lo, b_hi) in &intervals {
                    let lo = a_lo.max(b_lo);
                    let hi = a_hi.min(b_hi);
                    if hi > lo {
                        new_common.push((lo, hi));
                    }
                }
            }
            if new_common.is_empty() {
                return None;
            }
            common_intervals = new_common;
        }
    }

    common_intervals
        .into_iter()
        .max_by(|a, b| {
            let a_span = a.1 - a.0;
            let b_span = b.1 - b.0;
            a_span.partial_cmp(&b_span).unwrap_or(std::cmp::Ordering::Equal)
        })
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::common_x_interval_for_y_span`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn common_x_interval_for_y_span (poly : & Polygon < f64 > , y0 : f64 , y1 : f64 , probes : usize ,) -> Option < (f64 , f64) >
```

<details>
<summary>Source</summary>

```rust
fn common_x_interval_for_y_span(
    poly: &Polygon<f64>,
    y0: f64,
    y1: f64,
    probes: usize,
) -> Option<(f64, f64)> {
    if y1 <= y0 {
        return None;
    }

    let ys: Vec<f64> = if probes == 3 {
        vec![y0, (y0 + y1) * 0.5, y1]
    } else {
        let step = (y1 - y0) / (probes - 1) as f64;
        (0..probes).map(|i| y0 + step * i as f64).collect()
    };

    let mut common_intervals: Vec<(f64, f64)> = Vec::new();

    for &y in &ys {
        let intervals = horizontal_line_intervals(poly, y);
        if intervals.is_empty() {
            return None;
        }
        if common_intervals.is_empty() {
            common_intervals = intervals;
        } else {
            let mut new_common = Vec::new();
            for &(a_lo, a_hi) in &common_intervals {
                for &(b_lo, b_hi) in &intervals {
                    let lo = a_lo.max(b_lo);
                    let hi = a_hi.min(b_hi);
                    if hi > lo {
                        new_common.push((lo, hi));
                    }
                }
            }
            if new_common.is_empty() {
                return None;
            }
            common_intervals = new_common;
        }
    }

    common_intervals
        .into_iter()
        .max_by(|a, b| {
            let a_span = a.1 - a.0;
            let b_span = b.1 - b.0;
            a_span.partial_cmp(&b_span).unwrap_or(std::cmp::Ordering::Equal)
        })
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::rect_covers`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn rect_covers (poly : & Polygon < f64 > , x0 : f64 , y0 : f64 , x1 : f64 , y1 : f64) -> bool
```

<details>
<summary>Source</summary>

```rust
fn rect_covers(poly: &Polygon<f64>, x0: f64, y0: f64, x1: f64, y1: f64) -> bool {
    if x1 - x0 < 1e-10 || y1 - y0 < 1e-10 {
        return false;
    }

    let corners = [
        Point::new(x0, y0),
        Point::new(x1, y0),
        Point::new(x1, y1),
        Point::new(x0, y1),
    ];

    if !corners.iter().all(|p| poly.contains(p)) {
        return false;
    }

    true
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::compute_support_score`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn compute_support_score (poly : & Polygon < f64 > , x0 : f64 , y0 : f64 , x1 : f64 , y1 : f64) -> f64
```

<details>
<summary>Source</summary>

```rust
fn compute_support_score(poly: &Polygon<f64>, x0: f64, y0: f64, x1: f64, y1: f64) -> f64 {
    let bbox = match poly.bounding_rect() {
        Some(b) => b,
        None => return 0.0,
    };

    let diag = ((bbox.max().x - bbox.min().x).powi(2) + (bbox.max().y - bbox.min().y).powi(2)).sqrt();
    let threshold = diag * 0.015;

    let sdf_l = polygon_sdf(poly, x0, (y0 + y1) * 0.5).abs();
    let sdf_r = polygon_sdf(poly, x1, (y0 + y1) * 0.5).abs();
    let sdf_b = polygon_sdf(poly, (x0 + x1) * 0.5, y0).abs();
    let sdf_t = polygon_sdf(poly, (x0 + x1) * 0.5, y1).abs();

    let near = [sdf_l, sdf_r, sdf_b, sdf_t]
        .iter()
        .filter(|&&d| d < threshold)
        .count() as f64;

    near / 4.0
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::generate_vertical_pair_candidates`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn generate_vertical_pair_candidates (frame : & RotFrame , max_ratio : f64 , _min_ratio : f64 , current_best_area : f64 , top_k : usize ,) -> Vec < EdgeCandidate >
```

<details>
<summary>Source</summary>

```rust
fn generate_vertical_pair_candidates(
    frame: &RotFrame,
    max_ratio: f64,
    _min_ratio: f64,
    current_best_area: f64,
    top_k: usize,
) -> Vec<EdgeCandidate> {
    let mut candidates = Vec::new();
    let (minx, miny, maxx, maxy) = frame.bbox;
    let diag = ((maxx - minx).powi(2) + (maxy - miny).powi(2)).sqrt();

    let min_width = diag * 0.02;

    let x_events = &frame.x_events;
    let max_pairs = (top_k * 2).min(x_events.len().saturating_sub(1));

    for i in 0..x_events.len().saturating_sub(1).min(max_pairs) {
        for j in (i + 1)..x_events.len().min(i + 4).min(x_events.len()) {
            let x_l = x_events[i];
            let x_r = x_events[j];

            if x_r - x_l < min_width {
                continue;
            }

            if max_ratio > 0.0 {
                let max_height = (x_r - x_l) / max_ratio;
                if let Some((y0, y1)) = common_y_interval_for_x_span(&frame.poly, x_l, x_r, 3) {
                    let span = y1 - y0;
                    if span > max_height * 1.5 {
                        continue;
                    }
                }
            }

            if let Some((y0, y1)) = common_y_interval_for_x_span(&frame.poly, x_l, x_r, 3) {
                let span = y1 - y0;
                if span < min_width {
                    continue;
                }

                if !rect_covers(&frame.poly, x_l, y0, x_r, y1) {
                    if let Some((y0_r, y1_r)) = common_y_interval_for_x_span(&frame.poly, x_l, x_r, 5) {
                        if !rect_covers(&frame.poly, x_l, y0_r, x_r, y1_r) {
                            continue;
                        }
                        let area = (x_r - x_l) * (y1_r - y0_r);
                        if area < current_best_area * 0.5 {
                            continue;
                        }
                        let support = compute_support_score(&frame.poly, x_l, y0_r, x_r, y1_r);
                        candidates.push(EdgeCandidate {
                            rect_rot: (x_l, y0_r, x_r, y1_r),
                            angle: 0.0,
                            area,
                            support_score: support,
                            validity_score: 1.0,
                            origin: EdgeOrigin::VerticalPair,
                        });
                    }
                    continue;
                }

                let area = (x_r - x_l) * span;
                if area < current_best_area * 0.3 {
                    continue;
                }

                let support = compute_support_score(&frame.poly, x_l, y0, x_r, y1);
                candidates.push(EdgeCandidate {
                    rect_rot: (x_l, y0, x_r, y1),
                    angle: 0.0,
                    area,
                    support_score: support,
                    validity_score: 1.0,
                    origin: EdgeOrigin::VerticalPair,
                });
            }
        }
    }

    candidates.sort_by(|a, b| {
        let score_a = a.area * (1.0 + a.support_score * 0.3);
        let score_b = b.area * (1.0 + b.support_score * 0.3);
        score_b.partial_cmp(&score_a).unwrap_or(std::cmp::Ordering::Equal)
    });

    candidates.truncate(top_k);
    candidates
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::generate_horizontal_pair_candidates`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn generate_horizontal_pair_candidates (frame : & RotFrame , max_ratio : f64 , _min_ratio : f64 , current_best_area : f64 , top_k : usize ,) -> Vec < EdgeCandidate >
```

<details>
<summary>Source</summary>

```rust
fn generate_horizontal_pair_candidates(
    frame: &RotFrame,
    max_ratio: f64,
    _min_ratio: f64,
    current_best_area: f64,
    top_k: usize,
) -> Vec<EdgeCandidate> {
    let mut candidates = Vec::new();
    let (minx, miny, maxx, maxy) = frame.bbox;
    let diag = ((maxx - minx).powi(2) + (maxy - miny).powi(2)).sqrt();

    let min_height = diag * 0.02;

    let y_events = &frame.y_events;
    let max_pairs = (top_k * 2).min(y_events.len().saturating_sub(1));

    for i in 0..y_events.len().saturating_sub(1).min(max_pairs) {
        for j in (i + 1)..y_events.len().min(i + 4).min(y_events.len()) {
            let y_b = y_events[i];
            let y_t = y_events[j];

            if y_t - y_b < min_height {
                continue;
            }

            if max_ratio > 0.0 {
                let max_width = (y_t - y_b) / max_ratio;
                if let Some((x0, x1)) = common_x_interval_for_y_span(&frame.poly, y_b, y_t, 3) {
                    let span = x1 - x0;
                    if span > max_width * 1.5 {
                        continue;
                    }
                }
            }

            if let Some((x0, x1)) = common_x_interval_for_y_span(&frame.poly, y_b, y_t, 3) {
                let span = x1 - x0;
                if span < min_height {
                    continue;
                }

                if !rect_covers(&frame.poly, x0, y_b, x1, y_t) {
                    if let Some((x0_r, x1_r)) = common_x_interval_for_y_span(&frame.poly, y_b, y_t, 5) {
                        if !rect_covers(&frame.poly, x0_r, y_b, x1_r, y_t) {
                            continue;
                        }
                        let area = (x1_r - x0_r) * (y_t - y_b);
                        if area < current_best_area * 0.5 {
                            continue;
                        }
                        let support = compute_support_score(&frame.poly, x0_r, y_b, x1_r, y_t);
                        candidates.push(EdgeCandidate {
                            rect_rot: (x0_r, y_b, x1_r, y_t),
                            angle: 0.0,
                            area,
                            support_score: support,
                            validity_score: 1.0,
                            origin: EdgeOrigin::HorizontalPair,
                        });
                    }
                    continue;
                }

                let area = span * (y_t - y_b);
                if area < current_best_area * 0.3 {
                    continue;
                }

                let support = compute_support_score(&frame.poly, x0, y_b, x1, y_t);
                candidates.push(EdgeCandidate {
                    rect_rot: (x0, y_b, x1, y_t),
                    angle: 0.0,
                    area,
                    support_score: support,
                    validity_score: 1.0,
                    origin: EdgeOrigin::HorizontalPair,
                });
            }
        }
    }

    candidates.sort_by(|a, b| {
        let score_a = a.area * (1.0 + a.support_score * 0.3);
        let score_b = b.area * (1.0 + b.support_score * 0.3);
        score_b.partial_cmp(&score_a).unwrap_or(std::cmp::Ordering::Equal)
    });

    candidates.truncate(top_k);
    candidates
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::generate_single_side_anchor_candidates`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn generate_single_side_anchor_candidates (frame : & RotFrame , max_ratio : f64 , min_ratio : f64 , current_best_area : f64 , top_k : usize ,) -> Vec < EdgeCandidate >
```

<details>
<summary>Source</summary>

```rust
fn generate_single_side_anchor_candidates(
    frame: &RotFrame,
    max_ratio: f64,
    min_ratio: f64,
    current_best_area: f64,
    top_k: usize,
) -> Vec<EdgeCandidate> {
    let mut candidates = Vec::new();
    let (minx, miny, maxx, maxy) = frame.bbox;
    let diag = ((maxx - minx).powi(2) + (maxy - miny).powi(2)).sqrt();

    let min_dim = diag * 0.02;

    let left_candidates = generate_side_anchored_at_x(&frame.poly, minx, miny, maxy, min_dim, max_ratio, min_ratio, current_best_area, &frame.x_events);
    candidates.extend(left_candidates);

    let right_candidates = generate_side_anchored_at_x(&frame.poly, maxx, miny, maxy, min_dim, max_ratio, min_ratio, current_best_area, &frame.x_events);
    candidates.extend(right_candidates);

    let bottom_candidates = generate_side_anchored_at_y(&frame.poly, minx, maxx, miny, min_dim, max_ratio, min_ratio, current_best_area, &frame.y_events);
    candidates.extend(bottom_candidates);

    let top_candidates = generate_side_anchored_at_y(&frame.poly, minx, maxx, maxy, min_dim, max_ratio, min_ratio, current_best_area, &frame.y_events);
    candidates.extend(top_candidates);

    for edge_chain in extract_dominant_chains(&frame.poly) {
        match edge_chain {
            EdgeChain::Vertical { x, y_min, y_max } => {
                let chain_candidates = generate_side_anchored_at_x(&frame.poly, x, y_min, y_max, min_dim, max_ratio, min_ratio, current_best_area, &frame.x_events);
                candidates.extend(chain_candidates);
            }
            EdgeChain::Horizontal { y, x_min, x_max } => {
                let chain_candidates = generate_side_anchored_at_y(&frame.poly, x_min, x_max, y, min_dim, max_ratio, min_ratio, current_best_area, &frame.y_events);
                candidates.extend(chain_candidates);
            }
        }
    }

    candidates.sort_by(|a, b| {
        let score_a = a.area * (1.0 + a.support_score * 0.3);
        let score_b = b.area * (1.0 + b.support_score * 0.3);
        score_b.partial_cmp(&score_a).unwrap_or(std::cmp::Ordering::Equal)
    });

    candidates.truncate(top_k);
    candidates
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::extract_dominant_chains`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn extract_dominant_chains (poly : & Polygon < f64 >) -> Vec < EdgeChain >
```

<details>
<summary>Source</summary>

```rust
fn extract_dominant_chains(poly: &Polygon<f64>) -> Vec<EdgeChain> {
    let mut chains = Vec::new();
    let diag = poly.bounding_rect().map(|b| {
        ((b.max().x - b.min().x).powi(2) + (b.max().y - b.min().y).powi(2)).sqrt()
    }).unwrap_or(1.0);

    let min_length = diag * 0.15;

    for ring in std::iter::once(poly.exterior()).chain(poly.interiors()) {
        let coords = ring.0.as_slice();
        for w in coords.windows(2) {
            let dx = w[1].x - w[0].x;
            let dy = w[1].y - w[0].y;
            let len = dx.hypot(dy);

            if len > min_length {
                if dx.abs() < 1e-6 || dy.abs() < dy.abs() {
                    let x = (w[0].x + w[1].x) * 0.5;
                    let y_min = w[0].y.min(w[1].y);
                    let y_max = w[0].y.max(w[1].y);
                    chains.push(EdgeChain::Vertical { x, y_min, y_max });
                } else {
                    let y = (w[0].y + w[1].y) * 0.5;
                    let x_min = w[0].x.min(w[1].x);
                    let x_max = w[0].x.max(w[1].x);
                    chains.push(EdgeChain::Horizontal { y, x_min, x_max });
                }
            }
        }
    }

    chains.truncate(8);
    chains
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::generate_side_anchored_at_x`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn generate_side_anchored_at_x (poly : & Polygon < f64 > , x_anchor : f64 , _y_min : f64 , _y_max : f64 , min_dim : f64 , _max_ratio : f64 , _min_ratio : f64 , current_best_area : f64 , _x_events : & [f64] ,) -> Vec < EdgeCandidate >
```

<details>
<summary>Source</summary>

```rust
fn generate_side_anchored_at_x(
    poly: &Polygon<f64>,
    x_anchor: f64,
    _y_min: f64,
    _y_max: f64,
    min_dim: f64,
    _max_ratio: f64,
    _min_ratio: f64,
    current_best_area: f64,
    _x_events: &[f64],
) -> Vec<EdgeCandidate> {
    let mut candidates = Vec::new();

    if let Some((y0, y1)) = common_y_interval_for_x_span(poly, x_anchor, x_anchor + min_dim * 2.0, 3) {
        let span = y1 - y0;
        if span < min_dim {
            return candidates;
        }

        let x_max = poly.bounding_rect().map(|b| b.max().x).unwrap_or(x_anchor + 100.0);
        let mut x1 = x_anchor + min_dim;
        while x1 < x_max {
            if let Some((fy0, fy1)) = common_y_interval_for_x_span(poly, x_anchor, x1, 3) {
                let fspan = fy1 - fy0;
                if fspan < span * 0.95 {
                    break;
                }

                if rect_covers(poly, x_anchor, fy0, x1, fy1) {
                    let area = (x1 - x_anchor) * fspan;
                    if area > current_best_area * 0.3 {
                        let support = compute_support_score(poly, x_anchor, fy0, x1, fy1);
                        candidates.push(EdgeCandidate {
                            rect_rot: (x_anchor, fy0, x1, fy1),
                            angle: 0.0,
                            area,
                            support_score: support,
                            validity_score: 0.9,
                            origin: EdgeOrigin::SingleSideAnchor,
                        });
                    }
                }
            }
            x1 += min_dim;
        }
    }

    if let Some((y0, y1)) = common_y_interval_for_x_span(poly, x_anchor - min_dim * 2.0, x_anchor, 3) {
        let span = y1 - y0;
        if span < min_dim {
            return candidates;
        }

        let x_min = poly.bounding_rect().map(|b| b.min().x).unwrap_or(x_anchor - 100.0);
        let mut x0 = x_anchor - min_dim;
        while x0 > x_min {
            if let Some((fy0, fy1)) = common_y_interval_for_x_span(poly, x0, x_anchor, 3) {
                let fspan = fy1 - fy0;
                if fspan < span * 0.95 {
                    break;
                }

                if rect_covers(poly, x0, fy0, x_anchor, fy1) {
                    let area = (x_anchor - x0) * fspan;
                    if area > current_best_area * 0.3 {
                        let support = compute_support_score(poly, x0, fy0, x_anchor, fy1);
                        candidates.push(EdgeCandidate {
                            rect_rot: (x0, fy0, x_anchor, fy1),
                            angle: 0.0,
                            area,
                            support_score: support,
                            validity_score: 0.9,
                            origin: EdgeOrigin::SingleSideAnchor,
                        });
                    }
                }
            }
            x0 -= min_dim;
        }
    }

    candidates
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::generate_side_anchored_at_y`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn generate_side_anchored_at_y (poly : & Polygon < f64 > , _x_min : f64 , _x_max : f64 , y_anchor : f64 , min_dim : f64 , _max_ratio : f64 , _min_ratio : f64 , current_best_area : f64 , _y_events : & [f64] ,) -> Vec < EdgeCandidate >
```

<details>
<summary>Source</summary>

```rust
fn generate_side_anchored_at_y(
    poly: &Polygon<f64>,
    _x_min: f64,
    _x_max: f64,
    y_anchor: f64,
    min_dim: f64,
    _max_ratio: f64,
    _min_ratio: f64,
    current_best_area: f64,
    _y_events: &[f64],
) -> Vec<EdgeCandidate> {
    let mut candidates = Vec::new();

    if let Some((x0, x1)) = common_x_interval_for_y_span(poly, y_anchor, y_anchor + min_dim * 2.0, 3) {
        let span = x1 - x0;
        if span < min_dim {
            return candidates;
        }

        let y_max = poly.bounding_rect().map(|b| b.max().y).unwrap_or(y_anchor + 100.0);
        let mut y1 = y_anchor + min_dim;
        while y1 < y_max {
            if let Some((fx0, fx1)) = common_x_interval_for_y_span(poly, y_anchor, y1, 3) {
                let fspan = fx1 - fx0;
                if fspan < span * 0.95 {
                    break;
                }

                if rect_covers(poly, fx0, y_anchor, fx1, y1) {
                    let area = fspan * (y1 - y_anchor);
                    if area > current_best_area * 0.3 {
                        let support = compute_support_score(poly, fx0, y_anchor, fx1, y1);
                        candidates.push(EdgeCandidate {
                            rect_rot: (fx0, y_anchor, fx1, y1),
                            angle: 0.0,
                            area,
                            support_score: support,
                            validity_score: 0.9,
                            origin: EdgeOrigin::SingleSideAnchor,
                        });
                    }
                }
            }
            y1 += min_dim;
        }
    }

    if let Some((x0, x1)) = common_x_interval_for_y_span(poly, y_anchor - min_dim * 2.0, y_anchor, 3) {
        let span = x1 - x0;
        if span < min_dim {
            return candidates;
        }

        let y_min = poly.bounding_rect().map(|b| b.min().y).unwrap_or(y_anchor - 100.0);
        let mut y0 = y_anchor - min_dim;
        while y0 > y_min {
            if let Some((fx0, fx1)) = common_x_interval_for_y_span(poly, y0, y_anchor, 3) {
                let fspan = fx1 - fx0;
                if fspan < span * 0.95 {
                    break;
                }

                if rect_covers(poly, fx0, y0, fx1, y_anchor) {
                    let area = fspan * (y_anchor - y0);
                    if area > current_best_area * 0.3 {
                        let support = compute_support_score(poly, fx0, y0, fx1, y_anchor);
                        candidates.push(EdgeCandidate {
                            rect_rot: (fx0, y0, fx1, y_anchor),
                            angle: 0.0,
                            area,
                            support_score: support,
                            validity_score: 0.9,
                            origin: EdgeOrigin::SingleSideAnchor,
                        });
                    }
                }
            }
            y0 -= min_dim;
        }
    }

    candidates
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::angle_diff`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn angle_diff (a : f64 , b : f64) -> f64
```

<details>
<summary>Source</summary>

```rust
fn angle_diff(a: f64, b: f64) -> f64 {
    let diff = (a - b).abs();
    diff.min(90.0 - diff)
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::center_distance`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn center_distance (a : & (f64 , f64 , f64 , f64) , b : & (f64 , f64 , f64 , f64)) -> f64
```

<details>
<summary>Source</summary>

```rust
fn center_distance(a: &(f64, f64, f64, f64), b: &(f64, f64, f64, f64)) -> f64 {
    let cx_a = (a.0 + a.2) * 0.5;
    let cy_a = (a.1 + a.3) * 0.5;
    let cx_b = (b.0 + b.2) * 0.5;
    let cy_b = (b.1 + b.3) * 0.5;
    ((cx_a - cx_b).powi(2) + (cy_a - cy_b).powi(2)).sqrt()
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::rect_iou`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn rect_iou (a : & (f64 , f64 , f64 , f64) , b : & (f64 , f64 , f64 , f64)) -> f64
```

<details>
<summary>Source</summary>

```rust
fn rect_iou(a: &(f64, f64, f64, f64), b: &(f64, f64, f64, f64)) -> f64 {
    let x0 = a.0.max(b.0);
    let y0 = a.1.max(b.1);
    let x1 = a.2.min(b.2);
    let y1 = a.3.min(b.3);

    if x1 <= x0 || y1 <= y0 {
        return 0.0;
    }

    let inter_area = (x1 - x0) * (y1 - y0);
    let area_a = (a.2 - a.0) * (a.3 - a.1);
    let area_b = (b.2 - b.0) * (b.3 - b.1);
    let union_area = area_a + area_b - inter_area;

    if union_area <= 0.0 {
        return 0.0;
    }

    inter_area / union_area
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::similar_rect`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn similar_rect (a : & EdgeCandidate , b : & EdgeCandidate , diag : f64) -> bool
```

<details>
<summary>Source</summary>

```rust
fn similar_rect(a: &EdgeCandidate, b: &EdgeCandidate, diag: f64) -> bool {
    angle_diff(a.angle, b.angle) < 0.75
        && center_distance(&a.rect_rot, &b.rect_rot) < 0.03 * diag
        && rect_iou(&a.rect_rot, &b.rect_rot) > 0.85
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::deduplicate_candidates`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn deduplicate_candidates (candidates : & mut Vec < EdgeCandidate > , diag : f64)
```

<details>
<summary>Source</summary>

```rust
fn deduplicate_candidates(candidates: &mut Vec<EdgeCandidate>, diag: f64) {
    let mut unique: Vec<EdgeCandidate> = Vec::new();

    for cand in candidates.drain(..) {
        let is_dup = unique.iter().any(|u| similar_rect(&cand, u, diag));
        if !is_dup {
            unique.push(cand);
        }
    }

    candidates.extend(unique);
}
```

</details>



### `ige-core::solvers::lir::oriented::edge_anchor::generate_edge_anchored_candidates`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn generate_edge_anchored_candidates (poly : & Polygon < f64 > , angle_deg : f64 , options : & LirOrientedOptions , current_best_area : f64 ,) -> Vec < EdgeCandidate >
```

<details>
<summary>Source</summary>

```rust
pub fn generate_edge_anchored_candidates(
    poly: &Polygon<f64>,
    angle_deg: f64,
    options: &LirOrientedOptions,
    current_best_area: f64,
) -> Vec<EdgeCandidate> {
    let frame = build_rot_frame(poly, angle_deg);

    let (minx, miny, maxx, maxy) = frame.bbox;
    let diag = ((maxx - minx).powi(2) + (maxy - miny).powi(2)).sqrt();

    let threshold = if current_best_area > 0.0 { current_best_area } else { diag * diag * 0.25 };

    let top_k = 12;

    let mut vertical_candidates = generate_vertical_pair_candidates(
        &frame,
        options.max_ratio,
        options.min_ratio,
        threshold,
        top_k,
    );

    let mut horizontal_candidates = generate_horizontal_pair_candidates(
        &frame,
        options.max_ratio,
        options.min_ratio,
        threshold,
        top_k,
    );

    let mut side_anchor_candidates = generate_single_side_anchor_candidates(
        &frame,
        options.max_ratio,
        options.min_ratio,
        threshold,
        top_k,
    );

    let mut all_candidates: Vec<EdgeCandidate> = Vec::new();
    all_candidates.append(&mut vertical_candidates);
    all_candidates.append(&mut horizontal_candidates);
    all_candidates.append(&mut side_anchor_candidates);

    for cand in &mut all_candidates {
        cand.angle = angle_deg;
    }

    deduplicate_candidates(&mut all_candidates, diag);

    all_candidates.sort_by(|a, b| {
        let score_a = a.area * (1.0 + a.support_score * 0.3) * a.validity_score;
        let score_b = b.area * (1.0 + b.support_score * 0.3) * b.validity_score;
        score_b.partial_cmp(&score_a).unwrap_or(std::cmp::Ordering::Equal)
    });

    all_candidates.truncate(top_k * 2);
    all_candidates
}
```

</details>



