# ige-core::solvers::lir::oriented::expand <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


SDF-guided boundary expansion.

Port of `_expand_rect_to_boundary` from `bcrs_fast_worker.py`.
Uses the signed-distance field at edge midpoints to bound binary search
for the maximum expansion of each side.

## Structs

### `ige-core::solvers::lir::oriented::expand::CoversIndex`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


Pre-built spatial index for fast rect-covers queries.

Builds once per `expand_rect_to_boundary` call. Sorts polygon edges by
their x-range minimum and uses binary search + AABB pre-filter to find
candidate edges that might intersect the query rect, instead of scanning
all N polygon edges for every binary search step.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `edges_a` | `Vec < Coord < f64 > >` |  |
| `edges_b` | `Vec < Coord < f64 > >` |  |
| `xmin` | `Vec < f64 >` |  |
| `xmax` | `Vec < f64 >` |  |
| `ymin` | `Vec < f64 >` |  |
| `ymax` | `Vec < f64 >` |  |
| `order` | `Vec < usize >` | Indices into the above arrays, sorted by `xmin`. |

#### Methods

##### `from_polygon` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn from_polygon (poly : & Polygon < f64 >) -> Self
```

<details>
<summary>Source</summary>

```rust
    fn from_polygon(poly: &Polygon<f64>) -> Self {
        let mut edges_a = Vec::new();
        let mut edges_b = Vec::new();

        let mut add_ring = |ring: &LineString<f64>| {
            let n = ring.0.len();
            for i in 0..n.saturating_sub(1) {
                edges_a.push(ring.0[i]);
                edges_b.push(ring.0[i + 1]);
            }
        };
        add_ring(poly.exterior());
        for hole in poly.interiors() {
            add_ring(hole);
        }

        let n = edges_a.len();
        let mut xmin = Vec::with_capacity(n);
        let mut xmax = Vec::with_capacity(n);
        let mut ymin = Vec::with_capacity(n);
        let mut ymax = Vec::with_capacity(n);
        for i in 0..n {
            let (a, b) = (edges_a[i], edges_b[i]);
            xmin.push(a.x.min(b.x));
            xmax.push(a.x.max(b.x));
            ymin.push(a.y.min(b.y));
            ymax.push(a.y.max(b.y));
        }

        let mut order: Vec<usize> = (0..n).collect();
        order.sort_unstable_by(|&i, &j| xmin[i].partial_cmp(&xmin[j]).unwrap());

        CoversIndex { edges_a, edges_b, xmin, xmax, ymin, ymax, order }
    }
```

</details>



##### `has_crossing` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn has_crossing (& self , x0 : f64 , y0 : f64 , x1 : f64 , y1 : f64) -> bool
```

Returns `true` if any polygon edge crosses one of the four rect edges.

<details>
<summary>Source</summary>

```rust
    fn has_crossing(&self, x0: f64, y0: f64, x1: f64, y1: f64) -> bool {
        if self.xmin.is_empty() {
            return false;
        }

        let rect_edges = [
            (Coord { x: x0, y: y0 }, Coord { x: x1, y: y0 }),
            (Coord { x: x1, y: y0 }, Coord { x: x1, y: y1 }),
            (Coord { x: x1, y: y1 }, Coord { x: x0, y: y1 }),
            (Coord { x: x0, y: y1 }, Coord { x: x0, y: y0 }),
        ];

        for &idx in &self.order {
            if self.xmin[idx] > x1 {
                break;
            }
            if self.xmax[idx] < x0 || self.ymax[idx] < y0 || self.ymin[idx] > y1 {
                continue;
            }
            let (a, b) = (self.edges_a[idx], self.edges_b[idx]);
            for &(ra, rb) in &rect_edges {
                if segments_intersect(ra, rb, a, b) {
                    return true;
                }
            }
        }
        false
    }
```

</details>





## Functions

### `ige-core::solvers::lir::oriented::expand::multi_probe_sdf_v`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn multi_probe_sdf_v (poly : & Polygon < f64 > , x_fixed : f64 , y_lo : f64 , y_hi : f64 , probes : usize ,) -> f64
```

Sample the SDF at `probes` points along a vertical line (fixed x, varying y) using recursive SDF evaluation: the Lipschitz property (|SDF(a)-SDF(b)| ≤ |a-b|) means a far-inside evaluation guarantees nearby probes are also inside, eliminating redundant distance computations.

Evaluates probes left-to-right.  After each evaluation at position yᵢ with
SDF = dᵢ, any probe within distance dᵢ of yᵢ is guaranteed inside (SDF > 0)
and is skipped.  The returned value is a conservative lower bound on the
minimum SDF across all probes — safe for use as the binary-search ceiling.

<details>
<summary>Source</summary>

```rust
fn multi_probe_sdf_v(
    poly: &Polygon<f64>,
    x_fixed: f64,
    y_lo: f64,
    y_hi: f64,
    probes: usize,
) -> f64 {
    if probes == 0 {
        return f64::MAX;
    }
    let span = y_hi - y_lo;
    let mut min_sdf = f64::MAX;
    let mut last_y = f64::NAN;
    let mut last_sdf = f64::NAN;

    for i in 0..probes {
        let t = (i as f64 + 0.5) / probes as f64;
        let y = y_lo + span * t;

        // Lipschitz skip: if last evaluated probe guarantees this point is inside
        if last_sdf.is_finite() {
            let dist = (y - last_y).abs();
            if last_sdf - dist > 0.0 {
                // Conservative bound: actual SDF(y) ≥ last_sdf - dist
                let bound = last_sdf - dist;
                if bound < min_sdf {
                    min_sdf = bound;
                }
                continue;
            }
        }

        let sdf = polygon_sdf(poly, x_fixed, y);
        last_y = y;
        last_sdf = sdf;
        if sdf < min_sdf {
            min_sdf = sdf;
        }
    }
    min_sdf
}
```

</details>



### `ige-core::solvers::lir::oriented::expand::multi_probe_sdf_h`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn multi_probe_sdf_h (poly : & Polygon < f64 > , y_fixed : f64 , x_lo : f64 , x_hi : f64 , probes : usize ,) -> f64
```

Sample the SDF at `probes` points along a horizontal line (fixed y, varying x) using the same Lipschitz-skip optimisation.

<details>
<summary>Source</summary>

```rust
fn multi_probe_sdf_h(
    poly: &Polygon<f64>,
    y_fixed: f64,
    x_lo: f64,
    x_hi: f64,
    probes: usize,
) -> f64 {
    if probes == 0 {
        return f64::MAX;
    }
    let span = x_hi - x_lo;
    let mut min_sdf = f64::MAX;
    let mut last_x = f64::NAN;
    let mut last_sdf = f64::NAN;

    for i in 0..probes {
        let t = (i as f64 + 0.5) / probes as f64;
        let x = x_lo + span * t;

        if last_sdf.is_finite() {
            let dist = (x - last_x).abs();
            if last_sdf - dist > 0.0 {
                let bound = last_sdf - dist;
                if bound < min_sdf {
                    min_sdf = bound;
                }
                continue;
            }
        }

        let sdf = polygon_sdf(poly, x, y_fixed);
        last_x = x;
        last_sdf = sdf;
        if sdf < min_sdf {
            min_sdf = sdf;
        }
    }
    min_sdf
}
```

</details>



### `ige-core::solvers::lir::oriented::expand::rect_covers`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn rect_covers (index : & CoversIndex , poly : & Polygon < f64 > , x0 : f64 , y0 : f64 , x1 : f64 , y1 : f64) -> bool
```

Proper geometric containment check: verifies all 4 corners are inside AND no rect edge intersects the polygon boundary. Equivalent to Shapely's `prep.covers(box(x0,y0,x1,y1))`.

Uses a pre-built spatial index for the edge-crossing test to avoid
O(n) ring traversal on every call.

<details>
<summary>Source</summary>

```rust
fn rect_covers(index: &CoversIndex, poly: &Polygon<f64>, x0: f64, y0: f64, x1: f64, y1: f64) -> bool {
    if x1 - x0 < 1e-12 || y1 - y0 < 1e-12 {
        return false;
    }

    // Stage 1: fast corner check (4 point-in-polygon tests)
    let corners = [
        Point::new(x0, y0),
        Point::new(x1, y0),
        Point::new(x1, y1),
        Point::new(x0, y1),
    ];
    if !corners.iter().all(|p| poly.contains(p)) {
        return false;
    }

    // Stage 2: use the spatial index for edge-crossing check
    !index.has_crossing(x0, y0, x1, y1)
}
```

</details>



### `ige-core::solvers::lir::oriented::expand::segments_intersect`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn segments_intersect (a : Coord < f64 > , b : Coord < f64 > , c : Coord < f64 > , d : Coord < f64 > ,) -> bool
```

<details>
<summary>Source</summary>

```rust
fn segments_intersect(
    a: Coord<f64>,
    b: Coord<f64>,
    c: Coord<f64>,
    d: Coord<f64>,
) -> bool {
    fn orientation(p: Coord<f64>, q: Coord<f64>, r: Coord<f64>) -> f64 {
        (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
    }

    let o1 = orientation(a, b, c);
    let o2 = orientation(a, b, d);
    let o3 = orientation(c, d, a);
    let o4 = orientation(c, d, b);

    // General case: segments straddle
    if o1 * o2 < 0.0 && o3 * o4 < 0.0 {
        return true;
    }

    // Collinear boundary cases -- consider as non-intersecting for containment
    // (the corner check already verifies endpoints are fine; collinear overlaps
    // on the boundary are acceptable for `covers`)
    false
}
```

</details>



### `ige-core::solvers::lir::oriented::expand::clamp_aspect_ratio`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn clamp_aspect_ratio (mut x0 : f64 , mut y0 : f64 , mut x1 : f64 , mut y1 : f64 , max_ratio : f64 , min_ratio : f64) -> (f64 , f64 , f64 , f64)
```

<details>
<summary>Source</summary>

```rust
fn clamp_aspect_ratio(mut x0: f64, mut y0: f64, mut x1: f64, mut y1: f64, max_ratio: f64, min_ratio: f64) -> (f64, f64, f64, f64) {
    let rw = x1 - x0;
    let rh = y1 - y0;
    if rw <= 0.0 || rh <= 0.0 {
        return (x0, y0, x1, y1);
    }
    let ls = rw.max(rh);
    let ss = rw.min(rh);
    let current_ratio = ls / ss;
    if max_ratio > 0.0 && current_ratio > max_ratio {
        let nl = ss * max_ratio;
        if rw >= rh {
            let cx = (x0 + x1) * 0.5;
            x0 = cx - nl * 0.5;
            x1 = cx + nl * 0.5;
        } else {
            let cy = (y0 + y1) * 0.5;
            y0 = cy - nl * 0.5;
            y1 = cy + nl * 0.5;
        }
    } else if min_ratio > 0.0 && current_ratio < min_ratio {
        let nl = ss * min_ratio;
        if rw >= rh {
            let cx = (x0 + x1) * 0.5;
            x0 = cx - nl * 0.5;
            x1 = cx + nl * 0.5;
        } else {
            let cy = (y0 + y1) * 0.5;
            y0 = cy - nl * 0.5;
            y1 = cy + nl * 0.5;
        }
    }
    (x0, y0, x1, y1)
}
```

</details>



### `ige-core::solvers::lir::oriented::expand::expand_rect_to_boundary`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn expand_rect_to_boundary (rot_poly : & Polygon < f64 > , x0 : f64 , y0 : f64 , x1 : f64 , y1 : f64 , max_ratio : f64 , min_ratio : f64 ,) -> (f64 , f64 , f64 , f64)
```

<details>
<summary>Source</summary>

```rust
pub fn expand_rect_to_boundary(
    rot_poly: &Polygon<f64>,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    max_ratio: f64,
    min_ratio: f64,
) -> (f64, f64, f64, f64) {
    // Build spatial index once for all rect_covers queries
    let idx = CoversIndex::from_polygon(rot_poly);

    let bb = match rot_poly.bounding_rect() {
        Some(b) => b,
        None => return (x0, y0, x1, y1),
    };
    let minx = bb.min().x;
    let miny = bb.min().y;
    let maxx = bb.max().x;
    let maxy = bb.max().y;

    let mut x0 = x0;
    let mut y0 = y0;
    let mut x1 = x1;
    let mut y1 = y1;

    // Shrink to valid start if seed slightly exceeds bounds
    if !rect_covers(&idx, rot_poly, x0, y0, x1, y1) {
        let cx_c = (x0 + x1) * 0.5;
        let cy_c = (y0 + y1) * 0.5;
        let hw = (x1 - x0) * 0.5;
        let hh = (y1 - y0) * 0.5;
        let mut lo = 0.0_f64;
        let mut hi = 1.0_f64;
        for _ in 0..36 {
            let mid = (lo + hi) * 0.5;
            if rect_covers(&idx, rot_poly, cx_c - hw * mid, cy_c - hh * mid, cx_c + hw * mid, cy_c + hh * mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        if lo < 1e-9 {
            return (x0, y0, x1, y1);
        }
        x0 = cx_c - hw * lo;
        y0 = cy_c - hh * lo;
        x1 = cx_c + hw * lo;
        y1 = cy_c + hh * lo;
    }

    for _ in 0..EXPAND_ITERS {
        let mut any_changed = false;

        // Sort sides by gap size (largest first) for faster convergence
        let gap_left = if x0 > minx { x0 - minx } else { 0.0 };
        let gap_right = if x1 < maxx { maxx - x1 } else { 0.0 };
        let gap_bottom = if y0 > miny { y0 - miny } else { 0.0 };
        let gap_top = if y1 < maxy { maxy - y1 } else { 0.0 };

        let mut expansions: [(usize, f64); 4] = [
            (0, gap_left), (1, gap_right), (2, gap_bottom), (3, gap_top),
        ];
        expansions.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        for &(side, _) in &expansions {
            match side {
                0 if x0 > minx => { // Left
                    let min_sdf = multi_probe_sdf_v(rot_poly, x0, y0, y1, SDF_PROBES);
                    let hi_d = if min_sdf < 0.0 { gap_left.min(min_sdf.abs()) } else { gap_left };
                    if hi_d > 1e-12 {
                        let mut lo_d = 0.0_f64;
                        let mut hi_d = hi_d;
                        for _ in 0..BINARY_STEPS {
                            let mid = (lo_d + hi_d) * 0.5;
                            if rect_covers(&idx, rot_poly, x0 - mid, y0, x1, y1) {
                                lo_d = mid;
                            } else {
                                hi_d = mid;
                            }
                        }
                        if lo_d > 1e-10 { x0 -= lo_d; any_changed = true; }
                    }
                }
                1 if x1 < maxx => { // Right
                    let min_sdf = multi_probe_sdf_v(rot_poly, x1, y0, y1, SDF_PROBES);
                    let hi_d = if min_sdf < 0.0 { gap_right.min(min_sdf.abs()) } else { gap_right };
                    if hi_d > 1e-12 {
                        let mut lo_d = 0.0_f64;
                        let mut hi_d = hi_d;
                        for _ in 0..BINARY_STEPS {
                            let mid = (lo_d + hi_d) * 0.5;
                            if rect_covers(&idx, rot_poly, x0, y0, x1 + mid, y1) {
                                lo_d = mid;
                            } else {
                                hi_d = mid;
                            }
                        }
                        if lo_d > 1e-10 { x1 += lo_d; any_changed = true; }
                    }
                }
                2 if y0 > miny => { // Bottom — fix y, vary x horizontally
                    let min_sdf = multi_probe_sdf_h(rot_poly, y0, x0, x1, SDF_PROBES);
                    let hi_d = if min_sdf < 0.0 { gap_bottom.min(min_sdf.abs()) } else { gap_bottom };
                    if hi_d > 1e-12 {
                        let mut lo_d = 0.0_f64;
                        let mut hi_d = hi_d;
                        for _ in 0..BINARY_STEPS {
                            let mid = (lo_d + hi_d) * 0.5;
                            if rect_covers(&idx, rot_poly, x0, y0 - mid, x1, y1) {
                                lo_d = mid;
                            } else {
                                hi_d = mid;
                            }
                        }
                        if lo_d > 1e-10 { y0 -= lo_d; any_changed = true; }
                    }
                }
                3 if y1 < maxy => { // Top
                    let min_sdf = multi_probe_sdf_h(rot_poly, y1, x0, x1, SDF_PROBES);
                    let hi_d = if min_sdf < 0.0 { gap_top.min(min_sdf.abs()) } else { gap_top };
                    if hi_d > 1e-12 {
                        let mut lo_d = 0.0_f64;
                        let mut hi_d = hi_d;
                        for _ in 0..BINARY_STEPS {
                            let mid = (lo_d + hi_d) * 0.5;
                            if rect_covers(&idx, rot_poly, x0, y0, x1, y1 + mid) {
                                lo_d = mid;
                            } else {
                                hi_d = mid;
                            }
                        }
                        if lo_d > 1e-10 { y1 += lo_d; any_changed = true; }
                    }
                }
                _ => {}
            }
        }

        if !any_changed { break; }
    }

    (x0, y0, x1, y1) = clamp_aspect_ratio(x0, y0, x1, y1, max_ratio, min_ratio);

    (x0, y0, x1, y1)
}
```

</details>



