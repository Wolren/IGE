# ige-core::solvers::lir::axis_aligned::containment <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Geometric containment verification for axis-aligned rectangles.

Provides a full containment check (corners + edge-intersection test) and
a per-side binary contraction that guarantees the result is fully inside
the polygon while maximising area.

## Functions

### `ige-core::solvers::lir::axis_aligned::containment::segments_intersect`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn segments_intersect (a : Coord < f64 > , b : Coord < f64 > , c : Coord < f64 > , d : Coord < f64 >) -> bool
```

<details>
<summary>Source</summary>

```rust
fn segments_intersect(a: Coord<f64>, b: Coord<f64>, c: Coord<f64>, d: Coord<f64>) -> bool {
    fn orient(p: Coord<f64>, q: Coord<f64>, r: Coord<f64>) -> f64 {
        (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
    }
    let o1 = orient(a, b, c);
    let o2 = orient(a, b, d);
    let o3 = orient(c, d, a);
    let o4 = orient(c, d, b);
    o1 * o2 < 0.0 && o3 * o4 < 0.0
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::containment::ring_intersects_rect`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn ring_intersects_rect (ring : & LineString < f64 > , x0 : f64 , y0 : f64 , x1 : f64 , y1 : f64) -> bool
```

<details>
<summary>Source</summary>

```rust
fn ring_intersects_rect(ring: &LineString<f64>, x0: f64, y0: f64, x1: f64, y1: f64) -> bool {
    let n = ring.0.len();
    if n < 2 {
        return false;
    }
    let edges: [(Coord<f64>, Coord<f64>); 4] = [
        (Coord { x: x0, y: y0 }, Coord { x: x1, y: y0 }),
        (Coord { x: x1, y: y0 }, Coord { x: x1, y: y1 }),
        (Coord { x: x1, y: y1 }, Coord { x: x0, y: y1 }),
        (Coord { x: x0, y: y1 }, Coord { x: x0, y: y0 }),
    ];
    for i in 0..n - 1 {
        let p1 = ring.0[i];
        let p2 = ring.0[i + 1];
        for &(a, b) in edges.iter() {
            if segments_intersect(a, b, p1, p2) {
                return true;
            }
        }
    }
    false
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::containment::rect_fully_contained`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn rect_fully_contained (poly : & Polygon < f64 > , x0 : f64 , y0 : f64 , x1 : f64 , y1 : f64 ,) -> bool
```

Check that an axis-aligned rect is fully inside `poly`.

Stage 1 -- corners: all 4 corners must be inside (boundary accepted).
Stage 2 -- edge crossings: no rect edge may cross a polygon edge.
Stage 3 -- vertex containment: no polygon vertex may lie strictly inside
the rect (catches concave notches where boundary edges coincide
with rect edges).

<details>
<summary>Source</summary>

```rust
pub fn rect_fully_contained(
    poly: &Polygon<f64>,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
) -> bool {
    if x1 - x0 < 1e-12 || y1 - y0 < 1e-12 {
        return false;
    }

    // Stage 1: corners -- accept boundary points (distance <= ε)
    const ON_BOUNDARY: f64 = crate::tuning::CONTAIN_BOUNDARY_EPS;
    let corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
    if !corners.iter().all(|&(cx, cy)| {
        let pt = Point::new(cx, cy);
        poly.contains(&pt) || poly.euclidean_distance(&pt) <= ON_BOUNDARY
    }) {
        return false;
    }

    // Stage 2: rect edges vs polygon rings (boundary crossings)
    if ring_intersects_rect(poly.exterior(), x0, y0, x1, y1) {
        return false;
    }
    for interior in poly.interiors() {
        if ring_intersects_rect(interior, x0, y0, x1, y1) {
            return false;
        }
    }

    // Stage 3: no polygon vertex lies strictly inside the rect.
    // Handles concave notches where boundary edges coincide with rect edges
    // (corners and edge-midpoint checks pass but the notch penetrates the interior).
    if ring_has_interior_vertex(poly.exterior(), x0, y0, x1, y1) {
        return false;
    }
    for interior in poly.interiors() {
        if ring_has_interior_vertex(interior, x0, y0, x1, y1) {
            return false;
        }
    }

    true
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::containment::ring_has_interior_vertex`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn ring_has_interior_vertex (ring : & LineString < f64 > , x0 : f64 , y0 : f64 , x1 : f64 , y1 : f64) -> bool
```

<details>
<summary>Source</summary>

```rust
fn ring_has_interior_vertex(ring: &LineString<f64>, x0: f64, y0: f64, x1: f64, y1: f64) -> bool {
    for c in ring.0.iter() {
        if c.x > x0 && c.x < x1 && c.y > y0 && c.y < y1 {
            return true;
        }
    }
    false
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::containment::contract_rect_to_boundary`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn contract_rect_to_boundary (poly : & Polygon < f64 > , mut x0 : f64 , mut y0 : f64 , mut x1 : f64 , mut y1 : f64 ,) -> Option < (f64 , f64 , f64 , f64) >
```

Given a candidate rect that may overflow the polygon, contract each side independently to the first valid position using binary search.

Returns `Some((x0, y0, x1, y1))` or `None` if even a point-sized rect fails.

<details>
<summary>Source</summary>

```rust
pub fn contract_rect_to_boundary(
    poly: &Polygon<f64>,
    mut x0: f64,
    mut y0: f64,
    mut x1: f64,
    mut y1: f64,
) -> Option<(f64, f64, f64, f64)> {
    // Fast path: already valid
    if rect_fully_contained(poly, x0, y0, x1, y1) {
        return Some((x0, y0, x1, y1));
    }

    // Binary shrink from centre to find a valid starting point
    let cx = (x0 + x1) * 0.5;
    let cy = (y0 + y1) * 0.5;
    let hw = (x1 - x0) * 0.5;
    let hh = (y1 - y0) * 0.5;
    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;

    for _ in 0..32 {
        let s = (lo + hi) * 0.5;
        if rect_fully_contained(poly, cx - hw * s, cy - hh * s, cx + hw * s, cy + hh * s) {
            lo = s;
        } else {
            hi = s;
        }
    }

    if lo < 1e-9 {
        return None;
    }

    x0 = cx - hw * lo;
    y0 = cy - hh * lo;
    x1 = cx + hw * lo;
    y1 = cy + hh * lo;

    // Per-side binary expansion -- push each side outward until just before it would exit
    const ITER: usize = crate::tuning::CONTRACT_BINARY_ITERS;
    let bb = poly.bounding_rect()?;
    let minx = bb.min().x;
    let miny = bb.min().y;
    let maxx = bb.max().x;
    let maxy = bb.max().y;

    // Left
    if x0 > minx {
        let mut lo_d = 0.0_f64;
        let mut hi_d = x0 - minx;
        for _ in 0..ITER {
            let d = (lo_d + hi_d) * 0.5;
            if rect_fully_contained(poly, x0 - d, y0, x1, y1) {
                lo_d = d;
            } else {
                hi_d = d;
            }
        }
        x0 -= lo_d;
    }

    // Right
    if x1 < maxx {
        let mut lo_d = 0.0_f64;
        let mut hi_d = maxx - x1;
        for _ in 0..ITER {
            let d = (lo_d + hi_d) * 0.5;
            if rect_fully_contained(poly, x0, y0, x1 + d, y1) {
                lo_d = d;
            } else {
                hi_d = d;
            }
        }
        x1 += lo_d;
    }

    // Bottom
    if y0 > miny {
        let mut lo_d = 0.0_f64;
        let mut hi_d = y0 - miny;
        for _ in 0..ITER {
            let d = (lo_d + hi_d) * 0.5;
            if rect_fully_contained(poly, x0, y0 - d, x1, y1) {
                lo_d = d;
            } else {
                hi_d = d;
            }
        }
        y0 -= lo_d;
    }

    // Top
    if y1 < maxy {
        let mut lo_d = 0.0_f64;
        let mut hi_d = maxy - y1;
        for _ in 0..ITER {
            let d = (lo_d + hi_d) * 0.5;
            if rect_fully_contained(poly, x0, y0, x1, y1 + d) {
                lo_d = d;
            } else {
                hi_d = d;
            }
        }
        y1 += lo_d;
    }

    Some((x0, y0, x1, y1))
}
```

</details>



