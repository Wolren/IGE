# ige-core::solvers::lir::oriented::certify <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Rectangle certification — SDF-based containment verification and shrink.

These functions verify that a candidate inscribed rectangle is fully inside
the polygon (using the signed-distance field from ``lir_axis_aligned::sdf``),
and if not, shrink it symmetrically until it is contained.

## Functions

### `ige-core::solvers::lir::oriented::certify::rect_local_frame`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn rect_local_frame (corners : & [(f64 , f64)]) -> Option < (f64 , f64 , f64 , f64 , f64 , f64 , f64 , f64) >
```

<details>
<summary>Source</summary>

```rust
fn rect_local_frame(corners: &[(f64, f64)]) -> Option<(f64, f64, f64, f64, f64, f64, f64, f64)> {
    if corners.len() < 5 { return None; }
    let p0 = (corners[0].0, corners[0].1);
    let p1 = (corners[1].0, corners[1].1);
    let p2 = (corners[2].0, corners[2].1);
    let e0 = (p1.0 - p0.0, p1.1 - p0.1);
    let e1 = (p2.0 - p1.0, p2.1 - p1.1);
    let l0 = (e0.0 * e0.0 + e0.1 * e0.1).sqrt();
    let l1 = (e1.0 * e1.0 + e1.1 * e1.1).sqrt();
    if l0 < 1e-14 || l1 < 1e-14 { return None; }
    let cx = (p0.0 + p2.0) / 2.0;
    let cy = (p0.1 + p2.1) / 2.0;
    let (ux, uy, vx, vy, a, b) = if l0 >= l1 {
        (e0.0 / l0, e0.1 / l0, e1.0 / l1, e1.1 / l1, l0 / 2.0, l1 / 2.0)
    } else {
        (e1.0 / l1, e1.1 / l1, e0.0 / l0, e0.1 / l0, l1 / 2.0, l0 / 2.0)
    };
    Some((cx, cy, ux, uy, vx, vy, a, b))
}
```

</details>



### `ige-core::solvers::lir::oriented::certify::build_rect_from_frame`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn build_rect_from_frame (cx : f64 , cy : f64 , ux : f64 , uy : f64 , vx : f64 , vy : f64 , a : f64 , b : f64) -> Polygon < f64 >
```

<details>
<summary>Source</summary>

```rust
fn build_rect_from_frame(cx: f64, cy: f64, ux: f64, uy: f64, vx: f64, vy: f64, a: f64, b: f64) -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            Coord { x: cx + a * ux + b * vx, y: cy + a * uy + b * vy },
            Coord { x: cx - a * ux + b * vx, y: cy - a * uy + b * vy },
            Coord { x: cx - a * ux - b * vx, y: cy - a * uy - b * vy },
            Coord { x: cx + a * ux - b * vx, y: cy + a * uy - b * vy },
            Coord { x: cx + a * ux + b * vx, y: cy + a * uy + b * vy },
        ]),
        vec![],
    )
}
```

</details>



### `ige-core::solvers::lir::oriented::certify::clamp_half_sides_to_ratio`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn clamp_half_sides_to_ratio (a : f64 , b : f64 , max_ratio : f64) -> (f64 , f64)
```

<details>
<summary>Source</summary>

```rust
fn clamp_half_sides_to_ratio(a: f64, b: f64, max_ratio: f64) -> (f64, f64) {
    if max_ratio <= 0.0 || a <= 0.0 || b <= 0.0 {
        return (a, b);
    }
    let (mut long, short) = if a >= b { (a, b) } else { (b, a) };
    if short > 0.0 && long / short > max_ratio {
        long = short * max_ratio;
    }
    if a >= b {
        (long, short)
    } else {
        (short, long)
    }
}
```

</details>



### `ige-core::solvers::lir::oriented::certify::rect_sdf_max_poly`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">pub(crate)</span>


```rust
fn rect_sdf_max_poly (poly : & Polygon < f64 > , rect : & Polygon < f64 >) -> f64
```

Maximum SDF at the 8 sample points (4 corners + 4 edge midpoints) of an oriented rect, expressed as a ``Polygon``.

<details>
<summary>Source</summary>

```rust
pub(crate) fn rect_sdf_max_poly(poly: &Polygon<f64>, rect: &Polygon<f64>) -> f64 {
    let coords: Vec<_> = rect.exterior().0.iter().cloned().collect();
    let n = coords.len();
    let mut best = polygon_sdf(poly, coords[0].x, coords[0].y);
    for i in 1..n.saturating_sub(1) {
        let v = polygon_sdf(poly, coords[i].x, coords[i].y);
        if v > best { best = v; }
        let mx = (coords[i - 1].x + coords[i].x) * 0.5;
        let my = (coords[i - 1].y + coords[i].y) * 0.5;
        let v = polygon_sdf(poly, mx, my);
        if v > best { best = v; }
    }
    best
}
```

</details>



### `ige-core::solvers::lir::oriented::certify::certify_and_adjust`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">pub(crate)</span>


```rust
fn certify_and_adjust (poly : & Polygon < f64 > , rect : & Polygon < f64 > , max_ratio : f64 , cert_eps : f64 , cert_max_shrink : f64 ,) -> Option < (Polygon < f64 > , f64) >
```

Certify that an oriented rect is fully inside ``poly``. If the max SDF exceeds ``cert_eps``, shrink symmetrically from centre up to ``cert_max_shrink`` fraction of the shorter half-side. Returns ``(certified_rect, area)`` or ``None`` if unfixable.

<details>
<summary>Source</summary>

```rust
pub(crate) fn certify_and_adjust(
    poly: &Polygon<f64>,
    rect: &Polygon<f64>,
    max_ratio: f64,
    cert_eps: f64,
    cert_max_shrink: f64,
) -> Option<(Polygon<f64>, f64)> {
    let max_sdf = rect_sdf_max_poly(poly, rect);
    let corners: Vec<(f64, f64)> = rect.exterior().0.iter().map(|c| (c.x, c.y)).collect();
    let frame = rect_local_frame(&corners)?;
    let (cx, cy, ux, uy, vx, vy, a, b) = frame;

    if max_sdf <= cert_eps {
        let (a0, b0) = clamp_half_sides_to_ratio(a, b, max_ratio);
        let final_rect = build_rect_from_frame(cx, cy, ux, uy, vx, vy, a0, b0);
        if rect_sdf_max_poly(poly, &final_rect) > cert_eps * 10.0 { return None; }
        let area = final_rect.unsigned_area();
        return Some((final_rect, area));
    }
    let shrink = max_sdf + cert_eps;
    if shrink > a.min(b) * cert_max_shrink { return None; }
    let new_a = a - shrink;
    let new_b = b - shrink;
    if new_a <= 0.0 || new_b <= 0.0 { return None; }
    let (new_a, new_b) = clamp_half_sides_to_ratio(new_a, new_b, max_ratio);
    let final_rect = build_rect_from_frame(cx, cy, ux, uy, vx, vy, new_a, new_b);
    if rect_sdf_max_poly(poly, &final_rect) > cert_eps * 10.0 { return None; }
    let area = final_rect.unsigned_area();
    Some((final_rect, area))
}
```

</details>



### `ige-core::solvers::lir::oriented::certify::best_effort_shrink_to_cover`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">pub(crate)</span>


```rust
fn best_effort_shrink_to_cover (poly : & Polygon < f64 > , rect : & Polygon < f64 > , max_ratio : f64 , cert_eps : f64 ,) -> Option < (Polygon < f64 > , f64) >
```

Best-effort variant that allows a slightly looser shrink (``max_sdf + cert_eps * 2.0``) and no max-shrink cap.

<details>
<summary>Source</summary>

```rust
pub(crate) fn best_effort_shrink_to_cover(
    poly: &Polygon<f64>,
    rect: &Polygon<f64>,
    max_ratio: f64,
    cert_eps: f64,
) -> Option<(Polygon<f64>, f64)> {
    let max_sdf = rect_sdf_max_poly(poly, rect);
    let corners: Vec<(f64, f64)> = rect.exterior().0.iter().map(|c| (c.x, c.y)).collect();
    let frame = rect_local_frame(&corners)?;
    let (cx, cy, ux, uy, vx, vy, a0, b0) = frame;
    if a0 <= 0.0 || b0 <= 0.0 { return None; }

    if max_sdf <= cert_eps {
        let (a, b) = clamp_half_sides_to_ratio(a0, b0, max_ratio);
        let final_rect = build_rect_from_frame(cx, cy, ux, uy, vx, vy, a, b);
        if rect_sdf_max_poly(poly, &final_rect) > cert_eps { return None; }
        let area = final_rect.unsigned_area();
        return Some((final_rect, area));
    }

    let shrink = max_sdf + cert_eps * 2.0;
    let a = a0 - shrink;
    let b = b0 - shrink;
    if a <= 0.0 || b <= 0.0 { return None; }
    let (a, b) = clamp_half_sides_to_ratio(a, b, max_ratio);
    if a <= 0.0 || b <= 0.0 { return None; }
    let final_rect = build_rect_from_frame(cx, cy, ux, uy, vx, vy, a, b);
    if rect_sdf_max_poly(poly, &final_rect) > cert_eps { return None; }
    let area = final_rect.unsigned_area();
    Some((final_rect, area))
}
```

</details>



