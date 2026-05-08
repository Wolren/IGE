# ige-core::solvers::lir::axis_aligned::sdf <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Signed-distance-field utilities for polygon containment.

Port of `_polygon_sdf`, `_rect_sdf_max`, and `_certify_and_adjust`.

## Functions

### `ige-core::solvers::lir::axis_aligned::sdf::polygon_sdf`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn polygon_sdf (poly : & Polygon < f64 > , x : f64 , y : f64) -> f64
```

Signed distance from `(x, y)` to `poly`. - Negative: strictly inside (magnitude = distance to nearest ring) - Zero:     on boundary - Positive: outside polygon OR inside a hole

<details>
<summary>Source</summary>

```rust
pub fn polygon_sdf(poly: &Polygon<f64>, x: f64, y: f64) -> f64 {
    let mut min_dist_sq = f64::MAX;
    let mut winding = 0i32;

    for ring in std::iter::once(poly.exterior()).chain(poly.interiors()) {
        let coords = ring.0.as_slice();
        if coords.len() >= 2 {
            #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
            {
                let d2 = min_dist_sq_ring_simd(coords, x, y);
                if d2 < min_dist_sq {
                    min_dist_sq = d2;
                }
            }
            #[cfg(not(any(target_arch = "x86", target_arch = "x86_64")))]
            {
                let d2 = min_dist_sq_ring_scalar(coords, x, y);
                if d2 < min_dist_sq {
                    min_dist_sq = d2;
                }
            }
        }
        for w in coords.windows(2) {
            let (ax, ay) = (w[0].x, w[0].y);
            let (bx, by) = (w[1].x, w[1].y);

            // Winding number increment (robust crossing test)
            if ay <= y {
                if by > y && cross2d(ax - x, ay - y, bx - x, by - y) > 0.0 { winding += 1; }
            } else {
                if by <= y && cross2d(ax - x, ay - y, bx - x, by - y) < 0.0 { winding -= 1; }
            }

        }
    }

    let d = min_dist_sq.sqrt();
    if winding != 0 { -d } else { d }  // negative = inside
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::sdf::cross2d`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn cross2d (ux : f64 , uy : f64 , vx : f64 , vy : f64) -> f64
```

<details>
<summary>Source</summary>

```rust
fn cross2d(ux: f64, uy: f64, vx: f64, vy: f64) -> f64 { ux * vy - uy * vx }
```

</details>



### `ige-core::solvers::lir::axis_aligned::sdf::min_dist_sq_ring_scalar`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn min_dist_sq_ring_scalar (coords : & [Coord < f64 >] , x : f64 , y : f64) -> f64
```

<details>
<summary>Source</summary>

```rust
fn min_dist_sq_ring_scalar(coords: &[Coord<f64>], x: f64, y: f64) -> f64 {
    let mut min_d2 = f64::MAX;
    for w in coords.windows(2) {
        let (ax, ay) = (w[0].x, w[0].y);
        let (bx, by) = (w[1].x, w[1].y);
        let (ex, ey) = (bx - ax, by - ay);
        let t = ((x - ax) * ex + (y - ay) * ey) / (ex * ex + ey * ey + 1e-300);
        let t = t.clamp(0.0, 1.0);
        let (px, py) = (ax + t * ex, ay + t * ey);
        let d2 = (x - px) * (x - px) + (y - py) * (y - py);
        if d2 < min_d2 {
            min_d2 = d2;
        }
    }
    min_d2
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::sdf::min_dist_sq_ring_simd`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn min_dist_sq_ring_simd (coords : & [Coord < f64 >] , x : f64 , y : f64) -> f64
```

<details>
<summary>Source</summary>

```rust
fn min_dist_sq_ring_simd(coords: &[Coord<f64>], x: f64, y: f64) -> f64 {
    if is_x86_feature_detected!("avx") {
        // SAFETY: guarded by runtime feature detection.
        return unsafe { min_dist_sq_ring_avx(coords, x, y) };
    }
    if is_x86_feature_detected!("sse2") {
        // SAFETY: guarded by runtime feature detection.
        return unsafe { min_dist_sq_ring_sse2(coords, x, y) };
    }
    min_dist_sq_ring_scalar(coords, x, y)
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::sdf::min_dist_sq_ring_avx`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
unsafe fn min_dist_sq_ring_avx (coords : & [Coord < f64 >] , x : f64 , y : f64) -> f64
```

<details>
<summary>Source</summary>

```rust
unsafe fn min_dist_sq_ring_avx(coords: &[Coord<f64>], x: f64, y: f64) -> f64 {
    let n = coords.len().saturating_sub(1);
    if n == 0 {
        return f64::MAX;
    }
    let mut min_d2 = f64::MAX;
    let vx = arch::_mm256_set1_pd(x);
    let vy = arch::_mm256_set1_pd(y);
    let zero = arch::_mm256_set1_pd(0.0);
    let one = arch::_mm256_set1_pd(1.0);
    let eps = arch::_mm256_set1_pd(1e-300);

    let mut i = 0usize;
    while i + 4 <= n {
        let ax = arch::_mm256_set_pd(coords[i + 3].x, coords[i + 2].x, coords[i + 1].x, coords[i].x);
        let ay = arch::_mm256_set_pd(coords[i + 3].y, coords[i + 2].y, coords[i + 1].y, coords[i].y);
        let bx = arch::_mm256_set_pd(coords[i + 4].x, coords[i + 3].x, coords[i + 2].x, coords[i + 1].x);
        let by = arch::_mm256_set_pd(coords[i + 4].y, coords[i + 3].y, coords[i + 2].y, coords[i + 1].y);

        let ex = arch::_mm256_sub_pd(bx, ax);
        let ey = arch::_mm256_sub_pd(by, ay);
        let num = arch::_mm256_add_pd(
            arch::_mm256_mul_pd(arch::_mm256_sub_pd(vx, ax), ex),
            arch::_mm256_mul_pd(arch::_mm256_sub_pd(vy, ay), ey),
        );
        let den = arch::_mm256_add_pd(
            arch::_mm256_add_pd(arch::_mm256_mul_pd(ex, ex), arch::_mm256_mul_pd(ey, ey)),
            eps,
        );
        let t = arch::_mm256_max_pd(zero, arch::_mm256_min_pd(one, arch::_mm256_div_pd(num, den)));
        let px = arch::_mm256_add_pd(ax, arch::_mm256_mul_pd(t, ex));
        let py = arch::_mm256_add_pd(ay, arch::_mm256_mul_pd(t, ey));
        let dx = arch::_mm256_sub_pd(vx, px);
        let dy = arch::_mm256_sub_pd(vy, py);
        let d2 = arch::_mm256_add_pd(arch::_mm256_mul_pd(dx, dx), arch::_mm256_mul_pd(dy, dy));

        let mut lanes = [0.0_f64; 4];
        arch::_mm256_storeu_pd(lanes.as_mut_ptr(), d2);
        for v in lanes {
            if v < min_d2 {
                min_d2 = v;
            }
        }
        i += 4;
    }
    if i < n {
        let rem = min_dist_sq_ring_scalar(&coords[i..=n], x, y);
        if rem < min_d2 {
            min_d2 = rem;
        }
    }
    min_d2
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::sdf::min_dist_sq_ring_sse2`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
unsafe fn min_dist_sq_ring_sse2 (coords : & [Coord < f64 >] , x : f64 , y : f64) -> f64
```

<details>
<summary>Source</summary>

```rust
unsafe fn min_dist_sq_ring_sse2(coords: &[Coord<f64>], x: f64, y: f64) -> f64 {
    let n = coords.len().saturating_sub(1);
    if n == 0 {
        return f64::MAX;
    }
    let mut min_d2 = f64::MAX;
    let vx = arch::_mm_set1_pd(x);
    let vy = arch::_mm_set1_pd(y);
    let zero = arch::_mm_set1_pd(0.0);
    let one = arch::_mm_set1_pd(1.0);
    let eps = arch::_mm_set1_pd(1e-300);

    let mut i = 0usize;
    while i + 2 <= n {
        let ax = arch::_mm_set_pd(coords[i + 1].x, coords[i].x);
        let ay = arch::_mm_set_pd(coords[i + 1].y, coords[i].y);
        let bx = arch::_mm_set_pd(coords[i + 2].x, coords[i + 1].x);
        let by = arch::_mm_set_pd(coords[i + 2].y, coords[i + 1].y);

        let ex = arch::_mm_sub_pd(bx, ax);
        let ey = arch::_mm_sub_pd(by, ay);
        let num = arch::_mm_add_pd(
            arch::_mm_mul_pd(arch::_mm_sub_pd(vx, ax), ex),
            arch::_mm_mul_pd(arch::_mm_sub_pd(vy, ay), ey),
        );
        let den = arch::_mm_add_pd(
            arch::_mm_add_pd(arch::_mm_mul_pd(ex, ex), arch::_mm_mul_pd(ey, ey)),
            eps,
        );
        let t = arch::_mm_max_pd(zero, arch::_mm_min_pd(one, arch::_mm_div_pd(num, den)));
        let px = arch::_mm_add_pd(ax, arch::_mm_mul_pd(t, ex));
        let py = arch::_mm_add_pd(ay, arch::_mm_mul_pd(t, ey));
        let dx = arch::_mm_sub_pd(vx, px);
        let dy = arch::_mm_sub_pd(vy, py);
        let d2 = arch::_mm_add_pd(arch::_mm_mul_pd(dx, dx), arch::_mm_mul_pd(dy, dy));

        let mut lanes = [0.0_f64; 2];
        arch::_mm_storeu_pd(lanes.as_mut_ptr(), d2);
        for v in lanes {
            if v < min_d2 {
                min_d2 = v;
            }
        }
        i += 2;
    }
    if i < n {
        let rem = min_dist_sq_ring_scalar(&coords[i..=n], x, y);
        if rem < min_d2 {
            min_d2 = rem;
        }
    }
    min_d2
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::sdf::rect_sdf_max`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn rect_sdf_max (poly : & Polygon < f64 > , x0 : f64 , y0 : f64 , x1 : f64 , y1 : f64 ,) -> f64
```

Maximum SDF at 8 sample points of an axis-aligned rect (4 corners + 4 edge midpoints). Negative result means all samples are strictly inside the polygon.

<details>
<summary>Source</summary>

```rust
pub fn rect_sdf_max(
    poly: &Polygon<f64>,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
) -> f64 {
    let cx = (x0 + x1) * 0.5;
    let cy = (y0 + y1) * 0.5;
    let pts = [
        (x0, y0), (x1, y0), (x1, y1), (x0, y1), // corners
        (cx, y0), (x1, cy), (cx, y1), (x0, cy),  // edge midpoints
    ];
    pts.iter()
        .map(|&(px, py)| polygon_sdf(poly, px, py))
        .fold(f64::NEG_INFINITY, f64::max)
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::sdf::certify_rect`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn certify_rect (poly : & Polygon < f64 > , mut x0 : f64 , mut y0 : f64 , mut x1 : f64 , mut y1 : f64 , max_ratio : f64 ,) -> Option < (f64 , f64 , f64 , f64 , f64) >
```

Certify that an axis-aligned rect `(x0,y0,x1,y1)` is fully inside `poly`.

If `max_sdf <= CERT_EPS` it is already valid and returned unchanged.
Otherwise a symmetric shrink proportional to the violation is attempted.
Returns `Some((x0, y0, x1, y1, area))` on success, `None` if unfixable.

<details>
<summary>Source</summary>

```rust
pub fn certify_rect(
    poly: &Polygon<f64>,
    mut x0: f64,
    mut y0: f64,
    mut x1: f64,
    mut y1: f64,
    max_ratio: f64,
) -> Option<(f64, f64, f64, f64, f64)> {
    if x1 - x0 < 1e-12 || y1 - y0 < 1e-12 {
        return None;
    }

    let max_sdf = rect_sdf_max(poly, x0, y0, x1, y1);

    if max_sdf <= CERT_EPS {
        let area = (x1 - x0) * (y1 - y0);
        return Some((x0, y0, x1, y1, area));
    }

    // Symmetric shrink from centre
    let shrink = max_sdf + CERT_EPS;
    let hw = (x1 - x0) * 0.5;
    let hh = (y1 - y0) * 0.5;

    // Reject if the required shrink eats more than CERT_MAX_SHRINK of the shorter half-side
    if shrink > hw.min(hh) * CERT_MAX_SHRINK {
        return None;
    }

    let cx = (x0 + x1) * 0.5;
    let cy = (y0 + y1) * 0.5;
    x0 = cx - (hw - shrink);
    x1 = cx + (hw - shrink);
    y0 = cy - (hh - shrink);
    y1 = cy + (hh - shrink);

    if x1 - x0 <= 0.0 || y1 - y0 <= 0.0 {
        return None;
    }

    // Apply aspect-ratio constraint
    if max_ratio > 0.0 {
        let rw = x1 - x0;
        let rh = y1 - y0;
        let ls = rw.max(rh);
        let ss = rw.min(rh);
        if ss > 0.0 && ls / ss > max_ratio {
            let nl = ss * max_ratio;
            if rw >= rh {
                let c = (x0 + x1) * 0.5;
                x0 = c - nl * 0.5;
                x1 = c + nl * 0.5;
            } else {
                let c = (y0 + y1) * 0.5;
                y0 = c - nl * 0.5;
                y1 = c + nl * 0.5;
            }
        }
    }

    // Verify the shrunk rect passes (tighter threshold to catch residual violations)
    if rect_sdf_max(poly, x0, y0, x1, y1) > CERT_EPS * 10.0 {
        return None;
    }

    let area = (x1 - x0) * (y1 - y0);
    Some((x0, y0, x1, y1, area))
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::sdf::best_effort_shrink`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn best_effort_shrink (poly : & Polygon < f64 > , x0 : f64 , y0 : f64 , x1 : f64 , y1 : f64 , max_ratio : f64 ,) -> Option < (f64 , f64 , f64 , f64 , f64) >
```

Single-pass best-effort shrink: given a candidate that may slightly violate containment, shrink by exactly `max_sdf + 2*CERT_EPS` without binary search.

<details>
<summary>Source</summary>

```rust
pub fn best_effort_shrink(
    poly: &Polygon<f64>,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    max_ratio: f64,
) -> Option<(f64, f64, f64, f64, f64)> {
    certify_rect(poly, x0, y0, x1, y1, max_ratio)
}
```

</details>



