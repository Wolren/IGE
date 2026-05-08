# ige-core::solvers::lir::axis_aligned::histogram <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Largest-rectangle-in-histogram kernels (variable-pitch and uniform-pitch).

Port of `_histogram_kernel_vp` and `_histogram_kernel`.

## Functions

### `ige-core::solvers::lir::axis_aligned::histogram::lrih_vp`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn lrih_vp (heights : & [usize] , xs : & [f64] , ys : & [f64] , row_idx : usize , max_ratio : f64 , min_ratio : f64 ,) -> (f64 , f64 , f64 , f64 , f64)
```

Run the variable-pitch LRIH for one row of a histogram sweep.

Uses the n+1 boundary convention: xs/ys have `n_cols+1` / `n_rows+1` entries,
and the top of row `row_idx` is `ys[row_idx + 1]`.

**Parameters:**

| Name | Type | Description |
|------|------|-------------|
| `heights` | `-` | - column heights in row-cell units (length = n_cols) |
| `xs` | `-` | - column boundary x-coordinates (length = n_cols + 1) |
| `ys` | `-` | - row boundary y-coordinates (length = n_rows + 1) |
| `row_idx` | `-` | - current row index (0-based) |
| `max_ratio` | `-` | - max aspect ratio (longer/shorter <= max_ratio); 0.0 = unconstrained |
| `min_ratio` | `-` | - min aspect ratio (longer/shorter >= min_ratio); 0.0 = unconstrained |


<details>
<summary>Source</summary>

```rust
pub fn lrih_vp(
    heights: &[usize],
    xs: &[f64],
    ys: &[f64],
    row_idx: usize,
    max_ratio: f64,
    min_ratio: f64,
) -> (f64, f64, f64, f64, f64) {
    let n = heights.len();
    let n_xs = xs.len();
    let n_ys = ys.len();
    // (start_col, height)
    let mut stack: Vec<(usize, usize)> = Vec::with_capacity(n + 1);
    let mut best_area = 0.0_f64;
    let mut best = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);

    for c in 0..=n {
        let h = if c < n { heights[c] } else { 0 };
        let mut start = c;

        while let Some(&(sc, sh)) = stack.last() {
            if sh <= h {
                break;
            }
            stack.pop();

            let xi = c.min(n_xs.saturating_sub(1));
            let mut x0 = xs[sc];
            let mut x1 = xs[xi];
            let ri0 = (row_idx + 1).saturating_sub(sh);
            let ri1 = (row_idx + 1).min(n_ys.saturating_sub(1));
            let mut y0 = ys[ri0];
            let mut y1 = ys[ri1];

            let rw = x1 - x0;
            let rh = y1 - y0;

            if rw > 0.0 && rh > 0.0 {
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
                let rw2 = x1 - x0;
                let rh2 = y1 - y0;
                if rw2 > 0.0 && rh2 > 0.0 {
                    let area = rw2 * rh2;
                    if area > best_area {
                        best_area = area;
                        best = (x0, y0, x1, y1);
                    }
                }
            }
            start = sc;
        }

        if c < n {
            stack.push((start, h));
        }
    }

    (best.0, best.1, best.2, best.3, best_area)
}
```

</details>



### `ige-core::solvers::lir::axis_aligned::histogram::lrih`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn lrih (heights : & [usize] , xs : & [f64] , ys : & [f64] , row_idx : usize , max_ratio : f64 , min_ratio : f64 ,) -> (f64 , f64 , f64 , f64 , f64)
```

Run the uniform-pitch LRIH for one row of a histogram sweep.

Used by the coarse grid solver. xs/ys are grid-point coordinates; a cell
at row `r` spans `ys[r]` to `ys[r+1]`.  The bottom of the height-h stack
ending at `row_idx` is at `ys[row_idx + 1]`.

<details>
<summary>Source</summary>

```rust
pub fn lrih(
    heights: &[usize],
    xs: &[f64],
    ys: &[f64],
    row_idx: usize,
    max_ratio: f64,
    min_ratio: f64,
) -> (f64, f64, f64, f64, f64) {
    let n = heights.len();
    let n_xs = xs.len();
    let n_ys = ys.len();
    let mut stack: Vec<(usize, usize)> = Vec::with_capacity(n + 1);
    let mut best_area = 0.0_f64;
    let mut best = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);

    for c in 0..=n {
        let h = if c < n { heights[c] } else { 0 };
        let mut start = c;

        while let Some(&(sc, sh)) = stack.last() {
            if sh <= h {
                break;
            }
            stack.pop();

            let w = c - sc;
            let xi = sc + w;
            let mut x0 = xs[sc];
            let mut x1 = xs[xi.min(n_xs.saturating_sub(1))];
            let ri0 = (row_idx + 1).saturating_sub(sh);
            let mut y0 = ys[ri0.min(n_ys.saturating_sub(1))];
            let mut y1 = ys[(row_idx + 1).min(n_ys.saturating_sub(1))];

            let rw = x1 - x0;
            let rh = y1 - y0;

            if rw > 0.0 && rh > 0.0 {
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
                let rw2 = x1 - x0;
                let rh2 = y1 - y0;
                if rw2 > 0.0 && rh2 > 0.0 {
                    let area = rw2 * rh2;
                    if area > best_area {
                        best_area = area;
                        best = (x0, y0, x1, y1);
                    }
                }
            }
            start = sc;
        }

        if c < n {
            stack.push((start, h));
        }
    }

    (best.0, best.1, best.2, best.3, best_area)
}
```

</details>



