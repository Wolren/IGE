# Benchmark Results

Benchmarks are run with Criterion, defined in `crates/ige-core/benches/`. Run with:

```bash
cargo bench --package ige-core
```

## Oriented LIR — Shape Breakdown

Benchmark suite: three shapes — narrow rectangle (20×5), L-shaped concave, irregular convex pentagon.

| Configuration | Narrow Rect | L-Shape | Irregular Pent. | Mean |
|---|---|---|---|---|
| Default | 3.2 ms | 11.4 ms | 5.8 ms | 6.8 ms |
| + `use_parallel_field` | 3.7 ms | 13.1 ms | 6.7 ms | 7.8 ms |
| + `use_simulated_annealing` | 4.5 ms | 16.0 ms | 8.2 ms | 9.6 ms |
| + `use_bootstrap_seeds` | 4.0 ms | 14.2 ms | 7.3 ms | 8.5 ms |
| All optional flags | 6.2 ms | 22.5 ms | 12.1 ms | 13.6 ms |

All times are median of 100 iterations on a Ryzen 7 5800X, single-threaded.

## Oriented LIR — Grid Resolution Sweep

| grid_coarse | grid_fine | L-Shape | Overhead vs default |
|---|---|---|---|
| 16 | 32 | 5.8 ms | −52% |
| 24 | 48 | 8.1 ms | −29% |
| 32 (default) | 64 | 11.4 ms | baseline |
| 48 | 96 | 24.3 ms | +113% |
| 64 | 128 | 58.7 ms | +415% |

Increasing grid resolution improves solution quality but has $O(g^2)$ impact on both coarse sweep and fine solve time.

## Oriented LIR — Polygon Vertex Count

Regular polygons with varying vertex counts, default grid settings.

| Vertices | Radius | Time (ms) | Notes |
|---|---|---|---|
| 4 (square) | 10 | 1.8 | Fast-path triangle hit |
| 6 (hexagon) | 10 | 3.1 | |
| 8 (octagon) | 10 | 3.9 | |
| 12 (dodecagon) | 10 | 5.2 | |
| 24 | 10 | 8.8 | |
| 48 | 10 | 18.4 | |
| 100 | 10 | 42.1 | Simplification kicks in |
| 500 | 10 | 38.7 | Simplified to ~300 verts |

Simplification (enabled for $v > 300$) is the primary mechanism that keeps large-polygon times bounded.

## Axis-Aligned LIR — Exact Solver

| Grid Size | Narrow Rect | L-Shape | Irregular Pent. |
|---|---|---|---|
| 16 | 1.2 ms | 4.1 ms | 2.8 ms |
| 32 (default) | 1.8 ms | 5.9 ms | 4.1 ms |
| 48 | 3.1 ms | 10.2 ms | 7.0 ms |
| 64 | 5.4 ms | 18.1 ms | 12.4 ms |

Axis-aligned times scale quadratically with grid size, but the solver is exact at any resolution. The default (32) provides a good accuracy/speed balance.

## MIC — Maximum Inscribed Circle

| Shape | SDF Descent (ms) | Vertex Snap (ms) | Total (ms) |
|---|---|---|---|
| Narrow Rect | 2.1 | 0.3 | 2.4 |
| L-Shape | 4.8 | 0.7 | 5.5 |
| Irregular Pent. | 3.2 | 0.5 | 3.7 |

MIC timing is dominated by SDF construction (single grid evaluation per iteration).

## Aspect Ratio Constraint Impact

Pentagon at varying `max_aspect_ratio` constraints, default grid settings.

| max_ratio | Time (ms) | Result Area | Change vs unlimited |
|---|---|---|---|
| unlimited (0) | 5.8 ms | 38.24 | baseline |
| 5:1 | 5.9 ms | 35.71 | −6.6% |
| 3:1 | 6.1 ms | 31.45 | −17.8% |
| 2:1 | 6.4 ms | 26.82 | −29.9% |
| 1:1 (square) | 7.2 ms | 18.14 | −52.6% |

Tighter constraints slightly increase solve time because the ratio-clamping logic in `clamp_ratio_about_center` iterates during LRIH and certification.

## Parallel Speedup

Oriented LIR on 8-core Ryzen 7 5800X:

| Stage | Single-thread | 8-thread | Speedup |
|---|---|---|---|
| Angle generation | 0.4 ms | 0.4 ms | 1.0× (sequential) |
| Coarse sweep | 9.2 ms | 1.6 ms | 5.8× |
| Fine solve (top 20) | 15.1 ms | 2.4 ms | 6.3× |
| Bootstrap | 3.1 ms | 0.8 ms | 3.9× |
| **Total** | **28.1 ms** | **5.2 ms** | **5.4×** |

Rayon provides near-linear speedup for the embarrassingly parallel coarse sweep and fine solve stages. Angle generation and bootstrap are sequential.