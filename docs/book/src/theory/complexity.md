# Complexity Analysis

## Oriented LIR (BCRS Pipeline)

| Stage | Time | Space | Notes |
|---|---|---|---|
| Polygon preparation | $O(v)$ | $O(v)$ | Simplification if $v > 300$ |
| Angle generation | $O(v)$ | $O(1)$ | Edge histogram + optional PCA |
| Upper bound (per angle) | $O(v_h)$ | $O(1)$ | $v_h$ = convex hull vertices |
| Coarse sweep (per angle) | $O(v + g_c^2)$ | $O(g_c^2)$ | $g_c$ = grid_coarse |
| Cross-ray (bootstrap, per angle) | $O(v + p \cdot e)$ | $O(1)$ | $p$ = sampled centers, $e$ = edges |
| Fine solve (per candidate) | $O(v + G^2)$ | $O(G^2)$ | $G$ = grid_fine |
| Bootstrap seeds | $O(v + G^2)$ | $O(G^2)$ | 1–3 seeds per best angle |

Where:
- $v$ = polygon vertex count
- $g_c$ = `grid_coarse` (default 32)
- $G$ = `grid_fine` (default 64)
- $n_a$ = number of angles evaluated (pruned by UB, typically 10–30)

**Total worst case:** $O(n_a \cdot (v + g_c^2 + G^2))$ — dominated by the two grid stages.

**With UB pruning:** Pruning reduces $n_a$ significantly. For well-conditioned polygons, $n_a \approx 15$–$30$ even for $g_c=32$.

## Oriented LIR — Stage-by-Stage Cost

```
Coarse sweep breakdown:
  rotate polygon          O(v)
  build edges             O(v)
  sort edges             O(v log v)
  scan rows               O(g_c²)  ← dominates

Fine solve breakdown:
  rotate polygon          O(v)
  build vertex grid       O(v log v) + O(G²)
  scan rows               O(G²)    ← dominates
  SDF expansion          O(e · log(range) · probes) per edge
  certification          O(e)
```

## Axis-Aligned LIR (Exact)

| Stage | Time | Space | Notes |
|---|---|---|---|
| Vertex grid build | $O(v \log v)$ | $O(g^2)$ | Sort + dedupe coordinates |
| Parallel scan fill | $O(v + g^2)$ | $O(g^2)$ | Rayon parallel over rows |
| LRIH per row | $O(g)$ per row, $O(g^2)$ total | $O(g)$ | Monotonic stack per row |

**Total:** $O(v \log v + g^2)$ time, $O(g^2)$ space.

**Exact guarantee:** The vertex-grid constraint makes the solution space finite. LRIH implicitly evaluates all $O(g_x^2 \cdot g_y^2)$ rectangle configurations.

## Maximum Inscribed Circle

| Stage | Time | Space | Notes |
|---|---|---|---|
| SDF construction | $O(v)$ | $O(w \cdot h)$ | Grid-based SDF |
| Descent (coarse) | $O(k \cdot w \cdot h)$ | $O(w \cdot h)$ | $k$ = iterations, $w,h$ = grid dims |
| Vertex snap + edge walk | $O(v)$ | $O(1)$ | Constant-time refinement |

**Total:** $O(v + k \cdot w \cdot h)$ — dominated by grid SDF construction and descent iterations.

## Scaling Summary

| Polygon Size | Oriented (default) | Axis-Aligned (default) |
|---|---|---|
| 10 vertices | ~3–5 ms | ~1–2 ms |
| 50 vertices | ~8–15 ms | ~5–10 ms |
| 200 vertices | ~20–40 ms | ~15–30 ms |
| 1000 vertices | ~50–120 ms | ~40–100 ms |

Timings are single-threaded on a modern desktop. Parallel stages (coarse sweep, fine solve) scale with physical core count.