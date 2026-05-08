# Algorithms

IGE provides three geometric solvers, each implementing a distinct algorithm from the computational geometry literature.

| Algorithm | Module | Type | Best For |
|---|---|---|---|
| Oriented LIR | `solvers/lir/oriented/` | Approximate | General polygons, any rotation |
| Axis-Aligned LIR | `solvers/lir/axis_aligned/` | Exact | Axis-aligned rectangles, provable correctness |
| Maximum Inscribed Circle | `solvers/mic/` | Exact | Circular buffers, facility placement |

## Oriented LIR

The oriented (rotated) rectangle solver is the primary algorithm in this library. It accepts a polygon and returns the largest axis-aligned rectangle placed at *any* rotation angle inside the polygon.

This is the **LIR-BCRS** algorithm — a multi-stage pipeline combining:

- SDF-guided coarse search
- LRIH (Largest Rectangle in Histogram) per scanline row
- Parallel candidate evaluation via Rayon
- Optional bootstrap seeding, PCA guidance, simulated annealing, and edge-anchored candidates

The oriented solver is **not** certified exact — it uses rasterization and SDF approximation. See [Complexity](../theory/complexity.md) for bounds.

## Axis-Aligned LIR

An implementation of the Daniels-Milenkovic-Roth exact vertex-grid algorithm. The rectangle corners are constrained to lie on polygon vertices (or bounding box corners). Since the search space is finite and fully enumerated, the result is **mathematically certified** as the global optimum.

Use this when:
- Rectangle must be axis-aligned
- Correctness is non-negotiable
- You need a proof of optimality

## Maximum Inscribed Circle (MIC)

A two-sweep exact algorithm using SDF descent to find the center, followed by vertex-snapping and edge-walking to produce a geometrically certified radius. Used when the output is a circle rather than a rectangle.