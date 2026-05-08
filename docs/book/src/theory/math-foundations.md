# Mathematical Foundations

## Signed Distance Field (SDF)

The **signed distance field** of a polygon $P$ is a function:

$$
d_P(\mathbf{x}) = \begin{cases}
+\min_{\mathbf{p} \in \partial P} \|\mathbf{x} - \mathbf{p}\| & \text{if } \mathbf{x} \in P \\
0 & \text{if } \mathbf{x} \in \partial P \\
-\min_{\mathbf{p} \in \partial P} \|\mathbf{x} - \mathbf{p}\| & \text{if } \mathbf{x} \notin P
\end{cases}
$$

The SDF is used in three ways in IGE:

1. **Upper bound** — the SDF gradient points toward the nearest boundary, used for SDF-descent in MIC
2. **Expansion ceiling** — the SDF value at a point gives a conservative lower bound on clearance, used for Lipschitz skipping in expansion probes
3. **Certification** — the minimum SDF across a rectangle's sample points indicates whether it's fully inside

### Lipschitz Property

The SDF satisfies the Lipschitz condition: for any two points $\mathbf{a}, \mathbf{b}$:

$$
|d_P(\mathbf{a}) - d_P(\mathbf{b})| \leq \|\mathbf{a} - \mathbf{b}\|
$$

This is used in `multi_probe_sdf_v` / `multi_probe_sdf_h` to skip redundant probes: after evaluating SDF at $y_i$ with result $d_i$, any probe $y_j$ within distance $d_i$ of $y_i$ is guaranteed to have $\text{SDF}(y_j) \geq 0$, so the probe is skipped.

## Largest Rectangle in Histogram (LRIH)

Given a histogram of heights $h_0, h_1, \ldots, h_{n-1}$ (non-negative), the largest axis-aligned rectangle contained in the histogram has area:

$$
A^* = \max_{0 \leq i \leq j < n} (j - i + 1) \cdot \min(h_i, h_{i+1}, \ldots, h_j)
$$

**Algorithm:** Monotonic increasing stack. For each position $k$, pop while top of stack has height $\leq h_k$. The rectangle extending from the popped index + 1 to $k-1$ has area $(k - 1 - \text{stack.top}()) \cdot h_{\text{popped}}$. This runs in $O(n)$ time.

LRIH is applied to each row of the occupancy grid (after rotation), where the "height" of column $c$ at row $r$ is the number of consecutive occupied cells from row $r$ upward.

## Convex Hull Upper Bound

For a candidate angle $\theta$, rotate the polygon (or its convex hull) by $-\theta$. The axis-aligned bounding box of the rotated shape has width $w(\theta)$ and height $h(\theta)$. The largest inscribed axis-aligned rectangle at that rotation has area at most:

$$
\text{UB}(\theta) = \min(w(\theta), h(\theta))^2
$$

This bound is **tight** — if the optimal rectangle touches opposite sides of the bounding box, the bound is achieved.

Using the convex hull instead of the original polygon strengthens the bound for concave polygons.

## PCA — Eigendecomposition of the Covariance Matrix

For a polygon with $n$ vertices $\{(x_i, y_i)\}$, the covariance matrix is:

$$
\mathbf{C} = \begin{pmatrix}
\sigma_{xx} & \sigma_{xy} \\
\sigma_{xy} & \sigma_{yy}
\end{pmatrix} = \frac{1}{n} \sum_{i=1}^n \begin{pmatrix} (x_i - \bar{x})^2 & (x_i - \bar{x})(y_i - \bar{y}) \\ (x_i - \bar{x})(y_i - \bar{y}) & (y_i - \bar{y})^2 \end{pmatrix}
$$

For a $2 \times 2$ symmetric matrix, eigenvalues are found via the quadratic formula:

$$
\lambda = \frac{\text{tr}(\mathbf{C}) \pm \sqrt{\text{tr}(\mathbf{C})^2 - 4\det(\mathbf{C})}}{2}
$$

The dominant eigenvector (largest $\lambda$) points along the direction of maximum variance in the vertex distribution — closely aligned with the polygon's primary elongation axis, a strong angle candidate for inscribed rectangles.

## Cross-Ray Center Certification

Given a center point $(c_x, c_y)$ in the rotated coordinate frame, cast four rays in cardinal directions:

$$
d_+ = \text{distance to boundary in } (+x, 0) \\
d_- = \text{distance to boundary in } (-x, 0) \\
u_+ = \text{distance to boundary in } (0, +y) \\
u_- = \text{distance to boundary in } (0, -y)
$$

The **cross-ray rectangle** centered at $(c_x, c_y)$ with half-widths $(d_+ + d_-)/2$ and $(u_+ + u_-)/2$ is the largest axis-aligned rectangle *centered exactly at* that point. This is used in bootstrap seeds and for refining SA-perturbed centers.

The cross-ray clearance distances are computed by intersecting each ray with all polygon edges (segment-segment intersection), taking the minimum positive intersection parameter $t$.

## Edge-Anchored Candidates

The edge-anchored module (`edge_anchor.rs`) generates candidates from boundary support relationships, complementing the center-driven LRIH approach:

- **Vertical pairs:** Find pairs of polygon edges that can serve as left/right supports at a given rotation
- **Horizontal pairs:** Find pairs of edges that can serve as bottom/top supports
- **Single-side anchor:** Pin one rectangle edge to a dominant boundary chain and grow the orthogonal dimension

Each candidate is scored by `support_score` (how much of the rectangle's perimeter is supported by polygon edges) and `validity_score` (how far inside the polygon the candidate lies).