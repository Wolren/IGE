# ige-core::solvers::mic::solver::exact <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


## Structs

### `ige-core::solvers::mic::solver::exact::SegmentLine`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


#### Fields

| Name | Type | Description |
|------|------|-------------|
| `nx` | `f64` |  |
| `ny` | `f64` |  |
| `c` | `f64` |  |



### `ige-core::solvers::mic::solver::exact::QuadCell`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


#### Fields

| Name | Type | Description |
|------|------|-------------|
| `0` | `f64` |  |
| `1` | `f64` |  |
| `2` | `f64` |  |
| `3` | `f64` |  |



## Functions

### `ige-core::solvers::mic::solver::exact::reflex_vertex_in_ring`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn reflex_vertex_in_ring (host : & HostPolygon , ring_idx : usize , vert_idx : usize) -> bool
```

Check if a vertex at `vert_idx` in ring `ring_idx` is reflex (interior angle > 180°).

<details>
<summary>Source</summary>

```rust
fn reflex_vertex_in_ring(host: &HostPolygon, ring_idx: usize, vert_idx: usize) -> bool {
    let meta = &host.rings[ring_idx];
    let coords = &host.coords[meta.start..meta.end];
    let n = if coords.len() >= 2 && coords.first() == coords.last() {
        coords.len() - 1
    } else {
        coords.len()
    };
    if n < 3 { return false; }
    let idx = vert_idx % n;
    let prev = coords[(idx + n - 1) % n];
    let cur = coords[idx];
    let next = coords[(idx + 1) % n];
    let cross = (cur[0]-prev[0])*(next[1]-cur[1]) - (cur[1]-prev[1])*(next[0]-cur[0]);
    if meta.is_hole { cross > 1e-14 } else { cross < -1e-14 }
}
```

</details>



### `ige-core::solvers::mic::solver::exact::count_reflex_vertices`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn count_reflex_vertices (host : & HostPolygon , seg_index : & SegmentIndex) -> usize
```

Count reflex vertices across all rings (concavity measure).

<details>
<summary>Source</summary>

```rust
fn count_reflex_vertices(host: &HostPolygon, seg_index: &SegmentIndex) -> usize {
    use std::collections::HashSet;
    let mut reflex_set = HashSet::new();
    for seg_idx in 0..seg_index.len() {
        let rid = seg_index.ring_id[seg_idx];
        let eid = seg_index.edge_id[seg_idx];
        if reflex_vertex_in_ring(host, rid, eid) || reflex_vertex_in_ring(host, rid, eid + 1) {
            reflex_set.insert((rid, eid));
        }
    }
    reflex_set.len()
}
```

</details>



### `ige-core::solvers::mic::solver::exact::caps_for`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn caps_for (seg_count : usize , hole_count : usize , reflex_count : usize) -> (usize , usize , usize , usize)
```

Compute adaptive cap tier based on polygon complexity. Complex polygons (reflex vertices, holes, high segment count) get extended caps to ensure the candidate set covers the true MIC's support segments.

<details>
<summary>Source</summary>

```rust
fn caps_for(seg_count: usize, hole_count: usize, reflex_count: usize) -> (usize, usize, usize, usize) {
    // Trigger extended caps if any of:
    //   - >3 holes (cross-ring MIC constraints)
    //   - >8 reflex vertices (deep concavity)
    //   - >200 segments (dense boundary)
    let complex = hole_count > 3 || reflex_count > 8 || seg_count > 200;
    if complex {
        (EXT_TRIPLE_CAP, EXT_SS_SEG_CAP, EXT_SS_VERT_CAP, EXT_SEGS_PER_RING)
    } else {
        (BASE_TRIPLE_CAP, BASE_SS_SEG_CAP, BASE_SS_VERT_CAP, BASE_SEGS_PER_RING)
    }
}
```

</details>



### `ige-core::solvers::mic::solver::exact::quantize_origin`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn quantize_origin (host : & HostPolygon) -> (f64 , f64)
```

<details>
<summary>Source</summary>

```rust
fn quantize_origin(host: &HostPolygon) -> (f64, f64) {
    let Some((min_x, min_y, max_x, _max_y)) = host.bounds() else {
        return (0.0, 0.0);
    };
    let span_x = (max_x - min_x).max(1.0);
    (min_x - span_x * 0.1, min_y - span_x * 0.1)
}
```

</details>



### `ige-core::solvers::mic::solver::exact::solve_exact`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn solve_exact (workspace : & mut MicWorkspace , opts : & MicOptions ,) -> Result < MicResult , MicError >
```

<details>
<summary>Source</summary>

```rust
pub fn solve_exact(
    workspace: &mut MicWorkspace,
    opts: &MicOptions,
) -> Result<MicResult, MicError> {
    workspace.clear_candidates();
    let mut seen = FxHashSet::<(i64, i64)>::default();
    let q_origin = quantize_origin(&workspace.host);

    // 1. Centroid
    if let Some(c) = workspace.host.polygon.centroid() {
        push_candidate(&mut workspace.candidate_buf, &mut seen, c.x(), c.y(), q_origin);
    }



    // 2. All boundary vertices
    let vertices = workspace.host.unique_vertices();
    for v in &vertices {
        push_candidate(&mut workspace.candidate_buf, &mut seen, v[0], v[1], q_origin);
    }

    // 3. Segment midpoints
    for seg_idx in 0..workspace.seg_index.len() {
        let (mx, my) = workspace.seg_index.midpoint(seg_idx);
        push_candidate(&mut workspace.candidate_buf, &mut seen, mx, my, q_origin);
    }

    // Compute adaptive caps based on polygon complexity.
    let hole_count = workspace.host.rings.iter().filter(|r| r.is_hole).count();
    let reflex_count = count_reflex_vertices(&workspace.host, &workspace.seg_index);
    let seg_count = workspace.seg_index.len();
    let (triple_cap, ss_seg_cap, ss_vert_cap, segs_per_ring) =
        caps_for(seg_count, hole_count, reflex_count);

    // 4. Segment-triple incenters — reflex-biased sampling (Gap B)
    generate_segment_triple_candidates(&workspace.seg_index, &workspace.host, &mut seen,
        &mut workspace.candidate_buf, q_origin, triple_cap, segs_per_ring);

    // 5. CDT circumcenters
    generate_cdt_candidates(&workspace.host, &mut seen, &mut workspace.candidate_buf, q_origin);

    // 6. Ear circumcenters — ALL rings including holes
    generate_ear_candidates_all_rings(&workspace.host, &mut seen, &mut workspace.candidate_buf, q_origin);

    // 7. Filtered: seg-seg-vertex bisector candidates + vertex-triple circumcenters
    if matches!(opts.robust_mode, RobustMode::Filtered) {
        let lines = precompute_segment_lines(&workspace.seg_index);
        generate_ssv_candidates(&workspace.seg_index, &lines, &vertices, &mut seen,
            &mut workspace.candidate_buf, q_origin, ss_seg_cap, ss_vert_cap);

        let sampled = sample_vertices(&vertices, 48);
        for i in 0..sampled.len() {
            for j in i + 1..sampled.len() {
                for k in j + 1..sampled.len() {
                    if let Some((cx, cy)) = circumcenter(sampled[i], sampled[j], sampled[k]) {
                        push_candidate(&mut workspace.candidate_buf, &mut seen, cx, cy, q_origin);
                    }
                }
            }
        }
    }

    if workspace.candidate_buf.is_empty() {
        return Err(MicError::NoCircleFound);
    }

    let candidate_count = workspace.candidate_buf.len();
    let pip_index = &workspace.pip_index;
    let nb_index = &workspace.nb_index;
    let mut best_any: Option<MicCandidate> = None;

    let candidate_buf = &mut workspace.candidate_buf;

    for cand in candidate_buf.iter_mut() {
        if !pip_index.contains_strict_xy(cand.x, cand.y) {
            continue;
        }
        let Some((radius_sq, _nearest_idx)) = nb_index.nearest_distance_sq(cand.x, cand.y) else {
            continue;
        };
        if !radius_sq.is_finite() || radius_sq <= 0.0 {
            continue;
        }
        cand.radius_sq = radius_sq;

        if best_any.as_ref().map(|b| cand.radius_sq > b.radius_sq).unwrap_or(true) {
            best_any = Some(cand.clone());
        }
    }

    let best = best_any.ok_or(MicError::NoCircleFound)?;

    // Phase 4: Quadtree refinement from best candidate.
    // Closes the gap between discrete candidate optimum and true MIC.
    let (min_x, min_y, max_x, max_y) = workspace.host.bounds()
        .unwrap_or((0.0, 0.0, 1.0, 1.0));
    let diameter = (max_x - min_x).hypot(max_y - min_y).max(1.0);
    let refine_tol = diameter * 1e-6;
    let seed_h = best.radius_sq.sqrt().max(1e-12) * 0.5;
    let (ref_x, ref_y, ref_r) = quadtree_search(
        best.x, best.y, best.radius_sq.sqrt(), seed_h,
        pip_index, nb_index, refine_tol, 100,
    );

    let center = Point::new(ref_x, ref_y);
    let ref_r_sq = ref_r * ref_r;
    let support_eps = ref_r_sq.max(1.0) * 1e-10;
    let support_segments =
        nb_index.supporting_segments(ref_x, ref_y, ref_r_sq, support_eps);

    Ok(MicResult {
        center,
        radius: ref_r,
        radius_sq: ref_r_sq,
        support_segments,
        candidate_count,
        used_engine: MicUsedEngine::Exact,
        component_index: None,
    })
}
```

</details>



### `ige-core::solvers::mic::solver::exact::fast_triangle`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn fast_triangle (host : & HostPolygon) -> Option < MicResult >
```

Triangle incenter — exact O(1) MIC. Trigger: ring_count==1, outer.len()==4.

<details>
<summary>Source</summary>

```rust
pub fn fast_triangle(host: &HostPolygon) -> Option<MicResult> {
    if host.ring_count() != 1 { return None; }
    let outer = host.outer_ring();
    if outer.len() != 4 { return None; }
    let a = outer[0]; let b = outer[1]; let c = outer[2];
    let la = (b[0]-c[0]).hypot(b[1]-c[1]);
    let lb = (a[0]-c[0]).hypot(a[1]-c[1]);
    let lc = (a[0]-b[0]).hypot(a[1]-b[1]);
    let perim = la + lb + lc;
    if perim <= 1e-14 { return None; }
    let cx = (la*a[0] + lb*b[0] + lc*c[0]) / perim;
    let cy = (la*a[1] + lb*b[1] + lc*c[1]) / perim;
    let area = ((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])).abs() * 0.5;
    let r = 2.0 * area / perim;
    if r <= 1e-6 { return None; }
    Some(MicResult {
        center: Point::new(cx, cy), radius: r, radius_sq: r*r,
        support_segments: vec![], candidate_count: 1,
        used_engine: MicUsedEngine::Exact, component_index: None,
    })
}
```

</details>



### `ige-core::solvers::mic::solver::exact::fast_convex_quad`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn fast_convex_quad (host : & HostPolygon) -> Option < MicResult >
```

Convex quadrilateral bisector-intersection — exact O(1) MIC. Trigger: ring_count==1, outer.len()==5, is_convex.

<details>
<summary>Source</summary>

```rust
pub fn fast_convex_quad(host: &HostPolygon) -> Option<MicResult> {
    if host.ring_count() != 1 { return None; }
    let outer = host.outer_ring();
    if outer.len() != 5 { return None; }
    // Check convexity: all cross products must have same sign (CCW positive)
    let mut sign = 0i8;
    for i in 0..4 {
        let p = outer[(i+3)%4]; let c = outer[i]; let n = outer[(i+1)%4];
        let cross = (c[0]-p[0])*(n[1]-c[1]) - (c[1]-p[1])*(n[0]-c[0]);
        if cross.abs() <= 1e-14 { return None; }
        let s = if cross > 0.0 { 1 } else { -1 };
        if sign == 0 { sign = s; } else if s != sign { return None; }
    }
    // Build edge line equations: inward normal * x = c
    let mut nx = [0.0f64; 4]; let mut ny = [0.0f64; 4]; let mut line_c = [0.0f64; 4];
    for i in 0..4 {
        let ax = outer[i][0]; let ay = outer[i][1];
        let bx = outer[(i+1)%4][0]; let by = outer[(i+1)%4][1];
        let dx = bx - ax; let dy = by - ay;
        let len = dx.hypot(dy);
        if len <= 1e-14 { return None; }
        nx[i] = -dy / len; ny[i] = dx / len; // inward normal (CCW)
        line_c[i] = nx[i]*ax + ny[i]*ay;
    }
    // Adjacent bisector intersections — 4 candidates, pick best interior
    let mut best_r2 = 0.0f64; let mut best_cx = 0.0; let mut best_cy = 0.0; let mut found = false;
    for i in 0..4 {
        let j = (i+1)%4;
        let det = nx[i]*ny[j] - ny[i]*nx[j];
        if det.abs() <= 1e-14 { continue; }
        let inv = 1.0 / det;
        let cx = (line_c[i]*ny[j] - ny[i]*line_c[j]) * inv;
        let cy = (nx[i]*line_c[j] - line_c[i]*nx[j]) * inv;
        if !cx.is_finite() || !cy.is_finite() { continue; }
        let mut ok = true;
        for k in 0..4 {
            if nx[k]*cx + ny[k]*cy - line_c[k] <= 0.0 { ok = false; break; }
        }
        if !ok { continue; }
        let r2 = nx[0]*cx + ny[0]*cy - line_c[0];
        let r2 = r2 * r2;
        if r2 > best_r2 { best_r2 = r2; best_cx = cx; best_cy = cy; found = true; }
    }
    if !found { return None; }
    let r = best_r2.sqrt();
    if r <= 1e-6 { return None; }
    Some(MicResult {
        center: Point::new(best_cx, best_cy), radius: r, radius_sq: best_r2,
        support_segments: vec![], candidate_count: 1,
        used_engine: MicUsedEngine::Exact, component_index: None,
    })
}
```

</details>



### `ige-core::solvers::mic::solver::exact::push_candidate`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn push_candidate (buf : & mut Vec < MicCandidate > , seen : & mut FxHashSet < (i64 , i64) > , x : f64 , y : f64 , q_origin : (f64 , f64) ,)
```

<details>
<summary>Source</summary>

```rust
fn push_candidate(
    buf: &mut Vec<MicCandidate>,
    seen: &mut FxHashSet<(i64, i64)>,
    x: f64,
    y: f64,
    q_origin: (f64, f64),
) {
    if !x.is_finite() || !y.is_finite() { return; }
    let qx = quantize(x - q_origin.0);
    let qy = quantize(y - q_origin.1);
    if !seen.insert((qx, qy)) { return; }
    buf.push(MicCandidate { x, y, radius_sq: 0.0 });
}
```

</details>



### `ige-core::solvers::mic::solver::exact::quantize`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn quantize (v : f64) -> i64
```

<details>
<summary>Source</summary>

```rust
fn quantize(v: f64) -> i64 { (v * CANDIDATE_QUANTIZE).round() as i64 }
```

</details>



### `ige-core::solvers::mic::solver::exact::sample_vertices`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn sample_vertices (vertices : & [[f64 ; 2]] , max_vertices : usize) -> Vec < [f64 ; 2] >
```

<details>
<summary>Source</summary>

```rust
fn sample_vertices(vertices: &[[f64; 2]], max_vertices: usize) -> Vec<[f64; 2]> {
    if vertices.len() <= max_vertices { return vertices.to_vec(); }
    let step = ((vertices.len() as f64) / (max_vertices as f64)).ceil() as usize;
    vertices.iter().step_by(step.max(1)).copied().collect()
}
```

</details>



### `ige-core::solvers::mic::solver::exact::sample_segments_ring_aware`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn sample_segments_ring_aware (seg_index : & SegmentIndex , max_total : usize , min_per_ring : usize) -> Vec < usize >
```

Sample segments with ring awareness — guarantees at least MIN_SEGS_PER_RING from each ring before distributing the remaining budget by segment count.

<details>
<summary>Source</summary>

```rust
fn sample_segments_ring_aware(seg_index: &SegmentIndex, max_total: usize, min_per_ring: usize) -> Vec<usize> {
    let n = seg_index.len();
    if n <= max_total || n == 0 { return (0..n).collect(); }
    if seg_index.ring_id.is_empty() { return Vec::new(); }

    // Find ring boundaries in the flat segment list
    let mut ring_starts: Vec<usize> = vec![0];
    let mut last_rid = seg_index.ring_id[0];
    for i in 1..n {
        if seg_index.ring_id[i] != last_rid { ring_starts.push(i); last_rid = seg_index.ring_id[i]; }
    }
    let num_rings = ring_starts.len();
    let mut ring_ends = ring_starts[1..].to_vec();
    ring_ends.push(n);

    // Allocate: min_per_ring guaranteed, remainder by segment count
    let guaranteed = min_per_ring * num_rings;
    let remaining = if max_total > guaranteed { max_total - guaranteed } else { 0 };

    let mut result = Vec::with_capacity(max_total);
    for ri in 0..num_rings {
        let start = ring_starts[ri];
        let end = ring_ends[ri];
        let count = end - start;
        let alloc = if count <= min_per_ring {
            count
        } else {
            let extra = remaining * count / n;
            min_per_ring + extra.min(count - min_per_ring)
        };
        if count <= alloc {
            for idx in start..end { result.push(idx); }
        } else {
            let step = count / alloc;
            for i in (0..count).step_by(step.max(1)).take(alloc) {
                result.push(start + i);
            }
        }
    }
    result
}
```

</details>



### `ige-core::solvers::mic::solver::exact::generate_segment_triple_candidates`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn generate_segment_triple_candidates (seg_index : & SegmentIndex , host : & HostPolygon , seen : & mut FxHashSet < (i64 , i64) > , candidate_buf : & mut Vec < MicCandidate > , q_origin : (f64 , f64) , triple_cap : usize , _segs_per_ring : usize ,)
```

<details>
<summary>Source</summary>

```rust
fn generate_segment_triple_candidates(
    seg_index: &SegmentIndex,
    host: &HostPolygon,
    seen: &mut FxHashSet<(i64, i64)>,
    candidate_buf: &mut Vec<MicCandidate>,
    q_origin: (f64, f64),
    triple_cap: usize,
    _segs_per_ring: usize,
) {
    let n = seg_index.len();
    if n < 3 { return; }
    let lines = precompute_segment_lines(seg_index);

    // Reflex-biased sampling (Gap B): unconditionally include segments
    // adjacent to reflex vertices, then fill remaining budget with
    // ring-aware uniform sampling from non-reflex segments.
    let sampled = if n <= triple_cap {
        (0..n).collect()
    } else {
        let mut result: Vec<usize> = (0..n)
            .filter(|&si| reflex_vertex_in_ring(host, seg_index.ring_id[si], seg_index.edge_id[si])
                || reflex_vertex_in_ring(host, seg_index.ring_id[si], seg_index.edge_id[si] + 1))
            .collect();
        result.sort_unstable();
        result.dedup();
        if result.len() >= triple_cap {
            result.truncate(triple_cap)
        } else {
            let reflex_set: FxHashSet<usize> = result.iter().copied().collect();
            let remaining = triple_cap - result.len();
            let non_reflex: Vec<usize> = (0..n).filter(|i| !reflex_set.contains(i)).collect();
            if !non_reflex.is_empty() {
                let step = (non_reflex.len() / remaining).max(1);
                result.extend(non_reflex.iter().step_by(step).take(remaining).copied());
            }
        }
        result
    };
    for ii in 0..sampled.len() {
        let i = sampled[ii];
        for jj in ii + 1..sampled.len() {
            let j = sampled[jj];
            for kk in jj + 1..sampled.len() {
                let k = sampled[kk];
                if let Some((cx, cy)) = segment_incenter(&lines, i, j, k) {
                    push_candidate(candidate_buf, seen, cx, cy, q_origin);
                }
            }
        }
    }
}
```

</details>



### `ige-core::solvers::mic::solver::exact::segment_incenter`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn segment_incenter (lines : & [SegmentLine] , i : usize , j : usize , k : usize) -> Option < (f64 , f64) >
```

<details>
<summary>Source</summary>

```rust
fn segment_incenter(lines: &[SegmentLine], i: usize, j: usize, k: usize) -> Option<(f64, f64)> {
    let li = &lines[i];
    let lj = &lines[j];
    let lk = &lines[k];
    let a_x = li.nx - lj.nx;
    let a_y = li.ny - lj.ny;
    let d_ij = li.c - lj.c;
    let b_x = li.nx - lk.nx;
    let b_y = li.ny - lk.ny;
    let d_ik = li.c - lk.c;
    let det = a_x * b_y - a_y * b_x;
    if det.abs() <= 1e-14 { return None; }
    let inv_det = 1.0 / det;
    let x = (d_ij * b_y - d_ik * a_y) * inv_det;
    let y = (a_x * d_ik - b_x * d_ij) * inv_det;
    if !x.is_finite() || !y.is_finite() { return None; }
    let d_i = li.nx * x + li.ny * y - li.c;
    if d_i <= 0.0 { return None; }
    Some((x, y))
}
```

</details>



### `ige-core::solvers::mic::solver::exact::precompute_segment_lines`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn precompute_segment_lines (seg_index : & SegmentIndex) -> Vec < SegmentLine >
```

<details>
<summary>Source</summary>

```rust
fn precompute_segment_lines(seg_index: &SegmentIndex) -> Vec<SegmentLine> {
    let mut lines = Vec::with_capacity(seg_index.len());
    for idx in 0..seg_index.len() {
        let dx = seg_index.dir_x[idx];
        let dy = seg_index.dir_y[idx];
        let len = seg_index.len_sq[idx].sqrt().max(1e-300);
        let inv_len = 1.0 / len;
        let is_hole = seg_index.is_hole_edge[idx];
        // Outer ring (CCW): inward = rotate left = (-dy, dx)
        // Hole ring (CW): inward = rotate right = (dy, -dx)
        let (nx, ny) = if !is_hole { (-dy * inv_len, dx * inv_len) } else { (dy * inv_len, -dx * inv_len) };
        let c = nx * seg_index.ax[idx] + ny * seg_index.ay[idx];
        lines.push(SegmentLine { nx, ny, c });
    }
    lines
}
```

</details>



### `ige-core::solvers::mic::solver::exact::generate_ssv_candidates`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn generate_ssv_candidates (seg_index : & SegmentIndex , lines : & [SegmentLine] , vertices : & [[f64 ; 2]] , seen : & mut FxHashSet < (i64 , i64) > , candidate_buf : & mut Vec < MicCandidate > , q_origin : (f64 , f64) , ss_seg_cap : usize , ss_vert_cap : usize ,)
```

<details>
<summary>Source</summary>

```rust
fn generate_ssv_candidates(
    seg_index: &SegmentIndex,
    lines: &[SegmentLine],
    vertices: &[[f64; 2]],
    seen: &mut FxHashSet<(i64, i64)>,
    candidate_buf: &mut Vec<MicCandidate>,
    q_origin: (f64, f64),
    ss_seg_cap: usize,
    ss_vert_cap: usize,
) {
    if seg_index.len() < 2 || vertices.is_empty() { return; }

    let sampled_segs = sample_segments_ring_aware(seg_index, ss_seg_cap, 2);
    let max_verts = ss_vert_cap.min(vertices.len());
    let sampled_verts: Vec<[f64; 2]> = if vertices.len() <= max_verts {
        vertices.to_vec()
    } else {
        let step = vertices.len() / max_verts;
        vertices.iter().step_by(step.max(1)).copied().take(max_verts).collect()
    };

    for ii in 0..sampled_segs.len() {
        let i = sampled_segs[ii];
        let li = &lines[i];
        for jj in ii + 1..sampled_segs.len() {
            let j = sampled_segs[jj];
            let lj = &lines[j];

            let nx = li.nx - lj.nx;
            let ny = li.ny - lj.ny;
            let n_len_sq = nx * nx + ny * ny;
            if n_len_sq <= 1e-14 { continue; }
            let d_ij = li.c - lj.c;
            let inv_n2 = 1.0 / n_len_sq;
            let c0x = nx * d_ij * inv_n2;
            let c0y = ny * d_ij * inv_n2;
            let n_len = n_len_sq.sqrt();
            let dx = -ny / n_len;
            let dy = nx / n_len;
            let dist0 = li.nx * c0x + li.ny * c0y - li.c;
            let nd = li.nx * dx + li.ny * dy;
            let coeff_a = 1.0 - nd * nd;
            if coeff_a.abs() <= 1e-14 { continue; }
            let inv_2a = 0.5 / coeff_a;

            for v in &sampled_verts {
                let dvx = c0x - v[0];
                let dvy = c0y - v[1];
                let delta_sq = dvx * dvx + dvy * dvy;
                let delta_dot_d = dvx * dx + dvy * dy;
                let coeff_b = 2.0 * (delta_dot_d - dist0 * nd);
                let coeff_c = delta_sq - dist0 * dist0;
                let disc = coeff_b * coeff_b - 4.0 * coeff_a * coeff_c;
                if disc < 0.0 { continue; }
                let sqrt_disc = disc.sqrt();
                for t in [(-coeff_b + sqrt_disc) * inv_2a, (-coeff_b - sqrt_disc) * inv_2a] {
                    let cx = c0x + t * dx;
                    let cy = c0y + t * dy;
                    if !cx.is_finite() || !cy.is_finite() { continue; }
                    if li.nx * cx + li.ny * cy - li.c <= 0.0 { continue; }
                    if lj.nx * cx + lj.ny * cy - lj.c <= 0.0 { continue; }
                    push_candidate(candidate_buf, seen, cx, cy, q_origin);
                }
            }
        }
    }
}
```

</details>



### `ige-core::solvers::mic::solver::exact::generate_cdt_candidates`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn generate_cdt_candidates (host : & HostPolygon , seen : & mut FxHashSet < (i64 , i64) > , candidate_buf : & mut Vec < MicCandidate > , q_origin : (f64 , f64) ,)
```

<details>
<summary>Source</summary>

```rust
fn generate_cdt_candidates(
    host: &HostPolygon,
    seen: &mut FxHashSet<(i64, i64)>,
    candidate_buf: &mut Vec<MicCandidate>,
    q_origin: (f64, f64),
) {
    let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> =
        ConstrainedDelaunayTriangulation::new();

    for ring in &host.rings {
        let coords = &host.coords[ring.start..ring.end];
        let n = if coords.len() >= 2 && coords.first() == coords.last() {
            coords.len() - 1
        } else {
            coords.len()
        };
        if n < 3 { continue; }

        // Insert vertices
        let mut handles = Vec::with_capacity(n);
        for i in 0..n {
            if let Ok(h) = cdt.insert(Point2::new(coords[i][0], coords[i][1])) {
                handles.push(h);
            }
        }
        // Add constraint edges
        for i in 0..handles.len() {
            let j = (i + 1) % handles.len();
            let _ = cdt.add_constraint(handles[i], handles[j]);
        }
    }

    for face in cdt.inner_faces() {
        let verts = face.vertices();
        let a = [verts[0].position().x, verts[0].position().y];
        let b = [verts[1].position().x, verts[1].position().y];
        let c = [verts[2].position().x, verts[2].position().y];
        if let Some((cx, cy)) = circumcenter(a, b, c) {
            push_candidate(candidate_buf, seen, cx, cy, q_origin);
        }
    }
}
```

</details>



### `ige-core::solvers::mic::solver::exact::generate_ear_candidates_all_rings`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn generate_ear_candidates_all_rings (host : & HostPolygon , seen : & mut FxHashSet < (i64 , i64) > , candidate_buf : & mut Vec < MicCandidate > , q_origin : (f64 , f64) ,)
```

<details>
<summary>Source</summary>

```rust
fn generate_ear_candidates_all_rings(
    host: &HostPolygon,
    seen: &mut FxHashSet<(i64, i64)>,
    candidate_buf: &mut Vec<MicCandidate>,
    q_origin: (f64, f64),
) {
    for ring in &host.rings {
        let coords = &host.coords[ring.start..ring.end];
        let n = if coords.len() >= 2 && coords.first() == coords.last() {
            coords.len() - 1
        } else {
            coords.len()
        };
        if n < 3 { continue; }

        let verts = &coords[..n];
        let is_hole = ring.is_hole;

        for i in 0..n {
            let prev = if i == 0 { n - 1 } else { i - 1 };
            let next = if i + 1 >= n { 0 } else { i + 1 };
            let a = verts[prev];
            let b = verts[i];
            let c = verts[next];
            let cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
            // Outer ring CCW: convex when cross > 0
            // Hole ring CW: convex when cross < 0
            let is_convex = if !is_hole { cross > 1e-14 } else { cross < -1e-14 };
            if is_convex {
                if let Some((cx, cy)) = circumcenter(a, b, c) {
                    push_candidate(candidate_buf, seen, cx, cy, q_origin);
                }
            }
        }
    }
}
```

</details>



### `ige-core::solvers::mic::solver::exact::quadtree_search`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn quadtree_search (init_x : f64 , init_y : f64 , init_r : f64 , seed_h : f64 , pip : & super :: super :: index :: PipIndex , nb : & NearestBoundaryIndex , tol : f64 , max_iters : usize ,) -> (f64 , f64 , f64)
```

Run quadtree refinement with configurable iteration budget. `seed_h`: initial cell half-side. For refinement (tight seed, ~best_r/2). For primary solve (wide seed, ~diameter/4).

<details>
<summary>Source</summary>

```rust
fn quadtree_search(
    init_x: f64, init_y: f64, init_r: f64,
    seed_h: f64,
    pip: &super::super::index::PipIndex,
    nb: &NearestBoundaryIndex,
    tol: f64,
    max_iters: usize,
) -> (f64, f64, f64) {
    let mut queue: BinaryHeap<QuadCell> = BinaryHeap::new();
    queue.push(QuadCell(init_x, init_y, seed_h, init_r + seed_h * SQRT_2));
    let mut best_x = init_x; let mut best_y = init_y; let mut best_r = init_r;
    let mut iters = 0usize;
    while let Some(QuadCell(cx, cy, h, upper)) = queue.pop() {
        iters += 1;
        if iters > max_iters { break; }
        if upper <= best_r + tol { break; }
        if h < tol { continue; }
        let h2 = h * 0.5;
        for (dx, dy) in [(-h2,-h2), (h2,-h2), (-h2,h2), (h2,h2)] {
            let nx = cx + dx; let ny = cy + dy;
            if !pip.contains_strict_xy(nx, ny) { continue; }
            let Some((r2, _)) = nb.nearest_distance_sq(nx, ny) else { continue; };
            let r = r2.sqrt();
            if r > best_r { best_r = r; best_x = nx; best_y = ny; }
            let ub = r + h2 * SQRT_2;
            if ub > best_r + tol { queue.push(QuadCell(nx, ny, h2, ub)); }
        }
    }
    (best_x, best_y, best_r)
}
```

</details>



### `ige-core::solvers::mic::solver::exact::circumcenter`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn circumcenter (a : [f64 ; 2] , b : [f64 ; 2] , c : [f64 ; 2]) -> Option < (f64 , f64) >
```

<details>
<summary>Source</summary>

```rust
fn circumcenter(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Option<(f64, f64)> {
    let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    if d.abs() <= 1e-14 { return None; }
    let a2 = a[0] * a[0] + a[1] * a[1];
    let b2 = b[0] * b[0] + b[1] * b[1];
    let c2 = c[0] * c[0] + c[1] * c[1];
    let ux = (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d;
    let uy = (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d;
    if ux.is_finite() && uy.is_finite() { Some((ux, uy)) } else { None }
}
```

</details>



