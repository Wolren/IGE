# ige-core::solvers::mic::input <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


## Structs

### `ige-core::solvers::mic::input::RingMeta`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Metadata describing a ring inside the flat coordinate buffer.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `start` | `usize` |  |
| `end` | `usize` |  |
| `is_hole` | `bool` |  |



### `ige-core::solvers::mic::input::HostPolygon`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Normalized polygon input used by MIC solvers.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `coords` | `Vec < [f64 ; 2] >` | Flat coordinate storage for all rings. |
| `rings` | `Vec < RingMeta >` | Ring offsets into `coords`. |
| `polygon` | `Polygon < f64 >` | Canonicalized geometry used for predicates. |

#### Methods

##### `from_polygon` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn from_polygon (poly : & Polygon < f64 >) -> Result < Self , MicError >
```

<details>
<summary>Source</summary>

```rust
    pub fn from_polygon(poly: &Polygon<f64>) -> Result<Self, MicError> {
        let outer = normalize_ring(poly.exterior(), false)?;
        let mut holes = Vec::with_capacity(poly.interiors().len());
        for hole in poly.interiors() {
            holes.push(normalize_ring(hole, true)?);
        }

        let mut coords = Vec::new();
        let mut rings = Vec::with_capacity(1 + holes.len());

        let push_ring = |coords: &mut Vec<[f64; 2]>,
                         rings: &mut Vec<RingMeta>,
                         ring: &[[f64; 2]],
                         is_hole: bool| {
            let start = coords.len();
            coords.extend_from_slice(ring);
            let end = coords.len();
            rings.push(RingMeta { start, end, is_hole });
        };

        push_ring(&mut coords, &mut rings, &outer, false);
        for hole in &holes {
            push_ring(&mut coords, &mut rings, hole, true);
        }

        let exterior = ring_to_linestring(&outer);
        let interior_rings: Vec<LineString<f64>> = holes.iter().map(|ring| ring_to_linestring(ring)).collect();
        let normalized_polygon = Polygon::new(exterior, interior_rings);
        if normalized_polygon.unsigned_area() <= NORMALIZE_EPS {
            return Err(MicError::InvalidInput(
                "polygon area is zero after normalization".to_string(),
            ));
        }

        Ok(Self {
            coords,
            rings,
            polygon: normalized_polygon,
        })
    }
```

</details>



##### `ring_coords` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn ring_coords (& self , ring_id : usize) -> & [[f64 ; 2]]
```

<details>
<summary>Source</summary>

```rust
    pub fn ring_coords(&self, ring_id: usize) -> &[[f64; 2]] {
        let meta = &self.rings[ring_id];
        &self.coords[meta.start..meta.end]
    }
```

</details>



##### `outer_ring` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn outer_ring (& self) -> & [[f64 ; 2]]
```

<details>
<summary>Source</summary>

```rust
    pub fn outer_ring(&self) -> &[[f64; 2]] {
        self.ring_coords(0)
    }
```

</details>



##### `ring_count` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn ring_count (& self) -> usize
```

<details>
<summary>Source</summary>

```rust
    pub fn ring_count(&self) -> usize {
        self.rings.len()
    }
```

</details>



##### `unique_vertices` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn unique_vertices (& self) -> Vec < [f64 ; 2] >
```

<details>
<summary>Source</summary>

```rust
    pub fn unique_vertices(&self) -> Vec<[f64; 2]> {
        let mut out = Vec::new();
        for ring_id in 0..self.rings.len() {
            let ring = self.ring_coords(ring_id);
            if ring.len() < 2 {
                continue;
            }
            for p in &ring[..ring.len() - 1] {
                out.push(*p);
            }
        }
        out
    }
```

</details>



##### `bounds` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn bounds (& self) -> Option < (f64 , f64 , f64 , f64) >
```

<details>
<summary>Source</summary>

```rust
    pub fn bounds(&self) -> Option<(f64, f64, f64, f64)> {
        if self.coords.is_empty() {
            return None;
        }
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;

        for p in &self.coords {
            min_x = min_x.min(p[0]);
            min_y = min_y.min(p[1]);
            max_x = max_x.max(p[0]);
            max_y = max_y.max(p[1]);
        }

        Some((min_x, min_y, max_x, max_y))
    }
```

</details>





### `ige-core::solvers::mic::input::SegmentIndex`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Struct-of-arrays segment table over all ring edges.

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `ax` | `Vec < f64 >` |  |
| `ay` | `Vec < f64 >` |  |
| `bx` | `Vec < f64 >` |  |
| `by` | `Vec < f64 >` |  |
| `ring_id` | `Vec < usize >` |  |
| `edge_id` | `Vec < usize >` |  |
| `is_hole_edge` | `Vec < bool >` |  |
| `bbox_minx` | `Vec < f64 >` |  |
| `bbox_maxx` | `Vec < f64 >` |  |
| `bbox_miny` | `Vec < f64 >` |  |
| `bbox_maxy` | `Vec < f64 >` |  |
| `dir_x` | `Vec < f64 >` |  |
| `dir_y` | `Vec < f64 >` |  |
| `len_sq` | `Vec < f64 >` |  |

#### Methods

##### `from_host` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn from_host (host : & HostPolygon) -> Self
```

<details>
<summary>Source</summary>

```rust
    pub fn from_host(host: &HostPolygon) -> Self {
        let mut index = Self {
            ax: Vec::new(),
            ay: Vec::new(),
            bx: Vec::new(),
            by: Vec::new(),
            ring_id: Vec::new(),
            edge_id: Vec::new(),
            is_hole_edge: Vec::new(),
            bbox_minx: Vec::new(),
            bbox_maxx: Vec::new(),
            bbox_miny: Vec::new(),
            bbox_maxy: Vec::new(),
            dir_x: Vec::new(),
            dir_y: Vec::new(),
            len_sq: Vec::new(),
        };

        for (rid, meta) in host.rings.iter().enumerate() {
            let ring = host.ring_coords(rid);
            if ring.len() < 2 {
                continue;
            }
            for eid in 0..ring.len() - 1 {
                let a = ring[eid];
                let b = ring[eid + 1];
                let dx = b[0] - a[0];
                let dy = b[1] - a[1];
                let len_sq = dx * dx + dy * dy;
                if len_sq <= NORMALIZE_EPS {
                    continue;
                }

                index.ax.push(a[0]);
                index.ay.push(a[1]);
                index.bx.push(b[0]);
                index.by.push(b[1]);
                index.ring_id.push(rid);
                index.edge_id.push(eid);
                index.is_hole_edge.push(meta.is_hole);
                index.bbox_minx.push(a[0].min(b[0]));
                index.bbox_maxx.push(a[0].max(b[0]));
                index.bbox_miny.push(a[1].min(b[1]));
                index.bbox_maxy.push(a[1].max(b[1]));
                index.dir_x.push(dx);
                index.dir_y.push(dy);
                index.len_sq.push(len_sq);
            }
        }

        index
    }
```

</details>



##### `len` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn len (& self) -> usize
```

<details>
<summary>Source</summary>

```rust
    pub fn len(&self) -> usize {
        self.ax.len()
    }
```

</details>



##### `is_empty` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn is_empty (& self) -> bool
```

<details>
<summary>Source</summary>

```rust
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
```

</details>



##### `midpoint` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn midpoint (& self , seg_idx : usize) -> (f64 , f64)
```

<details>
<summary>Source</summary>

```rust
    pub fn midpoint(&self, seg_idx: usize) -> (f64, f64) {
        (
            (self.ax[seg_idx] + self.bx[seg_idx]) * 0.5,
            (self.ay[seg_idx] + self.by[seg_idx]) * 0.5,
        )
    }
```

</details>



##### `point_segment_distance_sq` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn point_segment_distance_sq (& self , seg_idx : usize , x : f64 , y : f64) -> f64
```

<details>
<summary>Source</summary>

```rust
    pub fn point_segment_distance_sq(&self, seg_idx: usize, x: f64, y: f64) -> f64 {
        let ax = self.ax[seg_idx];
        let ay = self.ay[seg_idx];
        let dx = self.dir_x[seg_idx];
        let dy = self.dir_y[seg_idx];
        let len_sq = self.len_sq[seg_idx];

        let t = (((x - ax) * dx + (y - ay) * dy) / len_sq).clamp(0.0, 1.0);
        let px = ax + t * dx;
        let py = ay + t * dy;
        let ex = x - px;
        let ey = y - py;
        ex * ex + ey * ey
    }
```

</details>





## Functions

### `ige-core::solvers::mic::input::normalize_ring`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn normalize_ring (ring : & LineString < f64 > , is_hole : bool) -> Result < Vec < [f64 ; 2] > , MicError >
```

<details>
<summary>Source</summary>

```rust
fn normalize_ring(ring: &LineString<f64>, is_hole: bool) -> Result<Vec<[f64; 2]>, MicError> {
    let mut pts = Vec::<[f64; 2]>::new();
    for c in &ring.0 {
        if !c.x.is_finite() || !c.y.is_finite() {
            return Err(MicError::InvalidInput(
                "ring contains non-finite coordinates".to_string(),
            ));
        }
        let p = [c.x, c.y];
        if pts
            .last()
            .map(|last| approx_same(*last, p))
            .unwrap_or(false)
        {
            continue;
        }
        pts.push(p);
    }

    if pts.len() < 3 {
        return Err(MicError::InvalidInput(
            "ring has fewer than 3 distinct vertices".to_string(),
        ));
    }

    if approx_same(*pts.first().expect("ring has first"), *pts.last().expect("ring has last")) {
        pts.pop();
    }

    if pts.len() < 3 {
        return Err(MicError::InvalidInput(
            "ring collapsed after closure normalization".to_string(),
        ));
    }

    let signed_area = ring_signed_area_open(&pts);
    if signed_area.abs() <= NORMALIZE_EPS {
        return Err(MicError::InvalidInput(
            "ring area is zero after normalization".to_string(),
        ));
    }

    let should_be_ccw = !is_hole;
    let is_ccw = signed_area > 0.0;
    if should_be_ccw != is_ccw {
        pts.reverse();
    }

    pts.push(*pts.first().expect("normalized ring has first"));
    Ok(pts)
}
```

</details>



### `ige-core::solvers::mic::input::ring_to_linestring`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn ring_to_linestring (ring : & [[f64 ; 2]]) -> LineString < f64 >
```

<details>
<summary>Source</summary>

```rust
fn ring_to_linestring(ring: &[[f64; 2]]) -> LineString<f64> {
    let coords = ring
        .iter()
        .map(|p| Coord { x: p[0], y: p[1] })
        .collect::<Vec<_>>();
    LineString::from(coords)
}
```

</details>



### `ige-core::solvers::mic::input::approx_same`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn approx_same (a : [f64 ; 2] , b : [f64 ; 2]) -> bool
```

<details>
<summary>Source</summary>

```rust
fn approx_same(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).abs() <= NORMALIZE_EPS && (a[1] - b[1]).abs() <= NORMALIZE_EPS
}
```

</details>



### `ige-core::solvers::mic::input::ring_signed_area_open`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn ring_signed_area_open (open_ring : & [[f64 ; 2]]) -> f64
```

<details>
<summary>Source</summary>

```rust
fn ring_signed_area_open(open_ring: &[[f64; 2]]) -> f64 {
    let n = open_ring.len();
    let mut sum = 0.0;
    for i in 0..n {
        let a = open_ring[i];
        let b = open_ring[(i + 1) % n];
        sum += a[0] * b[1] - b[0] * a[1];
    }
    sum * 0.5
}
```

</details>



