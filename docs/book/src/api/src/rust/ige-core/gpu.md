# ige-core::gpu <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


GPU-accelerated rectangle candidate evaluation using WGSL compute shaders.

This module provides optional GPU acceleration for evaluating large batches
of rectangle candidates in parallel. Falls back gracefully to CPU when GPU
is unavailable or disabled.

## Structs

### `ige-core::gpu::PolygonDataGpu`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


**Derives:** `Copy`, `Clone`, `Pod`, `Zeroable`

Polygon data structure matching WGSL layout

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `vertex_count` | `u32` |  |
| `_pad0` | `u32` |  |
| `_pad1` | `u32` |  |
| `_pad2` | `u32` |  |
| `vertices` | `[f32 ; MAX_VERTICES]` |  |



### `ige-core::gpu::RectCandidateGpu`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


**Derives:** `Copy`, `Clone`, `Pod`, `Zeroable`

Rectangle candidate structure matching WGSL layout

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `x_min` | `f32` |  |
| `y_min` | `f32` |  |
| `x_max` | `f32` |  |
| `y_max` | `f32` |  |



### `ige-core::gpu::CandidateResultGpu`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


**Derives:** `Copy`, `Clone`, `Pod`, `Zeroable`

Candidate result structure matching WGSL layout

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `area` | `f32` |  |
| `is_valid` | `u32` |  |
| `_pad0` | `u32` |  |
| `_pad1` | `u32` |  |



### `ige-core::gpu::SdfRectInputGpu`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


**Derives:** `Copy`, `Clone`, `Pod`, `Zeroable`

SDF input rect for GPU batch evaluation

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `x0` | `f32` |  |
| `y0` | `f32` |  |
| `x1` | `f32` |  |
| `y1` | `f32` |  |



### `ige-core::gpu::SdfResultGpu`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


**Derives:** `Copy`, `Clone`, `Pod`, `Zeroable`

SDF output for one rect (8 sample points)

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `max_sdf` | `f32` |  |
| `_corner0` | `f32` |  |
| `_corner1` | `f32` |  |
| `_corner2` | `f32` |  |
| `_corner3` | `f32` |  |
| `_mid0` | `f32` |  |
| `_mid1` | `f32` |  |
| `_mid2` | `f32` |  |
| `_mid3` | `f32` |  |



### `ige-core::gpu::CandidateResult`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

Result from GPU candidate evaluation

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `area` | `f32` |  |
| `is_valid` | `bool` |  |



### `ige-core::gpu::GpuCoordTransform`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


**Derives:** `Debug`, `Clone`, `Copy`

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `origin_x` | `f64` |  |
| `origin_y` | `f64` |  |
| `scale` | `f64` |  |

#### Methods

##### `from_bbox` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn from_bbox (min_x : f64 , min_y : f64 , max_x : f64 , max_y : f64) -> Self
```

<details>
<summary>Source</summary>

```rust
    fn from_bbox(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Self {
        let span_x = max_x - min_x;
        let span_y = max_y - min_y;
        let span = span_x.max(span_y);
        let scale = if span > 0.0 { 1.0 / span } else { 1.0 };
        Self {
            origin_x: (min_x + max_x) * 0.5,
            origin_y: (min_y + max_y) * 0.5,
            scale,
        }
    }
```

</details>



##### `from_polygon` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn from_polygon (polygon : & Polygon < f64 >) -> Result < Self >
```

<details>
<summary>Source</summary>

```rust
    fn from_polygon(polygon: &Polygon<f64>) -> Result<Self> {
        let bb = polygon
            .bounding_rect()
            .ok_or_else(|| anyhow::anyhow!("Polygon has no bounding rectangle"))?;
        Ok(Self::from_bbox(bb.min().x, bb.min().y, bb.max().x, bb.max().y))
    }
```

</details>



##### `norm_x` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn norm_x (self , x : f64) -> f32
```

<details>
<summary>Source</summary>

```rust
    fn norm_x(self, x: f64) -> f32 {
        ((x - self.origin_x) * self.scale) as f32
    }
```

</details>



##### `norm_y` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn norm_y (self , y : f64) -> f32
```

<details>
<summary>Source</summary>

```rust
    fn norm_y(self, y: f64) -> f32 {
        ((y - self.origin_y) * self.scale) as f32
    }
```

</details>



##### `denorm_area` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn denorm_area (self , area_scaled : f32) -> f32
```

<details>
<summary>Source</summary>

```rust
    fn denorm_area(self, area_scaled: f32) -> f32 {
        if self.scale > 0.0 {
            (area_scaled as f64 / (self.scale * self.scale)) as f32
        } else {
            area_scaled
        }
    }
```

</details>



##### `denorm_distance` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn denorm_distance (self , dist_scaled : f32) -> f32
```

<details>
<summary>Source</summary>

```rust
    fn denorm_distance(self, dist_scaled: f32) -> f32 {
        if self.scale > 0.0 {
            (dist_scaled as f64 / self.scale) as f32
        } else {
            dist_scaled
        }
    }
```

</details>





### `ige-core::gpu::SdfResult`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`

SDF result for one rect

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `max_sdf` | `f32` |  |



### `ige-core::gpu::RectCandidate`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`, `Clone`, `Copy`

Rectangle candidate for evaluation

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `x_min` | `f64` |  |
| `y_min` | `f64` |  |
| `x_max` | `f64` |  |
| `y_max` | `f64` |  |

#### Methods

##### `new` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn new (x_min : f64 , y_min : f64 , x_max : f64 , y_max : f64) -> Self
```

<details>
<summary>Source</summary>

```rust
    pub fn new(x_min: f64, y_min: f64, x_max: f64, y_max: f64) -> Self {
        Self {
            x_min,
            y_min,
            x_max,
            y_max,
        }
    }
```

</details>



##### `area` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn area (& self) -> f64
```

<details>
<summary>Source</summary>

```rust
    pub fn area(&self) -> f64 {
        (self.x_max - self.x_min) * (self.y_max - self.y_min)
    }
```

</details>





### `ige-core::gpu::GpuContext`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


**Derives:** `Debug`

GPU compute context for rectangle evaluation

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `device` | `wgpu :: Device` |  |
| `queue` | `wgpu :: Queue` |  |
| `rect_pipeline` | `wgpu :: ComputePipeline` |  |
| `sdf_pipeline` | `wgpu :: ComputePipeline` |  |
| `grid_pipeline` | `wgpu :: ComputePipeline` |  |
| `bind_group_layout` | `wgpu :: BindGroupLayout` |  |
| `sdf_bind_group_layout` | `wgpu :: BindGroupLayout` |  |
| `grid_bind_group_layout` | `wgpu :: BindGroupLayout` |  |

#### Methods

##### `new` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn new () -> Result < Self >
```

Initialize GPU context (blocking)

<details>
<summary>Source</summary>

```rust
    pub fn new() -> Result<Self> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }))
        .ok_or_else(|| anyhow::anyhow!("Failed to find suitable GPU adapter"))?;

        let (device, queue) = pollster::block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("LIRiAP GPU Device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
            },
            None,
        ))?;

        // Load and compile rect validation shader
        let rect_shader = include_str!("shaders/oriented_lir.wgsl");
        let rect_sm = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Oriented LIR Shader"),
            source: wgpu::ShaderSource::Wgsl(rect_shader.into()),
        });

        // Load and compile SDF shader
        let sdf_shader = include_str!("shaders/lir_sdf.wgsl");
        let sdf_sm = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("LIR SDF Shader"),
            source: wgpu::ShaderSource::Wgsl(sdf_shader.into()),
        });

        // Shared polygon + candidates + results bind group layout
        fn make_shared_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some(label),
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: true },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: true },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 2,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                ],
            })
        }

        let bind_group_layout = make_shared_layout(&device, "Rect Bind Group Layout");
        let sdf_bind_group_layout = make_shared_layout(&device, "SDF Bind Group Layout");

        let rect_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Rect Pipeline Layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let sdf_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("SDF Pipeline Layout"),
            bind_group_layouts: &[&sdf_bind_group_layout],
            push_constant_ranges: &[],
        });

        let rect_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Oriented LIR Pipeline"),
            layout: Some(&rect_pipeline_layout),
            module: &rect_sm,
            entry_point: "main",
            compilation_options: Default::default(),
        });

        let sdf_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("LIR Grid Batch Pipeline"),
            layout: Some(&sdf_pipeline_layout),
            module: &sdf_sm,
            entry_point: "main",
            compilation_options: Default::default(),
        });

        // Load and compile batch grid scorer shader
        let grid_shader = include_str!("shaders/lir_grid_batch.wgsl");
        let grid_sm = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("LIR Grid Batch Shader"),
            source: wgpu::ShaderSource::Wgsl(grid_shader.into()),
        });

        let grid_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Grid Batch Bind Group Layout"),
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: true },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 2,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: true },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 3,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                ],
            });

        let grid_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Grid Batch Pipeline Layout"),
            bind_group_layouts: &[&grid_bind_group_layout],
            push_constant_ranges: &[],
        });

        let grid_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("LIR Grid Batch Pipeline"),
            layout: Some(&grid_pipeline_layout),
            module: &grid_sm,
            entry_point: "main",
            compilation_options: Default::default(),
        });

        Ok(Self {
            device,
            queue,
            rect_pipeline,
            sdf_pipeline,
            grid_pipeline,
            bind_group_layout,
            sdf_bind_group_layout,
            grid_bind_group_layout,
        })
    }
```

</details>



##### `upload_polygon` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn upload_polygon (& self , polygon : & Polygon < f64 > , tx : GpuCoordTransform) -> Result < wgpu :: Buffer >
```

<details>
<summary>Source</summary>

```rust
    fn upload_polygon(&self, polygon: &Polygon<f64>, tx: GpuCoordTransform) -> Result<wgpu::Buffer> {
        if !polygon.interiors().is_empty() {
            anyhow::bail!("GPU kernels currently support polygons without holes; use CPU backend");
        }
        let coords = polygon.exterior().0.clone();
        if coords.len() > MAX_VERTICES / 2 {
            anyhow::bail!(
                "Polygon has too many vertices for GPU (max {})",
                MAX_VERTICES / 2
            );
        }
        let mut poly_data = PolygonDataGpu {
            vertex_count: coords.len() as u32,
            _pad0: 0,
            _pad1: 0,
            _pad2: 0,
            vertices: [0.0; MAX_VERTICES],
        };
        for (i, coord) in coords.iter().enumerate() {
            poly_data.vertices[i * 2] = tx.norm_x(coord.x);
            poly_data.vertices[i * 2 + 1] = tx.norm_y(coord.y);
        }
        Ok(self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Polygon Buffer"),
                contents: bytemuck::cast_slice(&[poly_data]),
                usage: wgpu::BufferUsages::STORAGE,
            }))
    }
```

</details>



##### `read_staging` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn read_staging < T : Pod > (& self , staging : & wgpu :: Buffer , _count : usize) -> Result < Vec < T > >
```

<details>
<summary>Source</summary>

```rust
    fn read_staging<T: Pod>(&self, staging: &wgpu::Buffer, _count: usize) -> Result<Vec<T>> {
        let buffer_slice = staging.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            tx.send(result).unwrap();
        });
        self.device.poll(wgpu::Maintain::Wait);
        rx.recv().unwrap()?;
        let data = buffer_slice.get_mapped_range();
        let result: Vec<T> = bytemuck::cast_slice(&data).to_vec();
        drop(data);
        staging.unmap();
        Ok(result)
    }
```

</details>



##### `evaluate_candidates` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn evaluate_candidates (& self , polygon : & Polygon < f64 > , candidates : & [RectCandidate] ,) -> Result < Vec < CandidateResult > >
```

Evaluate rectangle candidates on GPU

<details>
<summary>Source</summary>

```rust
    pub fn evaluate_candidates(
        &self,
        polygon: &Polygon<f64>,
        candidates: &[RectCandidate],
    ) -> Result<Vec<CandidateResult>> {
        if candidates.is_empty() {
            return Ok(Vec::new());
        }
        let tx = GpuCoordTransform::from_polygon(polygon)?;
        let polygon_buffer = self.upload_polygon(polygon, tx)?;

        let gpu_candidates: Vec<RectCandidateGpu> = candidates
            .iter()
            .map(|c| RectCandidateGpu {
                x_min: tx.norm_x(c.x_min),
                y_min: tx.norm_y(c.y_min),
                x_max: tx.norm_x(c.x_max),
                y_max: tx.norm_y(c.y_max),
            })
            .collect();

        let num = gpu_candidates.len();

        let candidate_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Candidate Buffer"),
                contents: bytemuck::cast_slice(&gpu_candidates),
                usage: wgpu::BufferUsages::STORAGE,
            });

        let result_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Result Buffer"),
            size: (num * std::mem::size_of::<CandidateResultGpu>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Staging Buffer"),
            size: (num * std::mem::size_of::<CandidateResultGpu>()) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Rect Bind Group"),
            layout: &self.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: polygon_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: candidate_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: result_buffer.as_entire_binding(),
                },
            ],
        });

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Rect Encoder"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Rect Compute Pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.rect_pipeline);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.dispatch_workgroups((num as u32 + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE, 1, 1);
        }
        encoder.copy_buffer_to_buffer(
            &result_buffer,
            0,
            &staging,
            0,
            (num * std::mem::size_of::<CandidateResultGpu>()) as u64,
        );
        self.queue.submit(Some(encoder.finish()));

        let gpu_results: Vec<CandidateResultGpu> = self.read_staging(&staging, num)?;
        Ok(gpu_results
            .iter()
            .map(|r| CandidateResult {
                area: tx.denorm_area(r.area),
                is_valid: r.is_valid != 0,
            })
            .collect())
    }
```

</details>



##### `evaluate_rect_sdf_batch` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn evaluate_rect_sdf_batch (& self , polygon : & Polygon < f64 > , rects : & [(f64 , f64 , f64 , f64)] ,) -> Result < Vec < f32 > >
```

Evaluate SDF for a batch of rects against a polygon. Returns the max SDF for each rect (negative = fully inside).

<details>
<summary>Source</summary>

```rust
    pub fn evaluate_rect_sdf_batch(
        &self,
        polygon: &Polygon<f64>,
        rects: &[(f64, f64, f64, f64)],
    ) -> Result<Vec<f32>> {
        if rects.is_empty() {
            return Ok(Vec::new());
        }
        let tx = GpuCoordTransform::from_polygon(polygon)?;
        let polygon_buffer = self.upload_polygon(polygon, tx)?;

        let gpu_rects: Vec<SdfRectInputGpu> = rects
            .iter()
            .map(|&(x0, y0, x1, y1)| SdfRectInputGpu {
                x0: tx.norm_x(x0),
                y0: tx.norm_y(y0),
                x1: tx.norm_x(x1),
                y1: tx.norm_y(y1),
            })
            .collect();

        let num = gpu_rects.len();

        let rect_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("SDF Rect Buffer"),
                contents: bytemuck::cast_slice(&gpu_rects),
                usage: wgpu::BufferUsages::STORAGE,
            });

        let result_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("SDF Result Buffer"),
            size: (num * std::mem::size_of::<SdfResultGpu>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("SDF Staging"),
            size: (num * std::mem::size_of::<SdfResultGpu>()) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("SDF Bind Group"),
            layout: &self.sdf_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: polygon_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: rect_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: result_buffer.as_entire_binding(),
                },
            ],
        });

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("SDF Encoder"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("SDF Compute Pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.sdf_pipeline);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.dispatch_workgroups((num as u32 + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE, 1, 1);
        }
        encoder.copy_buffer_to_buffer(
            &result_buffer,
            0,
            &staging,
            0,
            (num * std::mem::size_of::<SdfResultGpu>()) as u64,
        );
        self.queue.submit(Some(encoder.finish()));

        let gpu_results: Vec<SdfResultGpu> = self.read_staging(&staging, num)?;
        Ok(gpu_results
            .iter()
            .map(|r| tx.denorm_distance(r.max_sdf))
            .collect())
    }
```

</details>



##### `evaluate_grid_batch` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn evaluate_grid_batch (& self , polygons : & [& Polygon < f64 >] , grid_steps : u32 ,) -> Result < Vec < u32 > >
```

Batch-scored grid masks for many polygons in one GPU dispatch. Returns flat `[u32]` mask: `mask[poly * grid_steps^2 + row * grid_steps + col]`.

<details>
<summary>Source</summary>

```rust
    pub fn evaluate_grid_batch(
        &self,
        polygons: &[&Polygon<f64>],
        grid_steps: u32,
    ) -> Result<Vec<u32>> {
        if polygons.is_empty() {
            return Ok(Vec::new());
        }
        if grid_steps == 0 {
            anyhow::bail!("grid_steps must be > 0");
        }
        // Build flat vertex buffer + headers
        let mut verts = Vec::<f32>::new();
        let mut headers = Vec::<GridPolyHeader>::new();
        for poly in polygons {
            if !poly.interiors().is_empty() {
                anyhow::bail!("GPU grid batch currently supports polygons without holes; use CPU backend");
            }
            let bb = poly
                .bounding_rect()
                .ok_or_else(|| anyhow::anyhow!("Polygon has no bounding rectangle"))?;
            let tx = GpuCoordTransform::from_bbox(bb.min().x, bb.min().y, bb.max().x, bb.max().y);
            let span_x_norm = ((bb.max().x - bb.min().x) * tx.scale) as f32;
            let span_y_norm = ((bb.max().y - bb.min().y) * tx.scale) as f32;
            let coords = poly.exterior();
            let v0 = verts.len() as u32 / 2;
            for c in coords.0.iter() {
                verts.push(tx.norm_x(c.x));
                verts.push(tx.norm_y(c.y));
            }
            let vc = coords.0.len() as u32;
            headers.push(GridPolyHeader {
                vertex_offset: v0,
                vertex_count: vc,
                min_x: -0.5 * span_x_norm,
                min_y: -0.5 * span_y_norm,
                max_x: 0.5 * span_x_norm,
                max_y: 0.5 * span_y_norm,
            });
        }

        let np = polygons.len() as u32;
        let uniforms = GridUniforms {
            max_grid_steps: grid_steps,
            n_polygons: np,
            _pad0: 0,
            _pad1: 0,
        };
        let mask_size = (np * grid_steps * grid_steps) as usize;

        let uniform_buf = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Grid Uniforms"),
                contents: bytemuck::cast_slice(&[uniforms]),
                usage: wgpu::BufferUsages::UNIFORM,
            });
        let vert_buf = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Grid Verts"),
                contents: bytemuck::cast_slice(&verts),
                usage: wgpu::BufferUsages::STORAGE,
            });
        let header_buf = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Grid Headers"),
                contents: bytemuck::cast_slice(&headers),
                usage: wgpu::BufferUsages::STORAGE,
            });
        let result_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Grid Mask"),
            size: (mask_size * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Grid Staging"),
            size: (mask_size * 4) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        let bg = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Grid Batch BG"),
            layout: &self.grid_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: uniform_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: vert_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: header_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: result_buf.as_entire_binding(),
                },
            ],
        });

        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Grid Batch Enc"),
            });
        {
            let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Grid Batch Pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.grid_pipeline);
            pass.set_bind_group(0, &bg, &[]);
            let wg_x = (grid_steps + 7) / 8;
            let wg_y = (grid_steps + 7) / 8;
            pass.dispatch_workgroups(wg_x, wg_y, np);
        }
        enc.copy_buffer_to_buffer(&result_buf, 0, &staging, 0, (mask_size * 4) as u64);
        self.queue.submit(Some(enc.finish()));

        let out: Vec<u32> = self.read_staging(&staging, mask_size)?;
        Ok(out)
    }
```

</details>





### `ige-core::gpu::GridUniforms`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


**Derives:** `Copy`, `Clone`, `Pod`, `Zeroable`

GPU data-types for batch grid scorer

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `max_grid_steps` | `u32` |  |
| `n_polygons` | `u32` |  |
| `_pad0` | `u32` |  |
| `_pad1` | `u32` |  |



### `ige-core::gpu::GridPolyHeader`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


**Derives:** `Copy`, `Clone`, `Pod`, `Zeroable`

#### Fields

| Name | Type | Description |
|------|------|-------------|
| `vertex_offset` | `u32` |  |
| `vertex_count` | `u32` |  |
| `min_x` | `f32` |  |
| `min_y` | `f32` |  |
| `max_x` | `f32` |  |
| `max_y` | `f32` |  |



## Enums

### `ige-core::gpu::WorkloadBackend` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


Backend selector for testable CPU/GPU workload execution.

#### Variants

- **`Cpu`**
- **`Gpu`**



### `ige-core::gpu::GridMaskEngine` <span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


Selector for grid-mask workload implementation.

#### Variants

- **`Cpu`**
- **`GpuSdf`** - Uses `lir_sdf.wgsl` by submitting degenerate rects `(x,y,x,y)`.
- **`GpuBatch`** - Uses `lir_grid_batch.wgsl` for one-polygon grid dispatch.



## Functions

### `ige-core::gpu::evaluate_candidates_cpu`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn evaluate_candidates_cpu (polygon : & Polygon < f64 > , candidates : & [RectCandidate] ,) -> Vec < CandidateResult >
```

<details>
<summary>Source</summary>

```rust
fn evaluate_candidates_cpu(
    polygon: &Polygon<f64>,
    candidates: &[RectCandidate],
) -> Vec<CandidateResult> {
    candidates
        .iter()
        .map(|c| CandidateResult {
            area: c.area() as f32,
            is_valid: rect_fully_contained(polygon, c.x_min, c.y_min, c.x_max, c.y_max),
        })
        .collect()
}
```

</details>



### `ige-core::gpu::evaluate_rect_sdf_batch_cpu`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn evaluate_rect_sdf_batch_cpu (polygon : & Polygon < f64 > , rects : & [(f64 , f64 , f64 , f64)]) -> Vec < f32 >
```

<details>
<summary>Source</summary>

```rust
fn evaluate_rect_sdf_batch_cpu(polygon: &Polygon<f64>, rects: &[(f64, f64, f64, f64)]) -> Vec<f32> {
    rects
        .iter()
        .map(|&(x0, y0, x1, y1)| rect_sdf_max(polygon, x0, y0, x1, y1) as f32)
        .collect()
}
```

</details>



### `ige-core::gpu::evaluate_grid_batch_cpu`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: var(--fg); color: white;">private</span>


```rust
fn evaluate_grid_batch_cpu (polygons : & [& Polygon < f64 >] , grid_steps : u32) -> Result < Vec < u32 > >
```

<details>
<summary>Source</summary>

```rust
fn evaluate_grid_batch_cpu(polygons: &[&Polygon<f64>], grid_steps: u32) -> Result<Vec<u32>> {
    if grid_steps == 0 {
        anyhow::bail!("grid_steps must be > 0");
    }
    let cells_per_poly = (grid_steps * grid_steps) as usize;
    let mut out = vec![0u32; polygons.len() * cells_per_poly];
    for (pi, poly) in polygons.iter().enumerate() {
        let bb = poly
            .bounding_rect()
            .ok_or_else(|| anyhow::anyhow!("Polygon has no bounding rectangle"))?;
        let span_x = bb.max().x - bb.min().x;
        let span_y = bb.max().y - bb.min().y;
        if span_x <= 0.0 || span_y <= 0.0 {
            continue;
        }
        for gy in 0..grid_steps {
            for gx in 0..grid_steps {
                let cx = bb.min().x + span_x * (gx as f64 + 0.5) / grid_steps as f64;
                let cy = bb.min().y + span_y * (gy as f64 + 0.5) / grid_steps as f64;
                let idx = pi * cells_per_poly + gy as usize * grid_steps as usize + gx as usize;
                out[idx] = u32::from(polygon_sdf(poly, cx, cy) <= crate::tuning::CONTAIN_BOUNDARY_EPS);
            }
        }
    }
    Ok(out)
}
```

</details>



### `ige-core::gpu::evaluate_candidates_with_backend`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn evaluate_candidates_with_backend (polygon : & Polygon < f64 > , candidates : & [RectCandidate] , backend : WorkloadBackend ,) -> Result < Vec < CandidateResult > >
```

<details>
<summary>Source</summary>

```rust
pub fn evaluate_candidates_with_backend(
    polygon: &Polygon<f64>,
    candidates: &[RectCandidate],
    backend: WorkloadBackend,
) -> Result<Vec<CandidateResult>> {
    match backend {
        WorkloadBackend::Cpu => Ok(evaluate_candidates_cpu(polygon, candidates)),
        WorkloadBackend::Gpu => {
            let ctx = get_gpu_context().ok_or_else(|| anyhow::anyhow!("GPU context unavailable"))?;
            ctx.evaluate_candidates(polygon, candidates)
        }
    }
}
```

</details>



### `ige-core::gpu::evaluate_rect_sdf_batch_with_backend`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn evaluate_rect_sdf_batch_with_backend (polygon : & Polygon < f64 > , rects : & [(f64 , f64 , f64 , f64)] , backend : WorkloadBackend ,) -> Result < Vec < f32 > >
```

<details>
<summary>Source</summary>

```rust
pub fn evaluate_rect_sdf_batch_with_backend(
    polygon: &Polygon<f64>,
    rects: &[(f64, f64, f64, f64)],
    backend: WorkloadBackend,
) -> Result<Vec<f32>> {
    match backend {
        WorkloadBackend::Cpu => Ok(evaluate_rect_sdf_batch_cpu(polygon, rects)),
        WorkloadBackend::Gpu => {
            let ctx = get_gpu_context().ok_or_else(|| anyhow::anyhow!("GPU context unavailable"))?;
            ctx.evaluate_rect_sdf_batch(polygon, rects)
        }
    }
}
```

</details>



### `ige-core::gpu::evaluate_grid_mask_with_engine`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn evaluate_grid_mask_with_engine (polygon : & Polygon < f64 > , grid_steps : u32 , engine : GridMaskEngine ,) -> Result < Vec < bool > >
```

<details>
<summary>Source</summary>

```rust
pub fn evaluate_grid_mask_with_engine(
    polygon: &Polygon<f64>,
    grid_steps: u32,
    engine: GridMaskEngine,
) -> Result<Vec<bool>> {
    match engine {
        GridMaskEngine::Cpu => {
            let cpu = evaluate_grid_batch_cpu(&[polygon], grid_steps)?;
            Ok(cpu.into_iter().map(|v| v != 0).collect())
        }
        GridMaskEngine::GpuBatch => {
            let ctx = get_gpu_context().ok_or_else(|| anyhow::anyhow!("GPU context unavailable"))?;
            let gpu = ctx.evaluate_grid_batch(&[polygon], grid_steps)?;
            Ok(gpu.into_iter().map(|v| v != 0).collect())
        }
        GridMaskEngine::GpuSdf => {
            let ctx = get_gpu_context().ok_or_else(|| anyhow::anyhow!("GPU context unavailable"))?;
            let bb = polygon
                .bounding_rect()
                .ok_or_else(|| anyhow::anyhow!("Polygon has no bounding rectangle"))?;
            if grid_steps == 0 {
                anyhow::bail!("grid_steps must be > 0");
            }
            let span_x = bb.max().x - bb.min().x;
            let span_y = bb.max().y - bb.min().y;
            let rects: Vec<(f64, f64, f64, f64)> = (0..grid_steps)
                .flat_map(|gy| {
                    (0..grid_steps).map(move |gx| {
                        let cx = bb.min().x + span_x * (gx as f64 + 0.5) / grid_steps as f64;
                        let cy = bb.min().y + span_y * (gy as f64 + 0.5) / grid_steps as f64;
                        (cx, cy, cx, cy)
                    })
                })
                .collect();
            let values = ctx.evaluate_rect_sdf_batch(polygon, &rects)?;
            Ok(values
                .into_iter()
                .map(|v| v as f64 <= crate::tuning::CONTAIN_BOUNDARY_EPS)
                .collect())
        }
    }
}
```

</details>



### `ige-core::gpu::try_create_gpu_context`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn try_create_gpu_context () -> Option < GpuContext >
```

Try to create GPU context, returning None if unavailable

<details>
<summary>Source</summary>

```rust
pub fn try_create_gpu_context() -> Option<GpuContext> {
    GpuContext::new().ok()
}
```

</details>



### `ige-core::gpu::get_gpu_context`

<span class="plissken-badge plissken-badge-visibility" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #4caf50; color: white;">pub</span>


```rust
fn get_gpu_context () -> Option < & 'static GpuContext >
```

Get or create a globally cached GPU context.

<details>
<summary>Source</summary>

```rust
pub fn get_gpu_context() -> Option<&'static GpuContext> {
    static CTX: std::sync::OnceLock<Option<GpuContext>> = std::sync::OnceLock::new();
    CTX.get_or_init(|| GpuContext::new().ok()).as_ref()
}
```

</details>



