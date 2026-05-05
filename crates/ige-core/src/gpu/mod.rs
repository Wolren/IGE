//! GPU-accelerated rectangle candidate evaluation using WGSL compute shaders.
//!
//! This module provides optional GPU acceleration for evaluating large batches
//! of rectangle candidates in parallel. Falls back gracefully to CPU when GPU
//! is unavailable or disabled.

use anyhow::Result;
use bytemuck::{Pod, Zeroable};
use geo::BoundingRect;
use geo_types::Polygon;
use wgpu::util::DeviceExt;

/// Maximum polygon vertices supported by GPU shader.
/// Must match `array<f32, 4096>` in WGSL -- 4096 floats = 2048 (x,y) pairs.
const MAX_VERTICES: usize = 4096;

/// Workgroup size (must match shader)
const WORKGROUP_SIZE: u32 = 256;

/// Polygon data structure matching WGSL layout
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct PolygonDataGpu {
    vertex_count: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
    vertices: [f32; MAX_VERTICES],
}

/// Rectangle candidate structure matching WGSL layout
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct RectCandidateGpu {
    x_min: f32,
    y_min: f32,
    x_max: f32,
    y_max: f32,
}

/// Candidate result structure matching WGSL layout
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct CandidateResultGpu {
    area: f32,
    is_valid: u32,
    _pad0: u32,
    _pad1: u32,
}

/// SDF input rect for GPU batch evaluation
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct SdfRectInputGpu {
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32,
}

/// SDF output for one rect (8 sample points)
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct SdfResultGpu {
    max_sdf: f32,
    _corner0: f32,
    _corner1: f32,
    _corner2: f32,
    _corner3: f32,
    _mid0: f32,
    _mid1: f32,
    _mid2: f32,
    _mid3: f32,
}

/// Result from GPU candidate evaluation
#[derive(Debug, Clone)]
pub struct CandidateResult {
    pub area: f32,
    pub is_valid: bool,
}

/// SDF result for one rect
#[derive(Debug, Clone)]
pub struct SdfResult {
    pub max_sdf: f32,
}

/// Rectangle candidate for evaluation
#[derive(Debug, Clone, Copy)]
pub struct RectCandidate {
    pub x_min: f64,
    pub y_min: f64,
    pub x_max: f64,
    pub y_max: f64,
}

impl RectCandidate {
    pub fn new(x_min: f64, y_min: f64, x_max: f64, y_max: f64) -> Self {
        Self {
            x_min,
            y_min,
            x_max,
            y_max,
        }
    }

    pub fn area(&self) -> f64 {
        (self.x_max - self.x_min) * (self.y_max - self.y_min)
    }
}

/// GPU compute context for rectangle evaluation
#[derive(Debug)]
pub struct GpuContext {
    device: wgpu::Device,
    queue: wgpu::Queue,
    rect_pipeline: wgpu::ComputePipeline,
    sdf_pipeline: wgpu::ComputePipeline,
    grid_pipeline: wgpu::ComputePipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    sdf_bind_group_layout: wgpu::BindGroupLayout,
    grid_bind_group_layout: wgpu::BindGroupLayout,
}

impl GpuContext {
    /// Initialize GPU context (blocking)
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

    fn upload_polygon(&self, polygon: &Polygon<f64>) -> Result<wgpu::Buffer> {
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
            poly_data.vertices[i * 2] = coord.x as f32;
            poly_data.vertices[i * 2 + 1] = coord.y as f32;
        }
        Ok(self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Polygon Buffer"),
                contents: bytemuck::cast_slice(&[poly_data]),
                usage: wgpu::BufferUsages::STORAGE,
            }))
    }

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

    /// Evaluate rectangle candidates on GPU
    pub fn evaluate_candidates(
        &self,
        polygon: &Polygon<f64>,
        candidates: &[RectCandidate],
    ) -> Result<Vec<CandidateResult>> {
        let polygon_buffer = self.upload_polygon(polygon)?;

        let gpu_candidates: Vec<RectCandidateGpu> = candidates
            .iter()
            .map(|c| RectCandidateGpu {
                x_min: c.x_min as f32,
                y_min: c.y_min as f32,
                x_max: c.x_max as f32,
                y_max: c.y_max as f32,
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
                area: r.area,
                is_valid: r.is_valid != 0,
            })
            .collect())
    }

    /// Evaluate SDF for a batch of rects against a polygon.
    /// Returns the max SDF for each rect (negative = fully inside).
    pub fn evaluate_rect_sdf_batch(
        &self,
        polygon: &Polygon<f64>,
        rects: &[(f64, f64, f64, f64)],
    ) -> Result<Vec<f32>> {
        let polygon_buffer = self.upload_polygon(polygon)?;

        let gpu_rects: Vec<SdfRectInputGpu> = rects
            .iter()
            .map(|&(x0, y0, x1, y1)| SdfRectInputGpu {
                x0: x0 as f32,
                y0: y0 as f32,
                x1: x1 as f32,
                y1: y1 as f32,
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
        Ok(gpu_results.iter().map(|r| r.max_sdf).collect())
    }
}

/// GPU data-types for batch grid scorer
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct GridUniforms {
    max_grid_steps: u32,
    n_polygons: u32,
    _pad0: u32,
    _pad1: u32,
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct GridPolyHeader {
    vertex_offset: u32,
    vertex_count: u32,
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
}

impl GpuContext {
    /// Batch-scored grid masks for many polygons in one GPU dispatch.
    /// Returns flat `[u32]` mask: `mask[poly * grid_steps^2 + row * grid_steps + col]`.
    pub fn evaluate_grid_batch(
        &self,
        polygons: &[&Polygon<f64>],
        grid_steps: u32,
    ) -> Result<Vec<u32>> {
        // Build flat vertex buffer + headers
        let mut verts = Vec::<f32>::new();
        let mut headers = Vec::<GridPolyHeader>::new();
        for poly in polygons {
            let bb = poly.bounding_rect().unwrap();
            let coords = poly.exterior();
            let v0 = verts.len() as u32 / 2;
            for c in coords.0.iter() {
                verts.push(c.x as f32);
                verts.push(c.y as f32);
            }
            let vc = coords.0.len() as u32;
            headers.push(GridPolyHeader {
                vertex_offset: v0,
                vertex_count: vc,
                min_x: bb.min().x as f32,
                min_y: bb.min().y as f32,
                max_x: bb.max().x as f32,
                max_y: bb.max().y as f32,
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
}

#[cfg(test)]

/// Try to create GPU context, returning None if unavailable
pub fn try_create_gpu_context() -> Option<GpuContext> {
    GpuContext::new().ok()
}

/// Get or create a globally cached GPU context.
pub fn get_gpu_context() -> Option<&'static GpuContext> {
    static CTX: std::sync::OnceLock<Option<GpuContext>> = std::sync::OnceLock::new();
    CTX.get_or_init(|| GpuContext::new().ok()).as_ref()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gpu_context_creation() {
        let _ctx = try_create_gpu_context();
    }
}
