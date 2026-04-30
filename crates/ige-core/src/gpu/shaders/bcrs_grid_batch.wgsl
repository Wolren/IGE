// BCRS Batch Grid Scorer — one dispatch processes all polygons in parallel.
//
// Each workgroup: (gid.x, gid.y) = grid cell position, gid.z = polygon index.
// Replaces per-polygon GPU dispatch with single batch dispatch.
// After this shader, CPU runs LRIH via rayon per polygon.

struct Uniforms {
    max_grid_steps: u32,  // grid cells per axis
    n_polygons: u32,
    pad0: u32,
    pad1: u32,
}

struct PolyHeader {
    vertex_offset: u32,
    vertex_count: u32,
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
}

@group(0) @binding(0) var<uniform> uniforms: Uniforms;
@group(0) @binding(1) var<storage, read> poly_verts: array<vec2<f32>>;
@group(0) @binding(2) var<storage, read> poly_headers: array<PolyHeader>;
@group(0) @binding(3) var<storage, read_write> grid_mask: array<u32>;

fn point_in_poly(px: f32, py: f32, v0: u32, vn: u32) -> bool {
    var inside = false;
    var j = vn - 1u;
    for (var i = v0; i < vn; i++) {
        let xi = poly_verts[i].x;
        let yi = poly_verts[i].y;
        let xj = poly_verts[j].x;
        let yj = poly_verts[j].y;
        if ((yi > py) != (yj > py)) {
            let intersect = px < (xj - xi) * (py - yi) / (yj - yi) + xi;
            if intersect {
                inside = !inside;
            }
        }
        j = i;
    }
    return inside;
}

@compute @workgroup_size(8, 8, 1)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let poly_idx = gid.z;
    if poly_idx >= uniforms.n_polygons { return; }

    let h = poly_headers[poly_idx];
    if gid.x >= uniforms.max_grid_steps || gid.y >= uniforms.max_grid_steps { return; }

    let span_x = h.max_x - h.min_x;
    let span_y = h.max_y - h.min_y;
    if span_x <= 0.0 || span_y <= 0.0 { return; }

    let cx = h.min_x + span_x * (f32(gid.x) + 0.5) / f32(uniforms.max_grid_steps);
    let cy = h.min_y + span_y * (f32(gid.y) + 0.5) / f32(uniforms.max_grid_steps);

    let inside = point_in_poly(cx, cy, h.vertex_offset, h.vertex_offset + h.vertex_count);

    let out_idx = poly_idx * uniforms.max_grid_steps * uniforms.max_grid_steps
                + gid.y * uniforms.max_grid_steps + gid.x;
    grid_mask[out_idx] = select(0u, 1u, inside);
}
