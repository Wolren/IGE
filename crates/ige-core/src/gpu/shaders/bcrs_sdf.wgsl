// BCRS SDF compute shader — GPU-accelerated signed-distance-field evaluation.
//
// Evaluates the signed distance from each candidate rectangle's sampling points
// (4 corners + 4 edge midpoints) to the polygon boundary.
// Used to accelerate certification and boundary expansion.

struct PolygonData {
    vertex_count: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
    vertices: array<f32, 4096>,
}

struct RectInput {
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32,
}

struct SdfOutput {
    max_sdf: f32,
    corner0_sdf: f32,
    corner1_sdf: f32,
    corner2_sdf: f32,
    corner3_sdf: f32,
    mid0_sdf: f32,
    mid1_sdf: f32,
    mid2_sdf: f32,
    mid3_sdf: f32,
}

@group(0) @binding(0) var<storage, read> polygon: PolygonData;
@group(0) @binding(1) var<storage, read> rects: array<RectInput>;
@group(0) @binding(2) var<storage, read_write> outputs: array<SdfOutput>;

fn point_to_edge_distance(px: f32, py: f32, ax: f32, ay: f32, bx: f32, by: f32) -> f32 {
    let abx = bx - ax;
    let aby = by - ay;
    let len2 = abx * abx + aby * aby;
    if len2 < 1e-20 {
        let dx = px - ax;
        let dy = py - ay;
        return sqrt(dx * dx + dy * dy);
    }
    var t = ((px - ax) * abx + (py - ay) * aby) / len2;
    t = clamp(t, 0.0, 1.0);
    let projx = ax + t * abx;
    let projy = ay + t * aby;
    let dx = px - projx;
    let dy = py - projy;
    return sqrt(dx * dx + dy * dy);
}

fn polygon_sdf(px: f32, py: f32) -> f32 {
    var inside = false;
    var j = polygon.vertex_count - 1u;
    var min_dist: f32 = 1e30;

    for (var i: u32 = 0u; i < polygon.vertex_count; i++) {
        let xi = polygon.vertices[i * 2u];
        let yi = polygon.vertices[i * 2u + 1u];
        let xj = polygon.vertices[j * 2u];
        let yj = polygon.vertices[j * 2u + 1u];

        if ((yi > py) != (yj > py)) {
            let intersect = (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
            if (intersect) {
                inside = !inside;
            }
        }

        let d = point_to_edge_distance(px, py, xi, yi, xj, yj);
        if d < min_dist {
            min_dist = d;
        }

        j = i;
    }

    if inside {
        return -min_dist;
    }
    return min_dist;
}

@compute
@workgroup_size(256)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let idx = gid.x;
    let r = rects[idx];

    // 4 corners
    let sdf_c0 = polygon_sdf(r.x0, r.y0);
    let sdf_c1 = polygon_sdf(r.x1, r.y0);
    let sdf_c2 = polygon_sdf(r.x1, r.y1);
    let sdf_c3 = polygon_sdf(r.x0, r.y1);

    // 4 edge midpoints
    let cx = (r.x0 + r.x1) * 0.5;
    let cy = (r.y0 + r.y1) * 0.5;
    let sdf_m0 = polygon_sdf(cx, r.y0);
    let sdf_m1 = polygon_sdf(r.x1, cy);
    let sdf_m2 = polygon_sdf(cx, r.y1);
    let sdf_m3 = polygon_sdf(r.x0, cy);

    var max_sdf = sdf_c0;
    max_sdf = max(max_sdf, sdf_c1);
    max_sdf = max(max_sdf, sdf_c2);
    max_sdf = max(max_sdf, sdf_c3);
    max_sdf = max(max_sdf, sdf_m0);
    max_sdf = max(max_sdf, sdf_m1);
    max_sdf = max(max_sdf, sdf_m2);
    max_sdf = max(max_sdf, sdf_m3);

    outputs[idx].max_sdf = max_sdf;
    outputs[idx].corner0_sdf = sdf_c0;
    outputs[idx].corner1_sdf = sdf_c1;
    outputs[idx].corner2_sdf = sdf_c2;
    outputs[idx].corner3_sdf = sdf_c3;
    outputs[idx].mid0_sdf = sdf_m0;
    outputs[idx].mid1_sdf = sdf_m1;
    outputs[idx].mid2_sdf = sdf_m2;
    outputs[idx].mid3_sdf = sdf_m3;
}
