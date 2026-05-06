struct PolygonData {
    vertex_count: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
    vertices: array<f32, 4096>,
}

struct RectCandidate {
    x_min: f32,
    y_min: f32,
    x_max: f32,
    y_max: f32,
}

struct CandidateResult {
    area: f32,
    is_valid: u32,
    _pad0: u32,
    _pad1: u32,
}

@group(0) @binding(0) var<storage, read> polygon: PolygonData;
@group(0) @binding(1) var<storage, read> candidates: array<RectCandidate>;
@group(0) @binding(2) var<storage, read_write> results: array<CandidateResult>;

fn point_in_polygon(px: f32, py: f32) -> bool {
    var inside = false;
    var j = polygon.vertex_count - 1;
    
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
        j = i;
    }
    
    return inside;
}

@compute
@workgroup_size(256)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let idx = gid.x;
    if idx >= arrayLength(&candidates) || idx >= arrayLength(&results) {
        return;
    }
    let cand = candidates[idx];
    
    // Check if all corners are inside polygon
    let c1 = point_in_polygon(cand.x_min, cand.y_min);
    let c2 = point_in_polygon(cand.x_max, cand.y_min);
    let c3 = point_in_polygon(cand.x_max, cand.y_max);
    let c4 = point_in_polygon(cand.x_min, cand.y_max);
    
    let valid = c1 && c2 && c3 && c4;
    
    let area = (cand.x_max - cand.x_min) * (cand.y_max - cand.y_min);
    
    results[idx].area = area;
    results[idx].is_valid = select(0u, 1u, valid);
}
