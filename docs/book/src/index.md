# IGE — Inscribed Geometry Engine

**IGE** (Inscribed Geometry Engine) is a Rust library for solving inscribed-shape problems in arbitrary polygons — primarily the *largest inscribed rectangle* (LIR) and *maximum inscribed circle* (MIC).

## Core Algorithms

| Solver | Orientation | Precision | Typical Use |
|---|---|---|---|
| `solve_lir_oriented` | Any rotation angle | Approximate | General polygons, max area |
| `solve_axis_aligned` | Axis-aligned only | Exact | Fast path, correctness-critical |
| `maximum_inscribed_circle` | Any | Exact | Circular buffer, facility placement |

## Crates

| Crate | Language | Purpose |
|---|---|---|
| `ige-core` | Rust | Core algorithms, the engine |
| `ige-py` | Python 3 | PyO3 bindings, pip-installable |
| `ige-c` | C | C FFI header, FFI bindings |

## Quick Start

```rust
use ige_core::solve_lir_oriented;

let polygon = geo_types::Polygon::new(exterior, vec![]);
let result = solve_lir_oriented(&polygon, &Default::default()).unwrap();
println!("Best area: {:.4}", result.area);
```

```python
from ige import solve_bcrs
result = solve_bcrs([(0,0), (10,0), (10,5), (0,5)])
print(f"Area: {result.area:.4}")
```