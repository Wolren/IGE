# Folder Layout

```
ige/
├── Cargo.toml                    # Workspace root
├── rust-toolchain.toml           # MSRV: 1.75
│
├── crates/
│   ├── ige-core/                # Core library (Rust)
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs            # Public API re-exports
│   │       ├── prelude.rs        # Convenience re-exports
│   │       ├── shared/
│   │       │   └── mod.rs       # Rectangle, Result, LirError, rotate_polygon
│   │       ├── tuning.rs         # All default constants
│   │       ├── solvers/
│   │       │   ├── lir/
│   │       │   │   ├── oriented/
│   │       │   │   │   ├── mod.rs          # LirOrientedOptions, LirOrientedResult
│   │       │   │   │   ├── candidates.rs   # edge_candidate_angles, pca_candidate_angles, upper_bound_area
│   │       │   │   │   ├── parallel.rs     # Main solver: solve_lir_oriented_parallel (~1300 lines)
│   │       │   │   │   ├── expand.rs       # SDF expansion: multi_probe_sdf_v/h, expand_rect_to_boundary
│   │       │   │   │   ├── certify.rs       # Certification: rect_sdf_max_poly, certify_and_adjust
│   │       │   │   │   ├── edge_anchor.rs  # Edge-anchored candidates: generate_edge_anchored_candidates
│   │       │   │   │   ├── fast.rs         # Fast path for convex polygons
│   │       │   │   │   ├── prepare.rs      # Polygon validation and simplification
│   │       │   │   │   └── histogram.rs    # LRIH implementation
│   │       │   │   └── axis_aligned/
│   │       │   │       ├── mod.rs          # Exact vertex-grid solver
│   │       │   │       ├── sdf.rs          # Polygon SDF implementation
│   │       │   │       ├── vertex_grid.rs  # Grid construction
│   │       │   │       └── exact.rs       # Daniels-Milenkovic-Roth exact solve
│   │       │   ├── ler/                   # Largest Empty Rectangle (experimental)
│   │       │   │   ├── mod.rs
│   │       │   │   └── axis_aligned.rs
│   │       │   └── mic/
│   │       │       ├── mod.rs             # MIC solver entry
│   │       │       ├── solver/
│   │       │       │   ├── exact.rs       # Two-sweep exact implementation
│   │       │       │   └── sdf.rs         # MIC-specific SDF descent
│   │       │       └── visualize.rs
│   │       ├── gpu/                       # GPU acceleration (behind "gpu" feature)
│   │       │   ├── mod.rs
│   │       │   └── sdf_wgpu.rs
│   │       ├── benches/                  # Criterion benchmarks
│   │       │   ├── lir_oriented_bench.rs
│   │       │   ├── lir_axis_aligned_bench.rs
│   │       │   ├── mic_bench.rs
│   │       │   └── real_world_bench.rs
│   │       ├── demos/
│   │       │   └── visualize.rs          # CLI tool: --visualize polygons
│   │       └── tests/
│   │
│   ├── ige-py/                # Python bindings via PyO3
│   │   ├── Cargo.toml
│   │   ├── pyproject.toml
│   │   ├── README.md
│   │   └── src/
│   │       └── lib.rs         # solve_oriented_lir_py, solve_bcrs_py, solve_axis_aligned_py, etc.
│   │
│   └── ige-c/                 # C FFI
│       ├── Cargo.toml
│       ├── ige.h              # Public C header
│       └── src/
│           └── lib.rs         # C-exported functions: solve_oriented_lir, solve_axis_aligned_lir, etc.
│
├── gis/                       # Python GIS integration
│   ├── library/
│   │   ├── __init__.py        # solve_bcrs, solve_axis_aligned, maximum_inscribed_circle wrappers
│   │   └── qgis_algorithm.py  # IgeAlgorithmProvider for QGIS Processing framework
│   └── qgis/
│       └── scripts/
│           ├── oriented_lir.py    # QGIS Processing algorithm: "oriented_lir"
│           └── axis_aligned_lir.py
│
└── docs/
    └── book/                  # mdBook documentation
        ├── book.toml
        └── src/
            ├── SUMMARY.md
            ├── index.md
            ├── getting-started/
            ├── algorithms/
            ├── theory/
            ├── performance/
            └── dev-reference/
```

## Key Files

| File | Role |
|---|---|
| `src/lib.rs` | Public API entry: `solve_lir_oriented`, `solve_axis_aligned`, `maximum_inscribed_circle` |
| `src/tuning.rs` | Single source of truth for all default constants |
| `src/solvers/lir/oriented/parallel.rs` | Core solver — ~1300 lines containing coarse sweep, fine solve, bootstrap |
| `src/solvers/lir/oriented/candidates.rs` | Angle generation — edge voting, PCA, UB computation |
| `src/solvers/lir/oriented/expand.rs` | SDF expansion — binary search per edge, Lipschitz skipping |
| `src/solvers/lir/oriented/certify.rs` | Certification — SDF sampling, shrink adjustment |
| `src/solvers/lir/oriented/edge_anchor.rs` | Edge-anchored candidates — ~1000 lines of support-based generation |
| `src/solvers/lir/axis_aligned/mod.rs` | Exact vertex-grid solver — smaller, simpler than oriented |
| `src/solvers/mic/solver/exact.rs` | MIC two-sweep exact algorithm |