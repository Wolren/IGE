# ige - Inscribed Geometry Engine

Fast largest inscribed rectangle algorithms written in Rust with Python bindings.

## Install

```bash
pip install ige
```

## Usage

```python
import ige

# Axis-aligned LIR
rect = ige.solve_axis_aligned_py(
    [(0, 0), (10, 0), (10, 10), (0, 10)],
    max_aspect_ratio=2.0
)
print(f"Area: {rect.area}")

# Oriented LIR (BCRS)
rect = ige.solve_oriented_lir_py(
    [(0, 0), (8, 1), (7, 7), (2, 8), (-1, 4)],
    rotation_degrees=45.0
)
print(f"Area: {rect.area}, Angle: {rect.angle_deg}")
```

## Algorithms

- **Axis-aligned**: Exact vertex-grid algorithm (Daniels et al. 1997)
- **Oriented**: BCRS algorithm with SDF-guided expansion

## See Also

- QGIS Processing Provider: https://github.com/anomalyco/IGE