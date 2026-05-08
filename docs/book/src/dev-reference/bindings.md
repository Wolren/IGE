# Using Bindings

## Rust

```rust
use ige_core::solve_lir_oriented;

let polygon = geo_types::Polygon::new(exterior, vec![]);
let mut opts = LirOrientedOptions::default();
opts.grid_coarse = 48;
opts.grid_fine = 96;
opts.max_ratio = 3.0;
opts.use_bootstrap_seeds = true;

match solve_lir_oriented(&polygon, &opts) {
    Ok(result) => {
        println!("area: {:.4}", result.area);
        println!("angle: {:.2}°", result.angle_deg);
        if let Some(rect) = result.rect_polygon {
            println!("WKT: {}", rect.to_wkt());
        }
    }
    Err(e) => eprintln!("solve failed: {e}"),
}
```

## Python — Direct

```python
from ige import solve_bcrs, solve_axis_aligned, maximum_inscribed_circle

# Oriented LIR
result = solve_bcrs(
    exterior=[(0,0), (10,0), (10,5), (0,5)],
    max_aspect_ratio=3.0,
    use_bootstrap_seeds=True,
    grid_coarse=48,
    grid_fine=96,
)
print(f"area: {result.area:.4f}, angle: {result.angle_deg:.2f}°")

# Axis-Aligned LIR (exact)
result = solve_axis_aligned(
    exterior=[(0,0), (10,0), (10,5), (0,5)],
    max_grid=64,
)
print(f"area: {result.area:.4f}")

# Maximum Inscribed Circle
result = maximum_inscribed_circle(
    exterior=[(0,0), (10,0), (10,5), (0,5)],
)
print(f"radius: {result.radius:.4f}")
```

## Python — QGIS Processing

```python
import sys
sys.path.append('/path/to/ige/gis/library')

from qgis_algorithm import IgeAlgorithmProvider
from qgis.core importQgApplication

app = QgApplication.instance() or QgApplication([])
provider = IgeAlgorithmProvider()
app.processingRegistry().addProvider(provider)

# Run via GUI or:
from processing.core.Processing import Processing
Processing.runAlgorithm("ige:oriented_lir", {
    'INPUT': '/path/to/polygons.gpkg',
    'OUTPUT': '/path/to/rectangles.gpkg',
    'MAX_RATIO': 3.0,
    'GRID_COARSE': 48,
    'GRID_FINE': 96,
    'USE_BOOTSTRAP': True,
})
```

## C / FFI

```c
#include "ige.h"

int main() {
    double exterior[] = {0.0, 0.0, 10.0, 0.0, 10.0, 5.0, 0.0, 5.0};
    IgeOrientedLirResult result;

    IgeOptions opts = ige_options_default();
    opts.max_aspect_ratio = 3.0;
    opts.use_bootstrap_seeds = 1;
    opts.grid_coarse = 48;
    opts.grid_fine = 96;

    int ok = solve_oriented_lir(exterior, 4, NULL, 0, &opts, &result);
    if (ok == 0) {
        printf("area: %.4f\n", result.area);
        printf("angle: %.2f°\n", result.angle_deg);
        printf("center: (%.4f, %.4f), size: %.4f x %.4f\n",
               result.center_x, result.center_y, result.width, result.height);
    }
    return 0;
}
```

Compile and link:

```bash
gcc -o example example.c -L./target/release -lige_c
```