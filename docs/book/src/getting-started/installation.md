# Installation

## Rust / Cargo

Add `ige-core` to your `Cargo.toml`:

```toml
[dependencies]
ige-core = { git = "https://github.com/wolren/ige" }
```

For local development against this repository:

```toml
ige-core = { path = "/path/to/ige/crates/ige-core" }
```

The core crate depends only on `geo` and `rayon`. No external system libraries required.

Minimum supported Rust version (MSRV): 1.75.

## Python

The `ige` package is published on PyPI:

```bash
pip install ige
```

Development builds from TestPyPI:

```bash
pip install --index-url https://test.pypi.org/simple/ ige
```

Requires Python 3.8+. The wheel bundles a pre-compiled Rust extension — no Rust toolchain needed on the consumer side.

## C / FFI

Build the C library from the `ige-c` crate:

```bash
cd crates/ige-c
cargo build --release
```

Output:
- **Windows:** `target/release/ige_c.dll` + `ige.h` header
- **Linux:** `target/release/libige_c.so` + `ige.h`
- **macOS:** `target/release/libige_c.dylib` + `ige.h`

Link with `-lige_c`. The header is at `crates/ige-c/ige.h`.

## GIS Integration

For QGIS Processing integration, copy or symlink `gis/library/` into your QGIS Python search path. See the [Bindings](../dev-reference/bindings.md) reference for the QGIS provider setup.

## Feature Flags

`ige-core` exposes optional features via Cargo:

| Feature | Effect |
|---|---|
| `gpu` | Enables GPU SDF via WGPU — adds `wgpu` and `pollster` deps |
| ` profiling` | Enables internal profiling counters |

Default build includes neither — pure CPU, no extra dependencies.