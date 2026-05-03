//! Centralised tuning constants for all IGE solvers.
//!
//! OpenEvolve targets this file with `--target tuning.rs --mode tune`.
//! All solver modules read from here -- no hardcoded constants elsewhere.
//! Edit the values here to affect all solvers consistently.
//!
//! Ranges after each constant show reasonable min/max for OpenEvolve evolution.

// -- BCRS standard pipeline (bcrs/mod.rs) ----------------------------------
pub(crate) const TOP_K: usize = 10;               // candidates forwarded to refine [1..10]
pub(crate) const GRID_COARSE: usize = 32;        // coarse sweep resolution [8..128]
pub(crate) const GRID_FINE: usize = 64;          // fine grid for fallback [8..256]
pub(crate) const POLISH_HALFWIDTH: f64 = 3.0;    // Brent polish +-range [1..6]
pub(crate) const POLISH_XATOL: f64 = 0.02;        // Brent convergence (deg) [0.005..0.1]
pub(crate) const PRUNE_MARGIN: f64 = 0.92;        // angle upper-bound prune [0.5..0.99]
pub(crate) const ANGLE_DELTA: f64 = 0.5;          // offset around polished angle (deg) [0.1..2.0]
pub(crate) const TOP_TRIALS: usize = 2;           // angle variants per candidate [1..5]
pub(crate) const CERT_EPS: f64 = 1e-7;            // certification SDF epsilon [1e-12..1e-6]
pub(crate) const CERT_MAX_SHRINK: f64 = 0.20;     // max shrink fraction [0.05..0.50]

// -- BCRS parallel field (bcrs/parallel.rs) --------------------------------
pub(crate) const FIELD_MIN_ANGLES: usize = 45;     // min regular-scan angles [8..90]
pub(crate) const FIELD_ANGLE_STEP: usize = 5;      // regular step (deg) [1..10]
pub(crate) const FIELD_MAX_COORDS: usize = 300;    // vertex-grid max coords [50..1000]

// -- Axis-aligned vertex-grid (axis_aligned/vertex_grid.rs) ----------------
pub(crate) const AA_SUBDIV_LEVELS_HIGH: u32 = 3;   // levels for <=4 unique coords
pub(crate) const AA_SUBDIV_LEVELS_MED: u32 = 2;    // for <=12 unique coords
pub(crate) const AA_SUBDIV_LEVELS_LOW: u32 = 1;    // for >12 unique coords
pub(crate) const AA_SMALL_VERTEX_CUTOFF: usize = 12;  // use per-cell contains <= this

// -- Axis-aligned grid solver (axis_aligned/bcrs_grid.rs) ------------------
pub(crate) const AA_GRID_MAX_COORDS: usize = 300;  // max coords before uniform fallback

// -- SDF expansion (bcrs/expand.rs) ----------------------------------------
pub(crate) const EXPAND_BINARY_STEPS: usize = 24;  // binary search depth per side [8..48]
pub(crate) const EXPAND_ITERS: usize = 3;          // full expansion outer passes [1..6]

// -- Containment verification (axis_aligned/containment.rs) ----------------
pub(crate) const CONTAIN_BOUNDARY_EPS: f64 = 1e-9; // corner-on-boundary tolerance
pub(crate) const CONTRACT_BINARY_ITERS: usize = 32; // per-side binary search depth [8..64]
