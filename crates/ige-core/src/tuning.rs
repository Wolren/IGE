//! Centralised tuning constants for all IGE solvers.
//!
//! OpenEvolve targets this file with `--target tuning.rs --mode tune`.
//! All solver modules read from here -- no hardcoded constants elsewhere.
//! Edit the values here to affect all solvers consistently.
//!
//! Ranges after each constant show reasonable min/max for OpenEvolve evolution.

// -- LIR Oriented standard pipeline (lir_oriented/mod.rs) --
pub const GRID_COARSE: usize = 32;
pub const GRID_FINE: usize = 64;
pub const TOP_K: usize = 1;
pub const POLISH_HALFWIDTH: f64 = 3.0;
pub const POLISH_XATOL: f64 = 0.02;
pub const PRUNE_MARGIN: f64 = 0.92;
pub const ANGLE_DELTA: f64 = 0.5;
pub const TOP_TRIALS: usize = 2;
pub const CERT_EPS: f64 = 1e-7;
pub const CERT_MAX_SHRINK: f64 = 0.20;

// -- LIR Oriented parallel field (lir_oriented/parallel.rs) --------
pub const FIELD_MIN_ANGLES: usize = 45;
pub const FIELD_ANGLE_STEP: usize = 5;
pub const FIELD_MAX_COORDS: usize = 300;

// -- SDF expansion (lir_oriented/expand.rs) -----------------------------
pub const EXPAND_BINARY_STEPS: usize = 24;
pub const EXPAND_ITERS: usize = 3;

// -- SDF gradient expansion (lir_oriented/expand.rs) --------------------
pub const GRADIENT_EXPAND_STEPS: usize = 15;
pub const GRADIENT_EXPAND_STEP_SIZE: f64 = 0.5;
pub const GRADIENT_EXPAND_GRADIENT_STEP: f64 = 1.0;
pub const GRADIENT_EXPAND_MAX_DIST: f64 = 10.0;
pub const GRADIENT_EXPAND_MARGIN: f64 = 0.3;

// -- Containment verification (lir_axis_aligned/containment.rs) -------------
pub const CONTAIN_BOUNDARY_EPS: f64 = 1e-9;
pub const CONTRACT_BINARY_ITERS: usize = 32;

// -- Axis-Aligned (lir_axis_aligned/*.rs) -------------------
pub const AA_SUBDIV_LEVELS_HIGH: usize = 12;
pub const AA_SUBDIV_LEVELS_MED: usize = 8;
pub const AA_SUBDIV_LEVELS_LOW: usize = 3;
pub const AA_SMALL_VERTEX_CUTOFF: usize = 24;
pub const AA_GRID_MAX_COORDS: usize = 51200;

// -- Axis-Aligned GRID (lir_axis_aligned/*.rs) -------------------

pub const AA_GRID_COARSE_STEPS: usize = 400;
pub const AA_GRID_FINE_STEPS: usize = 400;
pub const AA_BCRS_MAX_COORDS: usize = 4096;
pub const AA_BCRS_GRID_CAP: usize = 120;
pub const AA_EXACT_BINARY_ITERS: usize = 40;
pub const AA_EXACT_REFINE_ITERS: usize = 5;
pub const AA_EXACT_TOP_SEEDS: usize = 16;
pub const AA_EXACT_GRID_CAP: usize = 120;
pub const AA_EPS: f64 = 1e-12;

// -- MIC (mic/solver/exact.rs) -------------------
pub const MIC_BINARY_ITERS: usize = 40;
pub const MIC_EXPANSION_ITERS: usize = 50;
pub const MIC_MAX_SDF_SAMPLES: usize = 200;
pub const MIC_EPS: f64 = 1e-12;
pub const MIC_CANDIDATE_QUANTIZE: f64 = 1e9;
pub const MIC_BASE_TRIPLE_CAP: usize = 64;
pub const MIC_BASE_SS_SEG_CAP: usize = 32;
pub const MIC_BASE_SS_VERT_CAP: usize = 12;
pub const MIC_BASE_SEGS_PER_RING: usize = 3;
pub const MIC_EXT_TRIPLE_CAP: usize = 96;
pub const MIC_EXT_SS_SEG_CAP: usize = 64;
pub const MIC_EXT_SS_VERT_CAP: usize = 32;
pub const MIC_EXT_SEGS_PER_RING: usize = 5;

// -- Oriented expand (expand.rs) -------------------
pub const EXPAND_MAX_SCALE_ITERS: usize = 36;
