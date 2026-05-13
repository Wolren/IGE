//! Command-line argument parsing and solver configuration.

use ige_core::MaskBackend;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxisCliSolver {
    Vertex,
    Exact,
    Grid,
}

/// Parse the `--mask-backend` flag, respecting GPU feature gating.
#[cfg(feature = "gpu")]
pub fn parse_mask_backend(args: &[String]) -> MaskBackend {
    let value = args
        .iter()
        .position(|a| a == "--mask-backend")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("auto");
    match value {
        "cpu" => MaskBackend::Cpu,
        "gpu-sdf" => MaskBackend::GpuSdf,
        "gpu-grid" => MaskBackend::GpuGridBatch,
        "auto" => MaskBackend::Auto,
        _ => {
            eprintln!("Unknown --mask-backend '{value}', using auto");
            MaskBackend::Auto
        }
    }
}

#[cfg(not(feature = "gpu"))]
pub fn parse_mask_backend(args: &[String]) -> MaskBackend {
    if let Some(value) = args
        .iter()
        .position(|a| a == "--mask-backend")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
    {
        if value != "cpu" && value != "auto" {
            eprintln!("--mask-backend={value} requires --features gpu; using cpu");
        }
    }
    MaskBackend::Cpu
}

/// Human-readable backend name for display in titles and logs.
#[cfg(feature = "gpu")]
pub fn mask_backend_name(backend: MaskBackend) -> &'static str {
    match backend {
        MaskBackend::Cpu => "cpu",
        MaskBackend::GpuSdf => "gpu-sdf",
        MaskBackend::GpuGridBatch => "gpu-grid",
        MaskBackend::Auto => "auto",
    }
}

#[cfg(not(feature = "gpu"))]
pub fn mask_backend_name(_backend: MaskBackend) -> &'static str {
    "cpu"
}

/// Parse the `--axis-solver` flag.
pub fn parse_axis_solver(args: &[String]) -> AxisCliSolver {
    // Backward compatibility: old flag implied grid baseline mode.
    if args.contains(&"--baseline-grid".to_string()) {
        return AxisCliSolver::Grid;
    }

    let value = args
        .iter()
        .position(|a| a == "--axis-solver")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("vertex");

    match value {
        "vertex" | "vertex_grid" => AxisCliSolver::Vertex,
        "exact" => AxisCliSolver::Exact,
        "grid" => AxisCliSolver::Grid,
        _ => {
            eprintln!("Unknown --axis-solver '{value}', using vertex");
            AxisCliSolver::Vertex
        }
    }
}

/// Human-readable solver name for display in titles and logs.
pub fn axis_solver_name(solver: AxisCliSolver) -> &'static str {
    match solver {
        AxisCliSolver::Vertex => "vertex",
        AxisCliSolver::Exact => "exact",
        AxisCliSolver::Grid => "grid",
    }
}

/// Collect all CLI flags into a single configuration struct.
#[derive(Debug, Clone)]
pub struct CliConfig {
    pub use_mic_compare: bool,
    pub real_only: bool,
    pub use_parallel: bool,
    pub use_sa: bool,
    pub use_bootstrap_seeds: bool,
    pub use_pca_axes: bool,
    pub use_early_stopping: bool,
    pub use_edge_anchored: bool,
    pub use_gradient_expand: bool,
    pub use_ler: bool,
    pub use_approx_oriented: bool,
    pub use_json: bool,
    pub limit: Option<usize>,
    pub file_path: Option<String>,
    pub axis_solver: AxisCliSolver,
    pub mask_backend: MaskBackend,
}

impl CliConfig {
    pub fn from_args(args: &[String]) -> Self {
        let use_mic_compare = args.contains(&"--mic-compare".to_string());
        let real_only = args.contains(&"--real-only".to_string());
        let use_parallel = args.contains(&"--parallel".to_string());
        let use_sa = args.contains(&"--sa".to_string());
        let use_bootstrap_seeds = args.contains(&"--bootstrap-seeds".to_string());
        let use_pca_axes = args.contains(&"--pca-axes".to_string());
        let use_early_stopping = args.contains(&"--early-stop".to_string());
        let use_edge_anchored = args.contains(&"--edge-anchored".to_string());
        let use_gradient_expand = args.contains(&"--gradient-expand".to_string());
        let use_ler = args.contains(&"--ler".to_string());
        let use_approx_oriented = !args.contains(&"--baseline".to_string());
        let use_json = args.contains(&"--json".to_string());
        let limit = args.iter()
            .position(|a| a == "--limit")
            .and_then(|i| args.get(i + 1))
            .and_then(|s| s.parse::<usize>().ok());
        let file_path = args.iter()
            .position(|a| a == "--file")
            .and_then(|i| args.get(i + 1))
            .map(|s| s.clone()); // Clone the String to own it
        let axis_solver = parse_axis_solver(args);
        let mask_backend = parse_mask_backend(args);

        Self {
            use_mic_compare,
            real_only,
            use_parallel,
            use_sa,
            use_bootstrap_seeds,
            use_pca_axes,
            use_early_stopping,
            use_edge_anchored,
            use_gradient_expand,
            use_ler,
            use_approx_oriented,
            use_json,
            limit,
            file_path,
            axis_solver,
            mask_backend,
        }
    }
}

/// Generate a human-readable algorithm name for the current configuration.
/// Returns SIMD status string for display.
pub fn simd_status() -> &'static str {
    #[cfg(feature = "simd")]
    { " + SIMD" }
    #[cfg(not(feature = "simd"))]
    { "" }
}

/// Generate a human-readable algorithm name for the current configuration.
pub fn algo_name(config: &CliConfig) -> String {
    let simd = simd_status();
    if config.use_mic_compare {
        format!("MIC exact vs GEOS{}", simd)
    } else if config.use_ler {
        format!("LER Axis-Aligned{}", simd)
    } else if config.use_sa && config.use_parallel {
        format!("LIR Approx Oriented + SA + local angle polish{}", simd)
    } else if config.use_sa {
        format!("LIR Approx Oriented + SA rescue{}", simd)
    } else if config.use_bootstrap_seeds && config.use_edge_anchored {
        format!("LIR Approx Oriented + bootstrap seeds + edge-anchored{}", simd)
    } else if config.use_bootstrap_seeds && config.use_parallel {
        format!("LIR Approx Oriented + bootstrap seeds + local angle polish{}", simd)
    } else if config.use_bootstrap_seeds {
        format!("LIR Approx Oriented + bootstrap seeds{}", simd)
    } else if config.use_edge_anchored {
        format!("LIR Approx Oriented + edge-anchored{}", simd)
    } else if config.use_parallel {
        format!("LIR Approx Oriented + local angle polish{}", simd)
    } else if config.use_approx_oriented {
        format!("LIR Approx Oriented{}", simd)
    } else {
        match config.axis_solver {
            AxisCliSolver::Grid => {
                format!(
                    "axis-aligned (solver={}, mask={}){}",
                    axis_solver_name(config.axis_solver),
                    mask_backend_name(config.mask_backend),
                    simd
                )
            }
            _ => format!("axis-aligned (solver={}){}", axis_solver_name(config.axis_solver), simd),
        }
    }
}