#!/usr/bin/env python3
"""
Run IGE solver evolution with OpenEvolve.

Usage:
    # OpenCode Zen (MiniMax M2.5 Free):
    $env:OPENAI_API_KEY = "oc_zen_..."
    python examples/openevolve_ige/run_ige_evolution.py

    # Evolve a specific module:
    python examples/openevolve_ige/run_ige_evolution.py --target bcrs/expand.rs

    # Speed-mode, longer run:
    python examples/openevolve_ige/run_ige_evolution.py --iterations 100

    # Axis-aligned solver:
    python examples/openevolve_ige/run_ige_evolution.py --target axis_aligned/vertex_grid.rs

    # Custom mode:
    python examples/openevolve_ige/run_ige_evolution.py --target bcrs/parallel.rs --mode speed
"""

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

if sys.platform == "win32":
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")
    os.environ.setdefault("PYTHONIOENCODING", "utf-8")

from openevolve import run_evolution

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
OUTPUT_DIR = REPO_ROOT / "target" / "openevolve_runs"
CRATE_SRC = REPO_ROOT / "crates" / "ige-core" / "src"

# ── Target mapping (must match evaluator.py TARGETS) ──────────────────────

TARGETS = {
    "bcrs/parallel.rs":       {"flag": "--parallel"},
    "bcrs/candidates.rs":     {"flag": "--parallel"},
    "bcrs/certify.rs":        {"flag": "--parallel"},
    "bcrs/expand.rs":         {"flag": "--parallel"},
    "bcrs/fallback.rs":       {"flag": "--parallel"},
    "bcrs/fast.rs":           {"flag": "--parallel"},
    "bcrs/prepare.rs":        {"flag": "--parallel"},
    "bcrs/polish.rs":         {"flag": "--parallel"},
    "bcrs/refine.rs":         {"flag": "--parallel"},
    "axis_aligned/vertex_grid.rs":  {"flag": "--baseline"},
    "axis_aligned/histogram.rs":    {"flag": "--baseline"},
    "axis_aligned/bcrs_grid.rs":    {"flag": "--baseline"},
    "axis_aligned/sdf.rs":          {"flag": "--baseline"},
    "axis_aligned/containment.rs":  {"flag": "--baseline"},
    "tuning.rs":                    {"flag": "--parallel"},
    "tuning_bcrs.rs":               {"flag": "--parallel"},
    "tuning_axis_aligned.rs":       {"flag": "--baseline"},
}

DEFAULT_TARGET = "bcrs/parallel.rs"
VALID_MODES = ["balanced", "accuracy", "speed", "tune"]


def _detect_provider() -> str:
    key = os.environ.get("OPENAI_API_KEY", "")
    if key.startswith("oc_zen_"):
        return "opencode-zen"
    if key.startswith("AIza"):
        return "gemini"
    return "openai-compatible"


def _set_api_base():
    if "OPENAI_BASE_URL" in os.environ or "OPENAI_API_BASE" in os.environ:
        return
    key = os.environ.get("OPENAI_API_KEY", "")
    if key.startswith("oc_zen_"):
        os.environ["OPENAI_BASE_URL"] = "https://opencode.ai/zen/v1"
    elif key.startswith("AIza"):
        os.environ["OPENAI_BASE_URL"] = "https://generativelanguage.googleapis.com/v1beta/openai/"


def _load_source(target: str, mode: str) -> str:
    """
    Load the real source code from the crate and prepend marker comments.
    This guarantees the LLM evolves the correct module code.

    Targets like tuning_bcrs.rs / tuning_axis_aligned.rs are aliases that
    resolve to the same physical file (tuning.rs).
    """
    # Resolve alias targets to their physical files
    alias_map = {
        "tuning_bcrs.rs": "tuning.rs",
        "tuning_axis_aligned.rs": "tuning.rs",
    }
    physical = alias_map.get(target, target)
    src_path = CRATE_SRC / physical
    if not src_path.exists():
        raise FileNotFoundError(f"Source not found: {src_path}")

    source = src_path.read_text(encoding="utf-8")
    # OpenEvolve checks for EVOLVE-BLOCK-START to decide whether to wrap
    # the code in its own `#`-style markers.  We provide Rust-compatible
    # `//`-style markers ourselves so OpenEvolve skips the wrapping.
    markers = (
        f"// OPENEVOLVE-TARGET: {target}\n"
        f"// OPENEVOLVE-MODE: {mode}\n"
        f"// EVOLVE-BLOCK-START\n"
    )
    return markers + source + "\n// EVOLVE-BLOCK-END"


def _build_config(config_path, iterations):
    from openevolve.config import Config
    cfg = Config.from_yaml(str(config_path))
    if iterations:
        cfg.max_iterations = iterations
    api_key = os.environ.get("OPENAI_API_KEY")
    api_base = os.environ.get("OPENAI_BASE_URL") or os.environ.get("OPENAI_API_BASE")
    if api_key or api_base:
        cfg.llm.update_model_params({"api_key": api_key, "api_base": api_base})
        if api_base:
            cfg.llm.api_base = api_base
    return cfg


def main():
    parser = argparse.ArgumentParser(
        description="Run IGE solver evolution with OpenEvolve",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--target", default=DEFAULT_TARGET,
        choices=list(TARGETS.keys()),
        help=f"Module to evolve (default: {DEFAULT_TARGET})")
    parser.add_argument("--mode", default="balanced", choices=VALID_MODES,
        help="Optimisation objective (default: balanced)")
    parser.add_argument("--config",
        default=REPO_ROOT / "examples" / "openevolve_ige" / "config.yaml",
        type=Path, help="OpenEvolve config YAML")
    parser.add_argument("--evaluator",
        default=REPO_ROOT / "examples" / "openevolve_ige" / "evaluator.py",
        type=Path, help="Evaluator module path")
    parser.add_argument("--iterations", type=int, default=20,
        help="Number of evolution iterations (default 20, ~1 min)")
    args = parser.parse_args()

    _detect_provider()
    _set_api_base()

    if not os.environ.get("OPENAI_API_KEY"):
        print("❌  OPENAI_API_KEY not set.")
        print()
        print("    Get a FREE OpenCode Zen key (minimax-m2.5-free):")
        print("      1. Sign in at https://opencode.ai/auth")
        print("      2. Copy your API key")
        print("      3. Run:  $env:OPENAI_API_KEY = 'oc_zen_...'")
        print(f"      4. python {__file__}")
        sys.exit(1)

    # Load source from the real crate (no more manual initial_program.rs files)
    source = _load_source(args.target, args.mode)
    print(f"  Source: {args.target} ({len(source)} chars)")
    print(f"  Mode:   {args.mode}")
    print(f"  Iters:  {args.iterations}")
    print(f"  LLM:    {'minimax-m2.5-free (OpenCode Zen)' if 'oc_zen_' in os.environ.get('OPENAI_API_KEY','') else 'gemini'}")
    print(f"  Est:    {args.iterations * 2 / 60:.1f} min")
    print(f"{'='*60}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    cfg = _build_config(args.config, args.iterations)

    # Write initial program to a .rs file so OpenEvolve doesn't create a .py temp
    initial_path = OUTPUT_DIR / "initial_program.rs"
    initial_path.write_text(source, encoding="utf-8")

    t_start = time.perf_counter()
    result = run_evolution(
        initial_program=str(initial_path),  # pass as file path, not string
        evaluator=str(args.evaluator),
        config=cfg,
        iterations=args.iterations,
        output_dir=str(OUTPUT_DIR),
        cleanup=False,
    )
    elapsed = time.perf_counter() - t_start

    summary = {
        "target": args.target,
        "mode": args.mode,
        "iterations": args.iterations,
        "elapsed_s": round(elapsed, 1),
        "best_metrics": result.metrics,
    }
    summary_path = OUTPUT_DIR / "last_run_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2))

    print(f"\n{'='*60}")
    print(f"  ✅  Done — {elapsed/60:.1f} min ({args.iterations} iterations)")
    print(f"  Best fitness: {result.metrics}")
    if result.best_code:
        best_path = OUTPUT_DIR / "best_candidate.rs"
        best_path.write_text(result.best_code)
        print(f"  Best candidate: {best_path}")
    print(f"  Full summary:   {summary_path}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
