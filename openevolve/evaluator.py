"""
OpenEvolve evaluator for IGE — evolves Rust solver source code.

Modes (parsed from a ``// OPENEVOLVE-MODE: ...`` comment in the candidate):

  ``accuracy``  — maximise fill rate (area / polygon area).  Speed is ignored.
  ``speed``     — maximise (fill_rate / avg_time_ms).  LLM may restructure loops,
                  parallelism, and data layouts for performance.
  ``tune``      — adjust constants only (grid sizes, thresholds, step counts).
                  The LLM must NOT change algorithm structure or control flow.

Target (parsed from ``// OPENEVOLVE-TARGET: ...``):

  ``bcrs/parallel.rs``             — parallel ray-shooting field solver
  ``axis_aligned/vertex_grid.rs``  — exact vertex-grid axis-aligned solver

API: set OPENAI_API_KEY + optionally OPENAI_BASE_URL for any OpenAI-compatible
provider (OpenAI, Google Gemini, Ollama, vLLM, …).
"""

import subprocess
import re
import sys
import time
import json as json_mod
import shutil
from pathlib import Path

from openevolve.evaluation_result import EvaluationResult

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
BACKUP_DIR = REPO_ROOT / "examples" / "openevolve_ige"

TARGETS = {
    "bcrs/parallel.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "bcrs" / "parallel.rs",
        "benchmark_flag": "--parallel",
    },
    "bcrs/candidates.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "bcrs" / "candidates.rs",
        "benchmark_flag": "--parallel",
    },
    "bcrs/certify.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "bcrs" / "certify.rs",
        "benchmark_flag": "--parallel",
    },
    "bcrs/expand.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "bcrs" / "expand.rs",
        "benchmark_flag": "--parallel",
    },
    "bcrs/fallback.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "bcrs" / "fallback.rs",
        "benchmark_flag": "--parallel",
    },
    "bcrs/fast.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "bcrs" / "fast.rs",
        "benchmark_flag": "--parallel",
    },
    "bcrs/prepare.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "bcrs" / "prepare.rs",
        "benchmark_flag": "--parallel",
    },
    "bcrs/polish.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "bcrs" / "polish.rs",
        "benchmark_flag": "--parallel",
    },
    "bcrs/refine.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "bcrs" / "refine.rs",
        "benchmark_flag": "--parallel",
    },
    "axis_aligned/vertex_grid.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "axis_aligned" / "vertex_grid.rs",
        "benchmark_flag": "--baseline",
    },
    "axis_aligned/histogram.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "axis_aligned" / "histogram.rs",
        "benchmark_flag": "--baseline",
    },
    "axis_aligned/bcrs_grid.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "axis_aligned" / "bcrs_grid.rs",
        "benchmark_flag": "--baseline",
    },
    "axis_aligned/sdf.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "axis_aligned" / "sdf.rs",
        "benchmark_flag": "--baseline",
    },
    "axis_aligned/containment.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "axis_aligned" / "containment.rs",
        "benchmark_flag": "--baseline",
    },
    "tuning.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "tuning.rs",
        "benchmark_flag": "--parallel",
    },
    "tuning_bcrs.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "tuning.rs",
        "benchmark_flag": "--parallel",
    },
    "tuning_axis_aligned.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "tuning.rs",
        "benchmark_flag": "--baseline",
    },
    "axis_aligned/containment.rs": {
        "module": REPO_ROOT / "crates" / "ige-core" / "src" / "axis_aligned" / "containment.rs",
        "benchmark_flag": "--baseline",
    },
}

DEFAULT_TARGET = "bcrs/parallel.rs"
DEFAULT_MODE = "balanced"

MODES = {
    "accuracy": {
        "fitness": lambda fill_rate, avg_ms, n_ok, n_total:
            fill_rate * (1.0 + n_ok / max(n_total, 1)),
        "desc": "maximise fill-rate",
    },
    "balanced": {
        "fitness": lambda fill_rate, avg_ms, n_ok, n_total:
            (fill_rate / max(avg_ms ** 0.61, 0.01)) * (1.0 + n_ok / max(n_total, 1)),
        "desc": "maximise fill-rate per sub-linear time penalty",
    },
    "speed": {
        "fitness": lambda fill_rate, avg_ms, n_ok, n_total:
            (fill_rate / max(avg_ms, 0.01)) * (1.0 + n_ok / max(n_total, 1)),
        "desc": "maximise fill-rate-per-millisecond",
    },
    "tune": {
        "fitness": lambda fill_rate, avg_ms, n_ok, n_total:
            fill_rate * (1.0 + n_ok / max(n_total, 1)),
        "desc": "adjust constants only",
    },
}


def _detect(line_prefix: str, source: str, default: str) -> str:
    m = re.search(rf"//\s*{re.escape(line_prefix)}:\s*(\S+)", source)
    return m.group(1) if m else default


def _backup_path(target_key: str) -> Path:
    safe = target_key.replace("/", "_").replace(".rs", "")
    return BACKUP_DIR / f".backup_{safe}.rs"


def evaluate(program_path: str) -> EvaluationResult:
    src_path = Path(program_path)
    if not src_path.exists():
        return _error("candidate file not found")

    source = src_path.read_text(encoding="utf-8")

    mode = _detect("OPENEVOLVE-MODE", source, DEFAULT_MODE)
    target_key = _detect("OPENEVOLVE-TARGET", source, DEFAULT_TARGET)

    if target_key not in TARGETS:
        return _error(f"unknown target '{target_key}'")
    if mode not in MODES:
        return _error(f"unknown mode '{mode}'")

    target = TARGETS[target_key]
    mode_info = MODES[mode]
    module_path = target["module"]
    bench_flag = target["benchmark_flag"]

    backup = _backup_path(target_key)
    if module_path.exists():
        shutil.copy2(module_path, backup)

    try:
        module_path.write_text(source, encoding="utf-8")

        # Build release binary (single invocation, no double build)
        t0 = time.perf_counter()
        result = subprocess.run(
            ["cargo", "build", "--package", "ige-core", "--release", "--example", "visualize"],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=180,
        )
        compile_time = time.perf_counter() - t0

        if result.returncode != 0:
            _log_stderr(result.stderr)
            return EvaluationResult(
                metrics=_metrics(fill_rate=0.0, compile_time_s=compile_time),
                artifacts={"stderr": _truncate(result.stderr, 4000), "error": "compilation failed"},
            )

        # Run benchmark binary directly (no cargo run overhead)
        binary = REPO_ROOT / "target" / "release" / "examples" / "visualize.exe"
        if not binary.exists():
            binary = REPO_ROOT / "target" / "release" / "examples" / "visualize"
        cmd = [str(binary), bench_flag, "--json"]

        t0 = time.perf_counter()
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=300,
                                cwd=REPO_ROOT)
        wall_ms = (time.perf_counter() - t0) * 1000

        if result.returncode != 0:
            _log_stderr(result.stderr)
            return EvaluationResult(
                metrics=_metrics(wall_time_ms=min(wall_ms, 999.0)),
                artifacts={"stderr": _truncate(result.stderr, 4000), "error": "benchmark crashed"},
            )

        # Parse JSON metrics
        try:
            data = json_mod.loads(result.stdout.strip())
        except (json_mod.JSONDecodeError, ValueError):
            return EvaluationResult(
                metrics=_metrics(wall_time_ms=min(wall_ms, 999.0)),
                artifacts={"stdout": _truncate(result.stdout or "", 2000), "error": "JSON parse failed"},
            )

        fill_rate = data.get("fill_rate", 0.0)
        avg_ms = data.get("avg_ms", 0.0)
        n_ok = data.get("success", 0)
        n_total = data.get("total", 20)
        per_shape = data.get("per_shape_pct", {})

        # Reward high median (typical coverage) over aggregate fill
        median_pct = per_shape.get("median", 0.0)
        mean_pct = per_shape.get("mean", 0.0)

        robust_fill = (fill_rate * 0.5 + median_pct * 0.5) / 100.0

        combined = mode_info["fitness"](robust_fill, avg_ms, n_ok, n_total)

        print(f"  ── fill={fill_rate*100:.1f}%  med={median_pct:.1f}%  avg={avg_ms:.2f}ms  ok={n_ok}/{n_total}  "
              f"build={compile_time:.1f}s  bench={wall_ms:.0f}ms  mode={mode}",
              file=sys.stderr)

        return EvaluationResult(
            metrics=_metrics(
                fill_rate=fill_rate,
                avg_time_ms=avg_ms,
                wall_time_ms=wall_ms,
                n_ok=float(n_ok),
                median_pct=median_pct,
                combined_score=combined,
                mode=mode,
            ),
            artifacts={
                "stdout": _truncate(result.stdout or "", 2000),
                "compile_time_s": compile_time,
                "fill_rate": fill_rate,
                "avg_time_ms": avg_ms,
                "median_pct": median_pct,
                "mode": mode,
                "target": target_key,
            },
        )

    except Exception as exc:
        import traceback
        traceback.print_exc()
        return _error(f"exception: {exc}")

    finally:
        if backup.exists():
            shutil.copy2(backup, module_path)
            backup.unlink()


def _error(msg: str) -> EvaluationResult:
    return EvaluationResult(
        metrics=_metrics(error=1.0),
        artifacts={"error": msg},
    )

def _metrics(**kw) -> dict:
    defaults = {"fill_rate": 0.0, "avg_time_ms": 999.0, "n_ok": 0.0, "combined_score": 0.0, "wall_time_ms": 999.0}
    defaults.update(kw)
    return defaults

def _truncate(text: str, limit: int = 4000) -> str:
    return text[:limit] + ("\n… [truncated]" if len(text) > limit else "")

def _log_stderr(text: str) -> None:
    lines = [l for l in text.strip().split("\n") if l.strip()]
    if lines:
        print("\n" + "!" * 60, file=sys.stderr)
        print("!  COMPILATION FAILED", file=sys.stderr)
        print("!" * 60, file=sys.stderr)
        for line in lines[:15]:
            print(f"!  {line}", file=sys.stderr)
        if len(lines) > 15:
            print(f"!  … ({len(lines) - 15} more lines)", file=sys.stderr)
        print("!" * 60 + "\n", file=sys.stderr)
