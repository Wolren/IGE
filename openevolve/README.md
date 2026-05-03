# OpenEvolve + IGE: Evolving the Solver

## Quick start

```powershell
$env:OPENAI_API_KEY = "oc_zen_..."
python examples/openevolve_ige/run_ige_evolution.py
```

## Targets

```powershell
python run_ige_evolution.py --target bcrs/parallel.rs         # parallel field solver
python run_ige_evolution.py --target bcrs/expand.rs           # SDF expansion
python run_ige_evolution.py --target bcrs/candidates.rs       # angle generation
python run_ige_evolution.py --target axis_aligned/sdf.rs      # signed-distance-field
python run_ige_evolution.py --target axis_aligned/histogram.rs # LRIH kernel
```

10 modules total. The source is auto-loaded from the real crate — no manual initial program files.

## Modes

```powershell
python run_ige_evolution.py --mode balanced   # default — fill_rate / time^0.61
python run_ige_evolution.py --mode accuracy   # fill only, ignore speed
python run_ige_evolution.py --mode speed      # fill_per_ms
python run_ige_evolution.py --mode tune       # constants only
```

## Iterations

```powershell
python run_ige_evolution.py --iterations 20   # default
python run_ige_evolution.py --iterations 100
```
