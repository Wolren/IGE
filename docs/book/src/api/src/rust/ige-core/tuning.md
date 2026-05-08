# ige-core::tuning <span class="plissken-badge plissken-badge-source" style="display: inline-block; padding: 0.1em 0.35em; font-size: 0.55em; font-weight: 600; border-radius: 0.2em; vertical-align: middle; background: #ff5722; color: white;">Rust</span>


Centralised tuning constants for all IGE solvers.

OpenEvolve targets this file with `--target tuning.rs --mode tune`.
All solver modules read from here -- no hardcoded constants elsewhere.
Edit the values here to affect all solvers consistently.
Ranges after each constant show reasonable min/max for OpenEvolve evolution.

