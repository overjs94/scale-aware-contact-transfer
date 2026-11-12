# Accepted complete-task plans

These compact, pickle-free certificates record the optimized keyframes and
all transitions for the panel, free box and two-body assembly tasks. The
manifest records the SHA-256 of the original accepted native trace. Raw native
traces are not included; this bundle is not a new execution or a fresh solve.

Run `tools/run_bundled_full_scenario.py --task assembly --output outputs/replay`
from the repository root to validate current independent mechanics and run a
new physical trial. `--validate-only` stops after stage validation. A new output
directory is required. For a fresh optimization use `tools/run_full_scenarios.py`.
See `docs/system-architecture-and-progress.md` for controller, source fallback,
integration-variant and physical-validation scope.

Install the Python dependencies and fetch the pinned native Robotiq assets
first. All P tools share the same spherical-tip URDF and EE transform. These
new tasks use three Panda arms; other embodiment previews are separate.

Each case preserves upstream model licenses/provenance and authored model
licenses under `provenance/`. Meshes are resolved from the pinned upstream
assets; CRISP SDK binaries and the manuscript are not included.
