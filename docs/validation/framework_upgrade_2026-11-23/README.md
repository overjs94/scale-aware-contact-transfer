# Alpha evidence bundle

`summary.json` indexes cold planning, new native trials and source/environment
identities. `plans/` contains the accepted numeric keyframes and intervals,
their validation reports and original cold-run manifests. It omits generated
models, failed numeric iterates, dense interpolation caches and large traces.
Model generation still uses the pinned native assets and original task inputs.

The old box native run in `plans/box/full/result.json` failed. It remains
unchanged; `native/box-conditioned.json` records a separate successful execution
after correcting the reference mesh. `native/` preserves all 12 executed
controller/time-step trials, including failures, plus their resolved policies.
These are single nominal trials and sensitivity checks, not statistical success
rates or robustness-polytope experiments.

To independently revalidate a stored plan and execute it again from a checkout:

```bash
contact-transfer assets
python tools/validate_execution_variant.py \
  --root docs/validation/framework_upgrade_2026-11-23/plans --task panel \
  --output outputs/replayed-panel
python tools/validate_execution_variant.py \
  --root docs/validation/framework_upgrade_2026-11-23/plans --task box \
  --allow-failed-baseline --output outputs/replayed-box
python tools/validate_execution_variant.py \
  --root docs/validation/framework_upgrade_2026-11-23/plans --task assembly \
  --output outputs/replayed-assembly-hybrid --control-mode hybrid_feedback \
  --force-normal-damping 160
```

The replay regenerates models in the new output directory and rechecks native
keyframe/interval mechanics; it does not rerun optimization or overwrite the
original plan/failure. Planning cache acceptance uses the stricter current
manifest contract. The historical cold manifests intentionally identify the
source used then; `release-source-snapshot.json` separately identifies the final
package source. See the [validation report](../../framework-upgrade-validation-2026-11-23.md)
for source-certificate and manuscript scope.

`checks/final-pytest.log` is the preceding 210-test run, before the last
reference/cache/controller changes. `checks/release-pytest.log`, when present,
is the final complete suite. Targeted regression results are named separately.
`checks/installed-wheel.json` was produced outside the source checkout, using
the actual wheel and native scene.
