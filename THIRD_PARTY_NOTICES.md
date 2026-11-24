# Third-party materials

The original framework is Apache-2.0. This grant does not replace upstream
conditions on dependencies, robot models, collision/visual assets, or optional
commercial solvers. Model-specific `LICENSE*` files and `model.json` provenance
are included in the Python package and must accompany redistribution.

The wheel's SPDX expression lists the aggregate of original code and bundled
model data: `Apache-2.0 AND BSD-3-Clause AND GPL-3.0-or-later`. In particular,
the existing KUKA iiwa7 URDF remains GPL-3.0-or-later; it has not been relicensed
as Apache-2.0. Other bundled model subsets are Apache-2.0 or BSD-3-Clause, as
declared in their individual `model.json` files. This aggregate metadata does
not change the Apache-2.0 grant on original framework code.

- Robot model subsets: see `src/contact_transfer/models/*/LICENSE*` and
  [model provenance](docs/models.md). Authored scene assets carry their own notices.
- Panda collision meshes: pinned to example-robot-data commit
  `78b6101a6387de721316b0db717351d899f97b4f`, Apache-2.0; fetched separately.
- Robotiq native mechanism/meshes: pinned MuJoCo Menagerie commit
  `370c43965323c0944b433b202aac5058b5f907a9`, BSD-2-Clause; fetched with its LICENSE.
- Native asset downloads are content-checked against the packaged
  `models/native_assets.json`; a mesh is not redistributed by merely installing the wheel.
- CRISP is optional and external. Its SDK is not shipped in the wheel/sdist.
  Obtain appropriate upstream rights for the intended use, including commercial use.
  The authored bridge source is in `backends/crisp`.
- SNOPT is optional and external. Its CasADi interface does not supply a solver license.
- CasADi, Pinocchio, MuJoCo, Clarabel, SCS, NumPy, and SciPy retain their upstream terms.

Visual appearance, published model limits and simulated contact capability are
model inputs; they do not constitute manufacturer hardware capability certification.
