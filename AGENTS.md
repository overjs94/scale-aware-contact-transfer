# Package conventions

- Read docs/conventions.md before modifying mechanics or contact constraints.
- Maintain angular-first ordering and mixed Jacobian power duality.
- URDF is authoritative for embodiment mechanics; do not add robot-specific
  dimensions or inertias to Python source.
- Preserve model provenance and upstream licenses.
- Independently validate dynamics and Jacobians against Pinocchio.
- A passed restricted NLP is not a full-geometry trajectory certificate.
- Label unimplemented components and untested hardware paths explicitly.
- Run relevant pytest checks and executable examples before committing changes.
