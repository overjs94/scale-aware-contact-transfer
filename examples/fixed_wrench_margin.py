"""Check the same frozen contact certificate using IPOPT and Clarabel."""

import numpy as np

from contact_transfer.conic import fixed_wrench_margin
from contact_transfer.contacts import Capability, SurfacePatch, TaskWrench
from contact_transfer.refinement import refine_contacts


task = TaskWrench(np.array([0., 0, 0, 0, 0, -9.81]), np.eye(6)[:, 5:], np.eye(1))
caps = [Capability(normal_max=20)]
nlp = refine_contacts([SurfacePatch.fixed([0, 0, 0], [0, 0, -1])], task, caps)
conic = fixed_wrench_margin(nlp.positions, nlp.rotations, task, caps)
assert abs(nlp.rho-conic.rho) < 1e-5
print({"ipopt_rho": nlp.rho, "clarabel_rho": conic.rho,
       "conic_primal_residual": conic.primal_residual})
