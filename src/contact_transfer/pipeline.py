"""Draft v12 source prior status and geometry-only scale transfer."""

from dataclasses import dataclass
import numpy as np

from .refinement import refine_contacts
from .optimization import SolverUnavailableError


@dataclass
class SourcePrior:
    positions: np.ndarray
    normals: np.ndarray
    group_ids: tuple | None
    source_status: str
    source_rho: float | None
    source_certificate: bool
    failure_reason: str | None = None
    failure_kind: str | None = None
    diagnostics: dict | None = None


def source_prior(patches, task, capabilities=None, *, group_ids=None, failure_policy='geometric_prior', structural_failures=None, **kwargs):
    if failure_policy not in {'geometric_prior','error'}:raise ValueError('Unknown source failure policy')
    if capabilities is not None and any(c.kind != "P" for c in capabilities):
        raise ValueError("The paper's source refinement uses unilateral point contacts.")
    if group_ids is not None and (len(group_ids) != len(patches) or len(set(group_ids)) != len(patches)):
        raise ValueError("group_ids must uniquely identify the supplied contact groups")
    ids = None if group_ids is None else tuple(group_ids)
    positions=np.array([np.asarray(p.position(p.initial)).reshape(3) for p in patches]);rotations=np.array([np.asarray(p.rotation(p.initial)) for p in patches])
    if structural_failures:
        message='Source unilateral-point wrench span is insufficient: '+str(structural_failures)
        if failure_policy=='error':raise RuntimeError(message)
        return SourcePrior(positions,rotations[:,:,2],ids,'structurally_infeasible',None,False,message,'structural_infeasibility',dict(span_failures=structural_failures))
    try:
        result = refine_contacts(patches, task, capabilities, **kwargs)
    except SolverUnavailableError:
        raise
    except RuntimeError as error:
        if failure_policy=='error':raise
        diagnostic={}
        # This checks the frozen anchors only. Moving surface coordinates can
        # change feasibility, so a failed conic check is not a global proof.
        try:
            import casadi as ca
            from .contacts import Capability,map_reaction
            from .generalized_conic import fixed_generalized_margin
            maps=[];w=ca.SX.sym('diagnostic_wrench',6)
            for i,(p,R) in enumerate(zip(positions,rotations)):
                image=kwargs['reaction_maps'][i](ca.DM(p),ca.DM(R),w) if kwargs.get('reaction_maps') is not None else map_reaction(p,R,w)
                maps.append(np.array(ca.Function('diagnostic_map',[w],[ca.jacobian(image,w)])(np.zeros(6))))
            check=fixed_generalized_margin(maps,task,capabilities or [Capability() for _ in patches])
            diagnostic=dict(frozen_anchor_status=check['status'],frozen_anchor_certificate=check['certificate'],frozen_anchor_rho=check['rho'],scope='fixed anchors only; not global surface-refinement feasibility')
        except (RuntimeError,ImportError) as diagnostic_error:
            diagnostic=dict(diagnostic_error=str(diagnostic_error))
        return SourcePrior(
            positions,rotations[:,:,2],
            ids, "failed_source_solve", None, False, str(error),
            'local_infeasibility_or_numerical_failure' if 'Infeasible' in str(error) else 'numerical_failure',diagnostic)
    return SourcePrior(result.positions, result.rotations[:, :, 2], ids,
                       "positive_margin" if result.rho > 1e-7 else "feasible_zero_margin",
                       result.rho, True)


def scale_prior(prior, scale):
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("scale must be positive and finite")
    # No source reactions are returned or scaled.
    return {"positions": scale*prior.positions, "normals": prior.normals.copy(),
            "group_ids": prior.group_ids, "source_status": prior.source_status}
