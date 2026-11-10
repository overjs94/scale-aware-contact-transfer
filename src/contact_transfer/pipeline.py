"""Draft v12 source prior status and geometry-only scale transfer."""

from dataclasses import dataclass
import numpy as np

from .refinement import refine_contacts


@dataclass
class SourcePrior:
    positions: np.ndarray
    normals: np.ndarray
    group_ids: tuple | None
    source_status: str
    source_rho: float | None
    source_certificate: bool
    failure_reason: str | None = None


def source_prior(patches, task, capabilities=None, *, group_ids=None, **kwargs):
    if capabilities is not None and any(c.kind != "P" for c in capabilities):
        raise ValueError("The paper's source refinement uses unilateral point contacts.")
    if group_ids is not None and (len(group_ids) != len(patches) or len(set(group_ids)) != len(patches)):
        raise ValueError("group_ids must uniquely identify the supplied contact groups")
    ids = None if group_ids is None else tuple(group_ids)
    try:
        result = refine_contacts(patches, task, capabilities, **kwargs)
    except RuntimeError as error:
        return SourcePrior(
            np.array([np.asarray(p.position(p.initial)).reshape(3) for p in patches]),
            np.array([np.asarray(p.rotation(p.initial))[:, 2] for p in patches]),
            ids, "failed_source_solve", None, False, str(error))
    return SourcePrior(result.positions, result.rotations[:, :, 2], ids,
                       "positive_margin" if result.rho > 1e-7 else "feasible_zero_margin",
                       result.rho, True)


def scale_prior(prior, scale):
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("scale must be positive and finite")
    # No source reactions are returned or scaled.
    return {"positions": scale*prior.positions, "normals": prior.normals.copy(),
            "group_ids": prior.group_ids, "source_status": prior.source_status}
