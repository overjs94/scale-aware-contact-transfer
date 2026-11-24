"""Scenario-consistent contact-type and injective arm-assignment enumeration."""

from dataclasses import dataclass,field
from itertools import product, permutations
import math


@dataclass
class SequenceCandidate:
    types: tuple
    assignments: tuple
    keyframes: tuple
    transitions: tuple
    objective: float


@dataclass
class SequenceSearchResult:
    best: SequenceCandidate
    candidates: list
    failures: list
    pruned: list = field(default_factory=list)


def choose_sequence(allowed_types_by_arm, assignment_sequences, keyframe_solver,
                    transition_solver, *, transition_weight=1., type_validator=None,
                    require_full_geometry=True, transition_cost_lower_bound=None):
    """Evaluate every specified discrete candidate, never stop at first feasible.

    keyframe_solver(k, types, assignment, previous) -> result with objective.
    transition_solver(k, left, right, types, assignments) -> checked trajectory.
    Optional type_validator can reject hardware-incompatible type combinations.
    This provides feasible-candidate comparison. Shared keyframe/interval NLP
    refinement is intentionally a separate problem-builder extension.
    An optional certified per-interval cost bound can skip dominated *fixed
    realized endpoints*. This is not a global bound over alternative IK or
    keyframe solutions and is recorded separately from infeasibility.
    """
    if transition_cost_lower_bound is not None and (not math.isfinite(transition_cost_lower_bound) or transition_weight<0):raise ValueError('Finite transition bound and nonnegative weight required')
    candidates, failures, pruned = [], [], []
    assignment_sequences = list(assignment_sequences)
    for types in product(*allowed_types_by_arm):
        if type_validator is not None and not type_validator(types):
            continue
        for assignments in assignment_sequences:
            try:
                keyframes = []
                for k, assignment in enumerate(assignments):
                    if len(set(assignment)) != len(assignment):
                        raise ValueError("assignments must be injective")
                    keyframes.append(keyframe_solver(k, types, assignment,
                                                     keyframes[-1] if keyframes else None))
                if transition_cost_lower_bound is not None and candidates:
                    bound=sum(r.objective for r in keyframes)+transition_weight*(len(keyframes)-1)*transition_cost_lower_bound
                    incumbent=min(c.objective for c in candidates)
                    if bound>incumbent+1e-6:
                        pruned.append(dict(types=tuple(types),assignments=assignments,reason='Dominated cost bound for these realized keyframes; no physical infeasibility claim',realized_keyframe_cost=sum(r.objective for r in keyframes),transition_cost_lower_bound=transition_cost_lower_bound,sequence_cost_lower_bound=bound,incumbent_objective=incumbent));continue
                transitions = [transition_solver(k, keyframes[k], keyframes[k+1], types, assignments)
                               for k in range(len(keyframes)-1)]
                if require_full_geometry and any(
                    not getattr(t, "full_geometry_checked", False) for t in transitions):
                    raise RuntimeError("transition lacks required full-geometry validation")
                value = sum(r.objective for r in keyframes)+transition_weight*sum(r.objective for r in transitions)
                if not math.isfinite(value):
                    raise RuntimeError("nonfinite objective")
                candidates.append(SequenceCandidate(tuple(types), tuple(assignments),
                                                     tuple(keyframes), tuple(transitions), value))
            except RuntimeError as error:
                failures.append({"types": tuple(types), "assignments": assignments, "reason": str(error)})
    if not candidates:
        raise RuntimeError(f"no feasible sequence among {len(failures)} attempted candidates")
    return SequenceSearchResult(min(candidates, key=lambda c: c.objective), candidates, failures,pruned)


def injective_assignments(contacts, arms):
    if contacts > arms:
        return []
    return list(permutations(range(arms), contacts))
