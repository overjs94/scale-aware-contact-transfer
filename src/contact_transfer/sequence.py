"""Scenario-consistent contact-type and injective arm-assignment enumeration."""

from dataclasses import dataclass
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


def choose_sequence(allowed_types_by_arm, assignment_sequences, keyframe_solver,
                    transition_solver, *, transition_weight=1., type_validator=None,
                    require_full_geometry=True):
    """Evaluate every specified discrete candidate, never stop at first feasible.

    keyframe_solver(k, types, assignment, previous) -> result with objective.
    transition_solver(k, left, right, types, assignments) -> checked trajectory.
    Optional type_validator can reject hardware-incompatible type combinations.
    This provides feasible-candidate comparison. Shared keyframe/interval NLP
    refinement is intentionally a separate problem-builder extension.
    """
    candidates, failures = [], []
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
    return SequenceSearchResult(min(candidates, key=lambda c: c.objective), candidates, failures)


def injective_assignments(contacts, arms):
    if contacts > arms:
        return []
    return list(permutations(range(arms), contacts))
