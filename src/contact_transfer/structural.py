"""Necessary wrench-span tests; passing is never a feasibility certificate."""
import numpy as np


def free_body_span_failures(obj, features, types, load):
    if not hasattr(obj, 'bodies'):
        return []
    failures = []
    for name in obj.names:
        kinds = [kind for feature, kind in zip(features, types) if feature.frame == name]
        maximum = 6 if 'G' in kinds or len(kinds) >= 3 else 5 if len(kinds) == 2 else 3*len(kinds)
        rows = obj.v_slices[name]
        required = int(np.linalg.matrix_rank(np.column_stack((load.nominal[rows], load.E[rows]))))
        if maximum < required:
            failures.append(dict(body=name, maximum_rank=maximum, required_rank=required,
                                 scope='necessary span bound for an initially independent free rigid body'))
    return failures
