"""Well-conditioned joint reference meshes that retain optimized state nodes."""
import numpy as np
from scipy.interpolate import CubicHermiteSpline


def subdivide_reference_times(nodes, max_step):
    """Subdivide each optimized cell; never union two nearly identical clocks.

    Every original node is retained exactly once. For an original cell longer
    than max_step, subdivisions are between max_step/2 and max_step. Shorter
    original cells are retained rather than deleting an optimization node.
    """
    nodes=np.asarray(nodes,float)
    if (nodes.ndim!=1 or len(nodes)<3 or not np.isfinite(nodes).all()
            or np.any(np.diff(nodes)<=0) or not np.isfinite(max_step) or max_step<=0):
        raise ValueError('Finite increasing optimized nodes and a positive reference step are required')
    counts=np.maximum(1,np.ceil(np.diff(nodes)/max_step).astype(int))
    times=np.r_[nodes[0],np.concatenate([np.linspace(a,b,n+1)[1:] for a,b,n in zip(nodes[:-1],nodes[1:],counts)])]
    indices=np.r_[0,np.cumsum(counts)]
    return times,indices


def conditioned_joint_spline(times,positions,node_indices,node_velocities):
    """Keep NLP velocities, including quasi-static internal endpoints, exact."""
    times=np.asarray(times,float);positions=np.asarray(positions,float);indices=np.asarray(node_indices,int);planned=np.asarray(node_velocities,float)
    if (positions.ndim!=2 or len(times)!=len(positions) or len(times)<3
            or planned.shape!=(len(indices),positions.shape[1]) or np.any(np.diff(times)<=0)
            or np.any(indices<0) or np.any(indices>=len(times))
            or not all(np.isfinite(a).all() for a in (times,positions,planned))):
        raise ValueError('Reference positions/velocities must match the optimized joint nodes')
    velocity=np.gradient(positions,times,axis=0,edge_order=2)
    velocity[indices]=planned
    spline=CubicHermiteSpline(times,positions,velocity)
    left=2*spline.c[1];right=left+6*spline.c[0]*np.diff(times)[:,None]
    diagnostics=dict(minimum_spacing_s=float(np.min(np.diff(times))),
                     maximum_joint_acceleration_rad_s2=float(max(np.max(abs(left)),np.max(abs(right)))),
                     optimized_node_velocities_preserved=True)
    return spline,velocity,diagnostics
