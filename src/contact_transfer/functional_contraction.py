"""Geometry contraction within supplied, link-aware functional contact groups."""
import numpy as np

def contract_dense_contacts(d):
    """Weighted planar P anchor / opposing-finger G grasp-frame centroid.

    Group IDs are GIVEN; this is the geometric contraction operator, not learned
    grouping. No target anchor is supplied independently of the source samples.
    """
    scale=float(d['source_to_target_scale']);out=[]
    if scale<=0 or not np.isfinite(scale):raise ValueError('Invalid scale')
    for group in d['dense_source_groups']:
        points=np.asarray(group['points_m'],float);weights=np.asarray(group['weights'],float)
        if points.ndim!=2 or points.shape[1]!=3 or not 4<=len(points)<=256 or weights.shape!=(len(points),) or not np.isfinite(np.r_[points.ravel(),weights]).all() or np.any(weights<=0):raise ValueError('Invalid dense group')
        anchor=np.average(points,axis=0,weights=weights)
        out.append(dict(group_id=group['group_id'],kind=group['kind'],link_id=group['link_id'],sample_count=len(points),source_points_m=points.tolist(),source_anchor_m=anchor.tolist(),target_anchor_m=(scale*anchor).tolist(),target_points_m=(scale*points).tolist(),inward_link=group['inward_link'],contraction_rms_m=float(np.sqrt(np.average(np.sum((points-anchor)**2,axis=1),weights=weights)))))
    if len(out)<2 or len({g['group_id'] for g in out})!=len(out):raise ValueError('Need at least two distinct functional groups')
    assignments=[a['group_id'] for a in d['arms']]
    if len(assignments)!=len(out) or set(assignments)!={g['group_id'] for g in out}:raise ValueError('Injective arm/group assignment required')
    return dict(method='weighted centroid within supplied functional groups; G centroid is a grasp frame, P centroid is on the planar contact patch',synthetic_input=True,network_inference_executed=False,functional_group_ids_supplied=True,contraction_executed=True,source_to_target_scale=scale,dense_sample_count=sum(g['sample_count'] for g in out),contracted_contact_count=len(out),groups=out,assignment=[dict(group_id=a['group_id'],arm_id=a['id']) for a in d['arms']],source_forces_transferred=False,target_dynamics_recomputed=True)
