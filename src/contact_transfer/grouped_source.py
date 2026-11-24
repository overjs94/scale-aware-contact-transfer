"""Explicit functional-group source extension with physical unilateral sites.

This is not a replacement of a single representative P by a free wrench box.
All supplied sites lie on source collision surfaces, move tangentially as one
group, and have separate bounded Coulomb reactions. Only the representative
surface geometry is transferred. The physical site expansion is disclosed.
"""
from importlib.resources import files
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import casadi as ca
import numpy as np

from .contacts import Capability, SurfacePatch, contact_basis
from .generalized_conic import fixed_generalized_margin
from .refinement import refine_contacts
from .urdf import origin


def fixture_path(reference):
    if not reference:
        raise ValueError('Functional source sites require an explicit fixture')
    return Path(files('contact_transfer').joinpath(reference[8:])) if reference.startswith('package:') else Path(reference)


def on_box_surface(robot, frame, point, normal, tolerance=1e-9):
    for node in ET.parse(robot.path).findall(f"link[@name='{frame}']/collision"):
        box=node.find('geometry/box')
        if box is None:
            continue
        R,p=origin(node);half=np.fromstring(box.get('size'),sep=' ')/2
        local=R.T@(point-p);n=R.T@normal
        for axis in range(3):
            if (abs(abs(n[axis])-1)<tolerance and
                abs(local[axis]-np.sign(n[axis])*half[axis])<=tolerance and
                np.all(abs(local)<=half+tolerance)):
                return True
    return False


def native_map(obj,q,frame,point,rotation):
    """Native Pinocchio point Jacobian; independent of the NLP's symbolic map."""
    T=obj.fk(q,frame);J=obj.jacobian(q,frame,'world_aligned');x,y,z=T[:3,:3]@point
    cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    Jp=np.vstack([J[:3],J[3:]-cross@J[:3]])
    R=np.zeros((6,6));R[:3,:3]=R[3:,3:]=T[:3,:3]@rotation
    return Jp.T@R


def verify_vertex_witnesses(maps,load,caps,rho,reactions,tolerance=2e-6):
    reactions=np.asarray(reactions)
    if not np.isfinite(rho) or rho<0:
        return dict(passed=False,reason='Finite nonnegative source margin required')
    if reactions.shape!=(len(load.vertices),len(caps),6) or not np.isfinite(reactions).all():
        return dict(passed=False,reason='Incomplete/nonfinite source vertex witnesses')
    error=dict(equilibrium=0.,circular_friction=0.,normal_bounds=0.,point_moments=0.)
    for forces,vertex in zip(reactions,load.vertices):
        required=np.asarray(load.nominal)+rho*np.asarray(load.E)@np.asarray(load.D)@vertex
        error['equilibrium']=max(error['equilibrium'],float(np.max(abs(sum(M@w for M,w in zip(maps,forces))-required))))
        for cap,w in zip(caps,forces):
            error['circular_friction']=max(error['circular_friction'],float(np.linalg.norm(w[3:5])-cap.mu*w[5]))
            error['normal_bounds']=max(error['normal_bounds'],float(-w[5]),float(w[5]-cap.normal_max))
            error['point_moments']=max(error['point_moments'],float(np.max(abs(w[:3]))))
    return dict(passed=max(error.values())<=tolerance,tolerance=tolerance,residuals=error,
        vertices_checked=len(load.vertices),mechanics='native Pinocchio point maps and unscaled NumPy circular-cone checks')


def refine_functional_groups(obj,q,load,features,*,scale,policy,enabled,
                             settings=None,conic_settings=None):
    from .generalized_load import link_reaction_map
    from .multi_object import RigidCollection
    definition=json.loads(fixture_path(policy.site_file).read_text())
    if (definition.get('schema_version')!=1 or definition.get('units')!='SI' or
        definition.get('contact_frame')!='source_link' or
        definition.get('inference_executed') is not False):
        raise ValueError('Source group fixtures require schema 1, SI, source_link and explicit inference provenance')
    groups=definition['groups'];ids=[g['group_id'] for g in groups]
    if ids!=[f.id for f in features]:
        raise ValueError('Source groups must match target feature IDs and ordering')
    patches=[];caps=[];labels=[];maps=[];anchors=[];frames=[];limits=[]
    symbolic=obj.symbolic()
    for index,(group,feature) in enumerate(zip(groups,features)):
        frame=group['link_id']
        if frame!=feature.frame:
            raise ValueError('Source and target link identities must agree')
        point=np.asarray(group['anchor_m'],float);normal=np.asarray(group['normal'],float)
        axes=np.asarray(group['translation_axes'],float).T
        if axes.size==0:axes=np.zeros((3,0))
        if (point.shape!=(3,) or normal.shape!=(3,) or axes.ndim!=2 or axes.shape[0]!=3 or
            not np.isfinite(np.r_[point,normal,axes.ravel()]).all() or
            not np.isclose(normal@normal,1.) or not np.allclose(axes.T@axes,np.eye(axes.shape[1]))):
            raise ValueError('Finite source anchor, unit normal and orthonormal tangent chart required')
        if not np.allclose(point*scale,feature.surface,atol=1e-10) or not np.allclose(normal,feature.normal):
            raise ValueError('Fixture representative must match the supplied contracted surface anchor')
        component=obj.bodies[frame] if isinstance(obj,RigidCollection) else obj
        local_frame=component.tree.root if isinstance(obj,RigidCollection) else frame
        # The representative is a genuine additional P site; finite patches
        # below are explicitly supplied physical hand contacts, not inferred.
        sites=[dict(position_m=point.tolist(),normal=normal.tolist(),normal_limit_N=group['anchor_normal_limit_N'])]
        sites+=group.get('sites',[])
        anchors.append(len(patches))
        for site in sites:
            p=np.asarray(site['position_m'],float);n=np.asarray(site['normal'],float)
            bound=float(site['normal_limit_N'])
            if p.shape!=(3,) or n.shape!=(3,) or not np.isfinite(np.r_[p,n,bound]).all() or not np.isclose(n@n,1.) or bound<=0:
                raise ValueError('Finite physical source sites and positive normal limits required')
            if not np.allclose(n@axes,0.,atol=1e-12):
                raise ValueError('Shared group translation must be tangent to every site')
            radius=policy.position_radius_m if enabled else 0.
            # Check all chart corners against actual source collision faces.
            from itertools import product
            corners=[p+axes@(radius*np.array(signs)) for signs in product((-1,1),repeat=axes.shape[1])]
            if any(not on_box_surface(component,local_frame,c,n) for c in corners):
                raise ValueError(f'Source physical site or trust chart leaves surface: {feature.id}, {p.tolist()}')
            u=ca.SX.sym('source_group_'+str(index),axes.shape[1] if enabled else 0)
            position=ca.DM(p)+radius*ca.DM(axes)@u if enabled else ca.DM(p)
            d=u.numel();half=np.ones(d)
            patches.append(SurfacePatch(ca.Function('site_p',[u],[position]),ca.Function('site_R',[u],[contact_basis(n)]),-half,half,np.zeros(d)))
            caps.append(Capability(mu=policy.mu,normal_max=bound));labels.append(index);frames.append(frame);limits.append(bound)
            maps.append(link_reaction_map(symbolic,q,frame))
    row_scale=np.maximum(abs(np.asarray(load.nominal)),np.linalg.norm(np.asarray(load.E)@np.asarray(load.D),axis=1))
    row_scale=np.where(row_scale>0,row_scale,np.min(row_scale[row_scale>0]))
    p0=[np.asarray(p.position(p.initial)).ravel() for p in patches];R0=[np.asarray(p.rotation(p.initial)) for p in patches]
    initial=fixed_generalized_margin([native_map(obj,q,f,p,R) for f,p,R in zip(frames,p0,R0)],load,caps,solver_settings=conic_settings)
    warm=None
    if initial['certificate']:
        values=initial['reactions'];warm=dict(rho=.5*initial['rho'],reactions=values.mean(axis=0,keepdims=True)+.5*(values-values.mean(axis=0,keepdims=True)))
    result=refine_contacts(patches,load,caps,coordinate_groups=labels,reaction_maps=maps,
        position_radius=policy.position_radius_m if enabled else 0.,normal_radius=0.,
        geometry_weight=policy.geometry_weight,settings=settings,
        equilibrium_scale=row_scale,normalize_trust_region=True,initial_witness=warm,point_force_coordinates='scaled_tangent')
    native=[native_map(obj,q,frame,p,R) for frame,p,R in zip(frames,result.positions,result.rotations)]
    check=fixed_generalized_margin(native,load,caps,fixed_rho=result.rho,solver_settings=conic_settings)
    witness=verify_vertex_witnesses(native,load,caps,result.rho,result.reactions)
    if not witness['passed'] or result.rho<=1e-7:
        raise RuntimeError('Functional source rejected by independent native witness: '+json.dumps(dict(conic_status=check['status'],nlp_rho=result.rho,conic_rho=check['rho'],native=witness)))
    budgets={}
    for group,patch,limit in zip(labels,patches,limits):
        normal=tuple(np.asarray(patch.rotation(patch.initial))[:,2]);entry=budgets.setdefault(ids[group],{})
        label=','.join(str(float(v)) for v in normal);entry[label]=entry.get(label,0.)+limit
    return dict(positions=result.positions[anchors]*scale,normals=result.rotations[anchors,:,2],
        record=dict(synthetic_input=True,network_inference_executed=False,
            source_model='functional_group_sites',source_model_extension=True,
            source_refinement_enabled=enabled,source_contact_positions_optimized=enabled,
            source_status='positive_margin',source_certificate=True,source_rho=result.rho,
            source_nlp_primal_residual=result.primal_residual,
            source_group_ids=ids,source_group_count=len(groups),source_site_count=len(patches),
            source_coordinate_groups=labels,source_anchor_indices=anchors,
            source_positions_before_m=[np.asarray(p.position(p.initial)).ravel().tolist() for p in patches],
            source_positions_after_m=result.positions.tolist(),source_site_frames=frames,
            source_normals=result.rotations[:,:,2].tolist(),source_normal_limits_N=limits,
            source_friction=policy.mu,source_E=np.asarray(load.E).tolist(),source_D=np.asarray(load.D).tolist(),
            source_equilibrium_row_scale=row_scale.tolist(),source_coordinates_normalized=True,
            source_initial_conic_status=initial['status'],source_warm_start_used=warm is not None,
            source_point_force_coordinates='scaled_tangent',source_native_vertex_witness=witness,
            source_normal_budgets_by_group_and_outward_normal_N=budgets,
            source_independent_validation=dict(certificate=True,rho=result.rho,residual=max(witness['residuals'].values()),status='VerifiedNLPVertexWitnesses',conic_solver_certificate=check['certificate'],conic_solver_status=check['status'],conic_solver_rho=check['rho'],mechanics='native Pinocchio/NumPy verification of every successful NLP vertex witness; independent conic solve reported separately'),
            source_geometry_fallback=False,source_forces_transferred=False,
            source_to_target_scale=scale,source_definition_origin=definition['origin']))
