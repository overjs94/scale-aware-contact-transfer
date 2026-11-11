"""Explicit small-twin source refinement and geometry-only scale transfer.

Input contraction is supplied, never inferred here. Synthetic fixtures are
labelled in every output and cannot be mistaken for learned/hand-demo results.
"""
import json
from pathlib import Path
import casadi as ca
import numpy as np
from .contacts import Capability,SurfacePatch,TaskWrench,contact_basis,gravity_wrench
from .pipeline import source_prior,scale_prior
from .conic import fixed_wrench_margin


def source_fixture():
    return {'schema_version':1,'units':'SI','contact_frame':'object','origin':'synthetic_contracted_fixture',
            'inference_executed':False,'source_to_target_scale':4.,
            'object':{'size':[.15,.10,.01],'mass':.03125,'com':[0,0,0]},
            'disturbance_scale':[.001,.001,.001,.05,.05,.05],
            'refinement':{'radius_m':.002,'normal_radius':0.,'geometry_weight':1000.,'normal_max_N':2.,'mu':.8},
            'groups':[{'group_id':'edge_grip','position':[-.075,0,0],'normal':[-1,0,0],'tangent_axes':[[0,1,0]],'half_size':[.003]},
                      {'group_id':'under_left','position':[.025,-.03,-.005],'normal':[0,0,-1],'tangent_axes':[[1,0,0],[0,1,0]],'half_size':[.003,.003]},
                      {'group_id':'under_right','position':[.025,.03,-.005],'normal':[0,0,-1],'tangent_axes':[[1,0,0],[0,1,0]],'half_size':[.003,.003]}]}


def load_source(path=None):
    d=source_fixture() if path is None else json.loads(Path(path).read_text())
    if d.get('schema_version')!=1 or d.get('units')!='SI' or d.get('contact_frame')!='object':raise ValueError('Source contacts need schema 1, SI and object frame')
    scale=d['source_to_target_scale'];mass=d['object']['mass'];cfg=d['refinement']
    if not isinstance(d.get('inference_executed',False),bool):raise ValueError('inference_executed must be boolean provenance')
    if not np.isfinite([scale,mass,*d['object']['com'],*d['disturbance_scale'],*cfg.values()]).all() or scale<=0 or mass<=0:raise ValueError('Source scalars must be finite and physically valid')
    if len(d['object']['com'])!=3 or len(d['disturbance_scale'])!=6 or min(d['disturbance_scale'])<=0 or cfg['radius_m']<0 or cfg['normal_radius']!=0 or cfg['geometry_weight']<0 or cfg['mu']<=0 or cfg['normal_max_N']<=0:raise ValueError('Unsupported source bounds; this planar schema fixes normals')
    groups=d['groups'];ids=[g['group_id'] for g in groups]
    if len(ids)<2 or len(ids)!=len(set(ids)):raise ValueError('Source group IDs must be unique')
    half=np.asarray(d['object']['size'],float)/2
    if half.shape!=(3,) or not np.isfinite(half).all() or np.any(half<=0) or d['object']['mass']<=0:raise ValueError('Invalid source geometry/mass')
    for g in groups:
        p,n=np.array(g['position'],float),np.array(g['normal'],float);B=np.array(g['tangent_axes'],float).T;extent=np.array(g['half_size'],float)
        if p.shape!=(3,) or n.shape!=(3,) or not np.isfinite(np.r_[p,n]).all() or not np.isclose(np.linalg.norm(n),1):raise ValueError('Invalid source contact')
        if not 1<=len(extent)<=2 or not np.isfinite(extent).all() or np.any(extent<=0) or B.shape!=(3,len(extent)) or not np.allclose(B.T@B,np.eye(len(extent))) or not np.allclose(n@B,0):raise ValueError('Source chart must be orthonormal and tangent')
        for signs in np.array(np.meshgrid(*([[-1,1]]*len(extent)))).T.reshape(-1,len(extent)):
            corner=p+B@(extent*signs)
            if np.any(abs(corner)>half+1e-10) or not any(np.isclose(corner[i],np.sign(n[i])*half[i]) and abs(n[i])>.999 for i in range(3)):raise ValueError('Source chart leaves the panel surface')
    return d


def refine_source_and_transfer(path=None,*,enabled=True,settings=None):
    d=load_source(path);patches=[]
    for i,g in enumerate(d['groups']):
        u=ca.SX.sym('source_chart_'+str(i),len(g['half_size']));p=ca.DM(g['position'])+ca.DM(np.array(g['tangent_axes']).T)@u;R=ca.DM(contact_basis(g['normal']));half=np.array(g['half_size'])
        patches.append(SurfacePatch(ca.Function('source_p_'+str(i),[u],[p]),ca.Function('source_R_'+str(i),[u],[R]),-half,half,np.zeros(len(half))))
    cfg=d['refinement'];caps=[Capability('P',mu=cfg['mu'],normal_max=cfg['normal_max_N']) for _ in patches]
    task=TaskWrench(gravity_wrench(d['object']['mass'],d['object']['com'],np.eye(3)),np.eye(6),np.diag(d['disturbance_scale']))
    before=np.array([g['position'] for g in d['groups']]);rotations=np.array([contact_basis(g['normal']) for g in d['groups']])
    baseline=fixed_wrench_margin(before,rotations,task,caps)
    prior=source_prior(patches,task,caps,group_ids=[g['group_id'] for g in d['groups']],
                       position_radius=cfg['radius_m'] if enabled else 0.,normal_radius=cfg['normal_radius'],geometry_weight=cfg['geometry_weight'],settings=settings)
    if not prior.source_certificate:raise RuntimeError('Source refinement failed: '+str(prior.failure_reason))
    validation=fixed_wrench_margin(prior.positions,rotations,task,caps)
    transferred=scale_prior(prior,d['source_to_target_scale'])
    return {'origin':d['origin'],'inference_executed':bool(d.get('inference_executed',False)),'synthetic_input':d['origin'].startswith('synthetic'),
            'source_refinement_enabled':enabled,'source_contact_positions_optimized':bool(enabled),'source_group_ids':list(prior.group_ids),
            'source_object':d['object'],'source_to_target_scale':d['source_to_target_scale'],
            'source_positions_before_m':before.tolist(),'source_positions_after_m':prior.positions.tolist(),
            'source_normals':prior.normals.tolist(),'source_rho_before':baseline.rho,'source_rho_after':validation.rho,
            'source_certificate':prior.source_certificate,'source_status':prior.source_status,'source_validation_residual':validation.primal_residual,
            'target_positions_m':transferred['positions'].tolist(),'target_normals':transferred['normals'].tolist(),
            'source_forces_transferred':False,'target_mass_reused_from_source':False}
