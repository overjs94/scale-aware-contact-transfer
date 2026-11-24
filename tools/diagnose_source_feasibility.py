"""Conservative convex outer test for the original representative-P source.

For a site shift ||dp||<=r, its generalized moment error in row j is bounded
by r*||J_angular[:,j]||*sqrt(1+mu^2)*f_normal. Independent errors inside these
bounds enlarge the feasible set. A verified conic dual infeasibility witness
therefore excludes every source contact location in the declared trust balls.
An outer-feasible result alone does not establish full source feasibility.
"""
import argparse
from pathlib import Path
from types import SimpleNamespace

import casadi as ca
import clarabel
import numpy as np
from scipy import sparse

from contact_transfer.contacts import Capability,SurfacePatch,contact_basis
from contact_transfer.full_pipeline import source_solver_settings
from contact_transfer.full_recording import write_json,atomic_npz
from contact_transfer.full_scenario import FullTask,FullProblem
from contact_transfer.generalized_conic import fixed_generalized_margin
from contact_transfer.generalized_load import link_reaction_map
from contact_transfer.grouped_source import native_map,verify_vertex_witnesses
from contact_transfer.multi_object import RigidCollection
from contact_transfer.paper_free import source_urdf
from contact_transfer.refinement import refine_contacts


def outer_test(obj,q,positions,normals,frames,load,caps,radius):
    nv=obj.nv;count=len(positions);blocks=[];coefficients=[]
    for point,normal,frame,cap in zip(positions,normals,frames,caps):
        A=native_map(obj,q,frame,point,contact_basis(normal))[:,3:]
        J=obj.jacobian(q,frame,'world_aligned')[:3];c=radius*np.linalg.norm(J,axis=0)*np.sqrt(1+cap.mu**2)
        rows=np.flatnonzero(c>1e-14);block=np.c_[A,np.eye(nv)[:,rows]]
        blocks.append((block,rows,cap));coefficients.append(c)
    widths=[b.shape[1] for b,_,_ in blocks];offsets=np.cumsum([0]+widths);n=offsets[-1]
    equilibrium=np.hstack([b for b,_,_ in blocks]);linear=[];bounds=[];cones=[]
    for i,((_,rows,cap),c) in enumerate(zip(blocks,coefficients)):
        start=offsets[i]
        for sign,limit in [(-1.,0.),(1.,cap.normal_max)]:
            row=np.zeros(n);row[start+2]=sign;linear.append(row);bounds.append(limit)
        for j,physical_row in enumerate(rows):
            for sign in [-1,1]:
                row=np.zeros(n);row[start+3+j]=sign;row[start+2]=-c[physical_row];linear.append(row);bounds.append(0.)
        cone=np.zeros((3,n));cone[0,start+2]=-cap.mu;cone[1,start]=-1.;cone[2,start+1]=-1.;cones.append(cone)
    A=sparse.csc_matrix(np.vstack([equilibrium,np.asarray(linear),*cones]));b=np.r_[load.nominal,bounds,np.zeros(3*count)]
    settings=clarabel.DefaultSettings();settings.verbose=False;settings.tol_feas=settings.tol_gap_abs=settings.tol_gap_rel=1e-10;settings.max_iter=300
    result=clarabel.DefaultSolver(sparse.csc_matrix((n,n)),np.zeros(n),A,b,
        [clarabel.ZeroConeT(nv),clarabel.NonnegativeConeT(len(linear)),*[clarabel.SecondOrderConeT(3) for _ in cones]],settings).solve()
    status=str(result.status);dual=None;verified=False
    if status in {'PrimalInfeasible','AlmostPrimalInfeasible'}:
        z=np.asarray(result.z);product=float(b@z)
        if np.isfinite(z).all() and product< -1e-12:
            z=z/(-product);null=float(np.max(abs(A.T@z)));violation=float(max(0.,-np.min(z[nv:nv+len(linear)])))
            at=nv+len(linear)
            for _ in cones:
                violation=max(violation,float(np.linalg.norm(z[at+1:at+3])-z[at]));at+=3
            verified=null<=1e-7 and violation<=1e-7 and b@z<=-.999999
            dual=dict(verified=bool(verified),stationarity_residual=null,dual_cone_residual=violation,b_dot_z=float(b@z))
    report=dict(status=status,nominal_globally_infeasible_within_trust_region=bool(verified),dual_certificate=dual,
        outer_feasible=status in {'Solved','AlmostSolved'},normals_fixed=True,position_radius_m=radius,
        relaxation='independent generalized-moment error bounds containing all allowed point shifts',
        passing_outer_test_is_not_source_certificate=True)
    return report,dict(A=A.toarray(),b=b,z=np.asarray(result.z),equality_rows=np.array(nv),linear_rows=np.array(len(linear)))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=Path('outputs/research-study/source-diagnosis'));args=parser.parse_args();summary={}
    for name in ['box','assembly']:
        folder=args.output/name;task=FullTask.load(name,folder/'task');p=FullProblem.create(task,folder/'models','PPP');obj=task.object;scale=task.scale
        def scaled(part,n):return source_urdf(SimpleNamespace(object=part),folder/('source_'+n+'.urdf'),scale,{k:l.mass/scale**3 for k,l in part.tree.links.items() if l.mass>0})
        src=RigidCollection({n:scaled(r,n) for n,r in obj.bodies.items()}) if isinstance(obj,RigidCollection) else scaled(obj,'object')
        positions=[f.surface/scale for f in task.features];normals=[f.normal for f in task.features];frames=[f.frame for f in task.features]
        cfg=task.configuration.source;caps=[Capability(mu=cfg.mu,normal_max=cfg.normal_max_N) for _ in positions];rows=[]
        for k,key in enumerate(task.keyframes):
            qo=key['object_q'];qs=np.concatenate([np.r_[qo[s][:3]/scale,qo[s][3:]] for s in obj.q_slices.values()]) if isinstance(obj,RigidCollection) else np.r_[qo[:3]/scale,qo[3:]]
            load=p.task_load(qs,source_object=src);result,certificate=outer_test(src,qs,positions,normals,frames,load,caps,cfg.position_radius_m)
            atomic_npz(folder/f'key{k}-outer-certificate.npz',**certificate)
            maps=[native_map(src,qs,f,p0,contact_basis(n)) for f,p0,n in zip(frames,positions,normals)]
            frozen=fixed_generalized_margin(maps,load,caps)
            result.update(keyframe=key['id'],frozen_anchor_status=frozen['status'],frozen_anchor_certificate=frozen['certificate'],frozen_anchor_rho=frozen['rho'])
            if not result['nominal_globally_infeasible_within_trust_region']:
                patches=[];symbolic=src.symbolic();reaction_maps=[]
                for i,(point,normal,frame) in enumerate(zip(positions,normals,frames)):
                    u=ca.SX.sym('normalized_source_'+str(i),2);B=contact_basis(normal)[:,:2];r=cfg.position_radius_m
                    patches.append(SurfacePatch(ca.Function('p',[u],[ca.DM(point)+r*B@u]),ca.Function('R',[u],[contact_basis(normal)]),np.full(2,-cfg.patch_half_size_m/r),np.full(2,cfg.patch_half_size_m/r),np.zeros(2)))
                    reaction_maps.append(link_reaction_map(symbolic,qs,frame))
                warm=None
                if frozen['certificate']:
                    values=frozen['reactions'];warm=dict(rho=.5*frozen['rho'],reactions=values.mean(axis=0,keepdims=True)+.5*(values-values.mean(axis=0,keepdims=True)))
                row_scale=np.maximum(abs(load.nominal),np.linalg.norm(load.E@load.D,axis=1));row_scale=np.where(row_scale>0,row_scale,np.min(row_scale[row_scale>0]))
                try:
                    refined=refine_contacts(patches,load,caps,reaction_maps=reaction_maps,position_radius=cfg.position_radius_m,normal_radius=0.,geometry_weight=cfg.geometry_weight,settings=source_solver_settings(),initial_witness=warm,point_force_coordinates='scaled_tangent',equilibrium_scale=row_scale,normalize_trust_region=True)
                    native=[native_map(src,qs,f,point,R) for f,point,R in zip(frames,refined.positions,refined.rotations)]
                    check=verify_vertex_witnesses(native,load,caps,refined.rho,refined.reactions)
                    result['equivalent_ratio_refinement']=dict(rho=refined.rho,independent=check,physical_model_changed=False)
                except RuntimeError as error:result['equivalent_ratio_refinement']=dict(accepted=False,failure=str(error),global_infeasibility_claimed=False)
            rows.append(result);write_json(folder/'result.json',rows);print(name,key['id'],result,flush=True)
        summary[name]=rows
    write_json(args.output/'summary.json',summary)


if __name__=='__main__':main()
