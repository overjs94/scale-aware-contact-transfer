"""Compare identical conic formulations against a pinned pre-upgrade revision."""
import argparse
import json
from pathlib import Path
import statistics
import subprocess
from time import perf_counter
import types

import numpy as np

from contact_transfer.contacts import Capability,contact_basis,map_reaction
from contact_transfer.generalized_conic import fixed_generalized_margin
from contact_transfer.generalized_load import GeneralizedTaskLoad
from contact_transfer.robot import Robot,_symbolic_template
from contact_transfer.embodiments import load


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-ref',default='4446273fa12645584a465969992db1a0e457da67')
    parser.add_argument('--repeats',type=int,default=5)
    parser.add_argument('--output',type=Path,default=Path('outputs/framework-upgrade/performance.json'))
    args=parser.parse_args()
    if args.repeats<1:parser.error('repeats must be positive')
    source=subprocess.check_output(['git','show',args.reference_ref+':src/contact_transfer/generalized_conic.py'],text=True)
    reference=types.ModuleType('contact_transfer._reference_conic');reference.__package__='contact_transfer'
    exec(compile(source,'reference_generalized_conic.py','exec'),reference.__dict__)
    points=np.array([[-.2,-.15,0.],[.2,-.15,0.],[0.,.2,0.]])
    rotations=[contact_basis([0,0,-1])]*3
    maps=[np.array(map_reaction(p,R,np.eye(6))) for p,R in zip(points,rotations)]
    task=GeneralizedTaskLoad(np.array([0.,0,0,0,0,-9.81]),np.eye(6),np.diag([.1]*3+[1.]*3))
    cases={}
    for name,caps in [('three_P',[Capability(mu=.8,normal_max=30.)]*3),
                      ('two_G_one_P',[Capability('G',H=np.eye(6),bounds=np.array([5.]*3+[30.]*3))]*2+[Capability(mu=.8,normal_max=30.)])]:
        timing={'reference':[],'current':[]};results={}
        for method,fn in [('reference',reference.fixed_generalized_margin),('current',fixed_generalized_margin)]:
            fn(maps,task,caps)
            for _ in range(args.repeats):
                start=perf_counter();result=fn(maps,task,caps);timing[method].append(perf_counter()-start)
                if not result['certificate']:raise RuntimeError(name+' rejected')
            results[method]=dict(rho=result['rho'],primal_residual=result['primal_residual'])
        if abs(results['reference']['rho']-results['current']['rho'])>2e-6:raise RuntimeError('Changed margin')
        medians={k:statistics.median(v) for k,v in timing.items()}
        cases[name]=dict(results=results,wall_seconds=timing,median_seconds=medians,speedup=medians['reference']/medians['current'])
    robot=load('franka_panda');_symbolic_template.cache_clear();timing={}
    for reuse in (False,True):
        robot.symbolic(reuse=reuse);times=[]
        for _ in range(args.repeats):
            start=perf_counter();robot.symbolic(reuse=reuse);times.append(perf_counter()-start)
        timing['cached' if reuse else 'fresh']=dict(wall_seconds=times,median_seconds=statistics.median(times))
    report=dict(reference_commit=args.reference_ref,repeats=args.repeats,
                scope='warm fixed-geometry conic calls and symbolic graph construction; not full-task or native execution speedup',
                cases=cases,symbolic=timing)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
