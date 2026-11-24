"""Check archived numerical source witnesses and dual infeasibility certificates."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def verify(root):
    manifest=json.loads((root/'files.sha256.json').read_text())
    for relative,expected in manifest.items():
        if hashlib.sha256((root/relative).read_bytes()).hexdigest()!=expected:raise ValueError(f'Artifact checksum differs: {relative}')
    source=[];duals=[]
    for file in sorted(root.rglob('*-source-witness.npz')):
        with np.load(file,allow_pickle=False) as a:
            maps,reactions=a['maps'],a['reactions'];nv=len(a['vertices'])
            if reactions.shape!=(nv,len(maps),6):raise ValueError('Incomplete source vertex coverage')
            total=np.einsum('inj,vij->vn',maps,reactions)
            required=a['nominal']+float(a['rho'])*(a['vertices']@a['D'].T@a['E'].T)
            error=max(float(np.max(abs(total-required))),float(np.max(abs(reactions[:,:,:3]))),
                float(np.max(np.linalg.norm(reactions[:,:,3:5],axis=2)-a['mu']*reactions[:,:,5])),
                float(np.max(-reactions[:,:,5])),float(np.max(reactions[:,:,5]-a['normal_max'])))
            if not np.isfinite(error) or error>2e-6:raise ValueError(f'Source witness rejected: {file}: {error}')
            source.append(dict(artifact=str(file.relative_to(root)),maximum_residual=error,vertices=nv))
    for file in sorted(root.rglob('*-outer-certificate.npz')):
        with np.load(file,allow_pickle=False) as a:
            A,b,z=a['A'],a['b'],a['z'];product=float(b@z)
            if product>=0:raise ValueError(f'No negative dual separator: {file}')
            z=z/(-product);eq=int(a['equality_rows']);linear=int(a['linear_rows'])
            error=max(float(np.max(abs(A.T@z))),float(max(0.,-np.min(z[eq:eq+linear]))))
            for at in range(eq+linear,len(z),3):error=max(error,float(np.linalg.norm(z[at+1:at+3])-z[at]))
            if not np.isfinite(error) or error>1e-7:raise ValueError(f'Dual certificate rejected: {file}: {error}')
            duals.append(dict(artifact=str(file.relative_to(root)),maximum_residual=error,b_dot_z=float(b@z)))
    if len(source)!=27 or len(duals)!=18:raise ValueError('This release requires 27 source witnesses and 18 original-model dual separators')
    return dict(artifacts=len(manifest),source_witnesses=source,original_model_dual_separators=duals,all_passed=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path,nargs='?',default=Path('docs/validation/research_2026-11-24'));args=parser.parse_args();print(json.dumps(verify(args.root),indent=2))
