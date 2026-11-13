"""Pinned official chassis collision meshes and wheel bounds; no SDK execution."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import urllib.request
import numpy as np
ROOT=Path(__file__).resolve().parents[1];SHA='300a585c297d1bdf8554312c3a1659610a9823f5';RAW=f'https://raw.githubusercontent.com/RainbowRobotics/rby1-sdk/{SHA}/models/rby1a/mujoco/assets/'
def main():
    cache=ROOT/'.cache/rby1';cache.mkdir(parents=True,exist_ok=True)
    paths=[f'base/base_collision_{i}.obj' for i in range(16)]+['WHEEL/WHEEL.obj']+[p.split('/assets/',1)[1] for p in json.loads((ROOT/'src/contact_transfer/scenarios/mobile_panel/collision_assets.json').read_text())]
    def fetch(path):
        dest=cache/path.rsplit('/',1)[-1]
        if not dest.exists():
            with urllib.request.urlopen(RAW+path,timeout=40) as r:data=r.read()
            if data.startswith(b'version https://git-lfs'):raise ValueError('LFS pointer')
            dest.write_bytes(data)
        vertices=np.array([list(map(float,l.split()[1:4])) for l in dest.read_text().splitlines() if l.startswith('v ')])
        return dest.name,{'min':vertices.min(axis=0).tolist(),'max':vertices.max(axis=0).tolist()}
    with ThreadPoolExecutor(max_workers=4) as pool:bounds=dict(pool.map(fetch,paths))
    (cache/'mesh_bounds.json').write_text(json.dumps(bounds,indent=2)+'\n');print('Fetched',len(bounds),'pinned RBY1 meshes')
if __name__=='__main__':main()
