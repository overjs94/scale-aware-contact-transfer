"""Fetch pinned RB5 collision geometry; no credentials or firmware access."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import urllib.request
ROOT=Path(__file__).resolve().parents[1]
def main():
    meta=json.loads((ROOT/'web/models/rb5/model.json').read_text());cache=ROOT/'.cache/rb5_meshes';cache.mkdir(parents=True,exist_ok=True)
    def fetch(url):
        p=cache/url.rsplit('/',1)[-1]
        if not p.exists():
            with urllib.request.urlopen(url,timeout=40) as r:data=r.read()
            if data.startswith(b'version https://git-lfs'):raise ValueError('LFS pointer, not collision mesh')
            p.write_bytes(data)
        return p.name
    with ThreadPoolExecutor(max_workers=4) as pool:print(list(pool.map(fetch,[u for u in meta['assets'] if '/collision/' in u])))
if __name__=='__main__':main()
