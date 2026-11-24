"""Cache pinned viewer assets for real-mesh QA and the anonymous static export."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import urllib.request


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--web',type=Path,default=Path('web'));parser.add_argument('--output',type=Path,default=Path('.cache/render-qa'));args=parser.parse_args()
    native=json.loads(Path('src/contact_transfer/models/native_assets.json').read_text())['components'];local={}
    for component,data in native.items():
        for file,record in data['files'].items():
            path=Path('.cache')/component/file
            if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest()==record['sha256']:local[record['url']]=path.resolve()
    urls=set();prefixes=('full_','robust_refinement_','counterfactual_','margin_','research_')
    for entry in json.loads((args.web/'data/index.json').read_text())['models']:
        if not entry['id'].startswith(prefixes):continue
        data=json.loads((args.web/'data'/(entry['id']+'.json')).read_text())
        for instance in data.get('instances',[]):
            model=json.loads((args.web/instance['manifest'].removeprefix('./')).read_text());urls.update(model.get('assets',[]))
    args.output.mkdir(parents=True,exist_ok=True)
    def fetch(url):
        if not url.startswith('https://raw.githubusercontent.com/'):raise ValueError('Pinned GitHub raw assets required')
        if url in local:return url,str(local[url])
        path=args.output/(hashlib.sha256(url.encode()).hexdigest()+Path(url).suffix)
        if not path.exists():
            with urllib.request.urlopen(url,timeout=60) as response:payload=response.read()
            if payload.startswith(b'version https://git-lfs'):raise ValueError('LFS pointer is not mesh geometry')
            temporary=path.with_suffix(path.suffix+'.part');temporary.write_bytes(payload);temporary.replace(path)
        return url,str(path.resolve())
    with ThreadPoolExecutor(max_workers=5) as pool:result=dict(pool.map(fetch,sorted(urls)))
    (args.output/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    audit={url:dict(path=path,sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),bytes=Path(path).stat().st_size) for url,path in result.items()}
    (args.output/'audit.json').write_text(json.dumps(audit,indent=2)+'\n');print(len(result),'real pinned mesh assets available')


if __name__=='__main__':main()
