"""Explicit, hash-verified native asset preparation; never download on import."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import urllib.request


def asset_manifest():
    return json.loads((Path(__file__).with_name('models')/'native_assets.json').read_text())


def fetch_native_assets(root='.cache', *, components=None, refresh=False):
    manifest=asset_manifest()['components'];components=tuple(components or manifest)
    unknown=set(components)-set(manifest)
    if unknown:raise ValueError(f'Unknown native asset components: {sorted(unknown)}')
    result={}
    for name in components:
        folder=Path(root)/name;folder.mkdir(parents=True,exist_ok=True);entry=manifest[name]
        for relative,record in entry['files'].items():
            target=folder/relative;target.parent.mkdir(parents=True,exist_ok=True)
            if target.is_file() and not refresh:
                if hashlib.sha256(target.read_bytes()).hexdigest()!=record['sha256']:
                    raise ValueError(f'Native asset hash mismatch: {target}; use an explicit refresh to restore the pinned asset')
                continue
            with urllib.request.urlopen(record['url'],timeout=30) as response:data=response.read()
            if hashlib.sha256(data).hexdigest()!=record['sha256']:
                raise ValueError(f'Downloaded native asset hash mismatch: {record["url"]}')
            with tempfile.NamedTemporaryFile(dir=target.parent,delete=False) as handle:
                handle.write(data);temporary=handle.name
            os.replace(temporary,target)
        (folder/'provenance.json').write_text(json.dumps(entry,indent=2)+'\n')
        result[name]=dict(directory=str(folder.resolve()),files=len(entry['files']),revision=entry['revision'])
    return result
