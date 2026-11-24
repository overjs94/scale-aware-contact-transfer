"""Freeze package/data/tools so long solver runs use an immutable code tree."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    repo=Path(__file__).resolve().parents[1];out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    for folder in ['src/contact_transfer','tools']:
        shutil.copytree(repo/folder,out/folder,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    paths=[f for folder in ['src/contact_transfer','tools'] for f in (out/folder).rglob('*') if f.is_file()]
    manifest=dict(parent_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
        working_tree_changes=subprocess.check_output(['git','status','--short'],cwd=repo,text=True),
        files={str(f.relative_to(out)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(paths)})
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(str(out/'src'))


if __name__=='__main__':main()
