"""Fetch pinned BSD-licensed native Robotiq geometry, retaining provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

REVISION='370c43965323c0944b433b202aac5058b5f907a9'
BASE=f'https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/{REVISION}/robotiq_2f85/'
FILES=['2f85.xml','LICENSE','README.md',*[f'assets/{name}.stl' for name in ['base_mount','base','driver','coupler','follower','pad','silicone_pad','spring_link']]]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',default='.cache/robotiq_2f85');args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True);records={}
    for name in FILES:
        target=out/name;target.parent.mkdir(parents=True,exist_ok=True)
        if not target.is_file():
            with urllib.request.urlopen(BASE+name,timeout=60) as response:target.write_bytes(response.read())
        records[name]=dict(sha256=hashlib.sha256(target.read_bytes()).hexdigest(),bytes=target.stat().st_size,url=BASE+name)
    (out/'provenance.json').write_text(json.dumps(dict(source='google-deepmind/mujoco_menagerie/robotiq_2f85',revision=REVISION,license='BSD-2-Clause (ROS-Industrial)',files=records),indent=2)+'\n')
    print(json.dumps(dict(directory=str(out.resolve()),files=len(records),total_bytes=sum(v['bytes'] for v in records.values()))))


if __name__=='__main__':main()
