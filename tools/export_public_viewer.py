"""Export only the built static viewer and recorded examples to a public repo."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
PUBLIC_REPO='https://github.com/overjs94/Contact_contraction'


def export(destination):
    destination=Path(destination).resolve()
    source=ROOT/'web/dist'
    if not (source/'index.html').is_file():raise SystemExit('Build the viewer first: npm --prefix web run build')
    target=destination/'site'
    target.mkdir(parents=True,exist_ok=True)
    shutil.copytree(source,target,dirs_exist_ok=True)
    html=(target/'index.html').read_text()
    for audit in ['draft_v12_code_check.md','manuscript-equation-audit.md']:
        html=html.replace('https://github.com/overjs94/scale-aware-contact-transfer/blob/main/docs/'+audit,PUBLIC_REPO+'#scope')
    html=html.replace('https://github.com/overjs94/scale-aware-contact-transfer/blob/main/docs/models.md',PUBLIC_REPO+'#models-and-licenses')
    html=html.replace('https://github.com/overjs94/scale-aware-contact-transfer',PUBLIC_REPO)
    html=html.replace('논문·코드 대조 ↗','사용 안내 ↗')
    (target/'index.html').write_text(html)
    reports=target/'reports';reports.mkdir(exist_ok=True)
    for source,name in [(ROOT/'docs/images/full-scenario-flow.svg','full-scenario-flow.svg'),(ROOT/'docs/full-scenario-results.json','full-scenario-results.json'),(ROOT/'docs/algorithm-comparison-results.json','algorithm-comparison-results.json'),(ROOT/'docs/images/source-robustness-comparison.svg','source-robustness-comparison.svg'),(ROOT/'docs/images/type-counterfactual-comparison.svg','type-counterfactual-comparison.svg')]:
        if source.is_file():shutil.copyfile(source,reports/name)
    (target/'.nojekyll').touch()
    shutil.copyfile(ROOT/'LICENSE',target/'CONTACT-TRANSFER-LICENSE.txt')
    metadata={'content':'static viewer and recorded simulation examples only',
              'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
              'python_backend_included':False,'live_optimization_available':False,
              'files':{str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in sorted(target.rglob('*')) if p.is_file() and p.name!='recording-provenance.json'}}
    (target/'recording-provenance.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(f'Exported {len(metadata["files"])} static files to {target}')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination',help='Public viewer repository checkout')
    export(parser.parse_args().destination)
