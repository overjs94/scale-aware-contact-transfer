"""Export a separate English, identifier-scrubbed static review snapshot.

Build web/dist first. The original source, recordings, licenses, and Git history
are never edited. This removes explicit identifiers, not indirect study linkage.
"""
import argparse
import hashlib
import html
import json
from pathlib import Path
import re
import secrets
import shutil
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
IDENTIFIERS = ('overjs94', 'Jeongseob Lee', 'Jeongseob', 'snu.ac.kr',
               'scale-aware-contact-transfer', 'Contact_contraction',
               'feat/conic-contact-validation')
PREFIXES = ('full_', 'robust_refinement_', 'counterfactual_')
NO_LICENSE = 'No redistribution license selected; see LICENSE.authored'


def numerical_leaves(value, path=()):
    """Include booleans and numeric array shape/indices, including empty arrays."""
    if isinstance(value, dict):
        for key, item in value.items():
            yield from numerical_leaves(item, path + (key,))
    elif isinstance(value, list):
        yield path + ('length',), len(value)
        for index, item in enumerate(value):
            yield from numerical_leaves(item, path + (index,))
    elif isinstance(value, (int, float, bool)):
        yield path, value


def page(title, content):
    return f'''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="robots" content="noindex,nofollow"><meta name="referrer" content="no-referrer">
<title>{html.escape(title)}</title><link rel="stylesheet" href="style.css">
<main style="max-width:940px;margin:auto;padding:40px 24px;line-height:1.8">
<p><a href="./">← Supplementary simulations</a></p><h1>{html.escape(title)}</h1>{content}</main></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='New or existing export directory, outside the source tree')
    parser.add_argument('--zip', type=Path)
    args = parser.parse_args()
    destination = args.output.resolve()
    if destination == ROOT or ROOT in destination.parents or destination == Path('/'):
        parser.error('Use an export directory outside the research checkout.')
    destination.mkdir(parents=True, exist_ok=True)
    if any(destination.iterdir()):
        parser.error('Output directory must be empty; existing files are never removed.')
    source = ROOT / 'web/dist'
    if not (source / 'index.html').is_file():
        parser.error('Run npm --prefix web run build first.')
    for name in ('index.html', 'app.js', 'style.css', 'THREE-LICENSE.txt', 'URDF-LOADER-LICENSE.txt'):
        shutil.copy2(source / name, destination / name)
    shutil.copy2(ROOT / 'LICENSE', destination / 'LICENSE.original-notice.txt')
    data_out = destination / 'data'
    data_out.mkdir()
    index = json.loads((source / 'data/index.json').read_text())
    models = [m for m in index['models'] if m['id'].startswith(PREFIXES)]
    assert len(models) == 37, 'Review cohort changed; inspect scope before exporting.'
    # Use the same tested English presentation layer as the normal application.
    subprocess.run(['node', '--input-type=module', '-e', '''
import fs from 'node:fs';
const [modulePath,source,dest,ids]=process.argv.slice(1);
const {englishRecording,recordingTitle}=await import(modulePath);
const models=JSON.parse(ids);
for(const model of models){
 const raw=JSON.parse(fs.readFileSync(`${source}/${model.id}.json`));
 fs.writeFileSync(`${dest}/${model.id}.json`,JSON.stringify(englishRecording(raw)));
 model.title=recordingTitle(model.id,model.title);
}
fs.writeFileSync(`${dest}/index.json`,JSON.stringify({models}));
''', (ROOT / 'web/src/presentation-en.js').as_uri(), str(source / 'data'),
                    str(data_out), json.dumps(models)], check=True)
    model_dirs = set()
    for model in models:
        record = json.loads((data_out / (model['id'] + '.json')).read_text())
        for instance in record['instances']:
            model_dirs.add(Path(instance['manifest']).parent.name)
    for name in sorted(model_dirs):
        shutil.copytree(source / 'models' / name, destination / 'models' / name)
    # Serve the real meshes from the review origin, without vendor network
    # requests. The URL-indexed cache is produced by the integration mesh audit.
    asset_cache = ROOT / '.cache/render-qa/manifest.json'
    cached = json.loads(asset_cache.read_text()) if asset_cache.is_file() else {}
    roots = sorted({url for name in model_dirs
                    for url in json.loads((source / 'models' / name / 'model.json').read_text()).get('packages', {}).values()})
    for index, url_root in enumerate(roots):
        for url, local in cached.items():
            if url.startswith(url_root + '/'):
                target = destination / 'assets' / f'vendor-{index}' / url[len(url_root) + 1:]
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(local, target)
    for name in model_dirs:
        path = destination / 'models' / name / 'model.json'
        meta = json.loads(path.read_text())
        for url in meta.get('assets', []):
            assert url in cached and Path(cached[url]).is_file(), f'Missing real mesh cache: {url}'
        meta['packages'] = {key: f'../../assets/vendor-{roots.index(value)}' for key, value in meta.get('packages', {}).items()}
        path.write_text(json.dumps(meta))
    for name in ('algorithm-comparisons.json', 'source-robustness-comparison.svg', 'type-counterfactual-comparison.svg'):
        shutil.copy2(source / 'data' / name, data_out / name)

    # Match comparisons after redaction, without publishing original trace/plan
    # fingerprints. This is metadata pseudonymization, not an integrity claim.
    salt = secrets.token_bytes(32)
    def digest(match):
        return hashlib.sha256(salt + match.group(0).encode()).hexdigest()

    commits = set(subprocess.check_output(['git', 'rev-list', '--all'], cwd=ROOT, text=True).split())
    for other in (ROOT.parent / 'Contact_contraction',):
        if (other / '.git').exists():
            commits.update(subprocess.check_output(['git', 'rev-list', '--all'], cwd=other, text=True).split())
    revisions = re.compile(r'(?<![a-fA-F0-9])(?:' + '|'.join(sorted({c[:7] for c in commits})) + r')[a-fA-F0-9]{0,33}(?![a-fA-F0-9])')

    def scrub_text(text):
        text = re.sub(r'https?://[^\s"<>]*?(?:overjs94|scale-aware-contact-transfer|Contact_contraction)[^\s"<>]*',
                      'review-scope.html', text, flags=re.I)
        text = re.sub(r'/(?:workspace|home|tmp|mnt)/[^\s"<>;,)]*', 'review-artifacts/withheld-path', text)
        for identifier in IDENTIFIERS:
            # Lowercase contact_contraction is a scientific schema key, not
            # the case-sensitive repository name Contact_contraction.
            text = re.sub(re.escape(identifier), 'withheld-for-review', text,
                          flags=0 if identifier == 'Contact_contraction' else re.I)
        text = revisions.sub('withheld-source-revision', text)
        # Vendor revision pins are 40 hex digits and remain intact.
        return re.sub(r'\b[a-fA-F0-9]{64}\b', digest, text)

    def scrub(value):
        if isinstance(value, dict):
            return {k: NO_LICENSE if k == 'license' and v == 'MIT' else scrub(v) for k, v in value.items()}
        if isinstance(value, list):
            return [scrub(v) for v in value]
        return scrub_text(value) if isinstance(value, str) else value

    checked = 0
    for path in sorted(destination.rglob('*')):
        if not path.is_file():
            continue
        if 'assets' in path.relative_to(destination).parts:
            continue  # Pinned third-party geometry is copied byte-for-byte.
        if path.suffix == '.json':
            raw = json.loads(path.read_text())
            clean = scrub(raw)
            assert list(numerical_leaves(raw)) == list(numerical_leaves(clean)), path
            if path.parent == data_out and path.stem.startswith(PREFIXES):
                original = json.loads((source / 'data' / path.name).read_text())
                assert list(numerical_leaves(original)) == list(numerical_leaves(clean)), path
                checked += 1
            path.write_text(json.dumps(clean, ensure_ascii=False, separators=(',', ':')) + '\n')
        elif path.name.startswith('LICENSE') or path.name.endswith('LICENSE.txt'):
            # Required legal notices are copied verbatim, never anonymized.
            continue
        else:
            path.write_text(scrub_text(path.read_text()))
    index_html = destination / 'index.html'
    text = index_html.read_text().replace('<body>', '<body class="anonymous-review">')
    text = text.replace('<meta name="theme-color"', '<meta name="robots" content="noindex,nofollow"><meta name="referrer" content="no-referrer"><meta name="theme-color"')
    text = re.sub(r'<nav><a href="review-scope.html".*?</nav>', '<nav><a href="review-scope.html">Implementation scope ↗</a><a href="model-licenses.html">Model licenses ↗</a></nav>', text, count=1, flags=re.S)
    text = re.sub(r'<a class="source-link".*?</a>', '<a class="source-link" href="model-licenses.html">Model provenance and licenses ↗</a>', text, count=1, flags=re.S)
    text = text.replace('Primary tasks, embodiment checks, and controlled diagnostics.', 'Three cooperative tasks and matched algorithm diagnostics.')
    index_html.write_text(text)
    # A static review snapshot has no backend; omit the health probe.
    app = destination / 'app.js'
    app.write_text(app.read_text().replace('fetch("./api/health")', 'Promise.resolve({ok:false})'))

    scope = '''<p>Anonymous supplementary material for <strong>Scale-Aware Functional Contact Transfer from Human Hands to Multi-Arm Robots</strong>.</p>
<h2>Method and terminology</h2><p>III-B: Task-Wrench-Robust Contact Refinement. III-C: Scale- and Embodiment-Aware Contact Realization. III-D: sequence-wide contact-type and arm-assignment selection. IV: Keyframe-Guided Contact-Implicit Trajectory Optimization.</p>
<p>P denotes unilateral point contact with a common spherical tool; G denotes bounded gripper-mediated contact. The robustness test enforces separate reactions at all vertices of a prescribed task-wrench disturbance box. This box-vertex certificate is distinct from a general dynamic stability guarantee.</p>
<p>Six-vectors are angular/moment first. Reactions are object-on-robot; opposite reactions act on the object. Only source contact geometry is scaled. Target mass, friction, reaction forces and robot torques are recomputed.</p>
<h2>Included evidence</h2><p>Panel reorientation/placement, free articulated box opening, and tabletop–leg assembly: seven conditions each, including all failed checks (21 recordings). Matched stationary source-robustness ON/OFF tests: 14 recordings. Alternative contact types: two executed recordings plus the assembly planning rejection in the comparison table. External forces are visualized with arrows.</p>
<p>Playback uses saved native simulation states; it does not run optimization or dynamics in the browser. Numeric arrays, result values, and pass/fail criteria are unchanged. Original trace/plan fingerprints are consistently pseudonymized; <a href="review-integrity.json">review-integrity.json</a> provides hashes of the distributed files.</p>
<h2>Validation limits</h2><ul><li>Contracted groups are supplied synthetic inputs; learned contraction is not executed.</li><li>Box/assembly source geometric fallback is reported and is not a successful source robustness certificate.</li><li>CITO uses restricted candidates and midpoint configuration-update variants; literal execution of every manuscript equation is not claimed.</li><li>The native arm controller uses joint impedance and planned reaction feedforward with force selection set to zero. These records do not establish measured-normal-force hybrid control.</li><li>The stationary force study fixes target material contacts to isolate source refinement. Static feasibility is not a closed-loop stability proof; failures outside the certified range are retained.</li><li>CRISP, hardware experiments, and general collision completeness are not validated here.</li></ul>
<h2>Licensing</h2><p>No new reuse license is granted for original research material. Earlier authored-model MIT metadata is corrected in this copy to match its bundled redistribution notice; no numerical data are changed. Third-party copyright, licenses, and vendor revision pins remain intact. See <a href="model-licenses.html">model licenses</a>.</p>'''
    (destination / 'review-scope.html').write_text(page('Implementation scope', scope))
    notices = []
    seen = set()
    for path in sorted((destination / 'models').rglob('LICENSE*')):
        fingerprint = hashlib.sha256(path.read_bytes()).hexdigest()
        if fingerprint not in seen:
            seen.add(fingerprint)
            relative = path.relative_to(destination).as_posix()
            notices.append(f'<li><a href="{relative}">{html.escape(path.name)}</a></li>')
    (destination / 'model-licenses.html').write_text(page('Model provenance and licenses', '<p>Original Panda and Robotiq URDF/mesh provenance and immutable vendor revisions remain in each model.json. Legal notices are preserved verbatim. Viewer libraries: <a href="THREE-LICENSE.txt">Three.js</a> and <a href="URDF-LOADER-LICENSE.txt">urdf-loader</a>.</p><ul>' + ''.join(notices) + '</ul>'))
    (destination / 'robots.txt').write_text('User-agent: *\nDisallow: /\n')

    # Audit all served bytes, including notices; fail rather than silently strip
    # an identifying legal notice. No Git history or local paths are distributed.
    for path in destination.rglob('*'):
        if path.is_file():
            content = path.read_bytes().decode('utf-8', errors='ignore')
            for identifier in IDENTIFIERS:
                found = identifier in content if identifier == 'Contact_contraction' else identifier.casefold() in content.casefold()
                assert not found, (path.name, identifier)
            assert not re.search(r'/(?:workspace|home|tmp|mnt)/', content), path
            if path.suffix != '.stl':  # Binary float bytes are not UI text.
                assert not re.search(r'[\uac00-\ud7a3]', content), ('Untranslated text', path)
    assert checked == 37
    integrity = {p.relative_to(destination).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in sorted(destination.rglob('*')) if p.is_file()}
    (destination / 'review-integrity.json').write_text(json.dumps(dict(
        format_version=1, recordings=checked, numeric_and_boolean_leaves_unchanged=True,
        fingerprints='Pseudonymous metadata; use these distributed-file hashes for integrity.',
        files=integrity), indent=2) + '\n')
    if args.zip:
        args.zip.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(args.zip, 'w', zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(destination.rglob('*')):
                if path.is_file():
                    archive.write(path, path.relative_to(destination))
    print(json.dumps(dict(recordings=checked, model_directories=len(model_dirs),
                          numerical_and_boolean_leaves='unchanged', explicit_identifier_findings=0)))


if __name__ == '__main__':
    main()
