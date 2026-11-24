"""Prepare full visual/collision URDFs at the mechanics models' pinned revisions.

Meshes stay at immutable upstream URLs. Use --fetch to populate sparse source
checkouts; normal builds and tests never need network access to regenerate URDFs.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from contact_transfer.urdf import URDFTree


ROOT = Path(__file__).resolve().parents[1]
PROFILES = {
    "franka_panda": ({"example-robot-data": "."}, {}, []),
    "rb3": ({"rbpodo_description": "rbpodo_description"}, {}, []),
    "rb5": ({"rbpodo_description": "rbpodo_description"}, {}, []),
    "kinova_gen3": ({"kortex_description": "kortex_description"}, {}, []),
    "unitree_g1": ({"g1_description": "robots/g1_description"}, {}, []),
    "kuka_iiwa7": ({"iiwa_description": "iiwa_description"}, {}, ["iiwa_description/urdf"]),
    "ur5e": ({"ur_description": "."}, {"ur_type": "ur5e", "name": "ur5e"}, ["urdf", "config/ur5e"]),
    "xarm7": ({"xarm_description": "xarm_description", "xarm_controller": "xarm_controller"},
              {"dof": "7", "robot_type": "xarm"}, ["xarm_description/urdf", "xarm_description/config", "xarm_controller/config"]),
}


def source_path(uri, urdf_path, packages, source_root):
    """Resolve package, relative and expanded absolute paths inside a source tree."""
    if uri.startswith("package://"):
        package, rel = uri[len("package://"):].split("/", 1)
        if package not in packages:
            raise ValueError(f"unmapped URDF package: {package}")
        path = source_root/packages[package]/rel
    elif "://" in uri:
        raise ValueError(f"unexpected remote URI in upstream URDF: {uri}")
    else:
        path = urdf_path.parent/uri
    path = path.resolve()
    if not path.is_relative_to(source_root.resolve()):
        raise ValueError(f"asset outside source repository: {uri}")
    return path.relative_to(source_root.resolve()).as_posix()


def validate_mechanics(full, subset, locked):
    """Reject a visual model that changes the recorded mechanical tree."""
    a, b = URDFTree(full, locked), URDFTree(subset, locked)
    if a.root != b.root or set(a.links) != set(b.links) or set(a.by_name) != set(b.by_name):
        raise ValueError("visual URDF does not match the mechanics link/joint tree")
    for name, joint in a.by_name.items():
        other = b.by_name[name]
        for field in ("kind", "parent", "child", "mimic"):
            if getattr(joint, field) != getattr(other, field):
                raise ValueError(f"mechanics mismatch: {name}.{field}")
        for field in ("R", "p", "axis", "lower", "upper", "effort", "velocity"):
            np.testing.assert_allclose(getattr(joint, field), getattr(other, field), atol=1e-10, rtol=1e-10)
    for name, link in a.links.items():
        for field in ("mass", "com", "inertia"):
            np.testing.assert_allclose(getattr(link, field), getattr(b.links[name], field), atol=1e-10, rtol=1e-10)


def fetch_sources(sources, models):
    repos = {}
    for name, meta in models.items():
        url, revision = meta["source_repository"], meta["commit"]
        key = url.removesuffix(".git").rsplit("/", 1)[-1]
        entry = repos.setdefault(key, {"url": url, "revision": revision, "paths": set()})
        entry["paths"].update([meta["source_path"], meta["license_path"], *PROFILES[name][2]])
    for key, entry in repos.items():
        repo = sources/key
        repo.mkdir(parents=True, exist_ok=True)
        def git(*args):
            return subprocess.run(["git", "-C", str(repo), *args], check=True)
        if not (repo/".git").is_dir():
            git("init", "-q")
            git("remote", "add", "origin", entry["url"])
        if not re.fullmatch(r"[0-9a-f]{40}", entry["revision"]):
            raise ValueError("source revision must be a full commit hash")
        git("sparse-checkout", "set", "--no-cone", *["/"+p for p in sorted(entry["paths"])] )
        git("fetch", "--depth=1", "--filter=blob:none", "origin", entry["revision"])
        git("checkout", "--detach", entry["revision"])


def prepare(name, meta, sources, destination):
    packages, mappings, _ = PROFILES[name]
    key = meta["source_repository"].removesuffix(".git").rsplit("/", 1)[-1]
    original = sources/key
    revision = subprocess.check_output(["git", "-C", str(original), "rev-parse", "HEAD"], text=True).strip()
    if revision != meta["commit"]:
        raise ValueError(f"{name}: source checkout must match {meta['commit']}")
    source_bytes = (original/meta["source_path"]).read_bytes()
    if hashlib.sha256(source_bytes).hexdigest() != meta["source_file_sha256"]:
        raise ValueError(f"{name}: source file hash differs from model provenance")
    with tempfile.TemporaryDirectory() as temp:
        work = Path(temp)/"source"
        shutil.copytree(original, work, ignore=shutil.ignore_patterns(".git"))
        path = work/meta["source_path"]
        if path.suffix == ".xacro":
            import xacro
            for f in work.rglob("*.xacro"):
                text = f.read_text()
                for package, rel in packages.items():
                    text = text.replace("$(find "+package+")", str(work/rel))
                f.write_text(text)
            xml = xacro.process_file(str(path), mappings=mappings).toxml()
        else:
            xml = path.read_text()
        full = work/"expanded.urdf"; full.write_text(xml)
        validate_mechanics(full, ROOT/"src/contact_transfer/models"/name/"robot.urdf", meta["locked_joints"])
        tree = ET.fromstring(xml)
        assets = set()
        for element in list(tree.iter("mesh"))+list(tree.iter("texture")):
            uri = element.get("filename")
            if uri:
                rel = source_path(uri, path, packages, work)
                assets.add(rel)
                element.set("filename", "package://upstream/"+rel)
        # Keep only model data understood by the browser loader.
        for child in list(tree):
            if child.tag not in {"link", "joint", "material"}:
                tree.remove(child)
        ET.indent(tree)
        dest = destination/name; dest.mkdir(parents=True, exist_ok=True)
        # Preserve upstream attribution, including comments outside <robot>.
        comments = dict.fromkeys(re.findall(r"<!--.*?-->", source_bytes.decode()+xml, flags=re.DOTALL))
        notices = "\n".join(comments).replace(str(work), "upstream").replace("\r\n", "\n")
        header = (f'<?xml version="1.0" encoding="utf-8"?>\n'
                  f'<!-- Modified for the viewer from {meta["source_repository"]} at {revision}. '
                  'Xacro expanded; mesh URIs rewritten; controller tags omitted; '
                  'upstream comments retained in this header. -->\n')
        (dest/"robot.urdf").write_text(header+notices+"\n"+ET.tostring(tree, encoding="unicode")+"\n")
        (dest/"LICENSE.upstream").write_bytes((original/meta["license_path"]).read_bytes())
        base = meta["source_repository"].removesuffix(".git").replace("https://github.com/", "https://raw.githubusercontent.com/")+"/"+revision
        manifest = {"id": name, "urdf": "robot.urdf", "packages": {"upstream": base},
                    "source_repository": meta["source_repository"], "commit": revision,
                    "source_path": meta["source_path"], "source_file_sha256": meta["source_file_sha256"],
                    "urdf_sha256": hashlib.sha256((dest/"robot.urdf").read_bytes()).hexdigest(),
                    "license": meta["license"], "license_file": "LICENSE.upstream",
                    "assets": [base+"/"+p for p in sorted(assets)],
                    "geometry": "upstream_visual_and_collision", "asset_transport": "pinned_upstream_https",
                    "modifications": ["xacro expanded where applicable", "mesh/texture URIs rewritten to pinned upstream package",
                                      "controller tags omitted", "upstream comments retained in header"],
                    "mechanics_verified": True}
        (dest/"model.json").write_text(json.dumps(manifest, indent=2)+"\n")
        print(f"Prepared {name}: {len(assets)} mesh/texture references", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sources", type=Path, required=True)
    parser.add_argument("--fetch", action="store_true")
    parser.add_argument("--models", nargs="+", choices=PROFILES, default=list(PROFILES))
    parser.add_argument("--output", type=Path, default=ROOT/"web/models")
    args = parser.parse_args()
    models = {name: json.loads((ROOT/"src/contact_transfer/models"/name/"model.json").read_text()) for name in args.models}
    if args.fetch:
        fetch_sources(args.sources, models)
    for name, meta in models.items():
        prepare(name, meta, args.sources, args.output)


if __name__ == "__main__":
    main()
