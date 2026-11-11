"""Fetch immutable Panda collision meshes into the ignored simulation cache."""

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def main():
    meta = json.loads((ROOT/"web/models/franka_panda/model.json").read_text())
    urls = [u for u in meta["assets"] if "/collision/" in u]
    cache = ROOT/".cache/panda_meshes"; cache.mkdir(parents=True, exist_ok=True)
    def fetch(url):
        target = cache/url.rsplit("/", 1)[-1]
        if not target.is_file():
            with urllib.request.urlopen(url, timeout=60) as response:
                data = response.read()
            if data.startswith(b"version https://git-lfs"):
                raise ValueError("Unexpected LFS pointer instead of a mesh")
            target.write_bytes(data)
        return target.name
    with ThreadPoolExecutor(max_workers=4) as pool:
        print(list(pool.map(fetch, urls)))


if __name__ == "__main__": main()
