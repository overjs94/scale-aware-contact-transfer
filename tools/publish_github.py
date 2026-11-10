"""Create a new private GitHub repo using the user's authenticated gh CLI."""

import argparse
from pathlib import Path
import subprocess


def run(*args):
    return subprocess.run(args, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--owner", required=True)
    parser.add_argument("--name", default="scale-aware-contact-transfer")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    run("gh", "auth", "status")
    if not (root/".git").is_dir():
        raise SystemExit("Initialize and commit the package before publishing.")
    dirty = subprocess.check_output(["git", "-C", str(root), "status", "--porcelain"], text=True)
    if dirty:
        raise SystemExit("Commit or set aside local changes before publishing.")
    run("gh", "repo", "create", f"{args.owner}/{args.name}", "--private",
        "--description", "URDF-driven scale-aware contact transfer and trajectory optimization",
        "--source", str(root), "--remote", "origin", "--push")


if __name__ == "__main__":
    main()
