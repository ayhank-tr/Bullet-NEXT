#!/usr/bin/env python3
"""Bullet NEXT Lab - fetches the pinned HARFANG 3D the Lab is built on (FL, CL.1).

Usage: fetch_harfang.py <dest_dir> [--no-patch]

Clones HARFANG 3D v3.2.7 and FABgen (its binding generator) into <dest_dir>/harfang and
<dest_dir>/fabgen, checks out the submodules the Lab build needs (no VR, no asset converters),
and applies integration/harfang/harfang-lab.patch. Run it again on an existing <dest_dir> to
reset both checkouts to the pins and re-apply the patch.

FABgen needs the Python package pypeg2 (pip install pypeg2).
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATCH = os.path.join(ROOT, "integration", "harfang", "harfang-lab.patch")

HARFANG_URL = "https://github.com/harfang3d/harfang3d"
HARFANG_PIN = "ef65b8a8f6d8285333286b13466d7728cb7bc911"  # tag v3.2.7, 14 Aug 2023
FABGEN_URL = "https://github.com/harfang3d/fabgen"
FABGEN_PIN = "241a246b981bc9e9c233d9ae7d95dc949a2cc08e"   # 23 Nov 2022, the last FABgen commit
SUBMODULES = [  # pinned by the HARFANG commit itself
    "extern/bgfx/bgfx", "extern/bgfx/bx", "extern/bgfx/bimg", "extern/glfw",
    "extern/openal-soft", "extern/recastnavigation", "extern/meshoptimizer", "extern/cmft",
]


# Keep HARFANG's own line endings on every system. Git for Windows turns LF into CRLF on checkout
# by default (core.autocrlf=true), and the Lab patch then no longer fits. "-c" also reaches the
# submodule clones, which git runs as child processes.
GIT = ["git", "-c", "core.autocrlf=false", "-c", "core.eol=lf"]


def git(args, cwd=None):
    print("git", " ".join(args), flush=True)
    subprocess.run(GIT + args, cwd=cwd, check=True)


def checkout(url, pin, path):
    if not os.path.isdir(os.path.join(path, ".git")):
        git(["clone", "-q", url, path])
    git(["fetch", "-q", "origin", pin], cwd=path)
    git(["reset", "-q", "--hard"], cwd=path)
    git(["clean", "-q", "-fdx"], cwd=path)
    git(["checkout", "-q", "--detach", pin], cwd=path)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    dest = os.path.abspath(sys.argv[1])
    os.makedirs(dest, exist_ok=True)
    hg = os.path.join(dest, "harfang")
    checkout(HARFANG_URL, HARFANG_PIN, hg)
    git(["submodule", "update", "--init", "--depth", "50", "--"] + SUBMODULES, cwd=hg)
    checkout(FABGEN_URL, FABGEN_PIN, os.path.join(dest, "fabgen"))
    if "--no-patch" not in sys.argv:
        git(["apply", "--whitespace=nowarn", PATCH], cwd=hg)
    print(f"HARFANG ready in {hg}")


if __name__ == "__main__":
    main()
