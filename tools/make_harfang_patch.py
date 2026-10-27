#!/usr/bin/env python3
"""Bullet NEXT Lab - writes integration/harfang/harfang-lab.patch from a working HARFANG checkout.

Usage: make_harfang_patch.py <dest_dir>/harfang

Every change the Lab makes to HARFANG lives in that one patch: edits and new files in the
checkout made by tools/fetch_harfang.py. Submodules are never patched.
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "integration", "harfang", "harfang-lab.patch")


def git(args, cwd):
    return subprocess.run(["git"] + args, cwd=cwd, check=True, capture_output=True, text=True).stdout


def main():
    hg = os.path.abspath(sys.argv[1])
    new = [l for l in git(["ls-files", "--others", "--exclude-standard"], hg).splitlines() if l]
    if new:
        git(["add", "-N", "--"] + new, hg)
    try:
        patch = git(["diff", "--ignore-submodules=all", "--", "."], hg)
    finally:
        if new:
            git(["reset", "-q", "--"] + new, hg)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", newline="\n") as f:
        f.write(patch)
    print(f"wrote {OUT} ({patch.count('diff --git')} files)")


if __name__ == "__main__":
    main()
