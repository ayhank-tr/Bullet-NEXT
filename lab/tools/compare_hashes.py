#!/usr/bin/env python3
"""Bullet NEXT Lab - compares a bnl_run output with lab/scenes/native_hashes.txt (FL, TL.8).

Usage: compare_hashes.py <run> [--reference FILE] [--require next,nextf,jolt]

<run> is a bnl_run output, or the JSON written by lab/harfang/lab_headless.py (HARFANG).

Prints, per engine, how many scenes reached the reference final hash. Exit code 1 when an engine
listed in --require differs on any scene; the others are only reported (legacy uses the system's
math library, as Blender 2.79b does, so it is not expected to match on another system).
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def load(path, columns):
    out = {}
    if columns == "run" and path.endswith(".json"):
        for scene, engines in json.load(open(path, encoding="utf-8")).items():
            for engine, r in engines.items():
                out[(scene, engine)] = r["hash"]
        return out
    for line in open(path, encoding="utf-8"):
        f = line.split()
        if columns == "run" and len(f) > 5 and f[2] == "frames":
            out[(f[0], f[1])] = f[5]
        elif columns == "ref" and len(f) == 3 and not line.startswith("#"):
            out[(f[0], f[1])] = f[2]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--reference", default=os.path.join(os.path.dirname(HERE), "scenes", "native_hashes.txt"))
    ap.add_argument("--require", default="next,nextf,jolt")
    a = ap.parse_args()
    ref, run = load(a.reference, "ref"), load(a.run, "run")
    required = set(x for x in a.require.split(",") if x)
    engines = sorted({e for _, e in run})
    failed = False
    for e in engines:
        keys = [k for k in run if k[1] == e and k in ref]
        diff = sorted(k[0] for k in keys if run[k] != ref[k])
        tag = "required" if e in required else "report only"
        print(f"{e:8s} {len(keys) - len(diff)}/{len(keys)} same as reference ({tag})" + (f"; differ: {', '.join(diff)}" if diff else ""))
        if diff and e in required:
            failed = True
        if not keys and e in required:
            print(f"{e:8s} no scenes to compare")
            failed = True
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
