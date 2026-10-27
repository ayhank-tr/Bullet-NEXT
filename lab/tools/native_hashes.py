#!/usr/bin/env python3
"""Bullet NEXT Lab - writes lab/scenes/native_hashes.txt (FL, CL.10 and TL.8).

Usage: native_hashes.py <bnl_run> [<out_file>]

Runs every scene on every engine of a native bnl_run, single thread, and keeps the final hashes
as "scene engine hash" lines. The browser viewer reads the file and says, at the end of a run,
whether the browser reached the same hash as the PC; tools compare it with the browser runner.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "scenes", "native_hashes.txt")


def main():
    runner = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else OUT
    p = subprocess.run([runner, "--threads", "0"], capture_output=True, text=True, check=True)
    lines = []
    for line in p.stdout.splitlines():
        f = line.split()
        if len(f) > 5 and f[2] == "frames":
            lines.append(f"{f[0]} {f[1]} {f[5]}")
    with open(out, "w") as f:
        f.write("# scene engine final_hash - native bnl_run, one thread (lab/tools/native_hashes.py)\n")
        f.write("\n".join(lines) + "\n")
    print(f"wrote {out} ({len(lines)} runs)")


if __name__ == "__main__":
    main()
