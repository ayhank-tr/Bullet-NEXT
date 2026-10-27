#!/usr/bin/env python3
"""Bullet NEXT Lab - runs HARFANG's own unit tests the way the FL gate does (FL, TL.1).

Usage: run_harfang_tests.py <harfang_build_dir> <harfang_source_dir> [--config Release] [--allow audio,...]

The test program reads its data from ./data next to itself, and on Linux loads the shared
libraries it was linked with from its own folder. The build folder has neither, so this copies
the program into <build>/hg_tests_run, adds HARFANG's test data and the shared libraries, and
runs it there. Passes when every failed test is in --allow (default: audio, because build
machines have no sound device). Run it as a normal user: two file tests check permissions,
which root ignores.
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("build")
    ap.add_argument("source")
    ap.add_argument("--config", default="Release")
    ap.add_argument("--allow", default="audio")
    a = ap.parse_args()
    build, source = os.path.abspath(a.build), os.path.abspath(a.source)
    win = sys.platform == "win32"

    tests = os.path.join(build, "harfang", "tests")
    exe = os.path.join(tests, a.config, "tests.exe") if win else os.path.join(tests, "tests")
    if not os.path.isfile(exe):
        sys.exit(f"no test program at {exe}")
    work = os.path.join(build, "hg_tests_run")
    shutil.rmtree(work, ignore_errors=True)
    shutil.copytree(os.path.dirname(exe), work, symlinks=False)
    shutil.copytree(os.path.join(source, "harfang", "tests", "data"), os.path.join(work, "data"), dirs_exist_ok=True)
    # the video test loads its plugin from ./data; Visual Studio builds it in data/<config>
    for plugin in glob.glob(os.path.join(tests, "**", "DummyVideoStream.*"), recursive=True):
        if os.path.splitext(plugin)[1] in (".dll", ".so", ".dylib"):
            shutil.copy(plugin, os.path.join(work, "data"))
    pattern = "*.dll" if win else "lib*.so*"
    for lib in glob.glob(os.path.join(build, "**", pattern), recursive=True):
        if os.path.commonpath([lib, work]) != work and os.path.isfile(lib):
            shutil.copy(lib, work)  # follows symbolic links

    p = subprocess.run([os.path.join(work, os.path.basename(exe))], cwd=work,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace")
    print(p.stdout)
    # a test's log lines can come between its "Test name..." line and its result
    current, failed, passed = None, [], 0
    for line in p.stdout.splitlines():
        if line.startswith("Test ") and "..." in line:
            current = line[5:line.index("...")]
        if "[ OK ]" in line:
            passed += 1
        if "[ FAILED ]" in line:
            failed.append(current or "?")
    allowed = [x.strip() for x in a.allow.split(",") if x.strip()]
    unexpected = [t for t in failed if not any(k in t for k in allowed)]
    print(f"HARFANG unit tests: {passed} passed, {len(failed)} failed"
          + (f" ({', '.join(failed)})" if failed else "")
          + (f"; allowed: {', '.join(allowed)}" if allowed else ""))
    if passed == 0 or unexpected:
        print("unexpected failures: " + (", ".join(unexpected) if unexpected else "no test ran"))
        sys.exit(1)


if __name__ == "__main__":
    main()
