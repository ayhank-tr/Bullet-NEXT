#!/usr/bin/env python3
"""Bullet NEXT - FL gate run (the Lab): tests TL.1-TL.8 and benchmarks BL.1-BL.3.

Usage: run_fl_gate.py --build B --out DIR [--harfang H] [--wasm W] [--asan A] [--tsan T]
                      [--chrono-results FILE] [--viewer-results FILE] [--browser-results FILE]

  --build    native Lab build (Release, harness and Lab tools)
  --harfang  native HARFANG build with Bullet NEXT (HG_BULLET_NEXT_DIR): Python module, tests
  --wasm     Emscripten build of the Lab runner (bnl_run.js)
  --asan     Lab build with -fsanitize=address,undefined; --tsan the same with -fsanitize=thread
  --chrono-results   JSON from tools/run_chrono_check.sh plus the Chrono engine runs (TL.4)
  --viewer-results   JSON: viewer screenshots per scene (TL.8)
  --browser-results  JSON: the browser page's BNL_RESULT lines per scene (TL.8)
Writes DIR/fl_gate.json (results and numbers for docs/FL_GATE_REPORT.md) and every raw output. Thresholds: docs/gates.json ("FL").
A test whose inputs are missing is reported as "not run here", never as passed.
"""
import argparse
import csv
import json
import os
import platform
import subprocess
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
F0_COMMIT = "3bedd98"  # F0 as pushed; legacy must not change after it
SCENES = json.load(open(os.path.join(ROOT, "lab", "scenes", "index.json")))["scenes"]
F0_SCENES = SCENES[:16]
RB_SCENES = [s for s in SCENES if s.startswith("rb_")]


def run(cmd, env=None, out=None, timeout=None):
    e = dict(os.environ)
    e.update(env or {})
    p = subprocess.run(cmd, capture_output=True, text=True, env=e, timeout=timeout)
    text = p.stdout + p.stderr
    if out:
        open(out, "w").write(text)
    return p.returncode, text


def runs(text):
    """bnl_run lines -> {(scene, engine): {'hash', 'ok', 'ms'}}"""
    r = {}
    for line in text.splitlines():
        f = line.split()
        if len(f) > 9 and f[2] == "frames":
            r[(f[0], f[1])] = {"hash": f[5], "ok": f[6] == "ok", "ms": float(f[8])}
    return r


def reference():
    ref = {}
    for line in open(os.path.join(ROOT, "lab", "scenes", "native_hashes.txt")):
        f = line.split()
        if len(f) == 3 and not line.startswith("#"):
            ref[(f[0], f[1])] = f[2]
    return ref


def summary(path):
    with open(path) as f:
        return {r["scene"]: r for r in csv.DictReader(f)}


def checker_reports(text):
    """Lines the address / undefined-behaviour / thread checkers print, grouped by place."""
    rep = defaultdict(int)
    for line in text.splitlines():
        if "runtime error:" in line:
            rep[line.split(": runtime error:")[0].split("/bullet-next/")[-1] + ": " + line.split("runtime error: ")[1][:60]] += 1
        elif "ERROR: AddressSanitizer" in line or "ERROR: LeakSanitizer" in line or "WARNING: ThreadSanitizer" in line:
            rep[line.split("==")[-1].strip()[:90]] += 1
    return dict(rep)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", required=True)
    ap.add_argument("--out", required=True)
    for k in ("harfang", "wasm", "asan", "tsan", "chrono-results", "viewer-results", "browser-results"):
        ap.add_argument("--" + k)
    ap.add_argument("--fuzz-pairs", type=int, default=100000)
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    b = os.path.abspath(a.build)
    gates = json.load(open(os.path.join(ROOT, "docs", "gates.json")))["FL"]
    ref = reference()
    res, notes = {}, {}

    # Native single-thread run of every scene on every engine: the reference itself, timings (BL.2)
    _, t = run([f"{b}/lab/bnl_run", "--threads", "0"], out=f"{out}/run_native_t0.txt")
    native = runs(t)
    _, t = run([f"{b}/lab/bnl_run", "--threads", str(os.cpu_count() or 2)], out=f"{out}/run_native_tall.txt")
    native_all = runs(t)
    ref_ok = all(native.get(k, {}).get("hash") == v for k, v in ref.items())

    # TL.1 builds: this Lab build; HARFANG with Bullet NEXT and its unit tests
    tl1 = {"lab": os.path.exists(f"{b}/lab/bnl_run")}
    if a.harfang:
        hg = os.path.abspath(a.harfang)
        tl1["harfang"] = all(os.path.exists(os.path.join(hg, p)) for p in
                             ("harfang/engine/libengine.a", "languages/hg_python/harfang.so", "harfang/bnl_viewer"))
        # HARFANG's own tests: run as a normal user (two of them check file permissions)
        tests = os.path.join(hg, "harfang", "tests")
        # in a copy the test user owns: the file tests write next to the program
        work = "/home/hgtest/hg_tests"
        run(["rm", "-rf", work])
        run(["cp", "-r", tests, work])
        for root, _, files in os.walk(hg):  # the shared libraries the test program loads from its folder
            for f in files:
                if f in ("libglfw3.so", "liblua54.so"):
                    run(["cp", "-L", os.path.join(root, f), work])
        # the test data lives in HARFANG's source tree (the build folder only has the video plugin)
        src = [l.split("=", 1)[1].strip() for l in open(os.path.join(hg, "CMakeCache.txt")) if l.startswith("CMAKE_HOME_DIRECTORY:")][0]
        run(["sh", "-c", f"cp -r {src}/harfang/tests/data/. {work}/data/"])
        run(["chown", "-R", "hgtest:hgtest", work])
        rc, t = run(["su", "hgtest", "-c", f"cd {work} && ./tests"], out=f"{out}/harfang_tests.txt", timeout=1800)
        failed = [l for l in t.splitlines() if "[ FAILED ]" in l]
        passed = t.count("[ OK ]")
        tl1["harfang_tests"] = f"{passed} passed, {len(failed)} failed" + (f" ({'; '.join(x.strip()[:60] for x in failed)})" if failed else "")
        tl1["harfang_tests_ok"] = all("audio" in x.lower() or "sound" in x.lower() for x in failed)
    res["TL.1"] = tl1["lab"] and tl1.get("harfang", False) and tl1.get("harfang_tests_ok", False)
    notes["TL.1"] = tl1

    # TL.2 legacy untouched: no change to legacy/ since F0, recordings equal the F0 goldens
    rc, _ = run(["git", "-C", ROOT, "diff", "--quiet", F0_COMMIT, "--", "legacy", "third_party/bullet284_orig"])
    d = f"{out}/tl2"
    run([f"{b}/harness/bn_harness_legacy", "run", "all", "--out", d, "--repeat", "1"])
    gold = {}
    for line in open(os.path.join(ROOT, "goldens", "f0-linux-gcc", "hashes.csv")):
        if "," in line and not line.startswith(("#", "scene,")):
            n, h = line.strip().split(",")
            gold[n] = h
    leg = summary(f"{d}/summary.legacy.csv")
    same = [n for n in F0_SCENES if leg.get(n, {}).get("final_hash") == gold.get(n)]
    res["TL.2"] = rc == 0 and len(same) == len(F0_SCENES)
    notes["TL.2"] = {"legacy_dir_unchanged_since_F0": rc == 0, "goldens": f"{len(same)}/{len(F0_SCENES)}",
                     "lab_runner_legacy_vs_goldens": f"{sum(native.get((n, 'legacy'), {}).get('hash') == gold.get(n) for n in F0_SCENES)}/{len(F0_SCENES)}"}

    # TL.3 same physics in HARFANG: every scene headless in HARFANG (Python) = the Lab's hash
    if a.harfang:
        hg = os.path.abspath(a.harfang)
        js = f"{out}/harfang_headless.json"
        run([sys.executable, os.path.join(ROOT, "lab", "harfang", "lab_headless.py"), "--engines", "legacy,next,nextf,jolt", "--out", js],
            env={"PYTHONPATH": os.path.join(hg, "languages", "hg_python"), "BN_THREADS": "0"}, out=f"{out}/harfang_headless.txt")
        hh = json.load(open(js))
        cnt = defaultdict(lambda: [0, 0])
        for s, es in hh.items():
            for e, r in es.items():
                cnt[e][1] += 1
                cnt[e][0] += r["hash"] == ref.get((s, e))
        res["TL.3"] = all(v[0] == v[1] for v in cnt.values()) and cnt["legacy"][1] >= 16 and cnt["nextf"][1] >= 16
        notes["TL.3"] = {e: f"{v[0]}/{v[1]}" for e, v in cnt.items()}
        # BL.1 host overhead: HARFANG's frame (node sync, scene update) over the physics step
        p = hh["pile_1000"]
        notes["BL.1"] = {e: {"step_ms": round(p[e]["step_ms"], 1), "frame_ms": round(p[e]["frame_ms"], 1),
                             "overhead_pct": round(100 * (p[e]["frame_ms"] / p[e]["step_ms"] - 1), 2),
                             "bare_lab_ms": native.get(("pile_1000", e), {}).get("ms")} for e in p}
        res["BL.1"] = all(v["overhead_pct"] <= gates["host_overhead_max_pct"] for v in notes["BL.1"].values())

    # TL.4 all engines: Jolt (and Chrono) run every scene they support with no NaN or blow-up
    jolt = [s for s in SCENES if native.get((s, "jolt"), {}).get("ok")]
    notes["TL.4"] = {"jolt": f"{len(jolt)}/{len(SCENES)} scenes without NaN or blow-up"}
    chrono_ok = None
    if a.chrono_results:
        cr = json.load(open(a.chrono_results))
        eng = cr.get("engine_runs", {})
        chrono_ok = bool(eng) and all(v.get("ok") for v in eng.values())
        notes["TL.4"]["chrono"] = f"{sum(1 for v in eng.values() if v.get('ok'))}/{len(eng)} scenes without NaN or blow-up" + \
            (f" ({', '.join(cr.get('skipped', []))} not supported)" if cr.get("skipped") else "")
        notes["TL.4"]["chrono_unit_tests"] = cr.get("tests_summary", "")
    res["TL.4"] = len(jolt) == len(SCENES) and chrono_ok is not False

    # TL.5 host profiles: bge279 = F0 goldens (TL.2); blender53-rb on Blender's Bullet = Blender's code
    d5 = f"{out}/tl5"
    run([f"{b}/harness/bn_harness_blender53", "run", "all", "--out", d5, "--repeat", "1"])
    prof = summary(f"{d5}/summary.blender53.csv")
    _, t = run([f"{b}/lab/bnl_rbref"], out=f"{out}/rbref.txt")
    rb = {l.split()[0]: l.split()[5] for l in t.splitlines() if " rbref " in l}
    rbsame = [s for s in RB_SCENES if prof.get(s, {}).get("final_hash") == rb.get(s)]
    res["TL.5"] = res["TL.2"] and len(rbsame) == len(RB_SCENES)
    notes["TL.5"] = {"bge279_vs_goldens": notes["TL.2"]["goldens"], "blender53_rb_vs_blender": f"{len(rbsame)}/{len(RB_SCENES)}"}

    # TL.6 recordings: playback reaches the live run's hash on every frame
    _, t = run([f"{b}/lab/bnl_run", "--threads", "0", "--replay-check", "1"], out=f"{out}/replay_check.txt")
    rep_lines = [l for l in t.splitlines() if len(l.split()) > 3 and l.split()[2] == "replay"]
    rep_bad = [l for l in rep_lines if l.split()[3] != "same"]
    res["TL.6"] = bool(rep_lines) and not rep_bad
    notes["TL.6"] = f"{len(rep_lines) - len(rep_bad)}/{len(rep_lines)} runs played back identically"

    # TL.7 check mode: checkers on every scene; random collision pairs
    tl7 = {}
    ok7 = True
    for kind, bdir in (("asan_ubsan", a.asan), ("tsan", a.tsan)):
        if not bdir:
            tl7[kind] = "not run here"
            ok7 = False
            continue
        bdir = os.path.abspath(bdir)
        env = {"UBSAN_OPTIONS": f"print_stacktrace=1:halt_on_error=0:suppressions={ROOT}/lab/tools/ubsan_legacy.supp",
               "ASAN_OPTIONS": "detect_leaks=1:halt_on_error=0", "TSAN_OPTIONS": "halt_on_error=0"}
        texts = []
        if kind == "asan_ubsan":
            texts.append(run([f"{bdir}/lab/bnl_run", "--threads", "0"], env=env, out=f"{out}/{kind}_run.txt")[1])
            texts.append(run([f"{bdir}/lab/bnl_run", "--engines", "next,nextf"], env=dict(env, BN_THREADS="2"), out=f"{out}/{kind}_run_threads.txt")[1])
            texts.append(run([f"{bdir}/lab/bnl_fuzz", "--pairs", str(a.fuzz_pairs)], env=env, out=f"{out}/{kind}_fuzz.txt")[1])
        else:
            texts.append(run([f"{bdir}/lab/bnl_run", "--engines", "nextf"], env=dict(env, BN_THREADS="4"), out=f"{out}/{kind}_run.txt")[1])
        reps = {}
        for x in texts:
            for k, v in checker_reports(x).items():
                reps[k] = reps.get(k, 0) + v
        tl7[kind] = reps or "no reports"
        ok7 = ok7 and not reps
    _, t = run([f"{b}/lab/bnl_fuzz", "--pairs", str(a.fuzz_pairs)], out=f"{out}/fuzz_native.txt")
    fz = [l for l in t.splitlines() if " pairs " in l]
    tl7["fuzz"] = fz
    ok7 = ok7 and len(fz) == 3 and all(" failed 0 " in l for l in fz)
    res["TL.7"] = ok7
    notes["TL.7"] = tl7

    # TL.8 viewer and browser
    tl8 = {}
    if a.viewer_results:
        v = json.load(open(a.viewer_results))
        tl8["viewer"] = f"{sum(1 for s in SCENES if v.get(s, {}).get('drawn'))}/{len(SCENES)} scenes drawn"
    web_ok = None
    if a.wasm:
        _, t = run(["node", f"{os.path.abspath(a.wasm)}/lab/bnl_run.js", "--threads", "0"], out=f"{out}/run_wasm.txt", timeout=3600)
        web = runs(t)
        per = {}
        for e in ("legacy", "next", "nextf", "jolt"):
            keys = [k for k in ref if k[1] == e]
            per[e] = f"{sum(web.get(k, {}).get('hash') == ref[k] for k in keys)}/{len(keys)}"
        tl8["browser_runner"] = per
        web_ok = all(per[e].split("/")[0] == per[e].split("/")[1] for e in ("next", "nextf", "jolt"))
        # BL.3 browser speed: physics time per scene in WebAssembly against the native build
        notes["BL.3"] = {s: {e: {"native_ms": native.get((s, e), {}).get("ms"), "wasm_ms": web.get((s, e), {}).get("ms")}
                             for e in ("legacy", "nextf", "jolt")} for s in SCENES}
    if a.browser_results:
        br = json.load(open(a.browser_results))
        tl8["browser_page"] = br
    viewer_ok = bool(a.viewer_results) and tl8.get("viewer", "").startswith(f"{len(SCENES)}/")
    page_ok = True
    if a.browser_results:  # Bullet NEXT and Jolt must say "same" in the page; legacy is reported
        page_ok = all(l.split()[-1] == "same" for r in br.values() for l in r["lines"] if l.split()[2] != "legacy")
    res["TL.8"] = viewer_ok and web_ok is True and page_ok
    notes["TL.8"] = tl8

    # BL.2 reference table: every engine on every scene, 1 thread and all threads
    notes["BL.2"] = {s: {e: (native.get((s, e), {}).get("ms"), native_all.get((s, e), {}).get("ms"))
                         for e in ("legacy", "next", "nextf", "jolt")} for s in SCENES}

    json.dump({"results": res, "notes": notes, "reference_matches_build": ref_ok}, open(f"{out}/fl_gate.json", "w"), indent=1)
    print(json.dumps(res, indent=1))
    print("reference hashes match this build:", ref_ok)


if __name__ == "__main__":
    main()
