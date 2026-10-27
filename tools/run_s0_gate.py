#!/usr/bin/env python3
"""Bullet NEXT - Stage 0 gate run (Ready for Rally FX): tests TS.1-TS.6 and benchmarks BS.1-BS.2.

Usage: run_s0_gate.py --build B --out DIR [--wasm W] [--chrono BIN] [--f1 DIR] [--f3c DIR]

  --build   native Lab build (Release): lab/bnl_run
  --wasm    Emscripten build of the Lab runner (bnl_run.js): the rally scenes in the browser build
  --chrono  the Chrono Lab engine (tools/build_chrono_engine.sh): its heightfield skip and its
            hashes on the earlier scenes
  --f1, --f3c  output folders of tools/run_f1_gate.py and tools/run_f3c_gate.py on this build
               (TS.2); without them TS.2 is "not run here"

Engines: legacy (Bullet 2.84 as in Blender 2.79b), next (Bullet NEXT without its features), nextf
(with them), nexts (with them and the Soft Step solver), jolt. Thresholds: docs/gates.json ("S0").
Writes DIR/s0_gate.json and every raw output. A test that cannot run is reported as "not run
here", never as passed.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENES_DIR = os.path.join(ROOT, "lab", "scenes")
ENGINES = ["legacy", "next", "nextf", "nexts", "jolt"]
RALLY = ["rally_slide", "rally_far", "rally_far_shift", "rally_stage"]


def run(cmd, out=None, timeout=None):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
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
        elif len(f) > 2 and f[2] == "skipped:":
            r[(f[0], f[1])] = {"hash": None, "ok": None, "skipped": " ".join(f[3:])}
    return r


def scene_dir(scenes, settings=None):
    d = tempfile.mkdtemp(prefix="s0_")
    for s in scenes:
        sc = json.load(open(os.path.join(SCENES_DIR, s + ".json")))
        sc["settings"].update(settings or {})
        json.dump(sc, open(os.path.join(d, s + ".json"), "w"))
    json.dump({"scenes": scenes}, open(os.path.join(d, "index.json"), "w"))
    return d


def metrics(bnl_run, scenes, engines, out, settings=None, threads=0):
    d = scene_dir(scenes, settings)
    try:
        m = os.path.join(d, "m.jsonl")
        _, text = run([bnl_run, "--engines", ",".join(engines), "--threads", str(threads), "--scenes", d, "--metrics", m], out=out)
        res = {}
        if os.path.exists(m):
            for line in open(m):
                r = json.loads(line)
                res[(r["scene"], r["engine"])] = r
        return res, runs(text)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def path_diff(a, b, offset):
    """Largest distance between two runs' tracks (every 30 frames), the first moved back by offset."""
    worst = 0.0
    for ta, tb in zip(a["track"], b["track"]):
        for pa, pb in zip(ta, tb):
            d = ((pa[0] - offset - pb[0]) ** 2 + (pa[1] - offset - pb[1]) ** 2 + (pa[2] - pb[2]) ** 2) ** 0.5
            worst = max(worst, d)
    return worst


BASE = "3e09963"  # main before Stage 0; its native_hashes.txt is the reference for TS.2


def reference():
    ref = {}
    text = subprocess.run(["git", "-C", ROOT, "show", BASE + ":lab/scenes/native_hashes.txt"], capture_output=True, text=True).stdout
    for line in text.splitlines():
        if line.startswith("#"):
            continue
        f = line.split()
        if len(f) == 3:
            ref[(f[0], f[1])] = f[2]
    return ref


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--wasm")
    ap.add_argument("--chrono")
    ap.add_argument("--f1")
    ap.add_argument("--f3c")
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    bnl_run = os.path.join(os.path.abspath(a.build), "lab", "bnl_run")
    g = json.load(open(os.path.join(ROOT, "docs", "gates.json")))["S0"]
    res, notes = {}, {}

    # TS.1 bounce: speed out / speed in against the set restitution, collision features on
    m, _ = metrics(bnl_run, ["bounce"], ENGINES, f"{out}/ts1_bounce.txt")
    t1 = {}
    for e in ENGINES:
        r = m.get(("bounce", e))
        if not r:
            continue
        rows = []
        for want, (vi, vo, _, _) in zip((0.0, 0.3, 0.6, 0.9), r["bounce"]):
            got = -vo / vi if vi else 0.0
            rows.append({"e": want, "measured": round(got, 4), "error": round(abs(got - want), 4)})
        t1[e] = rows
    res["TS.1"] = all(all(x["error"] <= g["bounce_max_restitution_error"] for x in t1.get(e, [{"error": 9}])) for e in ("nextf", "nexts"))
    notes["TS.1"] = t1

    # TS.2 no regressions: the F1 and F3c gates on this build, and every earlier scene's hash
    # against the reference (lab/scenes/native_hashes.txt as it was before Stage 0)
    t2 = {}
    f1_ok = f3c_ok = None
    if a.f1 and os.path.exists(os.path.join(a.f1, "F1_GATE_REPORT.md")):
        rep = open(os.path.join(a.f1, "F1_GATE_REPORT.md")).read()
        f1_ok = "**F1 gate (cloud part): PASSED.**" in rep
        t2["f1_gate"] = "PASSED" if f1_ok else "NOT PASSED"
    if a.f3c and os.path.exists(os.path.join(a.f3c, "f3c_gate.json")):
        r3 = json.load(open(os.path.join(a.f3c, "f3c_gate.json")))["results"]
        f3c_ok = all(r3.get(k) for k in ("T3.1", "T3.2", "T3.3", "T3.4", "T3.5", "T3.6", "B3.1"))
        t2["f3c_gate"] = {k: v for k, v in r3.items()}
    ref = reference()
    _, text = run([bnl_run, "--threads", "0"], out=f"{out}/ts2_all.txt")
    now = runs(text)
    changed = {}
    for (s, e), h in sorted(ref.items()):
        got = now.get((s, e), {}).get("hash")
        if got != h:
            changed.setdefault(e, []).append(s)
    t2["changed_vs_reference"] = changed
    t2["unchanged_engines_ok"] = not changed.get("legacy") and not changed.get("jolt")
    if a.chrono:
        _, ctext = run([a.chrono, "--scenes", SCENES_DIR, "--out", out], out=f"{out}/ts2_chrono.txt", timeout=7200)
        cnow = runs(ctext)
        t2["chrono_runs"] = {f"{s}": v.get("hash") or ("skipped: " + v.get("skipped", "")) for (s, _), v in cnow.items()}
    res["TS.2"] = (f1_ok is not None and f3c_ok is not None and f1_ok and f3c_ok and t2["unchanged_engines_ok"]) if (f1_ok is not None and f3c_ok is not None) else None
    notes["TS.2"] = t2

    # TS.3 bumps, TS.4 far from the origin: one metrics run of the slide scenes
    m, r_slide = metrics(bnl_run, ["rally_slide", "rally_far", "rally_far_shift"], ENGINES, f"{out}/ts3_ts4.txt")
    t3, t4 = {}, {}
    for e in ENGINES:
        s = m.get(("rally_slide", e))
        if not s:
            continue
        t3[e] = {"up_max_mps": round(s["up_max"], 4), "per_body": [[b, round(u, 4)] for b, u, _ in s["up_bodies"]],
                 "far_up_max_mps": round(m[("rally_far", e)]["up_max"], 4)}
        t4[e] = {"far_path_diff_mm": round(1000 * path_diff(m[("rally_far", e)], s, g["far_from_origin_m"]), 3),
                 "recentred_path_diff_mm": round(1000 * path_diff(m[("rally_far_shift", e)], s, 0.0), 3)}
    res["TS.3"] = all(t3.get(e, {}).get("up_max_mps", 9) <= g["bump_max_upward_speed_mps"] for e in ("nextf", "nexts"))
    res["TS.4"] = all(t4.get(e, {}).get("recentred_path_diff_mm", 9e9) <= 1000 * g["recentred_path_max_difference_m"] for e in ("nextf", "nexts"))
    notes["TS.3"], notes["TS.4"] = t3, t4

    # TS.5 replay: rally_stage at 1, 2, 4 and 8 threads, recorded and played back; the browser build
    d = scene_dir(["rally_stage"])
    hashes = {}
    try:
        for th in (1, 2, 4, 8):
            _, text = run([bnl_run, "--engines", "nextf,nexts", "--threads", str(th), "--scenes", d], out=f"{out}/ts5_t{th}.txt")
            for (s, e), v in runs(text).items():
                hashes.setdefault(e, {})[th] = v["hash"]
        rec = os.path.join(out, "rec")
        os.makedirs(rec, exist_ok=True)
        rc, text = run([bnl_run, "--engines", ",".join(ENGINES), "--threads", "0", "--scenes", d, "--record", rec, "--replay-check", "1"], out=f"{out}/ts5_replay.txt")
        replay_ok = rc == 0 and text.count(" replay same") == len(ENGINES) and "DIFFERENT" not in text
    finally:
        shutil.rmtree(d, ignore_errors=True)
    t5 = {"threads": hashes, "same_at_all_threads": {e: len(set(h.values())) == 1 for e, h in hashes.items()}, "replay_ok": replay_ok}
    if a.wasm:
        d = scene_dir(RALLY)
        try:
            _, text = run(["node", os.path.join(os.path.abspath(a.wasm), "lab", "bnl_run.js"), "--threads", "0", "--scenes", d], out=f"{out}/ts5_wasm.txt", timeout=7200)
        finally:
            shutil.rmtree(d, ignore_errors=True)
        web = runs(text)
        native = {k: v for k, v in now.items() if k[0] in RALLY}
        t5["browser_same"] = {e: [s for s in RALLY if web.get((s, e), {}).get("hash") == native.get((s, e), {}).get("hash")] for e in ("next", "nextf", "nexts", "jolt")}
    res["TS.5"] = all(t5["same_at_all_threads"].values()) and len(hashes) == 2 and replay_ok and \
        (not a.wasm or all(len(v) == len(RALLY) for v in t5["browser_same"].values()))
    notes["TS.5"] = t5

    # TS.6 engines: every rally scene runs on every engine (no NaN, no blow-up, nothing falls off)
    t6 = {f"{s} {e}": now.get((s, e), {}).get("ok") for s in RALLY for e in ENGINES}
    if a.chrono:
        t6["chrono"] = {s: notes["TS.2"].get("chrono_runs", {}).get(s) for s in RALLY}
    res["TS.6"] = all(v for k, v in t6.items() if k != "chrono")
    notes["TS.6"] = t6

    # BS.1 rally stage cost per engine; BS.2 the same terrain as a heightfield and as a mesh
    m, _ = metrics(bnl_run, ["rally_stage", "rally_stage_mesh"], ENGINES, f"{out}/bs.txt")
    notes["BS.1"] = {e: round(m[("rally_stage", e)]["ms"], 1) for e in ENGINES if ("rally_stage", e) in m}
    notes["BS.2"] = {e: {"heightfield_ms": round(m[("rally_stage", e)]["ms"], 1), "mesh_ms": round(m[("rally_stage_mesh", e)]["ms"], 1),
                         "heightfield_up_max_mps": round(m[("rally_stage", e)]["up_max"], 3), "mesh_up_max_mps": round(m[("rally_stage_mesh", e)]["up_max"], 3)}
                     for e in ENGINES if ("rally_stage", e) in m and ("rally_stage_mesh", e) in m}
    res["BS.1"] = res["BS.2"] = True  # records

    json.dump({"results": res, "notes": notes}, open(f"{out}/s0_gate.json", "w"), indent=1)
    for k in ["TS.1", "TS.2", "TS.3", "TS.4", "TS.5", "TS.6", "BS.1", "BS.2"]:
        print(k, {True: "PASS", False: "FAIL", None: "not run here"}[res.get(k)])
    return 0 if all(res.get(k) for k in ["TS.1", "TS.2", "TS.3", "TS.4", "TS.5", "TS.6"]) else 1


if __name__ == "__main__":
    sys.exit(main())
