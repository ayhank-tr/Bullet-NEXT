#!/usr/bin/env python3
"""Bullet NEXT - Stage 1 gate run (what Rally FX needs from Bullet NEXT): tests T1S.1-T1S.7 and
benchmarks B1S.1-B1S.3.

Usage: run_s1_gate.py --build B --out DIR [--chrono BIN] [--rigs JSON] [--f1 DIR] [--f3c DIR] [--s0 DIR]
                      [--runs N]

  --build   native Lab build (Release): lab/bnl_run and lab/bnl_edgecheck
  --chrono  the Chrono Lab engine (tools/build_chrono_engine.sh): T1S.6, run from two folders and in
            two scene orders
  --rigs    the Chrono vehicle rigs' output (tools/run_chrono_rigs.sh): T1S.6, B1S.3
  --f1, --f3c, --s0  output folders of the earlier gates on this build (T1S.7)
  --runs    repeat count for T1S.5 (default 60)

Thresholds: docs/gates.json ("S1"). Writes DIR/s1_gate.json and every raw output. A test that cannot
run is reported as "not run here", never as passed.
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENES_DIR = os.path.join(ROOT, "lab", "scenes")
ENGINES = ["legacy", "next", "nextf", "nexts", "jolt"]
S1_SCENES = ["rally_tilt", "rally_body", "rally_big", "rally_crash"]
BASE = "94bcc92"  # main after Stage 0; its native_hashes.txt is the reference for T1S.7


def run(cmd, out=None, timeout=None, env=None):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env)
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


def scene_dir(scenes, edit=None):
    """A folder with copies of the scenes (edit(name, scene) may change or rename them)."""
    d = tempfile.mkdtemp(prefix="s1_")
    names = []
    for s in scenes:
        sc = json.load(open(os.path.join(SCENES_DIR, s + ".json")))
        if edit:
            sc = edit(s, sc)
        json.dump(sc, open(os.path.join(d, sc["name"] + ".json"), "w"))
        names.append(sc["name"])
    json.dump({"scenes": names}, open(os.path.join(d, "index.json"), "w"))
    return d


def metrics_in(bnl_run, d, engines, out, extra=()):
    m = os.path.join(d, "m.jsonl")
    _, text = run([bnl_run, "--engines", ",".join(engines), "--threads", "0", "--scenes", d, "--metrics", m] + list(extra), out=out)
    res = {}
    if os.path.exists(m):
        for line in open(m):
            r = json.loads(line)
            res[(r["scene"], r["engine"])] = r
    return res, runs(text)


def metrics(bnl_run, scenes, engines, out, edit=None, extra=()):
    d = scene_dir(scenes, edit)
    try:
        return metrics_in(bnl_run, d, engines, out, extra)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def track_diff(a, b):
    """Distance between the first solid body's positions in two runs, sample by sample."""
    return [math.dist(p, q) for p, q in zip(a["track"][0], b["track"][0])]


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


def nudge(dx):
    """Scene edit: every moving body moved dx along x (the chaos bound of T1S.2)."""
    def edit(name, sc):
        sc["name"] = f"{name}_nudged"
        for o in sc["objects"]:
            b = o.get("body")
            if b and b.get("type", "rigid") == "rigid":
                b["at"]["p"][0] += dx
        return sc
    return edit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--chrono")
    ap.add_argument("--rigs")
    ap.add_argument("--f1")
    ap.add_argument("--f3c")
    ap.add_argument("--s0")
    ap.add_argument("--runs", type=int, default=60)
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    build = os.path.abspath(a.build)
    bnl_run = os.path.join(build, "lab", "bnl_run")
    edgecheck = os.path.join(build, "lab", "bnl_edgecheck")
    g = json.load(open(os.path.join(ROOT, "docs", "gates.json")))["S1"]
    res, notes = {}, {}

    # T1S.1 edges from the heights: every triangle of the rolling and the flat terrain against the bake
    t1 = []
    for s in ("rally_stage", "rally_slide"):
        rc, text = run([edgecheck, "check", os.path.join(SCENES_DIR, s + ".json")], out=f"{out}/t1s1_{s}.txt")
        for line in text.splitlines():
            if line.startswith("{"):
                t1.append(json.loads(line))
    res["T1S.1"] = bool(t1) and all(r["ok"] and r["max_angle_diff_rad"] <= g["edge_angle_max_difference_rad"] for r in t1)
    notes["T1S.1"] = t1

    # T1S.2 car body on terrain: speed off a tilted plane; heightfield and mesh of the same ground
    m, _ = metrics(bnl_run, ["rally_tilt"], ENGINES, f"{out}/t1s2_tilt.txt")
    tilt = {e: {"off_plane_max_mps": round(m[("rally_tilt", e)]["up_max"], 4),
                "per_body": [[b, round(u, 4)] for b, u, _ in m[("rally_tilt", e)]["up_bodies"]]} for e in ENGINES if ("rally_tilt", e) in m}
    every = ["--track-every", "1"]
    m, _ = metrics(bnl_run, ["rally_body", "rally_body_mesh"], ["nextf", "nexts", "jolt"], f"{out}/t1s2_body.txt", extra=every)
    mn, _ = metrics(bnl_run, ["rally_body", "rally_body_mesh"], ["nextf", "nexts", "jolt"], f"{out}/t1s2_body_nudged.txt",
                    edit=nudge(g["nudge_m"]), extra=every)
    body = {}
    early = int(g["identical_first_s"] * 60)
    for e in ("nextf", "nexts", "jolt"):
        h, me = m[("rally_body", e)], m[("rally_body_mesh", e)]
        d = track_diff(h, me)
        spread = max(track_diff(h, mn[("rally_body_nudged", e)])[-1], track_diff(me, mn[("rally_body_mesh_nudged", e)])[-1])
        body[e] = {"first_s_max_mm": round(1000 * max(d[:early]), 4), "at_10_s_mm": round(1000 * d[-1], 2),
                   "nudge_spread_at_10_s_mm": round(1000 * spread, 2)}
    res["T1S.2"] = all(tilt[e]["off_plane_max_mps"] <= g["off_plane_max_mps"] for e in ("nextf", "nexts")) and \
        all(body[e]["first_s_max_mm"] <= 1000 * g["identical_max_m"] and body[e]["at_10_s_mm"] <= body[e]["nudge_spread_at_10_s_mm"]
            for e in ("nextf", "nexts"))
    notes["T1S.2"] = {"tilt": tilt, "heightfield_vs_mesh": body}

    # T1S.3 full-size terrain: edges ready at once on a 3 km field at 1 m, and the slide on it
    rc, text = run([edgecheck, "bench", "3001"], out=f"{out}/t1s3_bench.txt", timeout=3600)
    bench = [json.loads(l) for l in text.splitlines() if l.startswith("{")]
    worked = next((b for b in bench if b["edges"] == "worked out"), None)
    m, _ = metrics(bnl_run, ["rally_big"], ENGINES, f"{out}/t1s3_big.txt")
    big = {e: round(m[("rally_big", e)]["up_max"], 4) for e in ENGINES if ("rally_big", e) in m}
    res["T1S.3"] = worked is not None and worked["seconds"] <= g["big_edges_max_s"] and worked["extra_memory_mb"] <= g["big_edges_max_mb"] and \
        all(big[e] <= g["off_plane_max_mps"] for e in ("nextf", "nexts"))
    notes["T1S.3"] = {"bench_3001": bench, "off_plane_max_mps": big,
                      "pen_max_m": {e: round(m[("rally_big", e)]["pen_max"], 4) for e in ENGINES if ("rally_big", e) in m}}

    # T1S.4 crash report: reported impulses against each body's change of momentum; nothing goes through
    m, _ = metrics(bnl_run, ["rally_crash"], ENGINES, f"{out}/t1s4_crash.txt")
    crash = {}
    for e in ENGINES:
        r = m.get(("rally_crash", e))
        if not r:
            continue
        ends = [t[-1][1] for t in r["track"]]
        crash[e] = {"impulse_error_max": r.get("impulse_error_max"), "end_y": [round(y, 2) for y in ends],
                    "through": sum(1 for y in ends if y > 30.0), "pen_max_m": round(r["pen_max"], 3),
                    "ccd_impulse_ns": [round(math.hypot(*i["ccd"]), 1) for i in r.get("impulses", [])]}
    res["T1S.4"] = all(crash.get(e, {}).get("impulse_error_max") is not None and crash[e]["impulse_error_max"] <= g["impulse_max_error"]
                       and crash[e]["through"] == 0 for e in ("nextf", "nexts"))
    notes["T1S.4"] = crash

    # T1S.5 threads under load: N runs at 8 threads with two busy loops beside them
    d = scene_dir(["pile_1000", "rally_body", "rally_crash"])
    hogs = [subprocess.Popen([sys.executable, "-c", "while True: pass"]) for _ in range(2)]
    seen = {}
    try:
        for i in range(a.runs):
            _, text = run([bnl_run, "--engines", "nextf,nexts", "--threads", "8", "--scenes", d], out=f"{out}/t1s5_run.txt")
            for k, v in runs(text).items():
                seen.setdefault(f"{k[0]} {k[1]}", set()).add(v["hash"])
    finally:
        for h in hogs:
            h.kill()
        shutil.rmtree(d, ignore_errors=True)
    res["T1S.5"] = len(seen) == 6 and all(len(v) == 1 for v in seen.values())
    notes["T1S.5"] = {"runs": a.runs, "hashes_per_scene": {k: sorted(v) for k, v in seen.items()}}

    # T1S.6 the Chrono reference: the same hashes from two folders and in reverse scene order; the rigs
    t6 = {}
    if a.chrono:
        results = []
        for label, order in (("folder_a", 1), ("folder_b_reversed", -1)):
            d = tempfile.mkdtemp(prefix=f"s1_chrono_{label}_")
            idx = json.load(open(os.path.join(SCENES_DIR, "index.json")))["scenes"][::order]
            for s in idx:
                shutil.copy(os.path.join(SCENES_DIR, s + ".json"), d)
            json.dump({"scenes": idx}, open(os.path.join(d, "index.json"), "w"))
            _, text = run([a.chrono, "--scenes", d, "--out", d], out=f"{out}/t1s6_chrono_{label}.txt", timeout=7200)
            results.append({k[0]: v.get("hash") for k, v in runs(text).items()})
            shutil.rmtree(d, ignore_errors=True)
        t6["chrono_differs"] = sorted(s for s in results[0] if results[0][s] != results[1].get(s))
        t6["chrono_scenes"] = len(results[0])
    if a.rigs and os.path.exists(a.rigs):
        rigs = json.load(open(a.rigs))
        t6["rigs"] = {"tyres": {n: [{k: v for k, v in l.items() if k != "curves"} for l in t["loads"]] for n, t in rigs["tyres"].items()},
                      "driveline": rigs["driveline"]}
        notes["B1S.3"] = {n: t["microseconds_per_evaluation"] for n, t in rigs["tyres"].items()}
    res["T1S.6"] = (not t6.get("chrono_differs") and t6.get("chrono_scenes", 0) > 0 and "rigs" in t6) if (a.chrono and a.rigs) else None
    notes["T1S.6"] = t6

    # T1S.7 no regressions: the earlier gates on this build; legacy and Jolt hashes as after Stage 0
    t7 = {}
    ok = []
    if a.f1 and os.path.exists(os.path.join(a.f1, "F1_GATE_REPORT.md")):
        t7["f1_gate"] = "**F1 gate (cloud part): PASSED.**" in open(os.path.join(a.f1, "F1_GATE_REPORT.md")).read()
        ok.append(t7["f1_gate"])
    if a.f3c and os.path.exists(os.path.join(a.f3c, "f3c_gate.json")):
        r3 = json.load(open(os.path.join(a.f3c, "f3c_gate.json")))["results"]
        t7["f3c_gate"] = all(r3.get(k) for k in ("T3.1", "T3.2", "T3.3", "T3.4", "T3.5", "T3.6", "B3.1"))
        ok.append(t7["f3c_gate"])
    if a.s0 and os.path.exists(os.path.join(a.s0, "s0_gate.json")):
        r0 = json.load(open(os.path.join(a.s0, "s0_gate.json")))["results"]
        t7["s0_gate"] = all(r0.get(k) for k in ("TS.1", "TS.2", "TS.3", "TS.4", "TS.5", "TS.6"))
        ok.append(t7["s0_gate"])
    ref = reference()
    _, text = run([bnl_run, "--threads", "0"], out=f"{out}/t1s7_all.txt")
    now = runs(text)
    changed = {}
    for (s, e), h in sorted(ref.items()):
        if now.get((s, e), {}).get("hash") != h:
            changed.setdefault(e, []).append(s)
    t7["changed_vs_stage0"] = changed
    t7["all_ok"] = all(v.get("ok") for k, v in now.items() if v.get("ok") is not None)
    res["T1S.7"] = (len(ok) == 3 and all(ok) and not changed.get("legacy") and not changed.get("jolt") and t7["all_ok"]) if len(ok) == 3 else None
    notes["T1S.7"] = t7

    # B1S.1 edge setup at three sizes, baked and worked out
    b1 = []
    for n in (513, 1025):
        _, text = run([edgecheck, "bench", str(n)], out=f"{out}/b1s1_{n}.txt")
        b1 += [json.loads(l) for l in text.splitlines() if l.startswith("{")]
    notes["B1S.1"] = b1 + bench
    # B1S.2 car body step cost: the heightfield (edges worked out) against the mesh (baked map), best of 3
    best = {}
    for _ in range(3):
        mm, _ = metrics(bnl_run, ["rally_body", "rally_body_mesh"], ["nextf", "nexts"], f"{out}/b1s2.txt")
        for k, v in mm.items():
            best[k] = min(best.get(k, 1e9), v["ms"])
    notes["B1S.2"] = {e: {"heightfield_ms": round(best[("rally_body", e)], 1), "mesh_ms": round(best[("rally_body_mesh", e)], 1),
                          "ratio": round(best[("rally_body", e)] / best[("rally_body_mesh", e)], 3)} for e in ("nextf", "nexts")}
    res["B1S.1"] = res["B1S.2"] = res["B1S.3"] = True  # records

    json.dump({"results": res, "notes": notes}, open(f"{out}/s1_gate.json", "w"), indent=1)
    keys = ["T1S.1", "T1S.2", "T1S.3", "T1S.4", "T1S.5", "T1S.6", "T1S.7", "B1S.1", "B1S.2", "B1S.3"]
    for k in keys:
        print(k, {True: "PASS", False: "FAIL", None: "not run here"}[res.get(k)])
    return 0 if all(res.get(k) for k in keys[:7]) else 1


if __name__ == "__main__":
    sys.exit(main())
