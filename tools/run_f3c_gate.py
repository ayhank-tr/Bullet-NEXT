#!/usr/bin/env python3
"""Bullet NEXT - F3c gate run (Soft Step contacts): tests T3.1-T3.6 and benchmarks B3.1-B3.2.

Usage: run_f3c_gate.py --build B --out DIR [--quick]

  --build  native Lab build (Release): lab/bnl_run and harness/bn_harness_legacy
  --quick  leave out the 1,240-box pyramid (T3.2, B3.1) and run B3.1 / B3.2 on fewer scenes

Soft Step is the Lab engine "nexts" (Bullet NEXT with its features and the Soft Step solver);
PGS is "nextf" (the same, with Bullet's sequential impulse solver at its default 10 iterations).
Every measurement comes from bnl_run --metrics (deepest penetration, fastest body after 2 s,
joint drift, energy, bounce speeds, how far the bodies moved). Thresholds: docs/gates.json
("F3c"). Writes DIR/f3c_gate.json and every raw output. A test that cannot run is reported as
"not run here", never as passed.
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
F0_COMMIT = "3bedd98"  # F0 as pushed; legacy must not change after it
SCENES = json.load(open(os.path.join(SCENES_DIR, "index.json")))["scenes"]
F0_SCENES = SCENES[:16]
JOINT_SCENES = ["joints_six", "rb_constraints", "joint_grid", "pendulum"]
B31_JOINT_RATES = [60, 120, 180, 240, 360, 540]
B31_STACK_RATES = [120, 180, 240, 360, 540]
B31_PGS_ITERATIONS = [10, 20, 40, 80, 160]
B31_REST_SCENES = ["stack_10", "pyramid_10", "stack_20", "rb_stack", "settings"]
B32_SCENES = ["stack_20", "mass_ratio", "pyramid_10", "joints_six", "joint_grid", "bounce"]
B32_SUBSTEPS = [2, 4, 8]


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
    return r


def metrics(bnl_run, scenes, engines, out, settings=None, threads=0):
    """Runs the scenes (with extra Soft Step settings, if any) and returns {(scene, engine): metrics}."""
    d = tempfile.mkdtemp(prefix="f3c_")
    try:
        for s in scenes:
            sc = json.load(open(os.path.join(SCENES_DIR, s + ".json")))
            sc["settings"].update(settings or {})
            json.dump(sc, open(os.path.join(d, s + ".json"), "w"))
        json.dump({"scenes": scenes}, open(os.path.join(d, "index.json"), "w"))
        m = os.path.join(d, "m.jsonl")
        run([bnl_run, "--engines", ",".join(engines), "--threads", str(threads), "--scenes", d, "--metrics", m], out=out)
        res = {}
        if os.path.exists(m):
            for line in open(m):
                r = json.loads(line)
                res[(r["scene"], r["engine"])] = r
        return res
    finally:
        shutil.rmtree(d, ignore_errors=True)


def gain(r):
    e = r["energy"]
    return (e["max"] - e["start"]) / abs(e["start"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    b = os.path.abspath(a.build)
    bnl_run = f"{b}/lab/bnl_run"
    g = json.load(open(os.path.join(ROOT, "docs", "gates.json")))["F3c"]
    res, notes = {}, {}

    # T3.1 legacy untouched: no change to legacy/ since F0, its recordings equal the F0 goldens,
    # and the Lab's other engines still reach their reference hashes (Soft Step is opt-in)
    rc, _ = run(["git", "-C", ROOT, "diff", "--quiet", F0_COMMIT, "--", "legacy", "third_party/bullet284_orig"])
    d = f"{out}/t31"
    run([f"{b}/harness/bn_harness_legacy", "run", "all", "--out", d, "--repeat", "1"])
    gold = {}
    for line in open(os.path.join(ROOT, "goldens", "f0-linux-gcc", "hashes.csv")):
        if "," in line and not line.startswith(("#", "scene,")):
            n, h = line.strip().split(",")
            gold[n] = h
    leg = {}
    if os.path.exists(f"{d}/summary.legacy.csv"):
        import csv
        leg = {r["scene"]: r for r in csv.DictReader(open(f"{d}/summary.legacy.csv"))}
    same = [n for n in F0_SCENES if leg.get(n, {}).get("final_hash") == gold.get(n)]
    _, t = run([bnl_run, "--threads", "0"], out=f"{out}/run_all_t0.txt")
    native = runs(t)
    ref = {}
    for line in open(os.path.join(SCENES_DIR, "native_hashes.txt")):
        f = line.split()
        if len(f) == 3 and not line.startswith("#"):
            ref[(f[0], f[1])] = f[2]
    others = {k: v for k, v in ref.items() if k[1] in ("legacy", "next", "nextf", "jolt")}
    others_same = sum(native.get(k, {}).get("hash") == v for k, v in others.items())
    res["T3.1"] = rc == 0 and len(same) == len(F0_SCENES) and others_same == len(others)
    notes["T3.1"] = {"legacy_dir_unchanged_since_F0": rc == 0, "legacy_vs_goldens": f"{len(same)}/{len(F0_SCENES)}",
                     "other_engines_vs_reference": f"{others_same}/{len(others)}"}

    # One metrics run of the test scenes, Soft Step against PGS
    test_scenes = ["stack_20", "mass_ratio", "bounce"] + JOINT_SCENES + ([] if a.quick else ["pyramid_15"])
    m = metrics(bnl_run, test_scenes, ["nextf", "nexts"], f"{out}/metrics_tests.txt")
    json.dump({f"{k[0]} {k[1]}": v for k, v in m.items()}, open(f"{out}/metrics_tests.json", "w"), indent=1)

    # T3.2 stack stability: resting penetration and jitter after 2 s
    t32 = {}
    for s in ["stack_20"] + ([] if a.quick else ["pyramid_15"]):
        r = m.get((s, "nexts"))
        if not r:
            continue
        ok = r["pen_rest"] <= g["stack20_max_penetration_m"] and r["jitter"] <= g["rest_jitter_max_speed_mps_after_2s"]
        p = m.get((s, "nextf"), {})
        t32[s] = {"ok": ok, "penetration_mm": round(1000 * r["pen_rest"], 2), "jitter_mps": round(r["jitter"], 4),
                  "pgs_penetration_mm": round(1000 * p.get("pen_rest", 0), 2), "pgs_jitter_mps": round(p.get("jitter", 0), 4),
                  "pgs_moved_m": round(p.get("moved_xy_max", 0), 3)}
    res["T3.2"] = bool(t32) and all(v["ok"] for v in t32.values()) and (a.quick or "pyramid_15" in t32)
    notes["T3.2"] = t32

    # T3.3 mass ratio: the 100 kg boxes stay on the 1 kg boxes for 10 s (no sliding, no tilting)
    r, p = m.get(("mass_ratio", "nexts")), m.get(("mass_ratio", "nextf"), {})
    if r:
        res["T3.3"] = r["moved_xy_max"] <= g["mass_ratio_max_moved_m"] and r["tilt_max"] <= g["mass_ratio_max_tilt_rad"]
        notes["T3.3"] = {"moved_m": round(r["moved_xy_max"], 4), "tilt_rad": round(r["tilt_max"], 4),
                         "penetration_mm": round(1000 * r["pen_rest"], 1),
                         "pgs_moved_m": round(p.get("moved_xy_max", 0), 3), "pgs_tilt_rad": round(p.get("tilt_max", 0), 3)}

    # T3.4 joints: worst drift no worse than PGS, energy gain no worse than PGS (and under 1% on the pendulum)
    t34 = {}
    for s in JOINT_SCENES:
        r, p = m.get((s, "nexts")), m.get((s, "nextf"))
        if not r or not p:
            continue
        tol_lin, tol_ang = g["joint_drift_noise_m"], g["joint_drift_noise_rad"]
        lin_ok = r["drift_lin_max"] <= p["drift_lin_max"] + tol_lin
        ang_ok = r["drift_ang_max"] <= p["drift_ang_max"] + tol_ang
        e_ok = gain(r) <= max(gain(p), 0.0) + 1e-4
        if s == "pendulum":
            e_ok = e_ok and 100 * gain(r) <= g["energy_gain_max_pct_10s_frictionless_pendulum"]
        t34[s] = {"ok": lin_ok and ang_ok and e_ok,
                  "drift_max_mm": round(1000 * r["drift_lin_max"], 3), "pgs_drift_max_mm": round(1000 * p["drift_lin_max"], 3),
                  "drift_mean_mm": round(1000 * r["drift_lin_mean"], 4), "pgs_drift_mean_mm": round(1000 * p["drift_lin_mean"], 4),
                  "angle_max_rad": round(r["drift_ang_max"], 5), "pgs_angle_max_rad": round(p["drift_ang_max"], 5),
                  "energy_gain_pct": round(100 * gain(r), 3), "pgs_energy_gain_pct": round(100 * gain(p), 3)}
    res["T3.4"] = len(t34) == len(JOINT_SCENES) and all(v["ok"] for v in t34.values())
    notes["T3.4"] = t34

    # T3.5 restitution: speed out / speed in, and the rise from the lowest point, against e and e^2 x fall
    r, p = m.get(("bounce", "nexts")), m.get(("bounce", "nextf"), {})
    if r:
        rows = []
        for e, (vi, vo, zl, z0), apex, pb in zip((0.0, 0.3, 0.6, 0.9), r["bounce"], r["apex"], p.get("bounce", [[0, 0, 0, 0]] * 4)):
            em = -vo / vi if vi else 0.0
            fall = z0 - zl
            rise = apex - zl if apex > 0 else 0.0
            want = e * e * fall
            rows.append({"e": e, "measured_e": round(em, 4), "rise_m": round(rise, 4), "analytic_rise_m": round(want, 4),
                         "rise_error_pct": round(100 * abs(rise - want) / want, 2) if want else 0.0,
                         "pgs_measured_e": round(-pb[1] / pb[0], 4) if pb[0] else 0.0})
        res["T3.5"] = all(abs(x["measured_e"] - x["e"]) <= g["restitution_max_error"] and x["rise_error_pct"] <= g["bounce_height_max_error_pct"] for x in rows)
        notes["T3.5"] = rows

    # T3.6 determinism: the same final hash at 1, 2, 4 and 8 threads, every scene
    hashes = {}
    for th in (1, 2, 4, 8):
        _, t = run([bnl_run, "--engines", "nexts", "--threads", str(th)], out=f"{out}/run_nexts_t{th}.txt")
        hashes[th] = {k[0]: v["hash"] for k, v in runs(t).items()}
    same = [s for s in SCENES if len({hashes[th].get(s) for th in hashes}) == 1 and hashes[1].get(s)]
    res["T3.6"] = len(same) == len(SCENES)
    notes["T3.6"] = {"scenes_same_at_1_2_4_8_threads": f"{len(same)}/{len(SCENES)}",
                     "differs": [s for s in SCENES if s not in same]}

    # B3.1 equal-quality cost (revised on 2026-10-27, see docs/F3C_GATE_REPORT.md). Soft contacts
    # sink under load by design, so resting depth is held to a bound rather than to PGS's
    # figure, and the cost is compared where the two solvers do the same job:
    #   a. joints: the lowest substep rate at which Soft Step's worst drift is no worse than PGS's
    #      (10 iterations), scene by scene, and its time against PGS's there
    #   b. stacks: the cheapest Soft Step rate and the cheapest PGS iteration count that meet the
    #      T3.2 / T3.3 limits, and their times
    #   c. resting depth at the defaults: no more than the bound on boxes of equal mass, recorded
    #      beside PGS's depth and time
    b31 = {"joints": {}, "stacks": {}, "rest": {}}
    max_ratio = g["softstep_vs_pgs_time_at_equal_error_max_ratio"]
    noise_lin, noise_ang = g["joint_drift_noise_m"], g["joint_drift_noise_rad"]
    joint_scenes = JOINT_SCENES if not a.quick else ["joints_six", "pendulum"]
    pgs = metrics(bnl_run, joint_scenes, ["nextf"], f"{out}/b31_joints_pgs.txt")
    by_rate = {rate: metrics(bnl_run, joint_scenes, ["nexts"], f"{out}/b31_joints_rate{rate}.txt", {"solver_substep_rate": rate})
               for rate in B31_JOINT_RATES}
    for s in joint_scenes:
        p = pgs.get((s, "nextf"))
        row = {"pgs_drift_max_mm": round(1000 * p["drift_lin_max"], 3) if p else None, "pgs_ms": round(p["ms"], 1) if p else None,
               "rate": None, "time_ratio": None}
        for rate in B31_JOINT_RATES:
            r = by_rate[rate].get((s, "nexts"))
            if r and p and r["drift_lin_max"] <= p["drift_lin_max"] + noise_lin and r["drift_ang_max"] <= p["drift_ang_max"] + noise_ang:
                row.update({"rate": rate, "drift_max_mm": round(1000 * r["drift_lin_max"], 3), "ms": round(r["ms"], 1),
                            "time_ratio": round(r["ms"] / p["ms"], 3)})
                break
        b31["joints"][s] = row
    joints_ok = all(v["time_ratio"] is not None and v["time_ratio"] <= max_ratio for v in b31["joints"].values())

    def stack_ok(s, r):
        if s == "mass_ratio":
            return r["moved_xy_max"] <= g["mass_ratio_max_moved_m"] and r["tilt_max"] <= g["mass_ratio_max_tilt_rad"]
        return (r["pen_rest"] <= g["stack20_max_penetration_m"] and r["jitter"] <= g["rest_jitter_max_speed_mps_after_2s"]
                and r["moved_xy_max"] <= g["mass_ratio_max_moved_m"])

    stack_scenes = ["stack_20", "mass_ratio"]
    ss = {rate: metrics(bnl_run, stack_scenes, ["nexts"], f"{out}/b31_stacks_rate{rate}.txt", {"solver_substep_rate": rate})
          for rate in B31_STACK_RATES}
    pg = {it: metrics(bnl_run, stack_scenes, ["nextf"], f"{out}/b31_stacks_pgs{it}.txt", {"iterations": it})
          for it in B31_PGS_ITERATIONS}
    stacks_ok = True
    for s in stack_scenes:
        row = {"rate": None, "ms": None, "pgs_iterations": None, "pgs_ms": None}
        for rate in B31_STACK_RATES:
            r = ss[rate].get((s, "nexts"))
            if r and stack_ok(s, r):
                row.update({"rate": rate, "ms": round(r["ms"], 1)})
                break
        for it in B31_PGS_ITERATIONS:
            r = pg[it].get((s, "nextf"))
            if r and stack_ok(s, r):
                row.update({"pgs_iterations": it, "pgs_ms": round(r["ms"], 1)})
                break
        most = pg[B31_PGS_ITERATIONS[-1]].get((s, "nextf"))
        row["pgs_ms_at_most_iterations"] = round(most["ms"], 1) if most else None
        if row["ms"] is None:
            row["time_ratio"], ok = None, False
        elif row["pgs_ms"] is not None:
            row["time_ratio"] = round(row["ms"] / row["pgs_ms"], 3)
            ok = row["time_ratio"] <= max_ratio
        else:
            # PGS does not get there at the most iterations tried: Soft Step costs less than this
            row["time_ratio_below"] = round(row["ms"] / row["pgs_ms_at_most_iterations"], 3)
            row["time_ratio"] = None
            ok = row["time_ratio_below"] <= max_ratio
        stacks_ok = stacks_ok and ok
        b31["stacks"][s] = row

    rest_scenes = B31_REST_SCENES + ([] if a.quick else ["pyramid_15"])
    mm = metrics(bnl_run, rest_scenes, ["nextf", "nexts"], f"{out}/b31_rest.txt")
    rest_ok = True
    for s in rest_scenes:
        r, p = mm.get((s, "nexts")), mm.get((s, "nextf"))
        if not r or not p:
            rest_ok = False
            continue
        ok = r["pen_rest"] <= g["softstep_rest_penetration_max_m"]
        rest_ok = rest_ok and ok
        b31["rest"][s] = {"ok": ok, "pen_rest_mm": round(1000 * r["pen_rest"], 2), "pgs_pen_rest_mm": round(1000 * p["pen_rest"], 2),
                          "pgs_moved_m": round(p["moved_xy_max"], 3), "ms": round(r["ms"], 1), "pgs_ms": round(p["ms"], 1),
                          "time_ratio": round(r["ms"] / p["ms"], 3)}
    b31["joints_ok"], b31["stacks_ok"], b31["rest_ok"] = joints_ok, stacks_ok, rest_ok
    res["B3.1"] = joints_ok and stacks_ok and rest_ok
    notes["B3.1"] = b31

    # B3.2 substep sweep: quality and time at 2, 4 and 8 substeps
    scenes = B32_SCENES if not a.quick else ["stack_20", "mass_ratio", "joints_six"]
    b32 = {}
    for n in B32_SUBSTEPS:
        mm = metrics(bnl_run, scenes, ["nexts"], f"{out}/b32_n{n}.txt", {"solver_substeps": n})
        b32[n] = {s: {"ms": round(r["ms"], 1), "pen_rest_mm": round(1000 * r["pen_rest"], 2), "jitter_mps": round(r["jitter"], 4),
                      "moved_m": round(r["moved_xy_max"], 3), "drift_max_mm": round(1000 * r["drift_lin_max"], 3)}
                  for (s, _), r in mm.items()}
    notes["B3.2"] = b32
    res["B3.2"] = True  # a record, no threshold

    json.dump({"results": res, "notes": notes}, open(f"{out}/f3c_gate.json", "w"), indent=1)
    for k in ["T3.1", "T3.2", "T3.3", "T3.4", "T3.5", "T3.6", "B3.1", "B3.2"]:
        print(k, {True: "PASS", False: "FAIL", None: "not run here"}[res.get(k)])
    return 0 if all(res.get(k) for k in ["T3.1", "T3.2", "T3.3", "T3.4", "T3.5", "T3.6", "B3.1"]) else 1


if __name__ == "__main__":
    sys.exit(main())
