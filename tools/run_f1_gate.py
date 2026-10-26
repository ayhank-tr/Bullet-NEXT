#!/usr/bin/env python3
"""Bullet NEXT - F1 gate run: tests T1.1-T1.7 and benchmarks B1.1-B1.3.

Usage: run_f1_gate.py <build_dir> <out_dir> [<chrono_results.json>]
Writes <out_dir>/F1_GATE_REPORT.md plus all recordings and metrics.
Thresholds come from docs/gates.json. T1.6 (Python API in the engine) and the 8-thread part of
B1.1 run on the Windows PC at project end.
"""
import csv
import json
import os
import platform
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import bnr  # noqa: E402

F0_COMMIT = "3bedd98"           # F0 as pushed; the legacy backend must not change after it
THREADS = (1, 2, 4, 8)
VARIANTS = range(5)             # gravity + 0.0005*k: the chaos band of each scene
NOT_QUALITY = {"tunnel_100", "projectile"}  # legacy passes through the walls there; see T1.4
TOL_REL, TOL_ABS_M = 0.10, 0.001


def run(cmd, env=None):
    e = dict(os.environ)
    e.update(env or {})
    p = subprocess.run(cmd, capture_output=True, text=True, env=e)
    return p.returncode, p.stdout + p.stderr


def scene_names(harness):
    _, out = run([harness, "list"])
    return [l.split()[0] for l in out.splitlines()[1:] if l.strip()]


def summary(path):
    with open(path) as f:
        return {r["scene"]: r for r in csv.DictReader(f)}


def metrics(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    pen = [float(r["max_penetration_m"]) for r in rows]
    energy = [float(r["energy_j"]) for r in rows]
    return {"mean_pen": sum(pen) / len(pen), "max_pen": max(pen),
            "drift": max(float(r["max_joint_drift_m"]) for r in rows),
            "e0": energy[0], "gain": max(energy) - energy[0]}


def avg(ms, key):
    return sum(m[key] for m in ms) / len(ms)


def main():
    build, out = sys.argv[1], sys.argv[2]
    chrono = sys.argv[3] if len(sys.argv) > 3 else None
    os.makedirs(out, exist_ok=True)
    gates = json.load(open(os.path.join(ROOT, "docs", "gates.json")))["F1"]
    h = {b: os.path.join(build, "harness", f"bn_harness_{b}") for b in ("orig", "legacy", "nextf")}
    cmp_tool = os.path.join(build, "harness", "bn_compare")
    golden = os.path.join(ROOT, "goldens", "f0-linux-gcc")
    sc = scene_names(h["legacy"])
    res, notes = {}, {}
    rep = ["# F1 gate report", "",
           f"Platform: {platform.system()} {platform.machine()}, {os.cpu_count()} CPUs, build `{build}`.", ""]

    # T1.1 legacy untouched: no change to legacy/ since F0; legacy == unrenamed original == goldens.
    rc, _ = run(["git", "-C", ROOT, "diff", "--quiet", F0_COMMIT, "--", "legacy", "third_party/bullet284_orig"])
    unchanged = rc == 0
    d = os.path.join(out, "t11")
    for b in ("orig", "legacy"):
        run([h[b], "run", "all", "--out", d, "--repeat", "1"])
    gold = {}
    for line in open(os.path.join(golden, "hashes.csv")):
        if line.startswith("#") or line.startswith("scene,"):
            continue
        name, hsh = line.strip().split(",")
        gold[name] = hsh
    leg = summary(f"{d}/summary.legacy.csv")
    rows = []
    for n in sc:
        same_orig = run([cmp_tool, f"{d}/{n}.orig.bnr", f"{d}/{n}.legacy.bnr"])[0] == 0
        same_gold = gold.get(n) == leg[n]["final_hash"]
        rows.append((n, same_orig, same_gold))
    res["T1.1"] = unchanged and all(a and b for _, a, b in rows)
    notes["T1.1"] = rows

    # T1.2 thread determinism: same final hash at 1, 2, 4 and 8 threads, stable over 2 runs.
    hashes = {}
    for t in THREADS:
        dd = os.path.join(out, f"t12_{t}")
        run([h["nextf"], "run", "all", "--out", dd, "--repeat", "2"], {"BN_THREADS": str(t)})
        hashes[t] = summary(f"{dd}/summary.nextf.csv")
    t12 = []
    for n in sc:
        hs = {hashes[t][n]["final_hash"] for t in THREADS}
        stable = all(hashes[t][n]["stable_over_2_runs"] == "yes" for t in THREADS)
        t12.append((n, hashes[1][n]["final_hash"], len(hs) == 1 and stable))
    res["T1.2"] = all(ok for _, _, ok in t12)
    notes["T1.2"] = t12
    t1 = os.path.join(out, "t12_1")  # the threaded world on one thread is what BGE NEXT runs

    # T1.3 ghost bumps on the mesh-slide scene.
    bump = bnr.ghost_bump(f"{t1}/mesh_slide.nextf.bnr")
    bump_legacy = bnr.ghost_bump(f"{d}/mesh_slide.legacy.bnr")
    res["T1.3"] = bump <= gates["ghost_bump_max_vertical_speed_mps"]

    # T1.4 tunnelling: 100 spheres at 300 m/s against a 5 cm wall.
    through, total = bnr.tunnelled(f"{t1}/tunnel_100.nextf.bnr")
    through_legacy, _ = bnr.tunnelled(f"{d}/tunnel_100.legacy.bnr")
    res["T1.4"] = through <= gates["tunnelling_max_count_of_100_at_300mps_through_5cm_wall"]

    # T1.5 quality vs F0: averaged over 5 gravity variants per scene (chaotic scenes vary between
    # runs that differ by 0.005 %), next may not exceed legacy by more than 10 % + 1 mm.
    q = []
    for n in sc:
        if n in NOT_QUALITY:
            continue
        m = {"legacy": [], "nextf": []}
        for k in VARIANTS:
            dd = os.path.join(out, f"t15_{k}")
            run([h["legacy"], "run", n, "--out", dd, "--repeat", "1", "--variant", str(k)])
            run([h["nextf"], "run", n, "--out", dd, "--repeat", "1", "--variant", str(k)], {"BN_THREADS": "1"})
            for b in m:
                m[b].append(metrics(f"{dd}/{n}.{b}.metrics.csv"))
        L, N = m["legacy"], m["nextf"]
        ok_pen = avg(N, "mean_pen") <= avg(L, "mean_pen") * (1 + TOL_REL) + TOL_ABS_M
        ok_drift = avg(N, "drift") <= avg(L, "drift") * (1 + TOL_REL) + TOL_ABS_M
        e_allow = max(avg(L, "gain"), 0.0) + 0.01 * abs(avg(L, "e0")) + 0.5
        ok_energy = avg(N, "gain") <= e_allow
        q.append((n, L, N, ok_pen, ok_drift, ok_energy))
    res["T1.5"] = all(a and b and c for *_, a, b, c in q)

    # T1.7 Chrono build: results of tools/run_chrono_check.sh (stock and patched Chrono).
    t17 = None
    if chrono and os.path.exists(chrono):
        t17 = json.load(open(chrono))
        # every test gives the stock result (some Chrono tests fail on stock Chrono as well)
        res["T1.7"] = t17["patched_builds"] and all(t["patched"] == t["stock"] and t["patched_features"] == t["stock"]
                                                    for t in t17["tests"])
    else:
        res["T1.7"] = False

    # B1.1 thread scaling (best of 3), B1.2 feature cost.
    b11 = {}
    for n in ("pile_1000", "joint_grid"):
        for t in THREADS:
            dd = os.path.join(out, "b11")
            run([h["nextf"], "run", n, "--out", dd, "--repeat", "3"], {"BN_THREADS": str(t)})
            b11[(n, t)] = float(summary(f"{dd}/summary.nextf.csv")[n]["best_ms"])
    feats = [(4, "Hull clipping"), (8, "Active edges"), (16, "CCD and speculative contacts"), (1, "World config (SAT, ordered pairs, reduction)")]
    b12 = []
    for n in ("shape_zoo", "mesh_slide", "pile_1000", "tunnel_100"):
        dd = os.path.join(out, "b12")
        run([h["nextf"], "run", n, "--out", dd, "--repeat", "3"], {"BN_THREADS": "1"})
        full = float(summary(f"{dd}/summary.nextf.csv")[n]["best_ms"])
        row = [n, full]
        for bit, _ in feats:
            run([h["nextf"], "run", n, "--out", dd, "--repeat", "3"], {"BN_THREADS": "1", "BN_FEATURES": str(31 - bit)})
            row.append(full - float(summary(f"{dd}/summary.nextf.csv")[n]["best_ms"]))
        b12.append(row)

    # ---- report ----
    names = {"T1.1": "Legacy untouched", "T1.2": "Thread determinism", "T1.3": "Ghost bumps",
             "T1.4": "Tunnelling", "T1.5": "Stacking and joint quality vs F0", "T1.7": "Chrono build"}
    rep += ["## Tests", "", "| Test | Result | Measured |", "| --- | --- | --- |"]
    meas = {
        "T1.1": f"legacy/ unchanged since F0: {'yes' if unchanged else 'NO'}; "
                f"{sum(1 for _, a, b in rows if a and b)} of {len(rows)} scenes bit-identical to original and goldens",
        "T1.2": f"{sum(1 for *_, ok in t12 if ok)} of {len(t12)} scenes identical at 1, 2, 4, 8 threads",
        "T1.3": f"{bump:.3f} m/s (limit {gates['ghost_bump_max_vertical_speed_mps']}; legacy {bump_legacy:.2f})",
        "T1.4": f"{through} of {total} through (legacy {through_legacy} of {total})",
        "T1.5": f"{sum(1 for *_, a, b, c in q if a and b and c)} of {len(q)} scenes within 10 % + 1 mm",
        "T1.7": ("Chrono " + t17["chrono_commit"][:8] + f": {sum(1 for t in t17['tests'] if t['patched'] == t['stock'] and t['patched_features'] == t['stock'])} of "
                 f"{len(t17['tests'])} tests give the stock result patched and with the features on "
                 f"({sum(1 for t in t17['tests'] if t['stock'] == 'pass')} pass on stock Chrono)") if t17 else "no Chrono results given",
    }
    for k in ("T1.1", "T1.2", "T1.3", "T1.4", "T1.5"):
        rep.append(f"| {k} {names[k]} | {'PASS' if res[k] else 'FAIL'} | {meas[k]} |")
    rep.append("| T1.6 Python API | Pending: engine on the Windows PC at project end | |")
    rep.append(f"| T1.7 {names['T1.7']} | {'PASS' if res['T1.7'] else 'FAIL'} | {meas['T1.7']} |")

    rep += ["", "### T1.1 and T1.2 per scene", "", "| Scene | Legacy = original | Legacy = golden | Next hash (1-8 threads) |",
            "| --- | --- | --- | --- |"]
    det = {n: (hsh, ok) for n, hsh, ok in t12}
    for n, a, b in rows:
        hsh, ok = det[n]
        rep.append(f"| {n} | {'yes' if a else 'NO'} | {'yes' if b else 'NO'} | {hsh}{'' if ok else ' DIFFERS'} |")

    rep += ["", "### T1.5 and B1.3 quality (mean of 5 gravity variants)", "",
            "| Scene | Mean penetration mm (legacy / next) | Max penetration mm (legacy / next) | Joint drift mm (legacy / next) | Energy gain J (legacy / next) | Result |",
            "| --- | --- | --- | --- | --- | --- |"]
    for n, L, N, a, b, c in q:
        rep.append(f"| {n} | {avg(L, 'mean_pen') * 1000:.2f} / {avg(N, 'mean_pen') * 1000:.2f} | "
                   f"{avg(L, 'max_pen') * 1000:.1f} / {avg(N, 'max_pen') * 1000:.1f} | "
                   f"{avg(L, 'drift') * 1000:.1f} / {avg(N, 'drift') * 1000:.1f} | "
                   f"{avg(L, 'gain'):.2f} / {avg(N, 'gain'):.2f} | {'pass' if a and b and c else 'FAIL'} |")

    if t17:
        rep += ["", "### T1.7 Chrono collision tests", "", f"Chrono commit {t17['chrono_commit']}.", "",
                "| Test | Stock | Patched | Patched, features on | Same output as stock (patched / features) |",
                "| --- | --- | --- | --- | --- |"]
        for t in t17["tests"]:
            rep.append(f"| {t['name']} | {t['stock']} | {t['patched']} | {t['patched_features']} | {t['same_output']} |")
        if "mesh_slide" in t17:
            ms = t17["mesh_slide"]
            rep += ["", "Chrono's tests do not slide anything over a mesh, so the features leave their output unchanged. "
                    "`integration/chrono/bn_mesh_slide.cpp` checks them directly in Chrono (box and sphere at 8 m/s "
                    "over a flat mesh, largest upward speed):", "",
                    "| Features | Box bump m/s | Sphere bump m/s | Box travel end x (m) |", "| --- | --- | --- | --- |",
                    f"| off | {ms['off']['box_bump_mps']:.3f} | {ms['off']['ball_bump_mps']:.3f} | {ms['off']['box_end_x_m']:.1f} |",
                    f"| on | {ms['on']['box_bump_mps']:.3f} | {ms['on']['ball_bump_mps']:.3f} | {ms['on']['box_end_x_m']:.1f} |"]

    rep += ["", "## Benchmarks", "", "### B1.1 thread scaling (whole run, best of 3 ms)", "",
            "| Scene | 1 thread | 2 | 4 | 8 | Speed-up at 8 |", "| --- | --- | --- | --- | --- | --- |"]
    for n in ("pile_1000", "joint_grid"):
        v = [b11[(n, t)] for t in THREADS]
        rep.append(f"| {n} | " + " | ".join(f"{x:.0f}" for x in v) + f" | {v[0] / v[-1]:.2f}x |")
    rep += ["", f"Threshold: {gates['thread_scaling_min_speedup_8_threads_1000_body_pile']}x at 8 threads on the "
            f"1,000-body pile. This machine has {os.cpu_count()} cores, so the threshold is checked on the PC.",
            "", "### B1.2 feature cost (ms per whole run, single thread; positive = the feature costs time)", "",
            "| Scene | All features ms | " + " | ".join(f for _, f in feats) + " |",
            "| --- | --- | " + " | ".join("---" for _ in feats) + " |"]
    for row in b12:
        rep.append(f"| {row[0]} | {row[1]:.1f} | " + " | ".join(f"{x:+.1f}" for x in row[2:]) + " |")

    gate = all(res.values())
    rep += ["", f"**F1 gate (cloud part): {'PASSED' if gate else 'NOT PASSED'}.** "
            "T1.6, the B1.1 threshold and the MSVC build remain for the Windows PC."]
    with open(os.path.join(out, "F1_GATE_REPORT.md"), "w") as f:
        f.write("\n".join(rep) + "\n")
    print("\n".join(rep))
    return 0 if gate else 1


if __name__ == "__main__":
    sys.exit(main())
