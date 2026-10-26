#!/usr/bin/env python3
"""Bullet NEXT - F0 gate run: tests T0.1-T0.7 and benchmarks B0.1-B0.2.

Usage: run_f0_gate.py <build_dir> <out_dir>
Writes <out_dir>/F0_GATE_REPORT.md plus all recordings and metrics.
T0.8 (BGE-in-the-loop) and the MSVC build run on the Windows PC at project end.
"""
import csv
import os
import platform
import subprocess
import sys


def run(cmd, **kw):
    p = subprocess.run(cmd, capture_output=True, text=True, **kw)
    return p.returncode, p.stdout + p.stderr


def scenes(harness):
    _, out = run([harness, "list"])
    rows = []
    for line in out.splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 5:
            rows.append((parts[0], parts[4]))
    return rows


def metrics_summary(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    pen = max(float(r["max_penetration_m"]) for r in rows)
    drift = max(float(r["max_joint_drift_m"]) for r in rows)
    e0, e1 = float(rows[0]["energy_j"]), float(rows[-1]["energy_j"])
    return pen, drift, e0, e1


def main():
    build, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    h = {b: os.path.join(build, "harness", f"bn_harness_{b}") for b in ("orig", "legacy", "next")}
    cmp_tool = os.path.join(build, "harness", "bn_compare")
    report = ["# F0 gate report", "", f"Platform: {platform.system()} {platform.machine()}, "
              f"{os.cpu_count()} CPUs, compiler from build `{build}`.", ""]
    results = {}

    # T0.1 build: all executables exist (the build ran before this script).
    exes = list(h.values()) + [cmp_tool, os.path.join(build, "harness", "bn_symcheck")]
    results["T0.1"] = all(os.path.exists(e) for e in exes)

    # T0.2 symbol isolation: one program with both libraries.
    rc, txt = run([os.path.join(build, "harness", "bn_symcheck")])
    results["T0.2"] = rc == 0
    t02 = txt.strip()

    # Runs: repeat 3 for determinism (T0.4); best-of-5 timing for B0.1.
    summ = {}
    for b in ("orig", "legacy", "next"):
        rc, txt = run([h[b], "run", "all", "--out", out, "--repeat", "5"])
        with open(os.path.join(out, f"summary.{b}.csv")) as f:
            summ[b] = {r["scene"]: r for r in csv.DictReader(f)}
    sc = scenes(h["legacy"])

    # T0.3 rename fidelity: legacy == orig, exact.
    t03 = []
    for name, _ in sc:
        rc, txt = run([cmp_tool, f"{out}/{name}.orig.bnr", f"{out}/{name}.legacy.bnr"])
        t03.append((name, rc == 0))
    results["T0.3"] = all(ok for _, ok in t03)

    # T0.4 determinism: hash stable over 5 runs for orig, legacy and next.
    t04 = [(n, all(summ[b][n]["stable_over_5_runs"] == "yes" for b in summ)) for n, _ in sc]
    results["T0.4"] = all(ok for _, ok in t04)

    # T0.5 compare self-test.
    rc, t05 = run([cmp_tool, "--selftest", f"{out}/{sc[0][0]}.legacy.bnr"])
    results["T0.5"] = rc == 0

    # T0.6 coverage: union of covered points == 1..12.
    covered = set()
    for _, c in sc:
        covered |= {int(x) for x in c.split(",")}
    results["T0.6"] = covered == set(range(1, 13))

    # T0.7 next baseline: runs every scene; record first divergence vs orig.
    t07 = []
    for name, _ in sc:
        rc, txt = run([cmp_tool, f"{out}/{name}.orig.bnr", f"{out}/{name}.next.bnr"])
        first = [l for l in txt.splitlines() if l.startswith("first divergence")]
        t07.append((name, "identical" if rc == 0 else first[0].replace("first divergence: ", "") if first else "differs"))
    results["T0.7"] = all(n in summ["next"] for n, _ in sc)

    report += ["## Tests", "", "| Test | Result |", "| --- | --- |"]
    names = {"T0.1": "Build (GCC, cloud)", "T0.2": "Symbol isolation", "T0.3": "Rename fidelity",
             "T0.4": "Determinism (5 of 5 runs)", "T0.5": "Compare self-test", "T0.6": "Coverage (12 points)",
             "T0.7": "Next baseline runs every scene"}
    for k in sorted(names):
        report.append(f"| {k} {names[k]} | {'PASS' if results[k] else 'FAIL'} |")
    report += ["| T0.8 BGE-in-the-loop | Pending: Windows PC at project end |", "",
               f"T0.2 output: `{t02}`", "", f"T0.5 output: `{t05.strip()}`", "",
               "### T0.3 rename fidelity and T0.7 next baseline (per scene)", "",
               "| Scene | Legacy = original | Next vs original (first divergence) |", "| --- | --- | --- |"]
    for (name, ok), (_, d) in zip(t03, t07):
        report.append(f"| {name} | {'bit-identical' if ok else 'DIFFERS'} | {d} |")

    # B0.1 step time, B0.2 quality baseline.
    report += ["", "## Benchmarks", "", "### B0.1 step time (best of 5 runs, single thread, whole scene)", "",
               "| Scene | Frames | Legacy ms | Next ms | Next / legacy |", "| --- | --- | --- | --- | --- |"]
    for name, _ in sc:
        lg, nx = float(summ["legacy"][name]["best_ms"]), float(summ["next"][name]["best_ms"])
        report.append(f"| {name} | {summ['legacy'][name]['frames']} | {lg:.2f} | {nx:.2f} | {nx / lg if lg else 0:.2f} |")
    report += ["", "### B0.2 quality baseline", "",
               "| Scene | Backend | Max penetration (mm) | Max joint drift (mm) | Energy start (J) | Energy end (J) |",
               "| --- | --- | --- | --- | --- | --- |"]
    for name, _ in sc:
        for b in ("legacy", "next"):
            pen, drift, e0, e1 = metrics_summary(f"{out}/{name}.{b}.metrics.csv")
            report.append(f"| {name} | {b} | {pen * 1000:.2f} | {drift * 1000:.2f} | {e0:.1f} | {e1:.1f} |")
    gate = all(results.values())
    report += ["", f"**F0 gate (cloud part): {'PASSED' if gate else 'NOT PASSED'}.** "
               "T0.8 and the MSVC build remain for the Windows PC."]
    with open(os.path.join(out, "F0_GATE_REPORT.md"), "w") as f:
        f.write("\n".join(report) + "\n")
    print("\n".join(report))
    return 0 if gate else 1


if __name__ == "__main__":
    sys.exit(main())
