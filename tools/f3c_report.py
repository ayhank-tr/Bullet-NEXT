#!/usr/bin/env python3
"""Bullet NEXT - writes the tables of docs/F3C_GATE_REPORT.md from a gate run (F3c).

Usage: f3c_report.py <gate_dir>/f3c_gate.json > tables.md
"""
import json
import sys

NAMES = {
    "T3.1": "Legacy untouched", "T3.2": "Stack stability", "T3.3": "Mass ratio", "T3.4": "Joint behaviour",
    "T3.5": "Restitution", "T3.6": "Determinism", "B3.1": "Equal-quality cost", "B3.2": "Substep sweep",
}


def verdict(r):
    return {True: "PASS", False: "FAIL", None: "not run"}[r]


def main():
    g = json.load(open(sys.argv[1]))
    res, n = g["results"], g["notes"]
    out = ["| Test | Result | Measured |", "| --- | --- | --- |"]
    t31 = n.get("T3.1", {})
    out.append(f"| T3.1 {NAMES['T3.1']} | {verdict(res.get('T3.1'))} | legacy/ unchanged since F0: {t31.get('legacy_dir_unchanged_since_F0')}; "
               f"legacy recordings = F0 goldens: {t31.get('legacy_vs_goldens')}; other engines = reference: {t31.get('other_engines_vs_reference')} |")
    t32 = n.get("T3.2", {})
    out.append(f"| T3.2 {NAMES['T3.2']} | {verdict(res.get('T3.2'))} | " + "; ".join(
        f"{s}: {v['penetration_mm']} mm, {v['jitter_mps']} m/s (PGS {v['pgs_penetration_mm']} mm, {v['pgs_jitter_mps']} m/s, moved {v['pgs_moved_m']} m)"
        for s, v in t32.items()) + " |")
    t33 = n.get("T3.3", {})
    if t33:
        out.append(f"| T3.3 {NAMES['T3.3']} | {verdict(res.get('T3.3'))} | moved {t33['moved_m']} m, tilt {t33['tilt_rad']} rad, "
                   f"penetration {t33['penetration_mm']} mm (PGS moved {t33['pgs_moved_m']} m, tilt {t33['pgs_tilt_rad']} rad) |")
    t34 = n.get("T3.4", {})
    out.append(f"| T3.4 {NAMES['T3.4']} | {verdict(res.get('T3.4'))} | " + "; ".join(
        f"{s}: drift {v['drift_max_mm']} mm (PGS {v['pgs_drift_max_mm']}), angle {v['angle_max_rad']} rad (PGS {v['pgs_angle_max_rad']}), "
        f"energy gain {v['energy_gain_pct']}% (PGS {v['pgs_energy_gain_pct']}%)" for s, v in t34.items()) + " |")
    t35 = n.get("T3.5", [])
    out.append(f"| T3.5 {NAMES['T3.5']} | {verdict(res.get('T3.5'))} | " + "; ".join(
        f"e {r['e']}: measured {r['measured_e']} (PGS {r['pgs_measured_e']}), rise {r['rise_m']} m vs {r['analytic_rise_m']} m" for r in t35) + " |")
    t36 = n.get("T3.6", {})
    out.append(f"| T3.6 {NAMES['T3.6']} | {verdict(res.get('T3.6'))} | {t36.get('scenes_same_at_1_2_4_8_threads')} scenes give the same hash at 1, 2, 4 and 8 threads |")
    b31 = n.get("B3.1", {})
    out.append(f"| B3.1 {NAMES['B3.1']} | {verdict(res.get('B3.1'))} | joints: {verdict(b31.get('joints_ok'))}, "
               f"stacks: {verdict(b31.get('stacks_ok'))}, resting depth: {verdict(b31.get('rest_ok'))} (tables below) |")
    out.append(f"| B3.2 {NAMES['B3.2']} | record | table below |")
    out += ["", "### B3.1a Joints: the lowest substep rate at which Soft Step drifts no more than PGS (10 iterations)", "",
            "| Scene | Substeps per second | Worst drift (mm), Soft Step / PGS | Time (ms), Soft Step / PGS | Time / PGS time |",
            "| --- | --- | --- | --- | --- |"]
    for s, r in b31.get("joints", {}).items():
        if r.get("rate"):
            out.append(f"| {s} | {r['rate']} | {r['drift_max_mm']} / {r['pgs_drift_max_mm']} | {r['ms']} / {r['pgs_ms']} | {r['time_ratio']}x |")
        else:
            out.append(f"| {s} | none tried | - / {r['pgs_drift_max_mm']} | - | - |")
    out += ["", "### B3.1b Stacks: the cheapest setting of each solver that keeps the stack standing still (T3.2 / T3.3 limits)", "",
            "| Scene | Soft Step: substeps per second, time (ms) | PGS: iterations, time (ms) | Time / PGS time |",
            "| --- | --- | --- | --- |"]
    for s, r in b31.get("stacks", {}).items():
        sst = f"{r['rate']}, {r['ms']}" if r.get("rate") else "none tried"
        if r.get("pgs_iterations"):
            pt, ratio = f"{r['pgs_iterations']}, {r['pgs_ms']}", f"{r['time_ratio']}x"
        else:
            pt = f"not reached at 160 ({r['pgs_ms_at_most_iterations']} ms at 160)"
            ratio = f"below {r['time_ratio_below']}x" if r.get("time_ratio_below") is not None else "-"
        out.append(f"| {s} | {sst} | {pt} | {ratio} |")
    out += ["", "### B3.1c Resting depth at the defaults (bound 2.5 mm), with PGS (10 iterations) beside it", "",
            "| Scene | Resting penetration (mm), Soft Step / PGS | PGS: how far a box moved (m) | Time (ms), Soft Step / PGS | Time / PGS time |",
            "| --- | --- | --- | --- | --- |"]
    for s, r in b31.get("rest", {}).items():
        out.append(f"| {s} | {r['pen_rest_mm']} / {r['pgs_pen_rest_mm']} | {r['pgs_moved_m']} | {r['ms']} / {r['pgs_ms']} | {r['time_ratio']}x |")

    b32 = n.get("B3.2", {})
    if b32:
        scenes = sorted({s for v in b32.values() for s in v})
        out += ["", "### B3.2 Substep sweep: Soft Step at a fixed number of substeps per 60 Hz step", "",
                "| Scene | " + " | ".join(f"{k} substeps: time, penetration, jitter, moved" for k in b32) + " |",
                "| --- | " + " | ".join("---" for _ in b32) + " |"]
        for s in scenes:
            cells = []
            for k, v in b32.items():
                r = v.get(s)
                cells.append(f"{r['ms']} ms, {r['pen_rest_mm']} mm, {r['jitter_mps']} m/s, {r['moved_m']} m" if r else "-")
            out.append(f"| {s} | " + " | ".join(cells) + " |")
    print("\n".join(out))


if __name__ == "__main__":
    main()
