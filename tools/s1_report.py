#!/usr/bin/env python3
"""Bullet NEXT - writes the tables of docs/S1_GATE_REPORT.md from a gate run (Stage 1).

Usage: s1_report.py <gate_dir>/s1_gate.json > tables.md
"""
import json
import sys

LABEL = {"legacy": "Bullet 2.84 (2.79b)", "next": "Bullet NEXT default", "nextf": "Bullet NEXT, all collision features",
         "nexts": "Soft Step", "jolt": "Jolt 5.6"}
NAMES = {"T1S.1": "Edges from the heights", "T1S.2": "Car body on terrain", "T1S.3": "Full-size terrain",
         "T1S.4": "Crash report", "T1S.5": "Threads under load", "T1S.6": "Chrono reference", "T1S.7": "No regressions"}


def verdict(r):
    return {True: "PASS", False: "FAIL", None: "not run here"}[r]


def main():
    g = json.load(open(sys.argv[1]))
    res, n = g["results"], g["notes"]
    out = ["| Test | Result | Measured |", "| --- | --- | --- |"]
    t1 = n.get("T1S.1", [])
    out.append(f"| T1S.1 {NAMES['T1S.1']} | {verdict(res.get('T1S.1'))} | " + "; ".join(
        f"{r['scene']} ({r['samples'][0]} x {r['samples'][1]}): {r['triangles']:,} triangles, largest angle difference "
        f"{r['max_angle_diff_rad']} rad, {r['mismatches']} mismatches, map entries {r['computed_map_entries']}" for r in t1) + " |")
    t2 = n.get("T1S.2", {})
    tilt = t2.get("tilt", {})
    hm = t2.get("heightfield_vs_mesh", {})
    out.append(f"| T1S.2 {NAMES['T1S.2']} | {verdict(res.get('T1S.2'))} | speed off the 15 % slope: " + ", ".join(
        f"{LABEL[e]} {v['off_plane_max_mps']} m/s" for e, v in tilt.items()) + "; heightfield against mesh, first 0.5 s / after 10 s "
        "(10 um nudge): " + ", ".join(f"{LABEL[e]} {v['first_s_max_mm']} mm / {v['at_10_s_mm']} mm ({v['nudge_spread_at_10_s_mm']} mm)"
                                      for e, v in hm.items()) + " |")
    t3 = n.get("T1S.3", {})
    worked = next((b for b in t3.get("bench_3001", []) if b["edges"] == "worked out"), {})
    baked = next((b for b in t3.get("bench_3001", []) if b["edges"] == "baked"), {})
    out.append(f"| T1S.3 {NAMES['T1S.3']} | {verdict(res.get('T1S.3'))} | 3001 x 3001 at 1 m: edges ready in {worked.get('seconds')} s, "
               f"{worked.get('extra_memory_mb')} MB (baked: {baked.get('seconds')} s, {baked.get('extra_memory_mb')} MB); speed off the slope: "
               + ", ".join(f"{LABEL[e]} {v} m/s" for e, v in t3.get("off_plane_max_mps", {}).items()) + " |")
    t4 = n.get("T1S.4", {})
    out.append(f"| T1S.4 {NAMES['T1S.4']} | {verdict(res.get('T1S.4'))} | reported impulses against momentum change, worst: " + ", ".join(
        f"{LABEL[e]} {v['impulse_error_max']:.2g}" for e, v in t4.items() if v.get("impulse_error_max") is not None)
        + "; bodies through a wall or trunk: " + ", ".join(f"{LABEL[e]} {v['through']} of {len(v['end_y'])}" for e, v in t4.items()) + " |")
    t5 = n.get("T1S.5", {})
    hp = t5.get("hashes_per_scene", {})
    out.append(f"| T1S.5 {NAMES['T1S.5']} | {verdict(res.get('T1S.5'))} | {t5.get('runs')} runs at 8 threads beside two busy loops: "
               + ", ".join(f"{k} {len(v)} hash{'es' if len(v) != 1 else ''}" for k, v in hp.items()) + " |")
    t6 = n.get("T1S.6", {})
    out.append(f"| T1S.6 {NAMES['T1S.6']} | {verdict(res.get('T1S.6'))} | Chrono on {t6.get('chrono_scenes', 0)} scenes from two folders, "
               f"forward and reversed: {len(t6.get('chrono_differs', [])) if 'chrono_differs' in t6 else 'not run'} differ; rigs: "
               f"{'run' if 'rigs' in t6 else 'not run'} (table below) |")
    t7 = n.get("T1S.7", {})
    ch = t7.get("changed_vs_stage0", {})
    out.append(f"| T1S.7 {NAMES['T1S.7']} | {verdict(res.get('T1S.7'))} | F1 {'PASSED' if t7.get('f1_gate') else 'not passed'}, F3c "
               f"{'PASSED' if t7.get('f3c_gate') else 'not passed'}, Stage 0 {'PASSED' if t7.get('s0_gate') else 'not passed'}; hashes changed "
               "against Stage 0: " + ", ".join(f"{LABEL[e]} {len(ch.get(e, []))}" for e in LABEL) + " |")
    out.append("| B1S.1 Edge setup | record | table below |")
    out.append("| B1S.2 Car body step cost | record | table below |")
    out.append("| B1S.3 Tyre rigs | record | table below |")

    out += ["", "### B1S.1: switching active edges on for a rolling field at 1 m", "",
            "| Samples | Triangles | Baked: time, extra memory | Worked out: time, extra memory |", "| --- | --- | --- | --- |"]
    rows = {}
    for b in n.get("B1S.1", []):
        rows.setdefault(b["samples"], {})[b["edges"]] = b
    for s in sorted(rows):
        bk, wo = rows[s].get("baked", {}), rows[s].get("worked out", {})
        out.append(f"| {s} x {s} | {bk.get('triangles', wo.get('triangles', 0)):,} | {bk.get('seconds')} s, {bk.get('extra_memory_mb')} MB | "
                   f"{wo.get('seconds')} s, {wo.get('extra_memory_mb')} MB |")
    out += ["", "### B1S.2: the car body on rolling terrain for 10 s, whole-run time (best of 3)", "",
            "| Engine | Heightfield (edges worked out) | The same ground as a mesh (baked edges) | Ratio |", "| --- | --- | --- | --- |"]
    for e, v in n.get("B1S.2", {}).items():
        out.append(f"| {LABEL[e]} | {v['heightfield_ms']} ms | {v['mesh_ms']} ms | {v['ratio']}x |")
    rigs = t6.get("rigs")
    if rigs:
        out += ["", "### T1S.6 and B1S.3: Chrono's tyres on tarmac, steady state", "",
                "| Tyre | Load | Peak grip, braking/driving (at slip) | Peak grip, cornering (at angle) | Cornering stiffness per unit load | Grip left at 4 degrees with 10 % slip | Time per evaluation |",
                "| --- | --- | --- | --- | --- | --- | --- |"]
        names = {"tmeasy_bmw_e90_front": "TMeasy, BMW E90 front (225/45 R17)", "pac02_sedan": "Pacejka 2002, sedan (245/40 R18)"}
        for t, loads in rigs["tyres"].items():
            for l in loads:
                share = l["mu_y_at_4_deg_slip_0_1"] / l["mu_y_at_4_deg_slip_0"] if l["mu_y_at_4_deg_slip_0"] else 0
                out.append(f"| {names.get(t, t)} | {l['fz_n'] / 1000:g} kN | {l['mu_x_peak']} ({l['slip_at_mu_x_peak']:.2f}) | "
                           f"{l['mu_y_peak']} ({l['angle_at_mu_y_peak_deg']:g} deg) | {l['cornering_stiffness_per_load_per_rad']} /rad | "
                           f"{share:.0%} | {n.get('B1S.3', {}).get(t, '')} us |")
        out += ["", "### T1S.6: Chrono's shafts as the Evo VI's 4WD, first gear, full throttle, split grip (left 0.3, right 0.9)", "",
                "| Locking | Torque at the wheels FL, FR, RL, RR (Nm) | Total | Front share | Torque bias front / rear |", "| --- | --- | --- | --- | --- |"]
        for c in rigs["driveline"]:
            out.append(f"| {c['case']} | {', '.join(f'{x:.0f}' for x in c['wheel_torque_nm'])} | {c['total_drive_nm']:.0f} | "
                       f"{c['front_share']:.0%} | {c['torque_bias_front']:.2f} / {c['torque_bias_rear']:.2f} |")
    if ch:
        out += ["", "### T1S.7: scenes whose final hash changed against Stage 0", "", "| Engine | Scenes |", "| --- | --- |"]
        for e in LABEL:
            if ch.get(e):
                out.append(f"| {LABEL[e]} | {', '.join(ch[e])} |")
    print("\n".join(out))


if __name__ == "__main__":
    main()
