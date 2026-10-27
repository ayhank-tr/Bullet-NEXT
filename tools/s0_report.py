#!/usr/bin/env python3
"""Bullet NEXT - writes the tables of docs/S0_GATE_REPORT.md from a gate run (Stage 0).

Usage: s0_report.py <gate_dir>/s0_gate.json > tables.md
"""
import json
import sys

NAMES = {"TS.1": "Bounce", "TS.2": "No regressions", "TS.3": "Heightfield bumps", "TS.4": "Far from the origin",
         "TS.5": "Replay", "TS.6": "Engines", "BS.1": "Rally stage cost", "BS.2": "Heightfield or mesh"}
ENGINES = ["legacy", "next", "nextf", "nexts", "jolt"]
LABEL = {"legacy": "Bullet 2.84 (2.79b)", "next": "Bullet NEXT default", "nextf": "Bullet NEXT, all collision features",
         "nexts": "Soft Step", "jolt": "Jolt 5.6"}


def verdict(r):
    return {True: "PASS", False: "FAIL", None: "not run here"}[r]


def main():
    g = json.load(open(sys.argv[1]))
    res, n = g["results"], g["notes"]
    out = ["| Test | Result | Measured |", "| --- | --- | --- |"]
    t1 = n.get("TS.1", {})
    out.append(f"| TS.1 {NAMES['TS.1']} | {verdict(res.get('TS.1'))} | restitution 0.9 comes back as: " + ", ".join(
        f"{LABEL[e]} {rows[3]['measured']}" for e, rows in t1.items()) + " |")
    t2 = n.get("TS.2", {})
    ch = t2.get("changed_vs_reference", {})
    out.append(f"| TS.2 {NAMES['TS.2']} | {verdict(res.get('TS.2'))} | F1 gate {t2.get('f1_gate', 'not run')}; F3c gate "
               f"{'PASSED' if t2.get('f3c_gate') and all(t2['f3c_gate'].values()) else 'not run' if not t2.get('f3c_gate') else 'NOT PASSED'}; "
               f"hashes changed against the reference: legacy {len(ch.get('legacy', []))}, Jolt {len(ch.get('jolt', []))}, "
               f"next {len(ch.get('next', []))}, nextf {len(ch.get('nextf', []))}, nexts {len(ch.get('nexts', []))} scenes |")
    t3 = n.get("TS.3", {})
    out.append(f"| TS.3 {NAMES['TS.3']} | {verdict(res.get('TS.3'))} | largest upward speed sliding at 30 and 50 m/s: " + ", ".join(
        f"{LABEL[e]} {v['up_max_mps']} m/s" for e, v in t3.items()) + " |")
    t4 = n.get("TS.4", {})
    out.append(f"| TS.4 {NAMES['TS.4']} | {verdict(res.get('TS.4'))} | path difference 10 km out, recentred / not: " + ", ".join(
        f"{LABEL[e]} {v['recentred_path_diff_mm']} / {v['far_path_diff_mm']} mm" for e, v in t4.items()) + " |")
    t5 = n.get("TS.5", {})
    br = t5.get("browser_same")
    out.append(f"| TS.5 {NAMES['TS.5']} | {verdict(res.get('TS.5'))} | same hash at 1, 2, 4, 8 threads: "
               + ", ".join(f"{LABEL[e]} {'yes' if v else 'no'}" for e, v in t5.get("same_at_all_threads", {}).items())
               + f"; recordings play back frame by frame: {'yes' if t5.get('replay_ok') else 'no'}"
               + (("; browser = PC on the rally scenes: " + ", ".join(f"{LABEL[e]} {len(v)}/4" for e, v in br.items())) if br else "")
               + " |")
    t6 = n.get("TS.6", {})
    bad = [k for k, v in t6.items() if k != "chrono" and not v]
    out.append(f"| TS.6 {NAMES['TS.6']} | {verdict(res.get('TS.6'))} | {len([k for k in t6 if k != 'chrono']) - len(bad)} of "
               f"{len([k for k in t6 if k != 'chrono'])} runs without NaN, blow-up or fall"
               + ("; Chrono: heightfields skipped, as planned" if t6.get("chrono") else "") + " |")
    out.append(f"| BS.1 {NAMES['BS.1']} | record | table below |")
    out.append(f"| BS.2 {NAMES['BS.2']} | record | table below |")

    out += ["", "### BS.1 and BS.2: the 60 s rally stage (one car, 2 km of rolling terrain), whole-run time in ms", "",
            "| Engine | Heightfield terrain | The same terrain as a triangle mesh |", "| --- | --- | --- |"]
    for e, v in n.get("BS.2", {}).items():
        out.append(f"| {LABEL[e]} | {v['heightfield_ms']} | {v['mesh_ms']} |")
    if ch:
        out += ["", "### TS.2: scenes whose final hash changed against the reference", "",
                "| Engine | Scenes |", "| --- | --- |"]
        for e in ENGINES:
            if ch.get(e):
                out.append(f"| {LABEL[e]} | {', '.join(ch[e])} |")
    print("\n".join(out))


if __name__ == "__main__":
    main()
