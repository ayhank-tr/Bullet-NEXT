#!/usr/bin/env python3
"""Bullet NEXT - writes the tables of docs/FL_GATE_REPORT.md from a gate run (FL, CL.12).

Usage: fl_report.py <gate_dir>/fl_gate.json > tables.md
"""
import json
import sys

NAMES = {
    "TL.1": "Builds", "TL.2": "Legacy untouched", "TL.3": "Same physics in HARFANG", "TL.4": "All engines",
    "TL.5": "Host profiles", "TL.6": "Recordings", "TL.7": "Check mode", "TL.8": "Viewer and browser",
    "BL.1": "Host overhead",
}


def fmt(v):
    if isinstance(v, dict):
        return "; ".join(f"{k}: {fmt(x)}" for k, x in v.items())
    if isinstance(v, list):
        return "; ".join(fmt(x) for x in v)
    return str(v)


def main():
    g = json.load(open(sys.argv[1]))
    res, notes = g["results"], g["notes"]
    out = ["| Test | Result | Measured |", "| --- | --- | --- |"]
    for k in ["TL.1", "TL.2", "TL.3", "TL.4", "TL.5", "TL.6", "TL.7", "TL.8", "BL.1"]:
        r = res.get(k)
        out.append(f"| {k} {NAMES[k]} | {'PASS' if r else ('FAIL' if r is False else 'not run')} | {fmt(notes.get(k, ''))} |")
    out += ["", "### BL.2 Reference table: physics time per scene (ms for the whole scene; 1 thread / all threads)", "",
            "| Scene | legacy | next | nextf | jolt |", "| --- | --- | --- | --- | --- |"]
    for s, es in notes["BL.2"].items():
        cells = []
        for e in ("legacy", "next", "nextf", "jolt"):
            a, b = es[e]
            cells.append(f"{a:.1f}" if e in ("legacy", "jolt") else f"{a:.1f} / {b:.1f}")
        out.append(f"| {s} | " + " | ".join(cells) + " |")
    if "BL.3" in notes:
        out += ["", "### BL.3 Browser speed: physics time per scene, native and WebAssembly (ms, one thread)", "",
                "| Scene | legacy native / wasm | nextf native / wasm | jolt native / wasm | nextf wasm / native |",
                "| --- | --- | --- | --- | --- |"]
        for s, es in notes["BL.3"].items():
            cells = [f"{es[e]['native_ms']:.1f} / {es[e]['wasm_ms']:.1f}" for e in ("legacy", "nextf", "jolt")]
            ratio = es["nextf"]["wasm_ms"] / es["nextf"]["native_ms"] if es["nextf"]["native_ms"] else 0
            out.append(f"| {s} | " + " | ".join(cells) + f" | {ratio:.1f}x |")
    print("\n".join(out))


if __name__ == "__main__":
    main()
