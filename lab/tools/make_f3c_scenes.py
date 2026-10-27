#!/usr/bin/env python3
"""Bullet NEXT Lab - writes the Soft Step test scenes (F3c, C3.9).

Usage: make_f3c_scenes.py   (writes lab/scenes/<name>.json and adds them to lab/scenes/index.json)

BGE 2.79b defaults (60 Hz, one substep, 10 iterations) unless a scene says otherwise.
The resting scenes keep their bodies awake ("no_sleep"), so that the solver, not
deactivation, holds them still. The bounce and pendulum scenes have no damping, so their
energy is the solver's alone.

  stack_20     20 boxes, one on another (T3.2: penetration, resting jitter)
  pyramid_15   square pyramid of 15 layers, 1,240 boxes (T3.2)
  mass_ratio   100 kg boxes resting on 1 kg boxes (T3.3)
  bounce       spheres of restitution 0 to 0.9 dropped from 2 m (T3.5)
  pendulum     frictionless pendulums: point-to-point and hinge (T3.4: energy)
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "scenes")

BGE = {"ticrate": 60.0, "substeps": 1, "gravity_z": -9.81, "contact_breaking": 0.02,
       "deactivation_time": 2.0, "lin_sleep": 0.8, "ang_sleep": 1.0}


def at(x, y, z, q=None):
    r = {"p": [x, y, z]}
    if q:
        r["q"] = q
    return r


def body(kind, shape, pos, q=None, **kw):
    b = {"type": kind, "at": at(*pos, q=q)}
    b.update(shape)
    b.update(kw)
    return {"body": b}


def box(hx, hy, hz):
    return {"shape": "box", "half": [hx, hy, hz]}


def ground(half=20.0, **kw):
    return body("static", box(half, half, 0.5), (0.0, 0.0, -0.5), name="ground", **kw)


def scene(name, covers, frames, objects, **st):
    s = dict(BGE)
    s.update(st)
    return {"name": name, "covers": covers, "profile": "bge279", "frames": frames,
            "settings": s, "objects": list(objects), "actions": []}


def stack_20():
    objs = [ground()]
    for i in range(20):
        objs.append(body("rigid", box(0.5, 0.5, 0.5), (0.0, 0.0, 0.5 + i), no_sleep=True))
    return scene("stack_20", "F3c T3.2", 600, objs)


def pyramid_15():
    objs = [ground(30.0)]
    layers = 15
    for k in range(layers):
        n = layers - k
        for i in range(n):
            for j in range(n):
                x = (i - (n - 1) / 2.0) * 1.0
                y = (j - (n - 1) / 2.0) * 1.0
                objs.append(body("rigid", box(0.5, 0.5, 0.5), (x, y, 0.5 + k), no_sleep=True))
    assert len(objs) - 1 == 1240
    return scene("pyramid_15", "F3c T3.2", 600, objs)


def mass_ratio():
    objs = [ground()]
    # column A: one 1 kg box under a 100 kg box
    objs.append(body("rigid", box(0.5, 0.5, 0.5), (-2.0, 0.0, 0.5), mass=1.0, no_sleep=True))
    objs.append(body("rigid", box(0.5, 0.5, 0.5), (-2.0, 0.0, 1.5), mass=100.0, no_sleep=True))
    # column B: three 1 kg boxes under a 100 kg box
    for i in range(3):
        objs.append(body("rigid", box(0.5, 0.5, 0.5), (2.0, 0.0, 0.5 + i), mass=1.0, no_sleep=True))
    objs.append(body("rigid", box(0.5, 0.5, 0.5), (2.0, 0.0, 3.5), mass=100.0, no_sleep=True))
    # column C: a small 1 kg box under a wide 100 kg slab
    objs.append(body("rigid", box(0.25, 0.25, 0.25), (6.0, 0.0, 0.25), mass=1.0, no_sleep=True))
    objs.append(body("rigid", box(1.0, 1.0, 0.25), (6.0, 0.0, 0.75), mass=100.0, no_sleep=True))
    return scene("mass_ratio", "F3c T3.3", 600, objs)


def bounce():
    # Bullet multiplies the two restitutions; the ground's 1.0 leaves each sphere's own.
    objs = [ground(restitution=1.0)]
    for i, e in enumerate((0.0, 0.3, 0.6, 0.9)):
        objs.append(body("rigid", {"shape": "sphere", "radius": 0.25}, (-3.0 + 2.0 * i, 0.0, 2.25),
                         restitution=e, lin_damp=0.0, ang_damp=0.0, no_sleep=True))
    return scene("bounce", "F3c T3.5", 240, objs)


def pendulum():
    objs = [ground()]
    # point to point: a 1 kg ball on a 2 m line, released level with the pivot
    objs.append(body("rigid", {"shape": "sphere", "radius": 0.1}, (2.0, 0.0, 4.0), mass=1.0,
                     lin_damp=0.0, ang_damp=0.0, no_sleep=True))
    objs.append({"constraint": {"type": "point_to_point", "a": 1, "b": -1,
                                "pivot_a": [-2.0, 0.0, 0.0], "pivot_b": [0.0, 0.0, 4.0]}})
    # hinge about the Y axis (the hinge axis is the frames' Z axis, turned onto Y)
    q = [-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
    objs.append(body("rigid", {"shape": "sphere", "radius": 0.1}, (2.0, 3.0, 4.0), mass=1.0,
                     lin_damp=0.0, ang_damp=0.0, no_sleep=True))
    objs.append({"constraint": {"type": "hinge", "a": 2, "b": -1,
                                "frame_a": {"p": [-2.0, 0.0, 0.0], "q": q},
                                "frame_b": {"p": [0.0, 3.0, 4.0], "q": q}}})
    return scene("pendulum", "F3c T3.4", 600, objs)


def write(s):
    # one object per line, as the other scene files
    head = {k: s[k] for k in ("covers", "frames", "name", "profile")}
    lines = ["{"]
    for k, v in head.items():
        lines.append('  "%s": %s,' % (k, json.dumps(v)))
    lines.append('  "settings": %s,' % json.dumps(s["settings"], sort_keys=True, separators=(",", ":")))
    lines.append('  "objects": [')
    objs = [json.dumps(o, sort_keys=True, separators=(",", ":")) for o in s["objects"]]
    lines.append(",\n".join("    " + o for o in objs))
    lines.append("  ],")
    lines.append('  "actions": [')
    lines.append("  ]")
    lines.append("}")
    with open(os.path.join(OUT, s["name"] + ".json"), "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    scenes = [stack_20(), pyramid_15(), mass_ratio(), bounce(), pendulum()]
    for s in scenes:
        write(s)
    idx_path = os.path.join(OUT, "index.json")
    idx = json.load(open(idx_path))
    for s in scenes:
        if s["name"] not in idx["scenes"]:
            idx["scenes"].append(s["name"])
    with open(idx_path, "w") as f:
        f.write('{\n  "scenes": [\n' + ",\n".join('    "%s"' % n for n in idx["scenes"]) + "\n  ]\n}\n")
    print("wrote", ", ".join(s["name"] for s in scenes))


if __name__ == "__main__":
    main()
