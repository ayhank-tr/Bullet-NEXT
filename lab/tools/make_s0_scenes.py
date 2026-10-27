#!/usr/bin/env python3
"""Bullet NEXT Lab - writes the Stage 0 (Ready for Rally FX) scenes (CS.4).

Usage: make_s0_scenes.py   (writes lab/scenes/<name>.json; the first four join lab/scenes/index.json)

BGE 2.79b defaults (60 Hz, one substep, 10 iterations). The terrain is a heightfield made by the
Lab's own generator (lab/core/bnl_scene.cpp), so every engine and platform gets the same heights.

  rally_slide       a box, a sphere and two car-sized boxes sliding (no friction) at 30 and 50 m/s
                    over a flat heightfield: every quad edge is a chance for a phantom bump (TS.3)
  rally_far         the same, 10 km from the origin on x and y (TS.4: float precision)
  rally_far_shift   rally_far, recentred on its first frame with shift_origin (TS.4)
  rally_stage       the Lab car on rolling heightfield terrain for 60 s, driven by a scripted
                    throttle, brake and steering timeline (TS.5 replay, BS.1 cost)
  rally_stage_mesh  rally_stage with the same terrain as a triangle mesh (BS.2; not in the index)
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "scenes")

BGE = {"ticrate": 60.0, "substeps": 1, "gravity_z": -9.81, "contact_breaking": 0.02,
       "deactivation_time": 2.0, "lin_sleep": 0.8, "ang_sleep": 1.0}
FAR = 10000.0


def at(x, y, z, q=None):
    r = {"p": [x, y, z]}
    if q:
        r["q"] = q
    return r


def body(kind, shape, pos, **kw):
    b = {"type": kind, "at": at(*pos)}
    b.update(shape)
    b.update(kw)
    return {"body": b}


def scene(name, covers, frames, objects, actions):
    return {"name": name, "covers": covers, "profile": "bge279", "frames": frames,
            "settings": dict(BGE), "objects": list(objects), "actions": list(actions)}


def slide(name, off, shift):
    flat = {"shape": "heightfield", "samples": [320, 320], "spacing": 2.0, "terrain": {"kind": "plane"}}
    objs = [body("static", flat, (off, off, 0.0), name="ground")]
    movers = [  # shape, lane y, height of the centre, speed (m/s), mass (kg)
        ({"shape": "box", "half": [0.5, 0.5, 0.5]}, -20.0, 0.51, 30.0, 1.0),
        ({"shape": "sphere", "radius": 0.4}, -10.0, 0.41, 30.0, 1.0),
        ({"shape": "box", "half": [2.2, 0.9, 0.4]}, 0.0, 0.41, 50.0, 1200.0),
        ({"shape": "box", "half": [2.2, 0.9, 0.4]}, 10.0, 0.41, 30.0, 1200.0),
        ({"shape": "sphere", "radius": 0.4}, 20.0, 0.41, 50.0, 1.0),
    ]
    actions = []
    if shift:
        actions.append({"op": "shift_origin", "body": 0, "from": 0, "to": 0, "v": [-off, -off, 0.0]})
    for k, (shape, lane, z, speed, mass) in enumerate(movers):
        # no friction: the bodies slide without tipping, so any upward speed is a bump from an edge
        objs.append(body("rigid", shape, (off - 280.0, off + lane, z), mass=mass, friction=0.0, no_sleep=True))
        actions.append({"op": "lin_vel", "body": k + 1, "from": 0, "to": 0, "v": [speed, 0.0, 0.0]})
    return scene(name, "Stage 0 TS.3, TS.4", 600, objs, actions)


TERRAIN = {"kind": "rolling", "amplitude": 3.0, "wavelength": 120.0, "octaves": 3, "seed": 7}  # 2 km square


def stage(name, as_mesh):
    if as_mesh:
        ground = {"shape": "mesh", "mesh": {"terrain_grid": {"samples": [512, 512], "spacing": 4.0, "terrain": TERRAIN}}}
    else:
        ground = {"shape": "heightfield", "samples": [512, 512], "spacing": 4.0, "terrain": TERRAIN}
    objs = [body("static", ground, (0.0, 0.0, 0.0), name="ground"),
            body("rigid", {"shape": "box", "half": [0.9, 2.0, 0.4]}, (0.0, -150.0, 7.0), mass=800.0, no_sleep=True)]
    objs.append({"vehicle": {"chassis": 1}})
    tuning = {"damping_compression": 2.0, "damping_relaxation": 3.0, "friction_slip": 2.0,
              "roll_influence": 0.1, "suspension_stiffness": 20.0}
    for x, y, front in ((0.9, 1.4, True), (-0.9, 1.4, True), (0.9, -1.4, False), (-0.9, -1.4, False)):
        objs.append({"wheel": {"vehicle": 0, "pos": [x, y, 0.0], "down": [0.0, 0.0, -1.0], "axle": [-1.0, 0.0, 0.0],
                               "radius": 0.4, "rest_length": 0.5, "steering": front, "tuning": tuning}})
    drive = [0.0, 0.0, 1500.0, 1500.0]
    brake = [40.0, 40.0, 40.0, 40.0]
    actions = [  # the scripted driver: 60 s of throttle, braking and steering, circling the middle
        {"op": "vehicle", "vehicle": 0, "from": 60, "to": 899, "engine": drive},
        {"op": "vehicle", "vehicle": 0, "from": 60, "to": 1799, "steer": [0.08, 0.08]},
        {"op": "vehicle", "vehicle": 0, "from": 900, "to": 1019, "brake": brake},
        {"op": "vehicle", "vehicle": 0, "from": 1080, "to": 2039, "engine": drive},
        {"op": "vehicle", "vehicle": 0, "from": 1800, "to": 2399, "steer": [-0.1, -0.1]},
        {"op": "vehicle", "vehicle": 0, "from": 2280, "to": 2879, "engine": drive},
        {"op": "vehicle", "vehicle": 0, "from": 2400, "to": 3239, "steer": [0.12, 0.12]},
        {"op": "vehicle", "vehicle": 0, "from": 3240, "to": -1, "brake": brake},
    ]
    return scene(name, "Stage 0 TS.5, BS.1, BS.2", 3600, objs, actions)


def write(s):
    head = {k: s[k] for k in ("covers", "frames", "name", "profile")}
    lines = ["{"]
    for k, v in head.items():
        lines.append('  "%s": %s,' % (k, json.dumps(v)))
    lines.append('  "settings": %s,' % json.dumps(s["settings"], sort_keys=True, separators=(",", ":")))
    lines.append('  "objects": [')
    lines.append(",\n".join("    " + json.dumps(o, sort_keys=True, separators=(",", ":")) for o in s["objects"]))
    lines.append("  ],")
    lines.append('  "actions": [')
    lines.append(",\n".join("    " + json.dumps(a, sort_keys=True, separators=(",", ":")) for a in s["actions"]))
    lines.append("  ]")
    lines.append("}")
    with open(os.path.join(OUT, s["name"] + ".json"), "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    indexed = [slide("rally_slide", 0.0, False), slide("rally_far", FAR, False), slide("rally_far_shift", FAR, True),
               stage("rally_stage", False)]
    extra = [stage("rally_stage_mesh", True)]
    for s in indexed + extra:
        write(s)
    idx_path = os.path.join(OUT, "index.json")
    idx = json.load(open(idx_path))
    for s in indexed:
        if s["name"] not in idx["scenes"]:
            idx["scenes"].append(s["name"])
    with open(idx_path, "w") as f:
        f.write('{\n  "scenes": [\n' + ",\n".join('    "%s"' % n for n in idx["scenes"]) + "\n  ]\n}\n")
    print("wrote", ", ".join(s["name"] for s in indexed + extra))


if __name__ == "__main__":
    main()
