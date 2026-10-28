#!/usr/bin/env python3
"""Bullet NEXT Lab - writes the Stage 1 scenes (what Rally FX needs from Bullet NEXT, C1S.2, C1S.4).

Usage: make_s1_scenes.py   (writes lab/scenes/<name>.json; all but rally_body_mesh join lab/scenes/index.json)

BGE 2.79b defaults (60 Hz, one substep). The car body is a compound of two convex hulls the size
of the Evo VI shell (4.35 x 1.77 x 1.41 m, 1,230 kg), length along y, the centre of mass at its
origin. Terrain comes from the Lab's own generator (lab/core/bnl_scene.cpp), so every engine and
platform gets the same heights; a "plane" terrain with a slope is the tilted plane.

  rally_tilt       a box, a ball and two car bodies sliding (no friction) across a 15 % slope at 30
                   and 50 m/s; any speed off the plane is a bump (T1S.2)
  rally_body       the car body thrown sideways and spinning onto rolling terrain, sliding and
                   tumbling with friction for 10 s (T1S.2, T1S.5, B1S.2)
  rally_body_mesh  rally_body with the same terrain as a triangle mesh (T1S.2, B1S.2; not in the index)
  rally_big        3 km of 15 % slope at 1 m (9 million heights), two car bodies and a box sliding
                   across it (T1S.3)
  rally_crash      no gravity: car bodies into walls at 10, 30 and 60 m/s and into a tree trunk at
                   30 m/s; the engines report every impulse (T1S.4, T1S.5)
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "scenes")

BGE = {"ticrate": 60.0, "substeps": 1, "gravity_z": -9.81, "contact_breaking": 0.02,
       "deactivation_time": 2.0, "lin_sleep": 0.8, "ang_sleep": 1.0}
CAR_MASS = 1230.0


def car_body():
    """Evo VI-sized shell: a lower body with sloped nose and tail, and a cabin."""
    lower = []
    for sx in (-1, 1):
        lower += [[sx * 0.885, 2.175, 0.10], [sx * 0.885, 2.175, -0.10], [sx * 0.85, 1.95, -0.30],
                  [sx * 0.885, -2.175, 0.12], [sx * 0.885, -2.175, -0.12], [sx * 0.85, -1.95, -0.30],
                  [sx * 0.885, 1.70, 0.20], [sx * 0.885, -1.80, 0.20]]
    cabin = []
    for sx in (-1, 1):
        cabin += [[sx * 0.85, 1.0, 0.15], [sx * 0.85, -1.6, 0.15], [sx * 0.68, 0.2, 0.82], [sx * 0.68, -1.1, 0.82]]
    return {"shape": "compound", "children": [{"shape": "hull", "points": lower, "at": {"p": [0, 0, 0]}},
                                              {"shape": "hull", "points": cabin, "at": {"p": [0, 0, 0]}}]}


CAR_BOTTOM = 0.30   # from the centre of mass down to the lowest point


def body(kind, shape, pos, q=None, **kw):
    b = {"type": kind, "at": {"p": list(pos)}}
    if q:
        b["at"]["q"] = list(q)
    b.update(shape)
    b.update(kw)
    return {"body": b}


def scene(name, covers, frames, objects, actions=(), settings=None):
    st = dict(BGE)
    st.update(settings or {})
    return {"name": name, "covers": covers, "profile": "bge279", "frames": frames,
            "settings": st, "objects": list(objects), "actions": list(actions)}


SLOPE = 0.15


def on_slope(x, y, lift):
    """A point `lift` above the plane z = SLOPE * x, and the rotation that lays a body on it."""
    a = math.atan(SLOPE)
    n = (-math.sin(a), 0.0, math.cos(a))
    q = [0.0, math.sin(-a / 2.0), 0.0, math.cos(a / 2.0)]
    return (x + n[0] * lift, y, SLOPE * x + n[2] * lift), q


def slope_scene(name, samples, spacing, start_y, frames, covers):
    ground = {"shape": "heightfield", "samples": [samples, samples], "spacing": spacing,
              "terrain": {"kind": "plane", "slope_x": SLOPE}}
    objs = [body("static", ground, (0.0, 0.0, 0.0), name="ground")]
    movers = [  # shape, lane x, lift (centre above the plane), speed along y (m/s), mass
        ({"shape": "box", "half": [0.5, 0.5, 0.5]}, -30.0, 0.51, 30.0, 1.0),
        ({"shape": "sphere", "radius": 0.4}, -15.0, 0.41, 30.0, 1.0),
        (car_body(), 0.0, CAR_BOTTOM + 0.05, 50.0, CAR_MASS),
        (car_body(), 15.0, CAR_BOTTOM + 0.05, 30.0, CAR_MASS),
    ]
    for shape, lane, lift, speed, mass in movers:
        p, q = on_slope(lane, start_y, lift)
        # no friction: the bodies slide without tipping, so any speed off the plane is a bump
        objs.append(body("rigid", shape, p, q, mass=mass, friction=0.0, no_sleep=True, lin_vel=[0.0, speed, 0.0]))
    return scene(name, covers, frames, objs)


TERRAIN = {"kind": "rolling", "amplitude": 3.0, "wavelength": 120.0, "octaves": 3, "seed": 7}  # as rally_stage


def terrain_hash(a, b, c):
    m = 0xffffffff
    h = ((a * 0x8da6b343) & m) ^ ((b * 0xd8163841) & m) ^ ((c * 0xcb1ab31f) & m)
    h ^= h >> 13
    h = (h * 0x5bd1e995) & m
    h ^= h >> 15
    return h


def terrain_height(x, y, t):
    """The Lab generator's height at (x, y) (bnl_scene.cpp, generateHeights), for placing bodies."""
    h, amp, wl = 0.0, t["amplitude"], t["wavelength"]
    for o in range(t["octaves"]):
        u, v = x / wl, y / wl
        fu, fv = math.floor(u), math.floor(v)
        iu, iv = int(fu), int(fv)
        sm = lambda s: s * s * (3.0 - 2.0 * s)
        su, sv = sm(u - fu), sm(v - fv)
        seed = t["seed"] + 7919 * o
        lat = lambda i, j: (terrain_hash(seed & 0xffffffff, i & 0xffffffff, j & 0xffffffff) & 0xffffff) / 8388607.5 - 1.0
        a, b, c, d = lat(iu, iv), lat(iu + 1, iv), lat(iu, iv + 1), lat(iu + 1, iv + 1)
        ab, cd = a + (b - a) * su, c + (d - c) * su
        h += amp * (ab + (cd - ab) * sv)
        amp *= 0.5
        wl *= 0.5
    return h


def body_scene(name, as_mesh):
    if as_mesh:
        ground = {"shape": "mesh", "mesh": {"terrain_grid": {"samples": [512, 512], "spacing": 4.0, "terrain": TERRAIN}}}
    else:
        ground = {"shape": "heightfield", "samples": [512, 512], "spacing": 4.0, "terrain": TERRAIN}
    x0, y0 = 0.0, -40.0
    top = max(terrain_height(x0 + dx, y0 + dy, TERRAIN) for dx in (-3, 0, 3) for dy in (-3, 0, 3))
    objs = [body("static", ground, (0.0, 0.0, 0.0), name="ground"),
            body("rigid", car_body(), (x0, y0, top + 1.5), mass=CAR_MASS, friction=0.8, no_sleep=True,
                 lin_vel=[14.0, 6.0, -2.0], ang_vel=[0.4, -0.3, 2.5])]
    return scene(name, "Stage 1 T1S.2, T1S.5, B1S.2", 600, objs)


def crash_scene():
    objs = []
    lanes = [(-30.0, 10.0, "wall"), (-10.0, 30.0, "wall"), (10.0, 60.0, "wall"), (30.0, 30.0, "tree")]
    for x, speed, what in lanes:
        if what == "wall":
            objs.append(body("static", {"shape": "box", "half": [4.0, 0.5, 2.0]}, (x, 30.0, 0.0), name="wall", friction=0.6))
        else:
            objs.append(body("static", {"shape": "cylinder_z", "half": [0.25, 0.25, 3.0]}, (x + 0.4, 30.0, 0.0),
                             name="tree", friction=0.6))
    for x, speed, what in lanes:
        # straight on, a little off centre and turned, so the hit also spins the car
        q = [0.0, 0.0, math.sin(0.05), math.cos(0.05)]
        objs.append(body("rigid", car_body(), (x, 30.0 - 0.5 - 2.4 - speed / 60.0 * 20.0, 0.0), q, mass=CAR_MASS,
                         friction=0.6, lin_damp=0.0, ang_damp=0.0, no_sleep=True, lin_vel=[0.0, speed, 0.0]))
    # a single hull (a loose wheel-sized crate) into a 10 cm plank at 60 m/s: a step moves it 1 m
    objs.append(body("static", {"shape": "box", "half": [2.0, 0.05, 2.0]}, (50.0, 30.0, 0.0), name="plank", friction=0.6))
    objs.append(body("rigid", {"shape": "box", "half": [0.3, 0.3, 0.3]}, (50.0, 10.0, 0.0), mass=25.0, friction=0.6,
                     lin_damp=0.0, ang_damp=0.0, no_sleep=True, lin_vel=[0.0, 60.0, 0.0]))
    return scene("rally_crash", "Stage 1 T1S.4, T1S.5", 120, objs,
                 settings={"gravity_z": 0.0, "report_impulses": True})


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
    indexed = [slope_scene("rally_tilt", 320, 2.0, -280.0, 600, "Stage 1 T1S.2"),
               body_scene("rally_body", False),
               slope_scene("rally_big", 3000, 1.0, -1400.0, 600, "Stage 1 T1S.3"),
               crash_scene()]
    extra = [body_scene("rally_body_mesh", True)]
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
