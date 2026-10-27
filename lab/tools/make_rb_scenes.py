#!/usr/bin/env python3
"""Bullet NEXT Lab - writes the Blender rigid body scenes (host profile blender53-rb; FL, CL.7).

Usage: make_rb_scenes.py   (writes lab/scenes/rb_*.json and adds them to lab/scenes/index.json)

Blender's defaults where a scene does not say otherwise: 24 frames per second, 10 substeps per
frame, 10 solver iterations; bodies with friction 0.5, restitution 0, damping 0.04 / 0.1,
collision margin 0.04. Blender cannot give bodies a starting velocity, so these scenes do not.
Passive bodies are "static" (and "animated" when keyframed), active ones
"rigid". Constraints sit in world space between two bodies, as in Blender.
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "scenes")


def quat(axis, angle):
    n = math.sqrt(sum(a * a for a in axis))
    s = math.sin(angle / 2) / n
    return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2)]


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


def ground(half=10.0):
    return body("static", box(half, half, 0.5), (0, 0, -0.5), name="ground")


def settings(**kw):
    s = {"ticrate": 24.0, "substeps": 10, "gravity_z": -9.81, "iterations": 10}
    s.update(kw)
    return s


def scene(name, covers, frames, objects, actions=(), **st):
    return {"name": name, "covers": covers, "profile": "blender53-rb", "frames": frames,
            "settings": settings(**st), "objects": list(objects), "actions": list(actions)}


def rb_stack():
    objs = [ground()]
    for i in range(8):
        objs.append(body("rigid", box(0.5, 0.5, 0.5), (0.02 * (i % 2), 0, 0.5 + i * 1.0), mass=1.0))
    return scene("rb_stack", "rb:bodies,stacking", 250, objs)


def rb_shapes():
    objs = [body("static", {"shape": "mesh", "mesh": {"grid": [16, 16], "size": 10.0}}, (0, 0, 0), name="mesh_ground")]
    hull = [[0.6 * math.cos(i * math.pi / 6), 0.6 * math.sin(i * math.pi / 6), 0.5 if i % 2 else -0.5] for i in range(12)]
    shapes = [
        box(0.5, 0.4, 0.3),
        {"shape": "sphere", "radius": 0.5},
        {"shape": "capsule_z", "radius": 0.3, "height": 1.0},
        {"shape": "cylinder_z", "half": [0.4, 0.4, 0.5]},
        {"shape": "cone_z", "radius": 0.5, "height": 1.0},
        {"shape": "hull", "points": hull},
        {"shape": "compound", "children": [dict(box(0.6, 0.15, 0.15), at=at(0, 0, 0)),
                                           dict({"shape": "sphere", "radius": 0.3}, at=at(0, 0, 0.45))]},
        {"shape": "mesh", "mesh": {"box": [0.5, 0.4, 0.3]}},
    ]
    for i, s in enumerate(shapes):
        objs.append(body("rigid", s, (-7 + 2 * i, 0, 3), q=quat((1, 0.5, 0), 0.3 * i), mass=1.0 + 0.25 * i,
                         friction=0.5, restitution=0.1 * (i % 3)))
    # Collision collections: this sphere is only in collection 2 and falls through the ground.
    objs.append(body("rigid", {"shape": "sphere", "radius": 0.3}, (0, 3, 4), mass=1.0, col_groups=2))
    return scene("rb_shapes", "rb:shapes,margins,collections", 250, objs)


def rb_constraints():
    objs = [ground()]
    cons = []
    x = -8.0
    kinds = [
        ("rb_point", {}),
        ("rb_fixed", {}),
        ("rb_hinge", {"limits": {"ang_z": [-0.8, 0.8]}}),
        ("rb_slider", {"limits": {"lin_x": [-0.5, 0.5]}}),
        ("rb_piston", {"limits": {"lin_x": [-0.5, 0.5], "ang_x": [-1.0, 1.0]}}),
        ("rb_generic", {"limits": {"lin_x": [0, 0], "lin_y": [0, 0], "lin_z": [-0.3, 0.3], "ang_x": [-0.5, 0.5], "ang_y": [0, 0], "ang_z": [0, 0]}}),
        ("rb_generic_spring", {"springs": {"lin_z": [40.0, 0.5], "ang_x": [5.0, 0.5]}, "spring_type": 2,
                               "limits": {"lin_x": [0, 0], "lin_y": [0, 0], "lin_z": [-1, 1], "ang_x": [-1, 1], "ang_y": [0, 0], "ang_z": [0, 0]}}),
        ("rb_generic_spring", {"springs": {"lin_z": [40.0, 0.5]}, "spring_type": 1,
                               "limits": {"lin_x": [0, 0], "lin_y": [0, 0], "lin_z": [-1, 1], "ang_x": [0, 0], "ang_y": [0, 0], "ang_z": [0, 0]}}),
        ("rb_motor", {"motor_ang": [3.0, 2.0]}),
    ]
    for kind, extra in kinds:
        anchor = len(objs)
        objs.append(body("static", box(0.2, 0.2, 0.2), (x, 0, 4)))
        moving = len(objs)
        objs.append(body("rigid", box(0.3, 0.3, 0.6), (x + 0.6, 0.2, 3.4), mass=1.0))
        c = {"type": kind, "a": anchor, "b": moving, "frame": at(x + 0.3, 0, 4, quat((0, 1, 0), 0.4))}
        c.update(extra)
        cons.append({"constraint": c})
        x += 2.0
    # A breakable chain: three boxes held by fixed constraints that give way under a falling weight.
    prev = len(objs)
    objs.append(body("static", box(0.2, 0.2, 0.2), (0, 6, 6)))
    for i in range(3):
        cur = len(objs)
        objs.append(body("rigid", box(0.2, 0.2, 0.4), (0, 6, 5.4 - i * 0.8), mass=1.0))
        cons.append({"constraint": {"type": "rb_fixed", "a": prev, "b": cur, "frame": at(0, 6, 5.8 - i * 0.8), "breaking_threshold": 12.0}})
        prev = cur
    objs.append(body("rigid", box(0.5, 0.5, 0.5), (0, 6, 9), mass=5.0))
    return scene("rb_constraints", "rb:constraints,breaking", 250, objs + cons)


def rb_kinematic():
    objs = [ground()]
    paddle = len(objs)
    objs.append(body("static", box(0.2, 2.0, 0.5), (-6, 0, 0.5), animated=True))
    lift = len(objs)
    objs.append(body("static", box(1.0, 1.0, 0.1), (5, 0, 0.2), animated=True))
    for i in range(5):
        objs.append(body("rigid", box(0.4, 0.4, 0.4), (-3 + i * 0.9, -0.5 + 0.25 * i, 0.4), mass=1.0, use_deactivation=True))
    objs.append(body("rigid", box(0.4, 0.4, 0.4), (5, 0, 0.7), mass=2.0))
    acts = [{"op": "kinematic_move", "from": 0, "to": 199, "body": paddle, "v": [0.06, 0, 0]},
            {"op": "kinematic_move", "from": 24, "to": 120, "body": lift, "v": [0, 0, 0.03]},
            {"op": "kinematic_move", "from": 121, "to": 200, "body": lift, "v": [0, 0, -0.02]}]
    return scene("rb_kinematic", "rb:animated passive,deactivation", 250, objs, acts)


def rb_settings():
    objs = [ground()]
    for r in range(6):
        for i in range(6 - r):
            objs.append(body("rigid", box(0.45, 0.45, 0.45), (-2.5 + r * 0.5 + i * 1.0, 0, 0.45 + r * 0.9), mass=1.0,
                             use_deactivation=True, start_deactivated=(r == 0)))
    objs.append(body("rigid", {"shape": "sphere", "radius": 0.4}, (-1.0, 0.3, 9.0), mass=3.0))
    return scene("rb_settings", "rb:substeps,iterations,split impulse,time scale", 250, objs,
                 substeps=20, iterations=30, split_impulse=True, time_scale=0.5)


def main():
    scenes = [rb_stack(), rb_shapes(), rb_constraints(), rb_kinematic(), rb_settings()]
    for s in scenes:
        with open(os.path.join(OUT, s["name"] + ".json"), "w") as f:
            json.dump(s, f, indent=1)
            f.write("\n")
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
