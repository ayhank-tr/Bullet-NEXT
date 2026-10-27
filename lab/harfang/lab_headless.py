"""Bullet NEXT Lab - runs the Lab scenes inside HARFANG without a window (FL, TL.3 and BL.1).

Usage: python lab_headless.py [--engines legacy,nextf] [--scene NAME] [--scenes DIR] [--out FILE]

Each scene is loaded with HARFANG's Lab physics module (hg.SceneLabPhysics), one HARFANG node per
body, and stepped frame by frame with UpdateScene: scene update, physics frame, nodes put on the
bodies, world matrices - as HARFANG's own SceneUpdateSystems does for its physics.
Writes the final hash and timings per scene and engine as JSON:
  {"scene": {"engine": {"hash": "...", "frames": N, "step_ms": total, "frame_ms": total}}}
step_ms is the physics alone; frame_ms adds HARFANG's work (node sync and scene update).
"""
import argparse
import json
import os
import sys
import time

import harfang as hg

HERE = os.path.dirname(os.path.abspath(__file__))
SCENES = os.path.join(os.path.dirname(HERE), "scenes")


def as_list(seq):
    """HARFANG returns C++ string lists as sequence objects."""
    return [seq[i] for i in range(len(seq))]


def run(path, engine):
    lab = hg.SceneLabPhysics()
    if not lab.LoadScene(path, engine):
        raise RuntimeError(lab.GetError())
    scene = hg.Scene()
    nodes = hg.NodeList()
    for i in range(lab.GetBodyCount()):
        node = scene.CreateNode(lab.GetBodyName(i) or "body%d" % i)
        node.SetTransform(scene.CreateTransform())
        nodes.push_back(node)
    step_ms = frame_ms = 0.0
    dt = hg.time_from_sec_f(lab.GetTimeStep())
    for _ in range(lab.GetFrameCount()):
        t0 = time.perf_counter()
        lab.UpdateScene(scene, nodes, dt)
        frame_ms += (time.perf_counter() - t0) * 1000.0
        step_ms += lab.GetLastStepMs()
    return {"hash": lab.GetFinalHash(), "frames": lab.GetFrame(), "step_ms": step_ms, "frame_ms": frame_ms}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--engines", default=",".join(as_list(hg.SceneLabPhysics.GetEngineNames())))
    ap.add_argument("--scene", default="all")
    ap.add_argument("--scenes", default=SCENES)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    results = {}
    for name in as_list(hg.SceneLabPhysics.GetSceneNames(a.scenes)):
        if a.scene != "all" and a.scene != name:
            continue
        results[name] = {}
        for engine in a.engines.split(","):
            r = run(os.path.join(a.scenes, name + ".json"), engine)
            results[name][engine] = r
            print("%-18s %-7s frames %4d hash %s step %8.2f ms frame %8.2f ms" %
                  (name, engine, r["frames"], r["hash"], r["step_ms"], r["frame_ms"]), flush=True)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(results, f, indent=1)


if __name__ == "__main__":
    sys.exit(main())
