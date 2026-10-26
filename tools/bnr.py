#!/usr/bin/env python3
"""Reader for harness recordings (.bnr) and the scene-specific F1 gate metrics."""
import struct

F = 13  # floats per body per frame


def load(path):
    d = open(path, "rb").read()
    assert d[:4] == b"BNR1", path
    o = 8

    def s():
        nonlocal o
        n, = struct.unpack_from("<I", d, o)
        o += 4
        v = d[o:o + n].decode()
        o += n
        return v

    backend, scene = s(), s()
    nb, nf = struct.unpack_from("<II", d, o)
    o += 8
    frames = []
    for _ in range(nf):
        st = struct.unpack_from("<%df" % (nb * F), d, o)
        o += nb * F * 4
        ne, = struct.unpack_from("<I", d, o)
        o += 4 + ne * 44 + 8
        frames.append(st)
    return backend, scene, nb, frames


def body(frame, i):
    return frame[i * F:(i + 1) * F]


def tunnelled(path, wall_x=10.0, first_body=2):
    """Number of bodies (from first_body on) that end up beyond the wall."""
    _, _, nb, fr = load(path)
    last = fr[-1]
    return sum(1 for i in range(first_body, nb) if body(last, i)[0] > wall_x + 0.2), nb - first_body


def ghost_bump(path, bodies=(1, 2), start=10):
    """Largest upward speed (m/s) of bodies sliding on a flat mesh after they settle.
    An internal edge that launches a body shows up here (a ghost bump)."""
    _, _, nb, fr = load(path)
    return max(body(f, i)[9] for f in fr[start:] for i in bodies)


def contact_dips(path, bodies=(1, 2), start=10):
    """Largest downward speed (m/s) on a flat mesh: contacts briefly lost, body sinks."""
    _, _, nb, fr = load(path)
    return max(-body(f, i)[9] for f in fr[start:] for i in bodies)
