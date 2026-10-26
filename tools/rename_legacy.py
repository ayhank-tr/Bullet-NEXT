#!/usr/bin/env python3
"""Bullet NEXT - legacy rename tool (commit C0.2).

Produces the frozen legacy backend: a copy of Blender 2.79b's Bullet 2.84 in which
every identifier that can become a linker symbol is renamed, so the legacy copy and
Bullet NEXT can be linked into one program without clashes.

Rules
  * identifiers starting with "bt" + [A-Z0-9_]   ->  "btl" + rest   (btVector3 -> btlVector3)
  * identifiers starting with gim_/GIM_          ->  "btl_" + name
  * any other exported name listed in tools/legacy_extra_symbols.txt -> "btl_" + name
  * string literals, character literals and comments are never touched
    (keeps #include paths and the .bullet serializer DNA names intact)

Usage
  rename_legacy.py tree  <src_dir> <dst_dir>     rename a whole source tree
  rename_legacy.py files <dst_dir> <file>...     rename single files (used for the harness)
  --extra <names.txt>   (before the mode, repeatable) more names to prefix with "btl_";
                        used for the game engine's legacy copy of its physics module
"""
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
IDENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
BT = re.compile(r"^bt[A-Z0-9_]")
GIM = re.compile(r"^(gim_|GIM_)")
NEVER = {"int", "void", "operator", "bt"}


def load_extra():
    path = os.path.join(HERE, "legacy_extra_symbols.txt")
    names = set()
    if os.path.exists(path):
        for line in open(path):
            line = line.strip()
            if line and not line.startswith("#"):
                names.add(line)
    return names - NEVER


EXTRA = load_extra()


def map_ident(tok):
    if tok in NEVER:
        return tok
    if BT.match(tok):
        return "btl" + tok[2:]
    if GIM.match(tok) or tok in EXTRA:
        return "btl_" + tok
    return tok


def rename_source(text):
    """Rename identifiers outside comments, strings and char literals."""
    out = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c == "/" and i + 1 < n and text[i + 1] == "/":
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(text[i:j]); i = j
        elif c == "/" and i + 1 < n and text[i + 1] == "*":
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(text[i:j]); i = j
        elif c == '"' or c == "'":
            j = i + 1
            while j < n and text[j] != c:
                if text[j] == "\\":
                    j += 1
                elif text[j] == "\n":
                    break
                j += 1
            j = min(j + 1, n)
            out.append(text[i:j]); i = j
        elif c.isalpha() or c == "_":
            m = IDENT.match(text, i)
            out.append(map_ident(m.group(0))); i = m.end()
        elif c.isdigit():
            m = re.compile(r"[0-9A-Za-z_.]+").match(text, i)
            out.append(m.group(0)); i = m.end()
        else:
            out.append(c); i += 1
    return "".join(out)


SRC_EXT = (".h", ".hpp", ".cpp", ".c", ".inl")


def rename_file(src, dst):
    with open(src, "r", encoding="latin-1") as f:
        text = f.read()
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(dst, "w", encoding="latin-1", newline="") as f:
        f.write(rename_source(text))


def do_tree(src_dir, dst_dir):
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir)
    count = 0
    for root, _, files in os.walk(src_dir):
        for name in files:
            s = os.path.join(root, name)
            d = os.path.join(dst_dir, os.path.relpath(s, src_dir))
            if name.endswith(SRC_EXT):
                rename_file(s, d); count += 1
            else:
                os.makedirs(os.path.dirname(d), exist_ok=True)
                shutil.copy2(s, d)
    print(f"renamed {count} source files into {dst_dir}")


def main(argv):
    argv = list(argv)
    while len(argv) > 2 and argv[1] == "--extra":
        for line in open(argv[2]):
            line = line.strip()
            if line and not line.startswith("#"):
                EXTRA.add(line)
        del argv[1:3]
    if len(argv) >= 4 and argv[1] == "tree":
        do_tree(argv[2], argv[3])
    elif len(argv) >= 4 and argv[1] == "files":
        dst = argv[2]
        for s in argv[3:]:
            rename_file(s, os.path.join(dst, os.path.basename(s)))
    else:
        print(__doc__); return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
