#!/usr/bin/env python3
"""Bullet NEXT - writes the hexadecimal float literals of third_party/musl_math as exact decimals.

Usage: musl_decimal_literals.py <file>...   (rewrites the files in place)

musl writes constants such as 0x1.921fb6p-1 (C99 hexadecimal floats), which Visual Studio's C
compiler does not accept in every version the game engine is built with. Each literal is
replaced by the exact decimal value of the same binary number (every binary fraction has a finite
decimal expansion), so the compiled constants are bit for bit the same. A float literal (suffix f)
must be exactly representable as a 32-bit float; the script stops if one is not.
"""
import re
import struct
import sys
from decimal import Decimal, getcontext

getcontext().prec = 2000
HEX = re.compile(r"0[xX]([0-9a-fA-F]*)(?:\.([0-9a-fA-F]*))?[pP]([+-]?\d+)([fFlL]?)")


def exact_decimal(m):
    whole, frac, exp, suffix = m.group(1), m.group(2) or "", int(m.group(3)), m.group(4)
    text = "0x" + (whole or "0") + "." + (frac or "0") + "p" + str(exp)
    v = float.fromhex(text)
    if suffix in "fF" and suffix:
        if struct.unpack("f", struct.pack("f", v))[0] != v:
            raise SystemExit(f"{m.group(0)} is not exactly a 32-bit float")
    if suffix in "lL" and suffix:
        raise SystemExit(f"{m.group(0)}: long double literals are not handled")
    d = "{:E}".format(Decimal(v))
    mant, e = d.split("E")
    if "." not in mant:
        mant += ".0"
    return mant + "E" + str(int(e)) + suffix


def main():
    for path in sys.argv[1:]:
        src = open(path, encoding="utf-8").read()
        out, n = HEX.subn(exact_decimal, src)
        if n:
            open(path, "w", encoding="utf-8").write(out)
        print(f"{path}: {n} literals")


if __name__ == "__main__":
    main()
