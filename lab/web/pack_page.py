#!/usr/bin/env python3
"""Bullet NEXT Lab - packs the browser viewer into one page (FL, CL.10).

Usage: pack_page.py <bnl_viewer.js> <out.html> [--standalone]

bnl_viewer.js is the Emscripten build of lab/viewer/bnl_viewer.cpp (one file: program, scenes
and shaders inside). The page is lab/web/shell.html with the program in place of /*BNL_PROGRAM*/.
Without --standalone the page has no <html>/<head>/<body> (the artifact host adds them);
--standalone adds them, for opening the file straight in a browser.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    src, out = sys.argv[1], sys.argv[2]
    standalone = "--standalone" in sys.argv[3:]
    shell = open(os.path.join(HERE, "shell.html"), encoding="utf-8").read()
    program = open(src, encoding="utf-8").read().replace("</script", "<\\/script")
    page = shell.replace("/*BNL_PROGRAM*/", program, 1)
    if standalone:
        page = ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
                '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
                '<style>body{margin:0}</style>\n</head>\n<body>\n' + page + '\n</body>\n</html>\n')
    with open(out, "w", encoding="utf-8") as f:
        f.write(page)
    print(f"wrote {out} ({os.path.getsize(out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
