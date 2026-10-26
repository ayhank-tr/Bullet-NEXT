#!/bin/bash
# Bullet NEXT - T1.7: the F1 collision changes built as Chrono's cbt copy.
#
# Usage: run_chrono_check.sh <chrono_checkout> <configured_build_dir> <out.json>
#   The build dir is a Chrono build configured with -DBUILD_TESTING=ON (Ninja).
# Runs Chrono's collision and contact unit tests three times: stock Chrono, Chrono with
# integration/chrono/chrono-cbt.patch (features compiled in, off), and the same with
# CH_BN_NEXT_FEATURES=1 (hull clipping, active edges, SAT for triangles, mesh contact reduction).
set -u
CHRONO=$(cd "$1" && pwd)
BUILD=$(cd "$2" && pwd)
OUT=$3
HERE=$(cd "$(dirname "$0")" && pwd)
PATCH=$HERE/../integration/chrono/chrono-cbt.patch
TESTS="utest_COLL_bullet_utils utest_COLL_rounded_shapes utest_SMC_cohesion utest_SMC_cor_normal
utest_SMC_rolling_gravity utest_SMC_sliding_gravity utest_SMC_spinning_gravity utest_SMC_stacking
utest_SMC_sphere_sphere utest_CH_compute_contact utest_CH_assembly_contact"

run_all() {  # $1 = label; prints "name result" lines
  for t in $TESTS; do
    if (cd "$BUILD/bin" && timeout 1200 "./$t" > "$BUILD/$t.$1.log" 2>&1); then echo "$t pass"; else echo "$t fail"; fi
  done
}

git -C "$CHRONO" apply -R --check "$PATCH" 2>/dev/null && git -C "$CHRONO" apply -R "$PATCH"
ninja -C "$BUILD" $TESTS > "$BUILD/build_stock.log" 2>&1 || { echo "stock build failed"; exit 1; }
STOCK=$(run_all stock)

git -C "$CHRONO" apply "$PATCH" || { echo "patch does not apply"; exit 1; }
if ninja -C "$BUILD" $TESTS > "$BUILD/build_patched.log" 2>&1; then BUILT=true; else BUILT=false; fi
PATCHED=$(run_all patched)
FEAT=$(CH_BN_NEXT_FEATURES=1 run_all features)

# Same printed output (timings removed) as stock?
same() { cmp -s <(grep -v ' ms)\| ms total' "$BUILD/$1.stock.log") <(grep -v ' ms)\| ms total' "$BUILD/$1.$2.log") && echo yes || echo no; }
SAME=$(for t in $TESTS; do echo "$t $(same $t patched)/$(same $t features)"; done)

# Mesh slide (integration/chrono/bn_mesh_slide.cpp): ghost bumps with the features off and on
SLIDE=$BUILD/bn_mesh_slide
c++ -std=c++17 -O2 -w -I"$CHRONO/src" -I"$CHRONO/src/chrono/collision/bullet" -I"$BUILD" -I/usr/include/eigen3 \
  "$HERE/../integration/chrono/bn_mesh_slide.cpp" -L"$BUILD/lib" -lChrono_core -Wl,-rpath,"$BUILD/lib" -o "$SLIDE"
SLIDE_OFF=$("$SLIDE")
SLIDE_ON=$(CH_BN_NEXT_FEATURES=1 "$SLIDE")

COMMIT=$(git -C "$CHRONO" rev-parse HEAD)
python3 - "$OUT" "$COMMIT" "$BUILT" <<EOF
import json, sys
def parse(s): return dict(l.split() for l in s.strip().splitlines())
stock, patched, feat, same = parse("""$STOCK"""), parse("""$PATCHED"""), parse("""$FEAT"""), parse("""$SAME""")
tests = [{"name": n, "stock": stock[n], "patched": patched.get(n, "fail"), "patched_features": feat.get(n, "fail"),
          "same_output": same.get(n, "no/no")} for n in stock]
def slide(s):
    box, ball, x = (float(v) for v in s.split())
    return {"box_bump_mps": box, "ball_bump_mps": ball, "box_end_x_m": x}
json.dump({"chrono_commit": sys.argv[2], "patched_builds": sys.argv[3] == "true", "tests": tests,
           "mesh_slide": {"off": slide("""$SLIDE_OFF"""), "on": slide("""$SLIDE_ON""")}}, open(sys.argv[1], "w"), indent=1)
for t in tests: print(t["name"], t["stock"], t["patched"], t["patched_features"], t["same_output"])
print("mesh slide off:", """$SLIDE_OFF""", "on:", """$SLIDE_ON""")
EOF
