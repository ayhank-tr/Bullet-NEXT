#!/bin/bash
# Bullet NEXT Lab - builds the Chrono Lab engine (FL, CL.6) against a Chrono build that carries
# integration/chrono/chrono-cbt.patch (see tools/run_chrono_check.sh).
# Usage: build_chrono_engine.sh <chrono_checkout> <chrono_build_dir> <output_binary>
# Stage 1: use a Chrono build configured with -DCH_USE_SIMD=OFF. With SIMD on (the default, which
# compiles for the build machine's CPU), Chrono's results depended on what was in memory before:
# the bounce scene gave three different results with different folder names or MALLOC_PERTURB_
# values; without SIMD they repeat. A reference engine has to repeat, speed does not matter here.
set -eu
CHRONO=$(cd "$1" && pwd)
BUILD=$(cd "$2" && pwd)
OUT=$3
HERE=$(cd "$(dirname "$0")" && pwd)
LAB=$HERE/../lab
c++ -std=c++17 -O2 -w -fno-fast-math -ffp-contract=off \
  -I"$CHRONO/src" -I"$CHRONO/src/chrono_thirdparty/yaml-cpp/include" -I"$CHRONO/src/chrono/collision/bullet" -I"$BUILD" -I/usr/include/eigen3 \
  -I"$LAB/core" -I"$HERE/../third_party" \
  "$LAB/engines/bnl_chrono.cpp" "$LAB/core/bnl_scene.cpp" "$LAB/core/bnl_mesh.cpp" "$LAB/core/bnl_recording.cpp" \
  -L"$BUILD/lib" -lChrono_core -Wl,-rpath,"$BUILD/lib" -o "$OUT"
echo "built $OUT"
