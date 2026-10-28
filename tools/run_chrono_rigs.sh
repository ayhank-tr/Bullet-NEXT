#!/bin/sh
# Bullet NEXT (Stage 1, C1S.6) - builds and runs the Chrono vehicle rigs (integration/chrono/bn_vehicle_rigs.cpp).
#
# Usage: run_chrono_rigs.sh <chrono_checkout> <chrono_core_build> <out.json>
#   <chrono_core_build>: a Chrono build with lib/libChrono_core.so (tools/run_chrono_check.sh makes one)
# The rigs need ten files of Chrono::Vehicle (the tyre classes and what they use). They are compiled
# from <chrono_checkout>/build_vehicle, configured with the vehicle module on, so the rest of the
# module is not built. The Pacejka 2002 sedan tyre file comes from Chrono's data folder at the same
# commit (da80252), fetched once into <chrono_checkout>/build_vehicle.
set -e
CHRONO=$(cd "$1" && pwd)
CORE=$(cd "$2" && pwd)
OUT=$3
HERE=$(cd "$(dirname "$0")/.." && pwd)
BV="$CHRONO/build_vehicle"

if [ ! -f "$BV/build.ninja" ]; then
  cmake -S "$CHRONO" -B "$BV" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCH_ENABLE_MODULE_VEHICLE=ON \
        -DCH_ENABLE_MODULE_VEHICLE_MODELS=ON -DCH_ENABLE_OPENMP=ON -DBUILD_DEMOS=OFF -DBUILD_TESTING=OFF \
        -DBUILD_BENCHMARKING=OFF > "$BV.cmake.log"
fi
D=src/chrono_vehicle/CMakeFiles/Chrono_vehicle.dir
OBJS=""
for f in wheeled_vehicle/tire/ChTMeasyTire wheeled_vehicle/tire/ChPac02Tire wheeled_vehicle/tire/ChForceElementTire \
         wheeled_vehicle/ChTire wheeled_vehicle/ChWheel wheeled_vehicle/ChSpindle ChPart ChVehicleDataPath \
         ChSubsysDefs ChTerrain ChWorldFrame; do
  OBJS="$OBJS $D/$f.cpp.o"
done
(cd "$BV" && ninja -j2 $OBJS > /dev/null)

TIR="$BV/Sedan_Pac02Tire.tir"
if [ ! -f "$TIR" ]; then
  curl -sSfL -o "$TIR" https://raw.githubusercontent.com/projectchrono/chrono/da80252/data/vehicle/sedan/tire/Sedan_Pac02Tire.tir
fi

BIN="$BV/bn_vehicle_rigs"
LINK=""
for o in $OBJS; do LINK="$LINK $BV/$o"; done
g++ -O3 -march=native -std=gnu++17 -fopenmp -DCH_IGNORE_DEPRECATED -DNDEBUG -D_ENABLE_EXTENDED_ALIGNED_STORAGE \
    -I"$CHRONO/src" -I"$BV" -I"$CHRONO/src/chrono_thirdparty/yaml-cpp/include" \
    -isystem "$CHRONO/src/chrono/collision/bullet" -isystem "$CHRONO/src/chrono_thirdparty" -isystem /usr/include/eigen3 \
    "$HERE/integration/chrono/bn_vehicle_rigs.cpp" $LINK -L"$CORE/lib" -lChrono_core -Wl,-rpath,"$CORE/lib" -o "$BIN"
"$BIN" "$TIR" > "$OUT" 2> "$OUT.log"
echo "wrote $OUT"
