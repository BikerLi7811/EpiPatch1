#!/usr/bin/env bash
# Builds the device's algorithm code for the PC with zig (pip install ziglang)
# against upstream CMSIS-DSP, generates synthetic recordings and runs the tests.
# usage: build.sh <dir containing CMSIS-DSP> [vector dir]
set -euo pipefail
V=${1:?path to dir with the CMSIS-DSP checkout}
H=$(cd "$(dirname "$0")" && (pwd -W 2>/dev/null || pwd))
SRC="$H/../../src"
OBJ="$H/obj"
VEC=${2:-$H/vectors}
DSP="$V/CMSIS-DSP/Source"
INC=(-I"$SRC" -I"$V/CMSIS-DSP/Include" -I"$V/CMSIS-DSP/PrivateInclude")
mkdir -p "$OBJ"

LIB="$OBJ/libcmsisdsp.a"
if [ ! -f "$LIB" ]; then
  for f in BasicMathFunctions/BasicMathFunctions.c FilteringFunctions/FilteringFunctions.c \
           CommonTables/CommonTables.c FastMathFunctions/FastMathFunctions.c \
           SupportFunctions/SupportFunctions.c StatisticsFunctions/StatisticsFunctions.c; do
    python -m ziglang cc -O2 -std=c11 -D__GNUC_PYTHON__ -w "${INC[@]}" -c "$DSP/$f" -o "$OBJ/$(basename "$f" .c).o"
  done
  python -m ziglang ar rcs "$LIB" "$OBJ"/*.o
fi

SRCS=("$H/host_tests.cpp" "$SRC/algo/anaphylaxis.cpp" "$SRC/algo/body_motion.cpp" "$SRC/algo/ecg_qrs.cpp"
      "$SRC/algo/ppg.cpp" "$SRC/algo/resp_rate.cpp" "$SRC/burst.cpp")
CXX=(python -m ziglang c++ -O2 -std=c++20 -D__GNUC_PYTHON__ -Wall -Wextra -Wno-nullability-completeness)

# 1. as on the application core (CMSIS-DSP biquads)
"${CXX[@]}" "${INC[@]}" "${SRCS[@]}" "$LIB" -o "$H/host_tests.exe"
# 2. as on the FLPR coprocessor (plain C++ biquads, no CMSIS-DSP)
"${CXX[@]}" -DDEV_PORTABLE_DSP -I"$SRC" "${SRCS[@]}" -o "$H/host_tests_portable.exe"

python -I "$H/gen_vectors.py" "$VEC"
echo "=== application core build ==="
"$H/host_tests.exe" "$VEC"
echo "=== FLPR coprocessor build (portable DSP) ==="
"$H/host_tests_portable.exe" "$VEC"
