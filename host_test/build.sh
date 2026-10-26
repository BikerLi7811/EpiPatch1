#!/usr/bin/env bash
# Builds the firmware's C++ DSP+NN code for the PC with zig (pip install ziglang)
# against upstream CMSIS-DSP / CMSIS-NN (compiled as C).
# usage: build.sh <dir containing CMSIS-DSP, CMSIS-NN>
set -euo pipefail
V=${1:?path to dir with CMSIS-DSP and CMSIS-NN checkouts}
H=$(cd "$(dirname "$0")" && pwd)
FW="$H/../wheeze/src"
OBJ="$H/obj"
DSP="$V/CMSIS-DSP/Source"
INC=(-I"$FW" -I"$V/CMSIS-DSP/Include" -I"$V/CMSIS-DSP/PrivateInclude" -I"$V/CMSIS-NN/Include")
mkdir -p "$OBJ"

# CMSIS libraries (C), built once and cached
C_SRC=(
  "$DSP/BasicMathFunctions/BasicMathFunctions.c"
  "$DSP/ComplexMathFunctions/ComplexMathFunctions.c"
  "$DSP/FilteringFunctions/FilteringFunctions.c"
  "$DSP/StatisticsFunctions/StatisticsFunctions.c"
  "$DSP/SupportFunctions/SupportFunctions.c"
  "$DSP/TransformFunctions/TransformFunctions.c"
  "$DSP/CommonTables/CommonTables.c"
  "$DSP/FastMathFunctions/FastMathFunctions.c"
  "$DSP/MatrixFunctions/MatrixFunctions.c"
)
mapfile -t -O "${#C_SRC[@]}" C_SRC < <(find "$V/CMSIS-NN/Source" -name '*.c' ! -name '*f16*' ! -name '*f32*')
LIB="$OBJ/libcmsis.a"
if [ ! -f "$LIB" ]; then
  for f in "${C_SRC[@]}"; do
    python -m ziglang cc -O2 -std=c11 -D__GNUC_PYTHON__ -w "${INC[@]}" -c "$f" -o "$OBJ/$(basename "$f" .c).o"
  done
  python -m ziglang ar rcs "$LIB" "$OBJ"/*.o
fi

# Firmware C++ (unmodified) + host driver
python -m ziglang c++ -O2 -std=c++20 -D__GNUC_PYTHON__ -Wall -Wextra -Wno-unused-parameter -Wno-nullability-completeness "${INC[@]}" \
  "$H/host_main.cpp" "$FW/feature_extractor.cpp" "$FW/wheeze_net.cpp" \
  "$FW/generated/dsp_tables.cpp" "$FW/generated/model_data.cpp" \
  "$LIB" -o "$H/host_test.exe"
echo "built $H/host_test.exe"
