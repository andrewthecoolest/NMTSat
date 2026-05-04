#!/usr/bin/env bash
set -e

REPO="$(cd "$(dirname "$0")" && pwd)"
BUILD="$REPO/build"

rm -f "$BUILD"/nmsat_sync "$BUILD"/libnmsat_core.a
rm -f "$REPO"/sync_sdr*.bin "$REPO"/sync_plot.png

cmake -S "$REPO" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" --target nmsat_sync -j"$(nproc)"

"$BUILD/nmsat_sync" || true

python3 "$REPO/tests/visualize_sync.py"
