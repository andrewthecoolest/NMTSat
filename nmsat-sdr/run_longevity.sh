#!/usr/bin/env bash
set -e

REPO="$(cd "$(dirname "$0")" && pwd)"
BUILD="$REPO/build"

cmake -S "$REPO" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" --target nmsat_longevity -j"$(nproc)"

"$BUILD/nmsat_longevity"
