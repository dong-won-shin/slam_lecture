#!/bin/bash

if [[ "$(uname -s)" == Darwin ]]; then
  set -euo pipefail
  cd "$(dirname "$0")"
  if [[ ! -x build/deps/bin/cmake || ! -d build/opengv-src/include ]]; then
    echo "Run ./build_3rdparty.sh first." >&2
    exit 1
  fi
  export PATH="$PWD/build/deps/bin:$PATH"
  cmake -S . -B build/macos -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PWD/build/deps"
  exec cmake --build build/macos --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
fi

# Build the Jetson lessons with their standalone CMake projects.
set -eu
cd "$(dirname "$0")"
for lesson in lessons/*/*; do
  # The OpenGV extension has its own dependency on Linux.
  [[ "$lesson" == */07_opengv_triangulation ]] && continue
  mkdir -p "$lesson/build"
  (cd "$lesson/build" && cmake .. && cmake --build .)
done
