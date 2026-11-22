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

build_function ()
{
    sudo rm -rf build
    mkdir build
    cd build
    cmake ../
    make -j1
}

ROOT_DIR=$PWD
echo $ROOT_DIR

# build rgbd slam examples
cd "$ROOT_DIR/3DPointCameraProjection"
build_function

cd "$ROOT_DIR/3DPointCloudProcessing"
build_function

cd "$ROOT_DIR/3DPointCloudRegistration"
build_function

cd "$ROOT_DIR/BundleAdjustment"
build_function

cd "$ROOT_DIR/Fundamental&EssentialMatrix"
build_function

cd "$ROOT_DIR/GraphOptimization"
build_function

cd "$ROOT_DIR/KalmanFilter"
build_function

cd "$ROOT_DIR/Keypoint&Descriptor"
build_function

cd "$ROOT_DIR/LeastSquares"
build_function

cd "$ROOT_DIR/LoopClosureDetection"
build_function

cd "$ROOT_DIR/Perspective3Points"
build_function

cd "$ROOT_DIR/Triangulation"
build_function
