#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

./pose_estimation_2d2d ../data/1.png ../data/2.png
