#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

./pose_graph_g2o_SE3 ../data/sphere.g2o
