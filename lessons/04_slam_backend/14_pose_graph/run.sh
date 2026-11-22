#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

case "${1:-kitti}" in
  kitti) exec ./pose_graph_g2o_SE3 ../data/kitti_00.g2o --trajectory ;;
  sphere) exec ./pose_graph_g2o_SE3 ../data/sphere.g2o ;;
  *) echo "Usage: $0 [kitti|sphere]" >&2; exit 1 ;;
esac
