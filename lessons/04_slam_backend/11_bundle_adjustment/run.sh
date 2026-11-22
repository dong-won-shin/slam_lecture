#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

./bundle_adjustment_ceres ../data/problem-16-22106-pre.txt
