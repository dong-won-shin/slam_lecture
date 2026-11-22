#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

./gauss_newton
./g2o_curve_fitting
./ceres_curve_fitting
