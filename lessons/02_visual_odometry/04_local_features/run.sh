#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

./orb_cv ../data/1.png ../data/2.png
