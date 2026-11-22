#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

./optical_flow ../data/1.png ../data/2.png
