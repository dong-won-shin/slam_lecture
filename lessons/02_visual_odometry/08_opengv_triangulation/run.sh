#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

./test_triangulation
