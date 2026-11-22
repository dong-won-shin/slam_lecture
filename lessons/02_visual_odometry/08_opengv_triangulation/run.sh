#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

exec ./test_triangulation "$@"
