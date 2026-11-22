#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

./triangulation ../data/1.png ../data/2.png
