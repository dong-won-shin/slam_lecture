#!/bin/bash
set -eu
cd "$(dirname "$0")/build"

case "${1:-lk}" in
  lk)  exec ./optical_flow ../data/1.png ../data/2.png ;;
  dis) exec ./dis_optical_flow ../data/1.png ../data/2.png ;;
  *)   echo "Usage: $0 [lk|dis]" >&2; exit 1 ;;
esac
