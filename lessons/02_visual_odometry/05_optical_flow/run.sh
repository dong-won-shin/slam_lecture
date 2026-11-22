#!/bin/bash
set -eu
lesson=$(cd "$(dirname "$0")" && pwd)

case "${1:-lk}" in
  lk)  program=optical_flow ;;
  dis) program=dis_optical_flow ;;
  *)   echo "Usage: $0 [lk|dis] [--video [input_video] [--output result.mp4]]" >&2; exit 1 ;;
esac
if [ "$#" -gt 0 ]; then shift; fi

if [ "$#" -eq 0 ]; then
  cd "$lesson/build"
  exec "./$program" ../data/1.png ../data/2.png
fi
if [ "$1" = --video ]; then
  shift
  if [ "$#" -eq 0 ] || [ "$1" = --output ]; then
    set -- "$lesson/data/freiburg1_xyz.mp4" "$@"
  fi
  exec "$lesson/build/$program" --video "$@"
fi
echo "Usage: $0 [lk|dis] [--video [input_video] [--output result.mp4]]" >&2
exit 1
