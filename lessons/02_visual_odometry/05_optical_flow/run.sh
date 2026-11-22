#!/bin/bash
set -eu
lesson=$(cd "$(dirname "$0")" && pwd)

case "${1:-lk}" in
  lk)  program=optical_flow ;;
  dis) program=dis_optical_flow ;;
  flownet)
    shift
    python=${FLOWNET_PYTHON:-"$lesson/build/flownet-env/bin/python"}
    if [[ -z "${FLOWNET_PYTHON:-}" && ! -x "$python" ]]; then
      python="$lesson/../04_local_features/build/superpoint-env/bin/python"
      [[ -x "$python" ]] || python=python3
    fi
    exec "$python" "$lesson/src/flownet.py" "$@"
    ;;
  *)   echo "Usage: $0 [lk|dis|flownet] [--video [input_video] [--output result.mp4]]" >&2; exit 1 ;;
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
echo "Usage: $0 [lk|dis|flownet] [--video [input_video] [--output result.mp4]]" >&2
exit 1
