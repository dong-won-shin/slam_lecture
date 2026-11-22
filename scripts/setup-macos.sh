#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
root=$PWD
case "$(uname -m)" in
  arm64)
    platform=osx-arm64
    digest=acef4eaab19ec2174ad5a449e5aad981ca326270db819502ff096a92fc31caa6 ;;
  x86_64)
    platform=osx-64
    digest=17b20c4271d65c030a5c86788f5ca86eb575187209f626edabdb8c6a875c6838 ;;
  *) echo "Unsupported macOS architecture: $(uname -m)" >&2; exit 1 ;;
esac

tools_dir="$root/build/tools"
mamba="$tools_dir/bin/micromamba"
mkdir -p "$tools_dir"
if [[ ! -x "$mamba" ]]; then
  archive=$(mktemp "$tools_dir/micromamba.XXXXXX")
  trap 'rm -f "$archive"' EXIT
  curl --fail --location --retry 3 \
    "https://conda.anaconda.org/conda-forge/$platform/micromamba-2.10.0-0.tar.bz2" \
    --output "$archive"
  echo "$digest  $archive" | LC_ALL=C shasum -a 256 -c -
  tar -xjf "$archive" -C "$tools_dir" bin/micromamba
fi

action=create
if [[ -f build/deps/conda-meta/history ]]; then action=install; fi
"$mamba" --no-rc "$action" -y --root-prefix "$root/build/mamba" --prefix "$root/build/deps" \
  --override-channels -c conda-forge cmake ninja 'eigen=3.4.0' 'libopencv=4.13.*=qt6*' \
  'pcl=1.15.1' 'ceres-solver=2.2.0'

opengv="$root/build/opengv-src"
revision=91f4b19c73450833a40e463ad3648aae80b3a7f3
if [[ ! -d "$opengv/.git" ]]; then
  git init -q "$opengv"
  git -C "$opengv" remote add origin https://github.com/laurentkneip/opengv.git
fi
if ! git -C "$opengv" rev-parse --verify HEAD >/dev/null 2>&1; then
  git -C "$opengv" fetch --depth 1 origin "$revision"
  git -C "$opengv" checkout --detach FETCH_HEAD
fi
if [[ "$(git -C "$opengv" rev-parse HEAD)" != "$revision" ]]; then
  echo "Expected OpenGV $revision in $opengv. Move the existing checkout and rerun setup." >&2
  exit 1
fi
