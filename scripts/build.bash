#!/usr/bin/env bash
set -eo pipefail
export BOT_SKIP_OVERLAY=1
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
unset BOT_SKIP_OVERLAY
cd "$BOT_WORKSPACE"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
colcon build --symlink-install --base-paths src --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "$@"
