#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
cd "$SIM_WORKSPACE"
if [[ ! -f install/social_nav_bringup/share/social_nav_bringup/package.xml ]]; then
  echo 'Build the workspace first: bash scripts/build.bash' >&2
  exit 1
fi
exec ros2 launch social_nav_bringup simulation.launch.py "$@"
