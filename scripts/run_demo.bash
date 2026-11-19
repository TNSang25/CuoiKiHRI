#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.bash"
cd "$BOT_WORKSPACE"
if [[ ! -f install/bot_bringup/share/bot_bringup/package.xml ]]; then
  echo 'Build the workspace first: bash scripts/build.bash' >&2
  exit 1
fi
exec ros2 launch bot_bringup simulation.launch.py "$@"
