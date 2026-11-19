#!/usr/bin/env bash
# Source this file before running the local bot_* packages.
export BOT_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f /usr/share/gazebo/setup.sh ]]; then
  source /usr/share/gazebo/setup.sh
fi

_bot_deps="$BOT_WORKSPACE/.deps/root"
if [[ -d "$_bot_deps/opt/ros/humble" ]]; then
  export AMENT_PREFIX_PATH="$_bot_deps/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$_bot_deps/opt/ros/humble:$_bot_deps/usr:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$_bot_deps/opt/ros/humble/lib:$_bot_deps/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$_bot_deps/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
  export PATH="$_bot_deps/opt/ros/humble/bin:$PATH"
  export PKG_CONFIG_PATH="$_bot_deps/usr/lib/x86_64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH:-}"
  export CPATH="$_bot_deps/usr/include:${CPATH:-}"
  export LIBRARY_PATH="$_bot_deps/usr/lib/x86_64-linux-gnu:${LIBRARY_PATH:-}"
  export GAZEBO_PLUGIN_PATH="$_bot_deps/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"
fi
unset _bot_deps

export ROS_DOMAIN_ID="${BOT_ROS_DOMAIN_ID:-65}"
export GAZEBO_MASTER_URI="${BOT_GAZEBO_MASTER_URI:-http://localhost:11365}"
export GAZEBO_MODEL_DATABASE_URI=""
export BOT_RUNTIME_DIR="${BOT_RUNTIME_DIR:-$BOT_WORKSPACE/.runtime}"
export BOT_RESULTS_DIR="${BOT_RESULTS_DIR:-$BOT_RUNTIME_DIR/results}"
export ROS_LOG_DIR="$BOT_RUNTIME_DIR/ros-log"
mkdir -p "$ROS_LOG_DIR" "$BOT_RESULTS_DIR"
if [[ "${BOT_SKIP_OVERLAY:-0}" != 1 && -f "$BOT_WORKSPACE/install/local_setup.bash" ]]; then
  source "$BOT_WORKSPACE/install/local_setup.bash"
fi
