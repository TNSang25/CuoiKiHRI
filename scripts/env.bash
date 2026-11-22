#!/usr/bin/env bash
# Source this file before running the workspace packages.
export SIM_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f /usr/share/gazebo/setup.sh ]]; then
  source /usr/share/gazebo/setup.sh
fi

_deps="$SIM_WORKSPACE/.deps/root"
if [[ -d "$_deps/opt/ros/humble" ]]; then
  export AMENT_PREFIX_PATH="$_deps/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$_deps/opt/ros/humble:$_deps/usr:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$_deps/opt/ros/humble/lib:$_deps/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$_deps/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
  export PATH="$_deps/opt/ros/humble/bin:$PATH"
  export PKG_CONFIG_PATH="$_deps/usr/lib/x86_64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH:-}"
  export CPATH="$_deps/usr/include:${CPATH:-}"
  export LIBRARY_PATH="$_deps/usr/lib/x86_64-linux-gnu:${LIBRARY_PATH:-}"
  export GAZEBO_PLUGIN_PATH="$_deps/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"
fi
unset _deps

export ROS_DOMAIN_ID="${SIM_ROS_DOMAIN_ID:-65}"
export GAZEBO_MASTER_URI="${SIM_GAZEBO_MASTER_URI:-http://localhost:11365}"
export GAZEBO_MODEL_DATABASE_URI=""
export SIM_RUNTIME_DIR="${SIM_RUNTIME_DIR:-$SIM_WORKSPACE/.runtime}"
export SIM_RESULTS_DIR="${SIM_RESULTS_DIR:-$SIM_RUNTIME_DIR/results}"
export ROS_LOG_DIR="$SIM_RUNTIME_DIR/ros-log"
mkdir -p "$ROS_LOG_DIR" "$SIM_RESULTS_DIR"
if [[ "${SIM_SKIP_OVERLAY:-0}" != 1 && -f "$SIM_WORKSPACE/install/local_setup.bash" ]]; then
  source "$SIM_WORKSPACE/install/local_setup.bash"
fi
