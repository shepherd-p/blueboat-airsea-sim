#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$LAB/h-spar-sim/ros2_ws"
# Limit build to vehicle modeling/control; the external current database is not required.
colcon build --packages-select blueboat_gazebo blueboat_description usv_control --merge-install \
  --build-base "$LAB/build" --install-base "$LAB/install" \
  --cmake-args -DBUILD_TESTING=OFF
echo 'Build complete. Launch with tools/run_wsl.sh.'
