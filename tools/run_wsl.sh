#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$LAB/install/setup.bash"
exec ros2 launch usv_control blueboat_bringup.launch.py "$@"
