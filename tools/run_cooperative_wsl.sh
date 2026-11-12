#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source "$LAB/tools/graphics_wsl.sh"
source /opt/ros/jazzy/setup.bash
source "$LAB/install/setup.bash"
export ROS_DOMAIN_ID=94 GZ_PARTITION=blueboat_airsea
export GZ_SIM_RESOURCE_PATH="$LAB/h-spar-sim/ros2_ws/src/blueboat_description/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$LAB/build/airsea_native:$LAB/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_GUI_PLUGIN_PATH="$LAB/build/airsea_native:$LAB/install/lib:${GZ_GUI_PLUGIN_PATH:-}"
"${BLUEBOAT_PYTHON:-$HOME/blueboat_sitl/venv/bin/python}" "$LAB/tools/cooperative/session.py" "$@"
