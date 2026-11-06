#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$LAB/install/setup.bash"
export GZ_PARTITION=blueboat_propulsion_check
export GZ_SIM_RESOURCE_PATH="$LAB/install/share/blueboat_description/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$LAB/install/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
mkdir -p "$LAB/results"
setsid gz sim -s -r "$LAB/install/share/blueboat_description/worlds/blueboat_water.sdf" > "$LAB/results/propulsion_launch.log" 2>&1 &
curve_pid=$!
cleanup(){ kill -INT -- "-$curve_pid" 2>/dev/null || true; }
trap cleanup EXIT
python3 "$LAB/tools/validate_propulsion.py"
