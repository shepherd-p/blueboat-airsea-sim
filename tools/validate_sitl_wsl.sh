#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${BLUEBOAT_ROS_DOMAIN_ID:-82}"
export GZ_PARTITION="${BLUEBOAT_GZ_PARTITION:-blueboat_sitl}"
"${BLUEBOAT_SITL_RUNTIME:-$HOME/blueboat_sitl}/venv/bin/python" "$LAB/tools/validate_sitl.py" "$@"
