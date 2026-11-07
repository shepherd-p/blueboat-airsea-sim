#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source "$LAB/tools/graphics_wsl.sh"
source /opt/ros/jazzy/setup.bash
"${BLUEBOAT_PYTHON:-$HOME/blueboat_sitl/venv/bin/python}" "$LAB/tools/airsea/replay.py" "$@"
