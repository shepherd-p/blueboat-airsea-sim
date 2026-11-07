#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source "$LAB/tools/graphics_wsl.sh"
source /opt/ros/jazzy/setup.bash
python3 "$LAB/tools/airsea/preview.py" "$@"
