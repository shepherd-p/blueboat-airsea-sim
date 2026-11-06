#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$LAB/install/setup.bash"
export ROS_DOMAIN_ID=75
export GZ_PARTITION=blueboat_video_demo
export GZ_GUI_PLUGIN_PATH="$LAB/results/video_demo/recorder_build:${GZ_GUI_PLUGIN_PATH:-}"
cd "$LAB/results/video_demo"
exec bash "$LAB/tools/run_wsl.sh" headless:=false world:="$LAB/results/video_demo/recording_world.sdf"
