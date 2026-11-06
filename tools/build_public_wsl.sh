#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
python3 "$LAB/tools/generate_public_assets.py"
python3 "$LAB/tools/generate_model.py"
python3 "$LAB/tools/generate_environment.py"
python3 "$LAB/tools/sea_states.py" --generate
bash "$LAB/tools/build_wsl.sh"
cmake -S "$LAB/tools/video_recorder" -B "$LAB/results/video_demo/recorder_build"
cmake --build "$LAB/results/video_demo/recorder_build" -j2
