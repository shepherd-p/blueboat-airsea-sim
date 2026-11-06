#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$LAB/install/setup.bash"
cmake -S "$LAB/tools/airsea/native" -B "$LAB/build/airsea_native" -DCMAKE_BUILD_TYPE=Release
cmake --build "$LAB/build/airsea_native" -j2
