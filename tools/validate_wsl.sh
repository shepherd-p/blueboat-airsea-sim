#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$LAB/install/setup.bash"
# Independent discovery/transport domain keeps this test separate from other labs.
export ROS_DOMAIN_ID=74
export GZ_PARTITION=blueboat_validation
mkdir -p "$LAB/results/baseline"
setsid bash "$LAB/tools/run_wsl.sh" headless:=true > "$LAB/results/baseline/launch.log" 2>&1 &
sim_pid=$!
cleanup() { kill -INT -- "-$sim_pid" 2>/dev/null || true; }
trap cleanup EXIT
python3 "$LAB/tools/validate_ros.py" --out "$LAB/results/baseline" 2>&1 | tee "$LAB/results/baseline/validation.log"
