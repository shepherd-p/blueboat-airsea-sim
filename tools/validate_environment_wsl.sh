#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$LAB/install/setup.bash"
export ROS_DOMAIN_ID=81
for case in calm waves wind current combined; do
  export GZ_PARTITION="blueboat_environment_${case}"
  out="$LAB/results/environment/$case"
  python3 "$LAB/tools/make_scenario.py" --out "$out" --case "$case" --rtf 2
  setsid ros2 launch usv_control blueboat_bringup.launch.py world:="$out/world.sdf" > "$out/launch.log" 2>&1 &
  sim_pid=$!
  trap 'kill -INT -- "-$sim_pid" 2>/dev/null || true' EXIT
  set +e
  python3 "$LAB/tools/validate_environment.py" --out "$out" --case "$case" > "$out/validation.log" 2>&1
  code=$?
  set -e
  kill -INT -- "-$sim_pid" 2>/dev/null || true
  wait "$sim_pid" || true
  trap - EXIT
  cat "$out/validation.log"
  if [ "$code" -ne 0 ]; then exit "$code"; fi
done
