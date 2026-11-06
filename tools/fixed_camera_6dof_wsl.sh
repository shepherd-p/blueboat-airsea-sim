#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
state="${1:-2}"
mode="${2:-full}"
source /opt/ros/jazzy/setup.bash
export BLUEBOAT_SITL_OUT="$LAB/results/fixed_camera_6dof_20261102/state_${state}_${mode}"
export BLUEBOAT_SITL_WORLD="$BLUEBOAT_SITL_OUT/world.sdf"
export BLUEBOAT_HEADLESS=false BLUEBOAT_SITL_PORT=9052 BLUEBOAT_MAVLINK_PORT=5810 BLUEBOAT_SITL_INSTANCE=5
export BLUEBOAT_ROS_DOMAIN_ID=89 BLUEBOAT_GZ_PARTITION="blueboat_fixed_6dof_${state}_${mode}"
export ROS_DOMAIN_ID="$BLUEBOAT_ROS_DOMAIN_ID" GZ_PARTITION="$BLUEBOAT_GZ_PARTITION"
mkdir -p "$BLUEBOAT_SITL_OUT"
if [ -f "$BLUEBOAT_SITL_OUT/pose_samples.csv" ]; then
  echo "Existing capture retained; use a new mode/output name for another run." >&2
  exit 1
fi
python3 "$LAB/tools/fixed_camera_6dof.py" prepare --state "$state" --out "$BLUEBOAT_SITL_OUT"
bash "$LAB/tools/run_sitl_wsl.sh" > "$BLUEBOAT_SITL_OUT/session.log" 2>&1 &
session_pid=$!
cleanup(){ kill -TERM "$session_pid" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
args=()
if [ "$mode" = pilot ]; then args+=(--pilot); fi
set +e
"$HOME/blueboat_sitl/venv/bin/python" "$LAB/tools/fixed_camera_6dof.py" record --state "$state" --out "$BLUEBOAT_SITL_OUT" "${args[@]}" > "$BLUEBOAT_SITL_OUT/recording.log" 2>&1
code=$?
set -e
cleanup
wait "$session_pid" || true
cat "$BLUEBOAT_SITL_OUT/recording.log"
exit "$code"
