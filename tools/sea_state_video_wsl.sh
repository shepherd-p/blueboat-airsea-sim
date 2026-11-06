#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
state="${1:-1}"
source /opt/ros/jazzy/setup.bash
export BLUEBOAT_SITL_OUT="$LAB/results/sea_states_20261101/video_state_$state"
export BLUEBOAT_SITL_WORLD="$BLUEBOAT_SITL_OUT/world.sdf"
export BLUEBOAT_HEADLESS=false BLUEBOAT_SITL_PORT=9042 BLUEBOAT_MAVLINK_PORT=5800 BLUEBOAT_SITL_INSTANCE=4
export BLUEBOAT_ROS_DOMAIN_ID=88 BLUEBOAT_GZ_PARTITION="blueboat_sea_state_video_$state"
export ROS_DOMAIN_ID="$BLUEBOAT_ROS_DOMAIN_ID" GZ_PARTITION="$BLUEBOAT_GZ_PARTITION"
mkdir -p "$BLUEBOAT_SITL_OUT"
python3 "$LAB/tools/sea_state_video.py" prepare --state "$state"
bash "$LAB/tools/run_sitl_wsl.sh" > "$BLUEBOAT_SITL_OUT/session.log" 2>&1 &
session_pid=$!
cleanup(){ kill -TERM "$session_pid" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
set +e
"$HOME/blueboat_sitl/venv/bin/python" "$LAB/tools/sea_state_video.py" record --state "$state" > "$BLUEBOAT_SITL_OUT/recording.log" 2>&1
code=$?
set -e
cleanup
wait "$session_pid" || true
cat "$BLUEBOAT_SITL_OUT/recording.log"
exit "$code"
