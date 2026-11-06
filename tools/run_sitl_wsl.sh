#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
RUNTIME="${BLUEBOAT_SITL_RUNTIME:-$HOME/blueboat_sitl}"
OUT="${BLUEBOAT_SITL_OUT:-$LAB/results/sitl}"
HEADLESS="${BLUEBOAT_HEADLESS:-true}"
source /opt/ros/jazzy/setup.bash
source "$LAB/install/setup.bash"
export ROS_DOMAIN_ID="${BLUEBOAT_ROS_DOMAIN_ID:-82}"
export GZ_PARTITION="${BLUEBOAT_GZ_PARTITION:-blueboat_sitl}"
mkdir -p "$OUT"
if [ -z "${BLUEBOAT_SITL_WORLD:-}" ]; then
  python3 "$LAB/tools/make_scenario.py" --out "$OUT" --case combined --sitl
fi
export GZ_GUI_PLUGIN_PATH="$LAB/results/video_demo/recorder_build:${GZ_GUI_PLUGIN_PATH:-}"
cd "$OUT"
setsid ros2 launch usv_control blueboat_bringup.launch.py headless:="$HEADLESS" controller:=false world:="${BLUEBOAT_SITL_WORLD:-$OUT/world.sdf}" > "$OUT/gazebo.log" 2>&1 &
sim_pid=$!
cleanup(){ kill -INT -- "-$sim_pid" 2>/dev/null || true; kill -TERM "${sitl_pid:-}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
cd "$OUT"
"$RUNTIME/ardurover" --model JSON --home 51.5246,-0.1340,10,90 --rate 250 --speedup 1 --instance "${BLUEBOAT_SITL_INSTANCE:-0}" --sim-port-out "${BLUEBOAT_SITL_PORT:-9002}" --defaults "$LAB/h-spar-sim/ros2_ws/src/blueboat_description/config/ardurover.parm" --serial0 "tcp:${BLUEBOAT_MAVLINK_PORT:-5760}" --serial1 none --serial2 none --wipe > "$OUT/ardurover.log" 2>&1 &
sitl_pid=$!
echo "$sitl_pid" > "$OUT/ardurover.pid"
wait "$sitl_pid"
