#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
for state in "${@:-1 2}"; do
  # With no arguments use both states (word splitting intentionally below).
  for number in $state; do
    export BLUEBOAT_SITL_OUT="$LAB/results/sea_states_20261101/sea_state_$number"
    export BLUEBOAT_SITL_PORT=9032 BLUEBOAT_MAVLINK_PORT=5790 BLUEBOAT_SITL_INSTANCE=3
    export BLUEBOAT_ROS_DOMAIN_ID=87 BLUEBOAT_GZ_PARTITION="blueboat_sea_state_$number"
    export ROS_DOMAIN_ID="$BLUEBOAT_ROS_DOMAIN_ID" GZ_PARTITION="$BLUEBOAT_GZ_PARTITION"
    mkdir -p "$BLUEBOAT_SITL_OUT"
    python3 "$LAB/tools/sea_states.py" --out "$BLUEBOAT_SITL_OUT" --state "$number" --rtf 1 --port 9032
    export BLUEBOAT_SITL_WORLD="$BLUEBOAT_SITL_OUT/world.sdf"
    bash "$LAB/tools/run_sitl_wsl.sh" > "$BLUEBOAT_SITL_OUT/session.log" 2>&1 &
    session_pid=$!
    cleanup(){ kill -TERM "$session_pid" 2>/dev/null || true; }
    trap cleanup EXIT INT TERM
    set +e
    "$HOME/blueboat_sitl/venv/bin/python" "$LAB/tools/validate_sea_state.py" --out "$BLUEBOAT_SITL_OUT" > "$BLUEBOAT_SITL_OUT/validation.log" 2>&1
    code=$?
    set -e
    cleanup
    wait "$session_pid" || true
    trap - EXIT INT TERM
    if [ "$code" -eq 0 ]; then
      python3 "$LAB/tools/analyze_sea_state.py" --out "$BLUEBOAT_SITL_OUT" >> "$BLUEBOAT_SITL_OUT/validation.log" 2>&1 || code=$?
    fi
    tail -n 100 "$BLUEBOAT_SITL_OUT/validation.log"
    if [ "$code" -ne 0 ]; then exit "$code"; fi
  done
done
