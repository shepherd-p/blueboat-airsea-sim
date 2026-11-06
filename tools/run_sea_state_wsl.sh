#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
state="${1:-1}"
if [ "$state" != 1 ] && [ "$state" != 2 ]; then echo 'Sea state must be 1 or 2' >&2; exit 2; fi
export BLUEBOAT_SITL_OUT="${BLUEBOAT_SITL_OUT:-$LAB/results/sea_state_interactive_$state}"
export BLUEBOAT_SITL_WORLD="$BLUEBOAT_SITL_OUT/world.sdf"
export BLUEBOAT_SITL_PORT="${BLUEBOAT_SITL_PORT:-9002}"
python3 "$LAB/tools/sea_states.py" --state "$state" --out "$BLUEBOAT_SITL_OUT" --port "$BLUEBOAT_SITL_PORT"
exec bash "$LAB/tools/run_sitl_wsl.sh"
