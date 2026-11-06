#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
export BLUEBOAT_SITL_OUT="$LAB/results/sitl_full"
export BLUEBOAT_SITL_PORT=9012 BLUEBOAT_MAVLINK_PORT=5770 BLUEBOAT_SITL_INSTANCE=1
export BLUEBOAT_ROS_DOMAIN_ID=83 BLUEBOAT_GZ_PARTITION=blueboat_sitl_validation
source /opt/ros/jazzy/setup.bash
python3 "$LAB/tools/make_scenario.py" --out "$BLUEBOAT_SITL_OUT" --sitl --case combined
python3 - "$BLUEBOAT_SITL_OUT/world.sdf" <<'PY'
import sys,xml.etree.ElementTree as E
p=sys.argv[1];t=E.parse(p);t.find(".//plugin[@name='blueboat::SITL']/port").text='9012';t.write(p)
PY
export BLUEBOAT_SITL_WORLD="$BLUEBOAT_SITL_OUT/world.sdf"
bash "$LAB/tools/run_sitl_wsl.sh" > "$BLUEBOAT_SITL_OUT/session.log" 2>&1 &
session_pid=$!
cleanup(){ kill -TERM "$session_pid" 2>/dev/null || true; }
trap cleanup EXIT
set +e
bash "$LAB/tools/validate_sitl_wsl.sh" --out "$BLUEBOAT_SITL_OUT" > "$BLUEBOAT_SITL_OUT/validation.log" 2>&1
code=$?
set -e
cleanup
wait "$session_pid" || true
cat "$BLUEBOAT_SITL_OUT/validation.log"
exit "$code"
