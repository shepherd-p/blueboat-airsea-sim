#!/usr/bin/env bash
set -eo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export GZ_PARTITION="${GZ_PARTITION:-blueboat_standalone}"
source /opt/ros/jazzy/setup.bash
python3 - "$LAB" <<'PY'
from pathlib import Path
import sys
root=Path(sys.argv[1]);sys.path.insert(0,str(root/'tools'))
from sea_states import make_case
make_case(root/'results/public_demo',1,False,1)
PY
bash "$LAB/tools/run_wsl.sh" headless:=false controller:=true world:="$LAB/results/public_demo/world.sdf"
