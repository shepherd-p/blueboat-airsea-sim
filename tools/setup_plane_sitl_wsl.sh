#!/usr/bin/env bash
set -euo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
RUNTIME="${BLUEBOAT_PLANE_RUNTIME:-$HOME/blueboat_plane_sitl}"
mkdir -p "$RUNTIME"
base=https://firmware.ardupilot.org/Plane/stable-4.6.3/SITL_x86_64_linux_gnu
for file in arduplane firmware-version.txt git-version.txt; do
  curl -L --fail --retry 2 "$base/$file" -o "$RUNTIME/$file"
done
echo 'df3e05f7577a98342d4d34c3d0e8b37212c012ca4fa3691e3db7be06c4db9ec3  '"$RUNTIME/arduplane" | sha256sum --check
chmod +x "$RUNTIME/arduplane"
cat "$RUNTIME/firmware-version.txt"
