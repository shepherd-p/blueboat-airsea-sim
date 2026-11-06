#!/usr/bin/env bash
set -euo pipefail
LAB="$(cd "$(dirname "$0")/.." && pwd)"
RUNTIME="${BLUEBOAT_SITL_RUNTIME:-$HOME/blueboat_sitl}"
mkdir -p "$RUNTIME" "$LAB/references/ardurover"
base=https://firmware.ardupilot.org/Rover/stable-4.7.2/SITL_x86_64_linux_gnu
for file in ardurover firmware-version.txt git-version.txt; do
  curl -L --fail --retry 2 "$base/$file" -o "$RUNTIME/$file"
done
chmod +x "$RUNTIME/ardurover"
echo 'f453fd8217b9d798c873e5c424e139a2a3597b52e31658f76b2c177b523391af  '"$RUNTIME/ardurover" | sha256sum --check
cp "$RUNTIME/firmware-version.txt" "$RUNTIME/git-version.txt" "$LAB/references/ardurover/"
sha256sum "$RUNTIME/ardurover" > "$LAB/references/ardurover/binary.sha256"
if [ "$(id -u)" -eq 0 ]; then
  apt-get update -qq
  apt-get install -y -qq python3-venv
else
  sudo apt-get update -qq
  sudo apt-get install -y -qq python3-venv
fi
python3 -m venv --system-site-packages "$RUNTIME/venv"
"$RUNTIME/venv/bin/pip" install 'pymavlink==2.4.49'
"$RUNTIME/ardurover" --help > "$LAB/references/ardurover/help.txt" 2>&1 || true
cat "$RUNTIME/firmware-version.txt" "$RUNTIME/git-version.txt"
