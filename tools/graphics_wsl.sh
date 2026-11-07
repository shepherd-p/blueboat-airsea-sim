#!/usr/bin/env bash
# Prefer the installed NVIDIA WSLg adapter over Mesa llvmpipe, process only.
# Other hosts keep their existing driver selection. Set BLUEBOAT_SOFTWARE_RENDER=1
# or an explicit GALLIUM_DRIVER to retain a chosen renderer.
if [[ "${BLUEBOAT_SOFTWARE_RENDER:-0}" != 1 && -e /dev/dxg && -x /usr/lib/wsl/lib/nvidia-smi ]]; then
  if /usr/lib/wsl/lib/nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | grep -q NVIDIA; then
    export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
    export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
  fi
fi
