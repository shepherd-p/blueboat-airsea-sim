$ErrorActionPreference = 'Stop'
python "$PSScriptRoot\tools\generate_model.py"
if ($LASTEXITCODE -ne 0) { throw 'Model generation failed' }
python "$PSScriptRoot\tools\generate_environment.py"
if ($LASTEXITCODE -ne 0) { throw 'Environment generation failed' }
python "$PSScriptRoot\tools\sea_states.py" --generate
if ($LASTEXITCODE -ne 0) { throw 'Sea-state generation failed' }
$labLinux = (wsl -d Ubuntu-24.04 --exec wslpath -a -u "$PSScriptRoot").Trim()
wsl -d Ubuntu-24.04 -- bash "$labLinux/tools/build_wsl.sh"
if ($LASTEXITCODE -ne 0) { throw 'WSL build failed' }

