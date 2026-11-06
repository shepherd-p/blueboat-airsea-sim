$ErrorActionPreference = 'Stop'
$labLinux = (wsl -d Ubuntu-24.04 --exec wslpath -a -u "$PSScriptRoot").Trim()
wsl -d Ubuntu-24.04 -- bash "$labLinux/tools/validate_wsl.sh"
if ($LASTEXITCODE -ne 0) { throw 'Validation failed; inspect results\baseline\validation.log' }

