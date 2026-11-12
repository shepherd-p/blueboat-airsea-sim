param([double]$Duration=70, [double]$RealTimeFactor=0.95)
$ErrorActionPreference = 'Stop'
$labLinux = (wsl -d Ubuntu-24.04 --exec wslpath -a -u "$PSScriptRoot").Trim()
if ($LASTEXITCODE -ne 0) { throw 'Ubuntu-24.04 WSL is required' }
$runName = 'cooperative_live_' + (Get-Date -Format 'yyyyMMdd_HHmmss')
Write-Host "Starting Sea State 1 mutual observation. Results: $PSScriptRoot\results\$runName"
wsl -d Ubuntu-24.04 -- bash "$labLinux/tools/run_cooperative_wsl.sh" --out "$labLinux/results/$runName" --rtf "$RealTimeFactor" --duration "$Duration"
if ($LASTEXITCODE -ne 0) { throw 'Simulation did not complete; inspect results logs' }
wsl -d Ubuntu-24.04 -- python3 "$labLinux/tools/cooperative/validate.py" --run "$labLinux/results/$runName"
if ($LASTEXITCODE -ne 0) { throw 'Independent acceptance failed; do not present this run as complete' }
