param([switch]$Headless, [switch]$SITL, [ValidateSet('Ocean','Calm','SeaState1','SeaState2')][string]$Scenario='Ocean')
$ErrorActionPreference = 'Stop'
$labLinux = (wsl -d Ubuntu-24.04 --exec wslpath -a -u "$PSScriptRoot").Trim()
$mode = if ($Headless) { 'true' } else { 'false' }
if ($Scenario -in @('SeaState1','SeaState2')) {
    $seaNumber = if ($Scenario -eq 'SeaState1') { '1' } else { '2' }
    wsl -d Ubuntu-24.04 -- env "BLUEBOAT_HEADLESS=$mode" bash "$labLinux/tools/run_sea_state_wsl.sh" "$seaNumber"
} elseif ($SITL) {
    wsl -d Ubuntu-24.04 -- env "BLUEBOAT_HEADLESS=$mode" bash "$labLinux/tools/run_sitl_wsl.sh"
} else {
    $worldName = if ($Scenario -eq 'Ocean') { 'blueboat_ocean.sdf' } else { 'blueboat_water.sdf' }
    wsl -d Ubuntu-24.04 -- bash "$labLinux/tools/run_wsl.sh" "headless:=$mode" "world:=$labLinux/h-spar-sim/ros2_ws/src/blueboat_description/worlds/$worldName"
}

