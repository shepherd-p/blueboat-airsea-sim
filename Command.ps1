param([double]$Forward = 0, [double]$YawRate = 0, [double]$Seconds = 5)
$ErrorActionPreference = 'Stop'
$labLinux = (wsl -d Ubuntu-24.04 --exec wslpath -a -u "$PSScriptRoot").Trim()
$culture = [System.Globalization.CultureInfo]::InvariantCulture
wsl -d Ubuntu-24.04 -- bash "$labLinux/tools/command_wsl.sh" $Forward.ToString($culture) $YawRate.ToString($culture) $Seconds.ToString($culture)

