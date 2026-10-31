[CmdletBinding(SupportsShouldProcess = $true, ConfirmImpact = 'Medium')]
param(
    [Alias('Home')][string]$BeamNGHome = 'D:\BEAMNG\BeamNG.tech.v0.39.4.0',
    [string]$User = 'D:\BEAMNG\rural-user',
    [string]$Python = (Join-Path $PSScriptRoot '..\beamng_scene\.beamng-venv\Scripts\python.exe'),
    [ValidateSet('connect', 'launch')][string]$Mode = 'connect',
    [ValidateSet('camera', 'all')][string]$Sensors = 'camera',
    [switch]$ManualDrive,
    [switch]$Dx11,
    [switch]$ConfirmReplace,
    [ValidateRange(1, 65535)][int]$Port = 64256
)

$ErrorActionPreference = 'Stop'
if ($Dx11 -and $Mode -ne 'launch') { throw '-Dx11 is launch-only; a running renderer is unchanged.' }
$current = Join-Path $User 'current'
if ((Split-Path $User -Leaf) -ieq 'current') { $current = $User }
$runner = Join-Path $PSScriptRoot 'run.py'
$paths = @(
    @($BeamNGHome, 'Container'), @($User, 'Container'), @($current, 'Container'),
    @($Python, 'Leaf'), @($runner, 'Leaf'),
    @((Join-Path $BeamNGHome 'Bin64\BeamNG.tech.x64.exe'), 'Leaf'),
    @((Join-Path $BeamNGHome 'content\levels\smallgrid.zip'), 'Leaf'),
    @((Join-Path $BeamNGHome 'content\vehicles\pickup.zip'), 'Leaf'),
    @((Join-Path $PSScriptRoot 'generated\fixture.json'), 'Leaf'),
    @((Join-Path $PSScriptRoot 'generated\package.json'), 'Leaf'),
    @((Join-Path $PSScriptRoot 'dist\nz_sensor_calibration.zip'), 'Leaf'),
    @((Join-Path $current 'mods\nz_sensor_calibration.zip'), 'Leaf')
)
foreach ($item in $paths) {
    if (-not (Test-Path -LiteralPath $item[0] -PathType $item[1])) {
        throw ('Missing preflight path: ' + $item[0] + '. See the entry README; no installation was attempted.')
    }
}
$processes = @(Get-Process | Where-Object { $_.ProcessName -like '*BeamNG*' })
Write-Output ('BeamNG processes: ' + $processes.Count)
foreach ($process in $processes) { Write-Output ('  ' + $process.ProcessName + ' PID=' + $process.Id) }
$listening = $null
try {
    # Passive OS inspection only: do not open even a probe connection in WhatIf.
    $listeners = @([System.Net.NetworkInformation.IPGlobalProperties]::GetIPGlobalProperties().GetActiveTcpListeners())
    $listening = @($listeners | Where-Object {
        $_.Port -eq $Port -and $_.Address.ToString() -in @('127.0.0.1', '0.0.0.0', '::')
    }).Count -gt 0
    Write-Output ('Local TCP port ' + $Port + ' listening: ' + $listening + ' (not a BeamNG protocol check)')
} catch { Write-Warning 'TCP listener status unavailable; runtime connection/bind checks remain authoritative.' }
if ($Mode -eq 'connect' -and $listening -eq $false) {
    Write-Warning 'No local API listener. A GUI process alone is not an API service. Save work and close BeamNG yourself, then explicitly use -Mode launch; no automatic launch or kill.'
}
if ($Mode -eq 'launch' -and ($processes.Count -gt 0 -or $listening -eq $true)) {
    Write-Warning 'Launch blocked by an existing process/listener. Use connect for an API-enabled instance or close the intended instance yourself.'
    if (-not $WhatIfPreference) { throw 'Refusing a second launch.' }
}
$runArgs = @($runner, '--home', $BeamNGHome, '--user', $User, ('--' + $Mode), '--port', "$Port",
    '--preview', '--preview-only', '--mounted')
if ($Sensors -eq 'all') { $runArgs += @('--lidar', '--mounted-lidar', '--imu') }
if ($ManualDrive) { $runArgs += '--manual-drive' }
if ($Dx11) { $runArgs += @('--gfx', 'dx11') }
if ($ConfirmReplace) { $runArgs += '--replace-current' }
Write-Output ('Preview: mode=' + $Mode + ', sensors=' + $Sensors + ', manual-drive=' + [bool]$ManualDrive + '; static camera also present; NOT CALIBRATED')
if ($WhatIfPreference) {
    Write-Output ('WhatIf: preflight only; no Python, socket connection, scene mutation or launch. Replace authorized: ' + [bool]$ConfirmReplace)
    return
}
if (-not $ConfirmReplace) { throw 'Save your scene first, then explicitly pass -ConfirmReplace to authorize replacement.' }
if ($Mode -eq 'connect' -and $listening -eq $false) { throw 'No API listener; no simulator connection attempted.' }
if ($PSCmdlet.ShouldProcess('BeamNG current scene', 'Replace with sensor fixture and start preview')) {
    & $Python @runArgs
    $code = $LASTEXITCODE
    if ($code -eq 2) { Write-Output 'Exit 2: fixture not calibrated (expected for preview-only); inspect runtime_report.json for errors.' }
    exit $code
}
