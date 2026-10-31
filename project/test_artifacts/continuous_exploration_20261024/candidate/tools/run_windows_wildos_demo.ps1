param(
    [string]$Query = "house",
    [string]$Spawn = "spawns_sawmill",
    [int]$MaxCycles = 30,
    [double]$DriveSeconds = 2.0,
    [double]$Speed = 1.0,
    [int]$VisionPeriod = 1,
    [switch]$SaveArtifacts
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$Py = "D:\python.exe"
$Home = "D:\BEAMNG\BeamNG.tech.v0.39.4.0"
$User = "D:\BEAMNG\rural-user"
$MapZip = "D:\BEAMNG\rural-user\current\mods\american_road_0.39.zip"
$CheckpointRoot = "C:\Users\Yu\ckpts"
$SourceRoot = "C:\Users\Yu\nebula2-wildos"
$Runner = Join-Path $Root "tools\local_wildos_demo.py"
$Output = Join-Path $Root "wildos_lite\windows_demo_out"
$Log = Join-Path $Output "latest_run.log"

Write-Host "== WildOS Windows visible demo preflight ==" -ForegroundColor Cyan
foreach ($Path in @($Py, $Home, $User, $MapZip, $CheckpointRoot, $SourceRoot, $Runner)) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Missing required path: $Path"
    }
    Write-Host "OK $Path"
}

$Beamng = Get-Process -Name "BeamNG.tech.x64" -ErrorAction SilentlyContinue
if ($null -ne $Beamng) {
    throw "BeamNG.tech.x64 is already running. Close it before this demo; refusing to kill an existing session."
}

New-Item -ItemType Directory -Path $Output -Force | Out-Null
Write-Host "== scope ==" -ForegroundColor Yellow
Write-Host "Visible third-person BeamNG + background RGB/LiDAR + semantic query + local A* short segments"
Write-Host "Not the original ROS2/Nav2 runtime; no extra sensor windows"
Write-Host "query=$Query spawn=$Spawn cycles=$MaxCycles speed=$Speed"

Write-Host "== Python/BeamNGpy preflight ==" -ForegroundColor Cyan
& $Py -s -c "import beamngpy, importlib.metadata; print('beamngpy', importlib.metadata.version('beamngpy'))"
if ($LASTEXITCODE -ne 0) { throw "BeamNGpy import failed" }

$Args = @(
    "-s", "-B", $Runner,
    "--home", $Home,
    "--user", $User,
    "--map-zip", $MapZip,
    "--source-root", $SourceRoot,
    "--checkpoint-root", $CheckpointRoot,
    "--worker-python", $Py,
    "--spawn", $Spawn,
    "--query", $Query,
    "--max-cycles", "$MaxCycles",
    "--drive-seconds", "$DriveSeconds",
    "--speed", "$Speed",
    "--vision-period", "$VisionPeriod",
    "--camera-size", "960x640",
    "--output", $Output,
    "--success-radius", "4",
    "--target-confirmations", "3",
    "--target-max-jump", "8",
    "--target-miss-tolerance", "2",
    "--motion-chunk-metres", "2"
)
if ($SaveArtifacts) { $Args += "--save-artifacts" }

Write-Host "== starting visible BeamNG demo ==" -ForegroundColor Green
Write-Host "BeamNG window should be the only visible window; RGB/LiDAR stay background-only."
& $Py @Args 2>&1 | Tee-Object -FilePath $Log
$Code = $LASTEXITCODE
Write-Host "== demo exit code: $Code =="

$Reports = @(Get-ChildItem -LiteralPath $Output -Filter "loop_report.json" -Recurse -File -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending)
if ($Reports.Count -gt 0) {
    Write-Host "latest report: $($Reports[0].FullName)"
    Get-Content -LiteralPath $Reports[0].FullName
}
exit $Code
