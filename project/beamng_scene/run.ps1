$ErrorActionPreference = 'Stop'
$python = Join-Path $PSScriptRoot '.beamng-venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) {
    throw 'Dedicated venv missing. See README.md for setup.'
}
& $python (Join-Path $PSScriptRoot 'scene.py') @args
exit $LASTEXITCODE
