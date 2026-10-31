$ErrorActionPreference = 'Stop'
$python = if ($env:RURAL_PYTHON) { $env:RURAL_PYTHON } else { Join-Path $PSScriptRoot '.venv\Scripts\python.exe' }
& $python (Join-Path $PSScriptRoot 'run.py') @args
exit $LASTEXITCODE
