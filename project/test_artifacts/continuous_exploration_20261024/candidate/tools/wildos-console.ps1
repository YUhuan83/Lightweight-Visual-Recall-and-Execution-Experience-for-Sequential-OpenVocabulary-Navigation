param(
    [ValidateSet('Start','Stop')][string]$Action = 'Start',
    [string]$Server = 'ly@192.168.31.91',
    [string]$RemoteRoot = '/home/ly/wildos-console/bundle',
    [string]$Config = '/home/ly/wildos-console/console.json',
    [string]$Session = '',
    [string]$Key = "$env:USERPROFILE\.ssh\wildos_192_168_31_91"
)
$ErrorActionPreference = 'Stop'
# ssh joins its remaining arguments through a remote shell. Restrict those
# arguments explicitly instead of pretending that PowerShell quoting escapes it.
if ($Server -notmatch '^[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+$') { throw 'Invalid Server' }
foreach ($remotePath in @($RemoteRoot, $Config)) {
    if ($remotePath -notmatch '^/[a-zA-Z0-9_./-]+$') { throw "Unsupported remote path: $remotePath" }
}
$sshArgs = @('-t','-o','ConnectTimeout=15','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=2')
if (Test-Path -LiteralPath $Key) { $sshArgs += @('-i', $Key) }
$sshArgs += @($Server,'python3',"$RemoteRoot/tools/wildos_console.py")
if ($Action -eq 'Stop') {
    if ($Session -notmatch '^/[a-zA-Z0-9_./-]+$') { throw 'Stop requires -Session with the session log directory' }
    $sshArgs += @('--stop', $Session)
} else {
    $sshArgs += @('--config', $Config)
}
& ssh @sshArgs
if ($LASTEXITCODE -ne 0) { throw "Console/SSH exited with code $LASTEXITCODE; inspect session_result.json" }
