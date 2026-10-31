# Windows Visible Demo

This is a local BeamNG.tech demonstration entry point. It shows only the
third-person BeamNG game view; Camera and LiDAR are mounted and used by the
runner but are not opened as extra windows.

Run from a PowerShell terminal in the repository root:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\tools\run_windows_wildos_demo.ps1 -Query house -Spawn spawns_sawmill -MaxCycles 30 -Speed 1
```

For a safer short smoke:

```powershell
.\tools\run_windows_wildos_demo.ps1 -Query house -Spawn spawns_sawmill -MaxCycles 8 -Speed 0.8
```

The demonstration uses the current local runner: persistent ExploRFM worker,
RGB/LiDAR capture, frontier/traversability outputs, target confirmation, local
LiDAR A*, and short BeamNG AI segments. It is not a claim that the original
ROS2/Nav2 WildOS runtime has been reproduced on Windows.

If the run needs postmortem images and arrays, add `-SaveArtifacts`; otherwise
per-cycle artifacts are removed to keep the run light.
