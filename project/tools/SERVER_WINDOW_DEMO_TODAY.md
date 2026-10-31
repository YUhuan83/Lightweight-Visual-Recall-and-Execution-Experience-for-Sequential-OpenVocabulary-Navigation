# Server Windowed WildOS Demo

Goal: visible BeamNG American Road window on the physical display, while ROS2
Camera/LiDAR, ExploRFM semantic/frontier/traversability inference, Nav2 planning,
and limited vehicle control run in the background.

## Upload

From local Windows PowerShell:

```powershell
scp "C:\Users\Yu\Desktop\拟真\wildos_ros2_adapter_window.zip" `
    "C:\Users\Yu\Desktop\拟真\wildos_nav2_adapter_window.zip" `
    "C:\Users\Yu\Desktop\拟真\tools\wildos_server_window_demo.sh" `
    "C:\Users\Yu\Desktop\拟真\tools\wildos_server_stop.sh" `
    "C:\Users\Yu\Desktop\拟真\tools\nav2_activate.py" `
    ly@192.168.31.91:/home/ly/
```

## Build

```bash
unzip -oq ~/wildos_ros2_adapter_window.zip -d ~/beamng_ros2_0394_ws/src/wildos_ros2_adapter
unzip -oq ~/wildos_nav2_adapter_window.zip -d ~/beamng_ros2_0394_ws/src/wildos_nav2_adapter
source /opt/ros/humble/setup.bash
source ~/beamng_ros2_0394_ws/.venv/bin/activate
cd ~/beamng_ros2_0394_ws
colcon build --packages-select wildos_ros2_adapter wildos_nav2_adapter --symlink-install
source ~/beamng_ros2_0394_ws/install/setup.bash
```

## Run

The physical desktop for `ly` is display `:3`, with X authority at
`/run/user/1011/gdm/Xauthority`. The script verifies that display before making
any changes.

```bash
chmod +x ~/wildos_server_window_demo.sh ~/wildos_server_stop.sh
tmux new -s wildos-window
bash ~/wildos_server_window_demo.sh 2>&1 | tee /tmp/wildos_window_demo.log
```

Expected final output:

```text
WINDOW_DEMO_RUNNING
```

The BeamNG game window should appear on display `:3`. Camera/LiDAR/frontier maps
remain hidden. The visible window shows the vehicle moving in American Road.
The script holds continuous brakes until sensor, odometry, Nav2, perception, and
the first Nav2 path are all healthy.

## Emergency Stop

From any SSH shell:

```bash
bash ~/wildos_server_stop.sh
```

Expected:

```text
WILDOS_MOTION_STOPPED
```

## Evidence

```bash
tail -n 30 /tmp/wildos_window_demo.log
tail -n 20 /tmp/wildos_window_perception.log
tail -n 20 /tmp/wildos_window_plan.log
tail -n 20 /tmp/wildos_window_nav2.log
tail -n 10 /tmp/wildos_window_control.log
```

The meaningful perception modes are:

```text
mode=explore  semantic target not localized; use ExploRFM frontier/traversability
mode=target   semantic mask + LiDAR localization produced a world target
```

## Scope

This demonstrates the complete project pipeline:

```text
American Road RGB/LiDAR/State/TF
-> persistent ExploRFM semantic/frontier/traversability inference
-> semantic LiDAR localization or learned frontier exploration target
-> Nav2 rolling LiDAR costmaps and NavFn planning
-> Regulated Pure Pursuit controller
-> bounded Ackermann cmd_vel adapter
-> BeamNG VehicleControl
```

Known non-paper-exact elements must remain explicit:

* CameraInfo is nominal (960x640, FOVy 60), not measured calibration.
* StateSensor is used as explicit simulator-truth odometry for the demo.
* A persistent ROS `NavigationGraph` mapper is not yet implemented; the current
  learned frontier output generates intermediate goals and the Nav2 costmap is
  rolling LiDAR obstacle mapping.
* This is therefore a functional windowed reproduction of the perception ->
  planning -> control stack, not an exact reproduction of all paper evaluation
  conditions or metrics.
