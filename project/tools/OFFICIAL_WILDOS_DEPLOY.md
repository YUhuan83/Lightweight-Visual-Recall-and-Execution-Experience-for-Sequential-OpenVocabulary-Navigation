# Official WildOS BeamNG Deployment

## Scope

The runtime preserves the upstream algorithm nodes:

```text
graphnav_mapper -> WildOS_Nav -> graphnav_planner -> path_follower
```

Project code is restricted to simulator integration and local execution:

```text
BeamNG messages/TF/OccupancyGrid -> upstream graph
upstream path-follower goal -> Nav2 local controller -> BeamNG VehicleControl
```

The following inputs are explicitly simulator or experiment inputs and are not
claimed as paper sensor-localization components:

- `/spot1/odom`: BeamNG StateSensor simulator truth.
- `/spot1/pose_graph`: upstream dummy single-node PoseGraph publisher.
- Initial `/spot1/imgnav_waypoint`: one-shot seed goal to break the upstream
  object-triangulation/motion startup dependency.

## Bundle Contents

- `official_adapter/`: BeamNG input adapter, seed publisher and semantic health gate.
- `nav2_adapter/`: Nav2 local execution and safe VehicleControl adapter.
- `tools/prepare_official_wildos.py`: packaging metadata/config preparation only.
- `tools/build_official_wildos.sh`: selected upstream package build.
- `tools/official_wildos_window_demo.sh`: visible BeamNG runtime with safety gates.
- `tools/wildos_server_stop.sh`: emergency brake and official-stack shutdown.
- `tools/nav2_activate.py`: lifecycle activation fallback.

## Deploy

From Windows PowerShell:

```powershell
scp .\official_wildos_beamng_bundle.zip ly@192.168.31.91:/home/ly/
```

On the server:

```bash
mkdir -p /home/ly/official_wildos_bundle
unzip -o /home/ly/official_wildos_beamng_bundle.zip -d /home/ly/official_wildos_bundle
rm -rf /home/ly/beamng_ros2_0394_ws/src/beamng_wildos_adapter
rm -rf /home/ly/beamng_ros2_0394_ws/src/wildos_nav2_adapter
cp -a /home/ly/official_wildos_bundle/official_adapter /home/ly/beamng_ros2_0394_ws/src/beamng_wildos_adapter
cp -a /home/ly/official_wildos_bundle/nav2_adapter /home/ly/beamng_ros2_0394_ws/src/wildos_nav2_adapter
cp -a /home/ly/official_wildos_bundle/tools/. /home/ly/
chmod +x /home/ly/build_official_wildos.sh /home/ly/official_wildos_window_demo.sh /home/ly/wildos_server_stop.sh
bash /home/ly/build_official_wildos.sh
```

## Preflight

Before launching the vehicle, verify the combined ROS/CUDA interpreter:

```bash
source /opt/ros/humble/setup.bash
/home/ly/vision-cu128/bin/python -c 'import rclpy, torch; print(torch.cuda.is_available())'
```

Expected result: `True`.

Verify local checkpoints:

```bash
test -f /home/ly/ckpts/c-radio_v3-b_half.pth.tar
test -f /home/ly/ckpts/frontier_head.ckpt
test -f /home/ly/ckpts/trav_head.ckpt
find /home/ly/ckpts/siglip2 -name config.json -print
```

## Run

The physical desktop must be active on `DISPLAY=:3`:

```bash
bash /home/ly/official_wildos_window_demo.sh
```

The script holds the vehicle disabled until all three semantic gates pass:

1. Valid sensor contracts and a raw NavigationGraph containing frontiers.
2. A scored NavigationGraph containing finite upstream `frontier_scores`.
3. A non-empty upstream graph path, upstream path-follower goal and Nav2 path.

Reports:

```text
/tmp/official_graph_health.json
/tmp/official_scored_health.json
/tmp/official_path_health.json
```

## Emergency Stop

```bash
bash /home/ly/wildos_server_stop.sh
```

The stop script publishes high-frequency full braking before killing the
control and planning nodes, then pauses BeamNG. It intentionally leaves the
BeamNG window and bridge process alive for diagnosis.

## Acceptance

The demo is accepted only when all conditions hold:

- Visible BeamNG American Road window on the physical display.
- `/spot1/nav_graph` has nodes, valid indices and at least one frontier point.
- `/spot1/scored_nav_graph` has finite `frontier_scores` from upstream WildOS.
- `/spot1/graphnav_planner/path` contains at least two poses.
- `/spot1/goal_pose` is produced by upstream `path_follower`.
- `/wildos/planned_path` contains at least two Nav2 poses.
- Vehicle speed remains below the configured overspeed limit.
- Emergency stop applies braking and pauses the simulation.

This validates an upstream WildOS GraphNav integration with simulator/oracle
localization inputs. It does not validate sensor-only LIO/elevation mapping.
