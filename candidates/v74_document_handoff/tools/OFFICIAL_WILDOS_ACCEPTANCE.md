# Official WildOS BeamNG Acceptance

Date: 2026-09-15
Server: `ly@192.168.31.91`
Simulator: BeamNG.tech 0.39.4, American Road

## Accepted Chain

```text
BeamNG Camera/LiDAR/State
  -> BeamNG interface adapter
  -> upstream graphnav_mapper
  -> upstream WildOS_Nav / ExploRFM
  -> upstream graphnav_planner / Dijkstra
  -> upstream path_follower
  -> Nav2 local execution
  -> BeamNG VehicleControl
```

## Build

- 17 ROS2 packages built successfully in `/home/ly/beamng_ros2_0394_ws`.
- Upstream GridMap Humble source was built in the workspace without sudo.
- `graphnav_mapper`, `graphnav_planner`, `path_follower`, `visual_navigation`,
  `beamng_wildos_adapter` and `wildos_nav2_adapter` executables were verified.
- Vision interpreter imported ROS2, Torch, WildOS and triangulation modules together.
- CUDA: Torch 2.7.0+cu128, two RTX 4090 devices visible.

## Graph Gate

Repeated runs passed semantic validation of:

- RGB Image and matching CameraInfo.
- `spot1/odom -> spot1/base_link` truth odometry.
- local-frame LiDAR and OccupancyGrid.
- NavigationGraph frame, indices, traversability class and frontier content.

Representative raw graph:

```text
nodes: 77
edges: 514
frontier nodes: 54
frontier points: 169
```

## WildOS Scoring Gate

Representative scored graph:

```text
scored frontier nodes: 51
score values: 816
score range: 0.024074 to 0.2999997
```

Another stability run produced 59 scored nodes and 944 finite scores.

## Path Gate

The semantic path gate passed all required outputs:

```text
/spot1/graphnav_planner/path: non-empty upstream graph path
/spot1/goal_pose: upstream path_follower output
/wildos/planned_path: non-empty Nav2 local path
```

Nav2 planner results contained approximately 37 to 40 poses per local request.
After publishing a new path, prior FollowPath actions were cancelled rather than
allowed to compete.

## Object Search

The closest American Road house to `spawns_sawmill` is a real
`eca_bld_house_02_a` instance approximately 23.1 m from the spawn.

After correcting the initial BeamNG yaw to face the house, the unmodified
upstream object threshold `0.09` succeeded:

```text
house similarity maximum: approximately 0.1225
object mask: approximately 80,000 to 83,000 pixels
```

The upstream LiDAR-in-mask triangulator repeatedly published object goals. A
typical estimate was approximately 3.9 m from the mapped house instance; other
frames varied due to the broad mask and simulator geometry.

The final target-chain run printed `OFFICIAL_OBJECT_GOAL_RECEIVED`, used no seed
goal, and passed scored-graph, graph-path, path-follower and Nav2-path gates with
`ENABLE_MOTION=0`.

## Motion And Safety

A separate low-speed execution run enabled the vehicle only after all three
semantic gates passed.

```text
configured speed limit: 0.6 m/s
spawn-to-final displacement: 12.276 m
post-path 15 s displacement: 0.000027 m
post-path sampled maximum speed: 0.185 m/s
```

No overspeed, stale-state or non-finite command error was logged. Nav2 local
cloud filtering produced zero TF drop errors after converting the rigid
`base_link -> lidar` transform to a static transform.

Final emergency-stop acceptance:

```text
BeamNG pause service: success=True
brake: 1.0
brake_input: 1.0
throttle: 0.0
throttle_input: 0.0
wheelspeed: approximately 0.015 m/s and decaying
```

All planner, perception, action bridge and control processes were terminated;
the BeamNG window and bridge were intentionally retained for diagnosis.

## Explicit Limitations

- Odometry is simulator truth, not sensor-only DLIO.
- PoseGraph input is the upstream dummy single-node publisher.
- OccupancyGrid is a BeamNG LiDAR interface adapter, not Elevation Mapping CuPy.
- Actual low-speed vehicle motion and actual object-goal-to-path generation were
  each validated. Target pursuit was intentionally not allowed to continue after
  observing frame-to-frame triangulation variation.
- The result is an accepted upstream WildOS GraphNav integration using explicit
  simulator/oracle localization inputs, not a sensor-only field-stack reproduction.
