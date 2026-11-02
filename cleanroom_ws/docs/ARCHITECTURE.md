# WildOS + visual recall + BeamNG cleanroom

## Scope

This workspace is an isolated ROS 2 Humble build based on the v74 runtime
sources. It keeps the WildOS perception and graph-scoring flow, the bounded
in-process visual stores, and the Ackermann BeamNG vehicle interface. It has one
colcon workspace and one trial entry point. It does not import historical
install overlays or save memory across process restarts.

## Frames and vehicle geometry

- `spot1/odom` is the world frame for odometry, graph nodes, target estimates,
  and navigation paths.
- `spot1/base_link` is located at the rear axle in XY, retains measured Z and
  full body attitude, and is used for rigid sensor geometry.
- `spot1/nav_base_link` shares the rear axle XY, uses Z=0 and yaw only for
  planar planning and command-frame checks.
- `spot1/terrain_base_link` retains measured height with yaw-only orientation
  for terrain projection. The virtual point-cloud frame remains at the
  StateSensor origin; `spot1/lidar_ray_origin` is the physical ray origin.
- SBR geometry is front 3.4 m, rear 1.05 m, half-width 1.05 m and wheelbase
  2.501 m. The calibrated steering sign is -1.0. Smac plans with a 12 m
  minimum radius; RPP and the actuator enforce an 8 m minimum radius.

## Data flow and topic contracts

```text
BeamNG sensors and vehicle state
  -> beamng_wildos_adapter
     /spot1/odom, /spot1/realsense/front/color/image_raw,
     /spot1/realsense/front/color/camera_info,
     /spot1/ouster/front/points_filtered,
     /spot1/ouster/front/points_for_localization
  -> cloud_filter -> /wildos/points + /wildos/clearing_points
  -> Nav2 global/local costmaps -> Smac (Reeds-Shepp) -> RPP
  -> /wildos/cmd_vel_stamped -> cmd_vel_to_control
  -> /wildos/native_control -> native_vehicle_control -> BeamNG

Camera + odometry + scored navigation graph
  -> WildOS frontier scoring -> /spot1/scored_nav_graph
  -> graphnav_mapper -> /spot1/nav_graph
  -> graphnav_planner -> /spot1/graphnav_planner/path
  -> graph_path_bridge -> ComputePathToPose / FollowPath

Query /spot1/object_search/query
  -> WildOS scoring and in-process recall
  -> object mask -> triangulation -> /spot1/raw_object_waypoint
  -> graph_path_bridge; current-query confirmation arrives on
     /spot1/object_confirmation
  -> /wildos/plan_status and /spot1/traversal_feedback
```

Confirmation freshness is checked on receipt (measurement age <=5 s and
acquisition after this query began). An accepted confirmed object remains
valid for that task; reaching its XY<3 m disk does not require a new image of
an elevated surface now outside the camera frustum. Query changes clear it.
Independent audit records both measurement age and original receipt delay;
the target and its observation must still spatially agree. This is not a
claim that a cached image is a current camera observation.

All object queries use the existing confounder comparison before publishing
their mask; colored queries keep same-object color alternatives. A target's
own named parts are excluded as competitors. These model scores and coherent
LiDAR returns remain estimates; independent visual review can reject them.

The BeamNG adapter maps simulator state to the rear-axle odometry frame and
publishes measurement timestamps. The 40 m point cloud feeds costmap marking
and clearing; the localization cloud retains its separate 80 m range. The
terrain grid feeds GraphNav mapping. Nav2 costmaps and RPP retain obstacle and
footprint checks. GraphNav supplies topological direction; Smac and RPP handle
vehicle curvature and local collision-aware execution.

The target estimate remains a 3D point in `spot1/odom`. Navigation completes
when the vehicle-to-target, target-to-confirmed-observation, and
vehicle-to-confirmed-observation horizontal distances are each strictly below
3 m, with a fresh confirmation for the active query. Confirmation is not
replaced by a remembered location.

## Visual recall stores

All four stores are created in the WildOS process with SQLite `:memory:` or
bounded in-memory arrays. They do not create a second geometric map or retain
RGB video.

- `ExecutionExperienceStore` keeps bounded visual descriptors and feedback on
  executed heading bins; retrieved feedback adjusts supported frontier scores.
- `GraphObservationStore` associates visual observations and confirmed object
  evidence with existing graph nodes and heading bins.
- `ConfirmedViewpointCache` keeps a bounded set of query-specific, confirmed
  viewpoints for in-process reuse.
- `PassiveLandmarkIndex` stores a fixed-size descriptor and LiDAR-supported
  position for scene retrieval; retrieval is a hint, never target confirmation.
- `RecallCandidatePolicy` ranks eligible remembered views against current
  position and heading. The live query and current observation still govern
  navigation and arrival.

## Build and run

Build only against ROS 2 Humble and this workspace's install:

```bash
source /opt/ros/humble/setup.bash
COLCON_EXECUTABLE=/path/to/colcon bash scripts/build.sh
```

`scripts/run_trial.sh [query] --seconds 120` starts one map and one search.
`scripts/run_trial.sh --targets "traffic light" "bus stop" "gas station" --seconds 300`
uses one map/process and one total deadline; each stage requires verified arrival.
Set `BEAMNG_HOME`, `BEAMNG_USER`, `VISION_PYTHON`, and model paths as needed.
`--attach` uses an already loaded map. Startup and shutdown are outside the
120-second search window. The run directory contains process logs, event and
confirmation JSONL, odometry samples, a final camera image, and a confirmation
image when available. It does not record MP4 by default.

Before and after a server run, inspect and clean up only processes belonging to
this trial. Stop rosbag recording before reading its SQLite database. A trial
passes only with a matching `object_found` event, independent current-query
arrival evidence, clean process shutdown, and no recorded errors.
