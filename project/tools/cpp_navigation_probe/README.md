# Nav2 snapshot diagnostics

This standalone test project calls the installed **Nav2 Humble 1.1.20** C++
collision checker, inflation layer and SmacHybrid planner. It does not connect
to BeamNG, send actions, or control a vehicle. Run in an isolated ROS domain.
It adds no dependency to the production controller package.

```sh
source /opt/ros/humble/setup.bash
cmake -S tools/cpp_navigation_probe -B /tmp/wildos_snapshot_probe
cmake --build /tmp/wildos_snapshot_probe -j2
ROS_DOMAIN_ID=85 ctest --test-dir /tmp/wildos_snapshot_probe --output-on-failure
/tmp/wildos_snapshot_probe/collision_snapshot_probe SNAPSHOT.txt
ROS_DOMAIN_ID=85 /tmp/wildos_snapshot_probe/smac_planner_snapshot_probe \
  SNAPSHOT.txt 3.75 RESULT.json
```

Snapshot format: dimensions/resolution/origin on line 1; actual odometry,
returned first pose and returned goal (x/y/yaw radians) on lines 2–4;
then nonzero OccupancyGrid cells as `linear_index occupancy_value`.
Only a closed bag may be used. All poses and map must share the same frame.

`collision_snapshot_probe` compares continuous boundary checks against Smac
angle-bin checks, including the configured inflation shortcut. The publication
is lossy: lethal, inscribed and unknown classes are exact; inverse soft costs
are approximate. It is **not** the original planner's private costmap.

`smac_planner_snapshot_probe` rebuilds old/new inflation from the same lethal
and unknown cells and calls the actual planner. It tests the narrow-field false
acceptance, covered-field rejection and free-map/recovery planning. This also
is a reconstructed map, not a bit-exact replay of the planner's private map.
An explicit rectangular footprint parameter is essential: calling
`setRobotFootprint()` alone leaves Costmap2DROS's radius-mode flag unchanged.

Increasing global inflation fixes a proven unsafe shortcut. It does not make
an invalid quantized start feasible, repair the vehicle-to-path join, or prove
visual arrival or full-search stability. Do not relax downstream checks to
compensate for planner rejection.

## Exact start connector diagnostic

The project also requires OMPL (recorded runs used 1.7). `anchor_connector_probe`
samples exact Reeds-Shepp curves from the measured start to native Smac lattice
anchors. It retains radius 8 m, the padded SBR rectangle, both map checks and
forward/reverse sections. It sends no actions and has no simulator client.

```sh
/tmp/wildos_snapshot_probe/anchor_connector_probe \
  GLOBAL_SNAPSHOT.txt LOCAL_SNAPSHOT.txt CURVES.csv SUMMARY.json
ROS_DOMAIN_ID=85 /tmp/wildos_snapshot_probe/smac_planner_snapshot_probe \
  ANCHOR_SNAPSHOT.txt 3.75 RESULT.json SUFFIX.csv
```

The optional suffix output records the actual installed planner's poses.
`tools/check_anchor_connector_replay.py` checks exact prefixes joined to these
suffixes with the production Python validators. The paired maps must come from
a closed bag and use the same frame and actual start.

The archived run examined 57,528 anchors and found 1,444 geometric candidates.
This exhaustive diagnostic has no measured production latency budget, no map
update/goal cancellation protocol and no vehicle execution. Do not deploy it
directly as the planning loop. Initial continuous-pose integration remains
absent from `GraphPathBridge`; see `docs/OFFLINE_PLANNING_AUDIT_20261011.md`.
