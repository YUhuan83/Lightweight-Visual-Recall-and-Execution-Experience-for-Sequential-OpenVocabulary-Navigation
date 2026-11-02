# Continuous start for Smac Hybrid

`wildos_ackermann_smac/Planner` extends installed Humble Nav2 **1.1.20**;
native Smac retains goal search and smoothing. It does not select semantic
targets, change goal identity, shrink the footprint or disable obstacles.

When the continuous actual start is safe but Smac's integer XY / nearest
heading-bin start collides, the plugin searches nearby grid centres and native
float heading bins. OMPL Reeds–Shepp curves obey the configured turning radius,
have homogeneous sections longer than `connector_min_section`, and are sampled
at most 0.05 m (or resolution / 4). Every sample checks the current padded
polygon boundary and known lethal interior; unknown handling follows Smac.
The native start checker is forced to evaluate the full polygon here.

Search stops at 6000 evaluations, six retained candidates, or 0.2 seconds.
Candidates are ranked by length plus cusp penalty, within half the turning
radius and total curve length no more than that radius. A failed native suffix
may retry only if the remaining **2 s total** budget still accommodates the
native **1.5 s** budget. Complete candidates are retained on search expiry;
partially validated curves are discarded. Native suffixes must meet the exact
anchor; outputs are rechecked against the latest global map before release.

These are bounded work/deadline checks, not hard real-time preemption: native
search, locks and scheduling can overrun before returning. Expired results are
rejected; the bridge retains its request timeout and epoch/cancel checks.
Global-map checking does not replace the bridge's fresh local-map swept-body,
curvature, direction and actual-pose entry checks. A geometric planner output
is not visual arrival evidence.

The planner namespace is frozen while active. Changes made while inactive
require cleanup/configure before reactivation. Map resolution changes and
radius-only footprints fail closed. Costmap rolling origins and updates remain
supported. Downsampling is rejected because anchor quantization must match the
backend. Build/install the new package before enabling its YAML plugin ID.

Tests use actual pluginlib/Smac/OMPL and synthetic obstacle controls plus an
archived live map. The archived map reconstruction preserves lethal/unknown
observations and recomputes inflation; it is not the old private map byte for
byte. Vehicle execution and complete visual search require separate checks.
