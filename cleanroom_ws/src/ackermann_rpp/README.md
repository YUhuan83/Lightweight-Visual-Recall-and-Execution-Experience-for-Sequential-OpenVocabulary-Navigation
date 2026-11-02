# Ackermann RPP contract

This plugin targets the installed Nav2 Humble RPP 1.1.20 protected API.
It retains upstream transforms, point selection, speed regulation, approach
slowing, collision sampling and lifecycle behavior. It requires real car
configuration: reversing, collision checking, no rotation in place.

The requested Pure Pursuit curvature still controls speed regulation. A
bounded curvature controls **both** the footprint collision prediction and
the published angular velocity. The downstream actuator remains a final
guard, rather than silently changing the arc after Nav2 checked another one.

Real forward/reverse junctions use a separate StoppedGoalChecker: 0.2 m,
0.35 rad, linear/angular stopped thresholds 0.05 m/s and 0.05 rad/s.
When that checker requests stopped velocity, the controller brakes inside
the true global endpoint's pose tolerances and retains footprint collision
checking. A cropped local plan endpoint cannot trigger this brake. Semantic
observation endpoints retain their existing checker and tolerances.
Stationary commands check both the current footprint boundary and center
cell. The existing unknown-space policy is retained. Collision prediction
uses the actual zero command, rather than a hypothetical moving arc.
The radius is frozen per controller configuration. Dynamic attempts to
disable collision checking or enable in-place rotation fail closed when
computing a command.

Each FollowPath action contains one forward or reverse section. `setPlan`
captures that direction from the full path's geometry and body headings before
pruning, rejects mixed directions, and updates the path and direction together.
The lookahead cannot introduce a direction change when a side-offset terminal
point crosses the body y-axis. Such an action aborts for the route owner to plan
from the actual pose; it does not report arrival or enlarge cusp tolerances.
Braking at a genuinely reached stopped cusp still takes priority. A newly
accepted path can explicitly change direction, with existing stopped handoff
and speed-loop gear guards.

GraphPathBridge predicts this same bounded arc at a reversing cusp, requires
the existing position/heading rejoin tolerance and checks the swept body on
both maps. It does not delete obstacles or accept arbitrary steering.

Upstream source and license:
https://github.com/ros-navigation/navigation2/tree/1.1.20/nav2_regulated_pure_pursuit_controller
Apache License 2.0. The compute flow derives from the implementation by
Shrijit Singh and Samsung Research America (2020).

The algebra tests do not establish collision checking or vehicle execution.
Actual plugin loading, footprint collision tests and compound motion must
pass before another complete search. A Nav2 version change requires a rebuild
and renewed compatibility checks.
