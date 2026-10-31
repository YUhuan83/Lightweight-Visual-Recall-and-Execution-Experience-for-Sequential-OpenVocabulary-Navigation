# Geometric and physical traversability of a graph node for one robot profile.
# Continuous scores use [0, 1]; feasible=false is a hard rejection.

string robot_profile_id
bool feasible
float32 physical_traversability

# Local support-surface geometry and footprint clearance, in metres/radians.
float32 minimum_clearance
float32 maximum_uphill_slope
float32 maximum_downhill_slope
float32 maximum_cross_slope
float32 maximum_step_height
float32 roughness

# Local dynamic occupancy/risk and its freshness.
float32 dynamic_risk
builtin_interfaces/Time observed_at
float32 confidence

# Platform or sensor-specific measurements.
KeyValue[] properties
