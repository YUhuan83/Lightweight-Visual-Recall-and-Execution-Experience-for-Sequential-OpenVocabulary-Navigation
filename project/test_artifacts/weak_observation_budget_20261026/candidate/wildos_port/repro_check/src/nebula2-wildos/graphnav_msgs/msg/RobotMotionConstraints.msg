# Robot-dependent limits used to turn geometric terrain measurements into
# physical traversability. Values smaller than zero mean "not specified".

string profile_id

# Axis-aligned body envelope in the robot base frame (length, width, height).
geometry_msgs/Vector3 body_size
float32 ground_clearance

# Terrain limits. Slope values are in radians and lengths are in metres.
float32 max_uphill_slope
float32 max_downhill_slope
float32 max_cross_slope
float32 max_step_height
float32 max_roughness

# Planar kinematic and dynamic limits.
bool holonomic
float32 min_turning_radius
float32 max_linear_velocity
float32 max_linear_acceleration
float32 max_linear_deceleration
float32 max_angular_velocity

# Platform-specific limits, for example wheelbase or allowable wheel slip.
KeyValue[] properties
