"""Measured geometry of the Town Hirochi SBR, in ROS body axes (x forward).

2026-10-02: FL/FR/RL/RR wheel hub nodes read from the live vehicle.
The navigation origin is the rear axle in XY, keeping StateSensor height.
Extents include a small margin around the measured body bounding box.
Other vehicles must supply their own geometry rather than reuse this profile.
"""
SBR_BASE_OFFSET = (-2.172, 0.330, 0.0)
SBR_WHEELBASE = 2.501
SBR_FRONT_EXTENT = 3.4
SBR_REAR_EXTENT = 1.05
SBR_HALF_WIDTH = 1.05
# SBR JBeam body roof is 1.21 m above its origin. Keep 0.29 m clearance
# for suspension/roll; 3 m branches must not become ground obstacles.
SBR_OBSTACLE_MAX_HEIGHT = 1.5
