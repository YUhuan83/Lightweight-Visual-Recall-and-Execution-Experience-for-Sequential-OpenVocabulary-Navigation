std_msgs/Header header

string[] trav_classes
Node[] nodes
Edge[] edges

uint32 current_node_idx # the node that the robot is currently at

# Robot-specific limits used when evaluating the graph's geometric and
# physical traversability. Multiple profiles allow one map to support
# different platforms without confusing semantic terrain classes with robots.
RobotMotionConstraints[] robot_profiles

# Dynamic obstacles and visual experiences are published/stored separately so
# high-rate graph updates do not repeatedly serialize time-varying or large
# embedding payloads. See DynamicObstacleArray and VisualExperienceArray.

