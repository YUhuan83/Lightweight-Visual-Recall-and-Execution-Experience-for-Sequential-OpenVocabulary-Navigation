uint32 to_idx
uint32 from_idx

EdgeTraversability[] traversability  # same order as NavigationGraph::trav_classes

# Geometry- and platform-specific assessment. Kept separate from semantic
# traversability classes so one graph can be evaluated for multiple robots.
RobotEdgeTraversability[] robot_traversability

KeyValue[] properties
