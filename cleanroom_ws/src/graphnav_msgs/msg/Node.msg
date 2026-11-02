UUID uuid

geometry_msgs/Pose pose
NodeTraversabilityProperties[] trav_properties  # same order as NavigationGraph::trav_classes

# Platform-specific local feasibility, separate from semantic traversability.
RobotNodeTraversability[] robot_traversability

# Direction-conditioned visual and physical candidates.  The planner can use
# these without expanding the graph into a full node-heading lattice yet.
HeadingCandidate[] heading_candidates

KeyValue[] properties
