# Result of attempting to traverse a graph edge. This is intentionally an
# observation rather than a command so mapping, planning, and learning nodes
# can consume the same execution evidence.

uint8 OUTCOME_UNKNOWN=0
uint8 OUTCOME_SUCCEEDED=1
uint8 OUTCOME_BLOCKED_GEOMETRY=2
uint8 OUTCOME_BLOCKED_DYNAMIC=3
uint8 OUTCOME_STUCK=4
uint8 OUTCOME_SLIPPED=5
uint8 OUTCOME_COLLISION=6
uint8 OUTCOME_TIMEOUT=7
uint8 OUTCOME_ABORTED=8

std_msgs/Header header
UUID from_node
UUID to_node
string traversability_class
string robot_profile_id

geometry_msgs/Pose start_pose
geometry_msgs/Pose end_pose
# Headings are radians in header.frame_id; distances are metres and duration seconds.
float32 commanded_heading
float32 observed_heading
float32 commanded_distance
float32 observed_distance
float32 duration
float32 energy_cost
float32 slip_ratio

uint8 outcome
bool retryable

# Additional measurements such as peak pitch, wheel current, or obstacle age.
KeyValue[] observations
