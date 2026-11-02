# Tracked dynamic obstacle and its time-indexed short-horizon prediction.

uint8 STATE_UNKNOWN=0
uint8 STATE_MOVING=1
uint8 STATE_STATIONARY=2
uint8 STATE_OCCLUDED=3
uint8 STATE_LOST=4

std_msgs/Header header
string track_id
string semantic_class
float32 confidence
uint8 state

geometry_msgs/Pose pose
geometry_msgs/Twist velocity

# Full axis-aligned extent in the pose frame (length, width, height).
geometry_msgs/Vector3 size

# Parallel arrays describing future occupancy. Producers must publish equal
# lengths; an empty prediction represents a current-only observation.
builtin_interfaces/Time[] prediction_stamps
geometry_msgs/Pose[] predicted_poses
float32[] occupancy_probabilities

# Tracker covariance, source sensor, interaction class, and other extensions.
KeyValue[] properties
