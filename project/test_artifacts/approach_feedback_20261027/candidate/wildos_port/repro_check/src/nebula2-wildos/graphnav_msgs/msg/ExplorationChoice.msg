# Read-only vehicle-cost comparison result. Header/route_endpoint identify
# the originally issued graph route, not an arbitrary replacement destination.
std_msgs/Header header
string exploration_id
geometry_msgs/Point route_endpoint
geometry_msgs/Pose evaluation_start
string chosen_uuid
float64 vehicle_cost
