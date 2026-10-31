# Physical and motion feasibility of a directed graph edge for one robot
# profile. Heading transitions are a flattened row-major NxN matrix: row is
# start-heading bin, column is end-heading bin. Angles are radians in the
# declared heading_frame, and a negative transition cost means infeasible.

string robot_profile_id
string heading_frame

float32 physical_traversability
float32 minimum_clearance
float32 maximum_slope
float32 maximum_step_height
float32 roughness
float32 dynamic_risk

float32 estimated_travel_time
float32 estimated_energy

float32 heading_bin_start
float32 heading_bin_size
uint16 heading_bin_count
float32[] heading_transition_costs

# Evidence freshness and uncertainty allow planners to discount stale terrain.
builtin_interfaces/Time observed_at
float32 confidence
KeyValue[] properties
