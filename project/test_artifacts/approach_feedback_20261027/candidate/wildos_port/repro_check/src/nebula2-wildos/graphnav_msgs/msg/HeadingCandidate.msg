# One candidate motion direction considered by the local visual navigator.
# Heading is radians in heading_frame. Supervision fields are valid only when
# has_supervision is true; target_utility is normalized to [0, 1].

string heading_frame
float32 heading

float32 frontier_score
float32 semantic_traversability
float32 physical_traversability
float32 minimum_clearance
float32 dynamic_risk
float32 motion_cost
float32 predicted_utility

bool selected
bool has_supervision
float32 target_utility
bool traversable
