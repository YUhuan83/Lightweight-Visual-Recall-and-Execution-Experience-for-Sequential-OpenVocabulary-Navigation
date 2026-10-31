# Task-level record for offline or continual LRN-style training. Experience
# payloads remain in the visual-memory store; ids keep this message compact.

std_msgs/Header header
string episode_id
string scene_id
string robot_profile_id

# Original instruction and its grounded relational representation.
string task_instruction
SemanticRelation[] task_relations

# Ordered references to VisualExperience records collected during this task.
string[] visual_experience_ids

# Actual execution outcomes, including failed attempts and recoveries.
TraversalFeedback[] traversal_feedback

# Episode-level supervision and efficiency signals.
bool completed
float32 task_success
float32 path_length
float32 duration
float32 energy_cost
uint32 blocked_attempts
uint32 recovery_count

# Dataset split, annotator, reward decomposition, and provenance metadata.
KeyValue[] properties
