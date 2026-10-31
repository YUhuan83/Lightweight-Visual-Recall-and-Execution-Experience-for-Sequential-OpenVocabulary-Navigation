# Compact visual memory/training sample associated with a graph node. Images
# stay in the sensor pipeline; this message stores reusable embeddings and the
# supervision needed for retrieval and continual traversability learning.

std_msgs/Header header
UUID node
geometry_msgs/Pose observation_pose

string scene_id
string encoder_id
uint32 embedding_dim
float32[] embedding

# Candidate action represented in the robot/world heading convention selected
# by the producer. The frame is stated explicitly to avoid hidden assumptions.
string heading_frame
float32 heading

# Model predictions captured before execution.
float32 frontier_score
float32 semantic_traversability
float32 physical_traversability
float32 fused_traversability
float32 novelty

# Candidate-heading features used for LRN-style learning-to-rank. The chosen
# action is marked selected; after execution, attach labels to evaluated
# candidates and keep detailed execution evidence in feedback.
HeadingCandidate[] heading_candidates

# Optional semantic and relational context.
string[] semantic_labels
SemanticRelation[] relations

# Ground-truth supervision becomes available after an attempted traversal.
bool has_feedback
TraversalFeedback feedback

# Dataset provenance, uncertainty, retrieval score, and other extensions.
KeyValue[] properties
