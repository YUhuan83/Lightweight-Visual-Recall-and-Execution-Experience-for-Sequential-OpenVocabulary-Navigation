# Batched visual-memory records. Publish on demand or at a low rate; do not
# attach this payload to every NavigationGraph update.

std_msgs/Header header
string scene_id
string encoder_id
uint64 batch_id
bool complete_snapshot
VisualExperience[] experiences
