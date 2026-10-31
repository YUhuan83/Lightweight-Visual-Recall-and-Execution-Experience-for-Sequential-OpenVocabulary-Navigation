# A grounded relation extracted from language or perception. Entity identifiers
# are stable within a scene and may refer to graph nodes or detected objects.

string subject_id
string predicate
string object_id
float32 confidence

# Coordinate frame used by spatial predicates such as left_of and behind.
string reference_frame

# Optional numeric grounding, for example distance or relative bearing.
KeyValue[] properties
