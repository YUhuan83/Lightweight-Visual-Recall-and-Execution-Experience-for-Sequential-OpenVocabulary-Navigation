"""Reuse the existing closed-bag audit with the user's new XYZ metric."""
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v70_upstream_arrival_120s')
text=(root/'audit_target_road.py').read_text()
text=text.replace("position=xy(pose.position)","position=[pose.position.x, pose.position.y, pose.position.z]")
text=text.replace("vehicle_body_distance(xy(pose.position), heading(pose.orientation), target, 'ackermann')",
                  "math.dist([pose.position.x, pose.position.y, pose.position.z], target)")
text=text.replace('body_distance_m','distance_3d_m')
text=text.replace("'target_approach_audit.json'","'target_approach_3d_audit.json'")
text=text.replace('Closed 120s bag and events.', 'Closed 120s bag and events; XYZ Euclidean distance, not body distance.')
(root/'audit_target_3d.py').write_text(text)
