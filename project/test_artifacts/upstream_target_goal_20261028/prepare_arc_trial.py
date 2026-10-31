"""Create independent field wrappers; do not alter v57's frozen sources."""
from pathlib import Path
import hashlib

root=Path('/home/ly/execution_continuity_review_20261022/v57_upstream_target_approach/full')
wrapper=(root/'console_graph_capture.py').read_text()
needle='EXTRA_POINT_TOPICS = ["/spot1/raw_object_waypoint",'
assert wrapper.count(needle)==1
wrapper=wrapper.replace(needle,'EXTRA_POINT_TOPICS = ["/lookahead_collision_arc", "/lookahead_point", "/spot1/raw_object_waypoint",')
driver=(root/'driver_full.py').read_text()
assert driver.count("ROOT/'console_graph_capture.py'")==1
driver=driver.replace("ROOT/'console_graph_capture.py'","ROOT/'console_graph_arc_capture.py'")
for name,data in [('console_graph_arc_capture.py',wrapper),('driver_arc_capture.py',driver)]:
    p=root/name
    assert not p.exists(),f'{p} already exists; inspect before reusing'
    p.write_text(data)
    print(name,hashlib.sha256(p.read_bytes()).hexdigest())
