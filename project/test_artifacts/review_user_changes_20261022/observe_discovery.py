"""Two independent passive DDS discovery windows; never writes control."""
from collections import Counter
from pathlib import Path
import json
import time
import rclpy

out=Path('/home/ly/execution_continuity_review_20261022/v9/discovery_observation.json')
rclpy.init()
windows=[]
try:
    for repeat in (1,2):
        node=rclpy.create_node('passive_discovery_audit_%d'%repeat)
        started=time.monotonic();rows=[];previous=None
        try:
            while time.monotonic()-started < 30.:
                rclpy.spin_once(node,timeout_sec=.1)
                counts=Counter(('/' if ns=='/' else ns+'/')+name
                               for name,ns in node.get_node_names_and_namespaces())
                status={name:counts[name] for name in
                    ('/beamng_wildos_adapter','/wildos_cmd_vel_to_control','/wildos_graph_path_bridge')}
                status['odom_producers']=[p.node_name for p in node.get_publishers_info_by_topic('/spot1/odom')]
                signature=json.dumps(status,sort_keys=True)
                elapsed=time.monotonic()-started
                if signature!=previous or not rows or elapsed-rows[-1]['elapsed_s']>=4.:
                    rows.append(dict(elapsed_s=elapsed,wall=time.time(),**status));previous=signature
        finally:
            node.destroy_node()
        windows.append(dict(repeat=repeat,rows=rows))
        out.write_text(json.dumps(windows,indent=2))
        print(json.dumps(windows[-1]),flush=True)
finally:
    rclpy.shutdown()
