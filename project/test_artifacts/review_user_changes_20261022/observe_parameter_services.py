"""Read-only service probes alongside startup; no parameter writes or actuation."""
from collections import Counter
from pathlib import Path
import json
import os
import time
import rclpy
from rcl_interfaces.srv import GetParameters

out=Path('/home/ly/execution_continuity_review_20261022/v9/parameter_service_observation.json')
report={'environment':{key:os.environ.get(key) for key in
    ('ROS_DOMAIN_ID','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE')},'windows':[]}
rclpy.init()
try:
    for repeat in (1,2):
        node=rclpy.create_node('passive_parameter_audit_%d'%repeat)
        clients={name:node.create_client(GetParameters,name+'/get_parameters') for name in
                 ('/wildos_cmd_vel_to_control','/wildos_graph_path_bridge')}
        start=time.monotonic();rows=[]
        try:
            while time.monotonic()-start<30.:
                rclpy.spin_once(node,timeout_sec=.1)
                counts=Counter(('/' if ns=='/' else ns+'/')+name for name,ns in node.get_node_names_and_namespaces())
                row=dict(elapsed_s=time.monotonic()-start,wall=time.time(),counts={
                    name:counts[name] for name in ('/beamng_wildos_adapter',*clients)})
                for name,client in clients.items():
                    request_start=time.monotonic()
                    if not client.service_is_ready():
                        row[name]={'ready':False};continue
                    future=client.call_async(GetParameters.Request(names=['enable' if 'cmd_vel' in name else 'follow_path']))
                    rclpy.spin_until_future_complete(node,future,timeout_sec=2.)
                    row[name]={'ready':True,'response':future.done(),'elapsed_s':time.monotonic()-request_start,
                               'error':str(future.exception()) if future.done() and future.exception() else None}
                    if not future.done():client.remove_pending_request(future)
                rows.append(row)
                wait_until=time.monotonic()+.8
                while time.monotonic()<wait_until:rclpy.spin_once(node,timeout_sec=.1)
        finally:node.destroy_node()
        report['windows'].append(dict(repeat=repeat,rows=rows));out.write_text(json.dumps(report,indent=2))
        print(json.dumps({'repeat':repeat,'samples':len(rows),'last':rows[-1]}),flush=True)
finally:rclpy.shutdown()
