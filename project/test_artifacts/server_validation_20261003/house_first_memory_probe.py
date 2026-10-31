"""Stationary mechanism probe: initial house query, first station query later.

This verifies historical recall with real camera/LiDAR/model data. It does not
claim driving success or completion of a house search.
"""
from pathlib import Path
import json
import time
from beamngpy import BeamNGpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

root = Path('/home/ly/town_memory_validation_20261003')
out = root/'lightweight_memory_probe'
out.mkdir(exist_ok=False)
rclpy.init()
b = BeamNGpy('127.0.0.1', 25252).open(launch=False)
n = Node('house_first_memory_probe')
events = []
query = n.create_publisher(String, '/spot1/object_search/query', 10)
started = time.monotonic()
def receive(kind, message):
    events.append(dict(kind=kind, elapsed=time.monotonic()-started, payload=json.loads(message.data)))
subscriptions = [n.create_subscription(String, topic, lambda m, kind=kind: receive(kind, m), 10)
                 for topic, kind in [('/spot1/remembered_object_candidates', 'candidates'),
                                     ('/spot1/object_confirmation', 'confirmation')]]
report = dict(scope=__doc__, vision_pid=int((root/'vision.pid').read_text()),
              initial_query='house', station_preseeded=False, passed=False)
try:
    b.control.resume()
    until = time.monotonic()+25
    while time.monotonic() < until:
        rclpy.spin_once(n, timeout_sec=.05)
    query.publish(String(data='house'))
    until = time.monotonic()+15
    while time.monotonic() < until:
        rclpy.spin_once(n, timeout_sec=.05)
    report['station_confirmations_before_query'] = [e for e in events if e['kind']=='confirmation'
        and e['payload'].get('query')=='gas station']
    switch = time.monotonic()
    query.publish(String(data='gas station'))
    until = switch+60
    while time.monotonic() < until:
        rclpy.spin_once(n, timeout_sec=.05)
        station = [e for e in events if e['kind']=='candidates' and e['payload'].get('query')=='gas station']
        fresh = [e for e in events if e['kind']=='confirmation' and e['payload'].get('query')=='gas station']
        if station and fresh:
            break
    station = [e for e in events if e['kind']=='candidates' and e['payload'].get('query')=='gas station']
    report['first_station_candidates'] = station[0]['payload'] if station else None
    report['station_confirmations_after_query'] = [e for e in events if e['kind']=='confirmation'
        and e['payload'].get('query')=='gas station']
    views = station[0]['payload']['views'] if station else []
    report['passed'] = bool(views and all(v.get('evidence')=='passive_semantic_landmarks' for v in views)
                           and not report['station_confirmations_before_query'])
    report['fresh_confirmation_obtained'] = bool(report['station_confirmations_after_query'])
finally:
    b.control.pause()
    b.disconnect()
    report['events'] = events
    (out/'probe.json').write_text(json.dumps(report, indent=2))
    n.destroy_node()
    rclpy.shutdown()
print(json.dumps({k:v for k,v in report.items() if k!='events'}, indent=2), flush=True)
raise SystemExit(0 if report['passed'] else 1)
