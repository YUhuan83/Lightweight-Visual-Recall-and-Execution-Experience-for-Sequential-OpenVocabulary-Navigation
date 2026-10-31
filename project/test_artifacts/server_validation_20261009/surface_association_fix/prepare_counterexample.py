"""Preserve the stopped trial14 counterexample before production changes."""
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace as S
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
CASE = ROOT/'test_artifacts/server_validation_20261009/obstacle_restore/server_evidence/trial14'
OUT = Path(__file__).resolve().parent
VIS = ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation'
sys.path[:0] = [str(ROOT/'nav2_adapter'), str(ROOT/'nav2_adapter/test'), str(VIS)]
from offline_ros_fakes import install
install()
from visual_navigation.explorfm_triangulation.localization_geometry import foreground_depth
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge
from nav_msgs.msg import Odometry
from std_msgs.msg import String

stamp = 1791513863084888339
original = CASE/f'blue_house_review/{stamp}_projection.npz'
row = next(r for r in json.loads((CASE/'blue_house_review/projection_review.json').read_text())['rows']
           if r['mask_stamp_ns'] == stamp)
with np.load(original) as sample:
    points, mask, k = sample['matched_pose_points_cam'], sample['mask'], np.array(row['K'])
    points = points[np.all(np.isfinite(points), axis=1) & (points[:, 2] > .1)]
    projected = points @ k.T
    pixels = projected[:, :2]/projected[:, 2:]
    inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < mask.shape[1])
              & (pixels[:, 1] >= 0) & (pixels[:, 1] < mask.shape[0]))
    points, pixels = points[inside], pixels[inside].astype(int)
    points = points[mask[pixels[:, 1], pixels[:, 0]]]
fixture = VIS/'test/fixtures/trial14_ambiguous_surfaces.npz'
np.savez_compressed(fixture, points_cam=points, k=k, mask=mask)
quality = foreground_depth(points, k, mask)
arrival = json.loads((CASE/'full_test.json').read_text())['green_house_arrival']['status']
node = GraphPathBridge()
now = stamp + 100_000_000
node.get_clock = lambda: S(now=lambda: S(nanoseconds=now, to_msg=lambda: S(sec=now//10**9, nanosec=now%10**9)))
node.query = 'green residential house'
node.query_started_ns = stamp-30_000_000_000
node.query_reset_pending = False
node.odom = Odometry()
node.odom.pose.pose.position.x, node.odom.pose.pose.position.y = arrival['vehicle_position']
node.odom_received_at = time.monotonic()
node.on_object_confirmation(String(data=json.dumps(arrival['confirmation'])))
report = dict(scope='Offline production geometry and arrival callback with ROS interface doubles',
              original_npz=str(original), original_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
              fixture=str(fixture), stamp_ns=stamp, surface_depth_m=float(quality['position'][2]),
              geometric_confidence=float(quality['confidence']), false_arrival=node.object_found,
              server_baseline='f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0')
(OUT/'before.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
print(json.dumps(report, ensure_ascii=False, indent=2))
node.destroy_node()
