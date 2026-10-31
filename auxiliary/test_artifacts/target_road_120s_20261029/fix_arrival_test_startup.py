"""Supply a frozen ROS parameter at node startup, as the field launch does."""
import hashlib
import json
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v70_upstream_arrival_120s')
test=root/'candidate/nav2_adapter/test/test_upstream_target_approach_ros.py'
text=test.read_text().replace('from rclpy.parameter import Parameter','import rclpy')
text=text.replace('        self.b.set_parameters([Parameter("object_arrival_distance", value=3.0)])\n','')
anchor='class UpstreamTargetApproachTests(unittest.TestCase):\n'
insert='''    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-p', 'object_arrival_distance:=3.0'])

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

'''
assert text.count(anchor)==1;test.write_text(text.replace(anchor,anchor+insert))
runner=root/'run_arrival_regression.py';text=runner.read_text().replace('rclpy.init()\n','');runner.write_text(text)
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
name='nav2_adapter/test/test_upstream_target_approach_ros.py'
manifest['members'][name]=hashlib.sha256(test.read_bytes()).hexdigest()
(root/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
