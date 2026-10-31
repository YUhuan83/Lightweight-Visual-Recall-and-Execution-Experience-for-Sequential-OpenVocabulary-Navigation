import ast,json
from copy import deepcopy
from pathlib import Path
import unittest
from unittest.mock import Mock
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from test_goal_confirmation_ros import GoalConfirmationTests

ROOT=Path(__file__).parents[2]

class ConfirmationProfileTests(unittest.TestCase):
    tearDown=GoalConfirmationTests.tearDown
    update_odom=GoalConfirmationTests.update_odom
    def setUp(self):
        GoalConfirmationTests.setUp(self)
        self.messages=json.loads((Path(__file__).parent/'fixtures/v29_r2_sparse_localizations.json').read_text())
        self.node.on_query(String(data=self.messages[0][1]['query']))
        self.node.publish_standoff=Mock(return_value='isolated_no_map')

    def feed(self,index,*,shift=0.,age=None):
        received,data=deepcopy(self.messages[index]);data['observation_stamp_ns']+=int(shift*1e9)
        self.now=received+int(shift*1e9) if age is None else data['observation_stamp_ns']+int(age*1e9)
        o=Odometry();o.header.frame_id='spot1/odom'
        o.header.stamp=Time(sec=self.now//10**9,nanosec=self.now%10**9)
        o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=data['observer_position']
        q=o.pose.pose.orientation;q.x,q.y,q.z,q.w=data['observer_orientation']
        self.node.on_odom(o)
        self.node.on_localization(String(data=json.dumps(data)))

    def test_runtime_profile_gives_sparse_confirmation_sixty_second_history(self):
        source=ROOT/'official_adapter/launch/official_graph_stack.launch.py'
        tree=ast.parse(source.read_text(encoding='utf8'))
        values=[value.value for node in ast.walk(tree) if isinstance(node,ast.Dict)
                for key,value in zip(node.keys,node.values)
                if isinstance(key,ast.Constant) and key.value=='confirmation_window_sec' and isinstance(value,ast.Constant)]
        self.assertEqual(values,[60.])

    def test_recorded_four_positions_confirm_without_lowering_support(self):
        self.node.confirmation_window_sec=60.
        for i in range(4):self.feed(i)
        result=json.loads(self.node.confirmation_pub.publish.call_args.args[0].data)
        self.assertTrue(result['search_confirmed']);self.assertEqual(result['inliers'],4)
        self.assertLess(result['radius'],.05)
        self.assertFalse(result['approach_goal_ready'])

    def test_recorded_thirty_second_window_loses_the_first_frame(self):
        self.node.confirmation_window_sec=30.
        for i in range(4):self.feed(i)
        self.assertEqual(len(self.node.observations),3)
        self.node.confirmation_pub.publish.assert_not_called()

    def test_three_frames_still_cannot_confirm(self):
        self.node.confirmation_window_sec=60.
        for i in range(3):self.feed(i)
        self.node.confirmation_pub.publish.assert_not_called()

    def test_expired_history_cannot_complete_on_a_new_single_frame(self):
        self.node.confirmation_window_sec=60.
        for i in range(3):self.feed(i)
        span=(self.messages[3][1]['observation_stamp_ns']-self.messages[0][1]['observation_stamp_ns'])/1e9
        self.feed(3,shift=61.-span)
        self.node.confirmation_pub.publish.assert_not_called()

    def test_latest_measurement_freshness_is_still_five_seconds(self):
        self.node.confirmation_window_sec=60.
        for i in range(3):self.feed(i)
        self.feed(3,age=6.)
        self.node.confirmation_pub.publish.assert_not_called()

    def test_repeated_frame_is_not_additional_support(self):
        self.node.confirmation_window_sec=60.
        for _ in range(4):self.feed(0)
        self.assertEqual(len(self.node.observations),1)
        self.node.confirmation_pub.publish.assert_not_called()

    def test_query_switch_still_discards_history(self):
        self.node.confirmation_window_sec=60.
        for i in range(3):self.feed(i)
        self.node.on_query(String(data='gas station'))
        self.feed(3)
        self.assertFalse(self.node.observations)
        self.node.confirmation_pub.publish.assert_not_called()
