import sys,unittest
from concurrent.futures import Future
from pathlib import Path
from unittest.mock import Mock
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\fresh_bearing_repair_20261025\candidate')
sys.path[:0]=[str(root/'nav2_adapter/test'),str(root/'nav2_adapter'),str(root/'official_adapter'),
    str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
from test_continuous_exploration_ros import ContinuousExplorationTests

class NearIntentTests(ContinuousExplorationTests):
    def test_new_nearby_intent_is_not_filtered_by_old_endpoint(self):
        n=self.moving()
        n.plan_client=Mock();n.plan_client.send_goal_async.return_value=Future()
        n.on_route(self.route((14.,0.),'frontier-B'))
        self.assertTrue(n.following)
        self.assertIsNone(n.follow_release)
        n.plan_client.send_goal_async.assert_called_once()
        requested=n.plan_client.send_goal_async.call_args.args[0].goal.pose.position
        self.assertAlmostEqual(requested.x,14.)

if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromName('test_new_nearby_intent_is_not_filtered_by_old_endpoint',NearIntentTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(not result.wasSuccessful())
