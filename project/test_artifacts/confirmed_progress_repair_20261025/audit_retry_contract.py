"""Before-fix counterexamples only; does not edit frozen production candidate."""
import sys,time,unittest
from pathlib import Path
from unittest.mock import Mock
root=Path(__file__).parent/'candidate'
sys.path[:0]=[str(root/'tools'),str(root/'nav2_adapter/test'),str(root/'nav2_adapter'),str(root/'official_adapter'),str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
import test_continuous_exploration_ros as fixture
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge

class RetryContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):fixture.ContinuousExplorationTests.setUpClass()
    @classmethod
    def tearDownClass(cls):fixture.ContinuousExplorationTests.tearDownClass()
    def setUp(self):
        self.f=fixture.ContinuousExplorationTests();self.f.setUp();self.addCleanup(self.f.doCleanups)
        n=self.n=self.f.moving()
        n.following=False;n.follow_handle=n.follow_active=n.active_follow_serial=None
        n.motion_queue=[];n.pending=False
        n.sync_recovery_progress(n.route)
        n.block_current_route(backoff=4.)
        n.last_status=dict(status='follow_failed')
        n.start_segment=Mock()
        self.incoming=self.f.route((12.,0.),'frontier-A')
    def test_base_preserves_unexpired_backoff_for_same_route(self):
        GraphPathBridge.on_route(self.n,self.incoming)
        self.n.start_segment.assert_not_called()
    def test_continuous_must_preserve_unexpired_backoff_for_same_route(self):
        self.n.on_route(self.incoming)
        self.n.start_segment.assert_not_called()
    def test_base_preserves_exhausted_retry_budget(self):
        self.n.recovery_attempts=3
        GraphPathBridge.on_route(self.n,self.incoming)
        self.n.start_segment.assert_not_called()
    def test_continuous_must_preserve_exhausted_retry_budget(self):
        self.n.recovery_attempts=3
        self.n.on_route(self.incoming)
        self.n.start_segment.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
