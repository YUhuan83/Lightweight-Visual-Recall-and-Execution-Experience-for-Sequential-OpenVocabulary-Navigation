"""Counterexample only: source may expire after check and before envelope issue."""
from pathlib import Path
import sys,unittest
from unittest.mock import patch
root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'nav2_adapter'),str(root/'nav2_adapter/test')]
from offline_ros_fakes import install
install()
from test_controller_source_ros import ControllerSourceTests,W

class ExpiryRace(ControllerSourceTests):
    def test_pause_between_remaining_check_and_issue(self):
        self.command();self.at(.499)
        # Scheduler pause crosses the source deadline, without any wall rollback.
        with patch('time.time_ns',side_effect=[W+499_000_000,W+501_000_000]):
            with self.assertRaisesRegex(ValueError,'source_lifetime'):
                self.publish()

suite=unittest.defaultTestLoader.loadTestsFromName('test_pause_between_remaining_check_and_issue',ExpiryRace)
result=unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(not result.wasSuccessful())
