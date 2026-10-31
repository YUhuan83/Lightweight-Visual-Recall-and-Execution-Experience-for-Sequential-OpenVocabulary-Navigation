"""Run new regressions against the previous callback/timeout rules in memory."""
from pathlib import Path
import inspect, sys, textwrap, unittest

root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'nav2_adapter/test'),str(root/'nav2_adapter'),
             str(root/'official_adapter'),
             str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
from wildos_nav2_adapter.navigation_transactions import NavigationTransactions
from wildos_nav2_adapter.visible_approach import VisibleTargetApproach
import wildos_nav2_adapter.visible_approach as visible
import test_navigation_transactions_ros as transactions
import test_visible_approach_ros as approaches

# Restore only the pre-change rules; no working-tree source is rewritten.
NavigationTransactions.add_navigation_done_callback = lambda self,f,c:f.add_done_callback(c)
namespace=dict(vars(visible))
source=textwrap.dedent(inspect.getsource(VisibleTargetApproach.activate_visible_approach))
source=source.replace('self.route is None and not self.following', 'not self.following')
exec(source,namespace)
VisibleTargetApproach.activate_visible_approach=namespace['activate_visible_approach']
source=textwrap.dedent(inspect.getsource(VisibleTargetApproach.check_visible_approach_deadline))
source=source.replace('        if self.execution_recovery_owns_motion():\n            return False\n','')
exec(source,namespace)
VisibleTargetApproach.check_visible_approach_deadline=namespace['check_visible_approach_deadline']
suite=unittest.TestSuite([
    unittest.defaultTestLoader.loadTestsFromTestCase(transactions.NavigationResultQueueTests),
    approaches.VisibleApproachTests('test_admitted_observation_route_is_not_timed_out_between_segments'),
    approaches.VisibleApproachTests('test_expired_observation_does_not_consume_physical_recovery_tick')])
result=unittest.TextTestRunner(verbosity=2).run(suite)
assert result.testsRun==4 and len(result.failures)==4 and not result.errors
print('Confirmed: all four new counterexamples fail under the previous rules.')
