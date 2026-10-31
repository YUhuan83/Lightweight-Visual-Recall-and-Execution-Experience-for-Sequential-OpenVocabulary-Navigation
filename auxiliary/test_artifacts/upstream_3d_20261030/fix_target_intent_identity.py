"""Keep upstream goal-coordinate refinement separate from query identity."""
import hashlib
import json
from pathlib import Path
import sys
root=Path('/home/ly/execution_continuity_review_20261022/v70_upstream_arrival_120s')
if '--apply' not in sys.argv:
    test=root/'candidate/nav2_adapter/test/test_continuous_exploration_ros.py'
    text=test.read_text();anchor='    def test_new_frontier_plans_while_old_action_continues(self):'
    methods='''    def test_upstream_position_refinement_keeps_inflight_computation(self):
        n=self.moving();n.upstream_target_approach=True;n.expected=(20.,0.)
        key=n.intent_key(n.route)
        n._desired_intent=n._planned_intent=n._motion_intent=key
        n.pending=True
        n.discard_intent_plan=Mock()
        n.restrict_current_direction=Mock(return_value=True)
        n.maybe_extend_path=Mock()
        n.expected=(20.04,.02)
        message=fixtures.route((0.,0.),(20.04,.02))
        n.on_route(message)
        self.assertIs(n.latest_route,message)
        n.discard_intent_plan.assert_not_called()
        n.restrict_current_direction.assert_not_called()
        n.maybe_extend_path.assert_called_once()

    def test_upstream_query_change_still_changes_intent(self):
        n=self.node;n.upstream_target_approach=True;n.expected=(20.,0.)
        path=fixtures.route((0.,0.),(20.,0.));before=n.intent_key(path)
        n.query_generation+=1
        self.assertNotEqual(before,n.intent_key(path))

    def test_recall_viewpoint_identity_is_not_changed_by_upstream_mode(self):
        n=self.node;n.upstream_target_approach=True;n.expected=n.memory_goal=(20.,0.)
        path=fixtures.route((0.,0.),(20.,0.));before=n.intent_key(path)
        n.expected=n.memory_goal=(25.,0.)
        self.assertNotEqual(before,n.intent_key(path))

'''
    assert text.count(anchor)==1;test.write_text(text.replace(anchor,methods+anchor))
else:
    path=root/'candidate/nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py'
    text=path.read_text();anchor='''        else:
            goal = self.expected or self.memory_goal or xy(route.poses[-1].pose)
'''
    replacement='''        elif (self.upstream_target_approach and self.expected is not None
              and self.memory_goal is None):
            # WildOS refines the same query's goal; coordinates are not task IDs.
            destination = ('object',)
        else:
            goal = self.expected or self.memory_goal or xy(route.poses[-1].pose)
'''
    assert text.count(anchor)==1
    (root/'before_3d/continuous_graph_path_bridge.py').write_text(text)
    path.write_text(text.replace(anchor,replacement))
    manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
    for name in manifest['members']:manifest['members'][name]=hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest()
    (root/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    scope=json.loads((root/'candidate_scope.json').read_text())
    scope['runtime_delta'].append('nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py')
    scope['tests_delta'].append('nav2_adapter/test/test_continuous_exploration_ros.py')
    scope['target_identity_fix']='Original-mode goal coordinates refine a query-scoped destination; they do not restart the task. Recall and query switching unchanged.'
    (root/'candidate_scope.json').write_text(json.dumps(scope,indent=2))
