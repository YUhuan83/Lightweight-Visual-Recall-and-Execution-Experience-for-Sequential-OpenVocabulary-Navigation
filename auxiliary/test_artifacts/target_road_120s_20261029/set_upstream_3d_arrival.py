"""Restore upstream 3D distance < 3 m, replacing the body-clearance adaptation."""
import ast
import hashlib
import json
from pathlib import Path
import sys

root=Path('/home/ly/execution_continuity_review_20261022/v70_upstream_arrival_120s')
candidate=root/'candidate';package=candidate/'nav2_adapter/wildos_nav2_adapter'
tests=candidate/'nav2_adapter/test/test_upstream_target_approach_ros.py'
if '--apply' not in sys.argv:
    text=tests.read_text()
    text=text.replace('from geometry_msgs.msg import PoseStamped','from geometry_msgs.msg import PoseStamped\nfrom rclpy.parameter import Parameter')
    text=text.replace('        self.b.upstream_target_approach = True\n','        self.b.upstream_target_approach = True\n        self.b.set_parameters([Parameter("object_arrival_distance", value=3.0)])\n')
    text=text.replace('z=8.', 'z=0.')
    text=text.replace('self.move(14.599)', 'self.move(17.0)').replace('self.move(14.6)', 'self.move(17.001)')
    text=text.replace('self.assertEqual(sent.pose.position.z, 8.)','self.assertEqual(sent.pose.position.z, 0.)')
    text=text.replace("[20., 0., 8.]", "[20., 0., 0.]")
    text=text.replace("self.assertAlmostEqual(self.b.last_status['object_distance_m'], 2.)", "self.assertAlmostEqual(self.b.last_status['object_distance_m'], 2.999)")
    text=text.replace('self.goal(5.4)', 'self.goal(2.999)').replace('self.b.expected, (5.4, 0.)', 'self.b.expected, (2.999, 0.)')
    text=text.replace('self.goal(0., 5.4)', 'self.goal(0., 2.999)')
    text=text.replace('def test_rotated_body_clearance_is_used_without_a_standoff', 'def test_orientation_does_not_change_euclidean_arrival')
    text=text.replace('point=(20., 0., 8.)', 'point=(20., 0., 0.)')
    anchor='    def test_real_arrival_event_is_accepted_only_with_independent_image_evidence(self):'
    extra='''    def test_vertical_gap_is_part_of_arrival(self):
        self.b.on_waypoint(self.goal(0., 0., 3.001))
        self.assertFalse(self.b.object_found)

    def test_diagonal_three_dimensional_distance_is_used(self):
        self.b.on_waypoint(self.goal(2., 0., 2.3))
        self.assertFalse(self.b.object_found)
        self.b.on_waypoint(self.goal(2., 0., 2.1))
        self.assertTrue(self.b.object_found)
        self.assertAlmostEqual(self.b.last_status['object_distance_m'], math.sqrt(8.41))

'''
    assert text.count(anchor)==1;tests.write_text(text.replace(anchor,extra+anchor))
else:
    bridge=package/'graph_path_bridge.py';text=bridge.read_text()
    before=root/'before_3d';before.mkdir(exist_ok=True)
    (before/'graph_path_bridge.py').write_text(text)
    old='''        self.declare_parameter("traffic_light_arrival_distance", BODY_OBJECT_ARRIVAL_DISTANCE_M)
        self.declare_parameter("object_arrival_distance", BODY_OBJECT_ARRIVAL_DISTANCE_M)'''
    new='''        arrival_default = 3.0 if self.upstream_target_approach else BODY_OBJECT_ARRIVAL_DISTANCE_M
        self.declare_parameter("traffic_light_arrival_distance", arrival_default)
        self.declare_parameter("object_arrival_distance", arrival_default)'''
    assert old in text;text=text.replace(old,new)
    tree=ast.parse(text);cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='GraphPathBridge')
    node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='check_object_goal_distance')
    method='''    def check_object_goal_distance(self):
        if (not self.upstream_target_approach or self.object_found
                or self.object_destination is None or self.odom is None
                or self.odom_received_at is None
                or time.monotonic() - self.odom_received_at > self.odom_timeout):
            return
        goal = self.object_destination.pose.position
        vehicle = self.odom.pose.pose.position
        position = [vehicle.x, vehicle.y, vehicle.z]
        object_position = [goal.x, goal.y, goal.z]
        goal_distance = math.dist(position, object_position)
        tolerance = float(self.get_parameter('object_arrival_distance').value)
        if not goal_distance < tolerance:
            return
        # WildOS PlannerNode: (goal_vec - odom_vec).norm() < goal_radius.
        self.object_found = True
        self.reset()
        self.memory_selecting = False
        self.pending_destination = None
        self.request_graph_reset()
        self.retry_graph_reset()
        self.status('object_found', query=self.query,
                    object_position=object_position,
                    arrival_basis='upstream_goal_distance_3d',
                    vehicle_position=position,
                    object_distance_m=goal_distance,
                    object_arrival_tolerance_m=tolerance,
                    memory_view_id=(self.memory_active.get('id')
                                    if self.memory_active else None))
'''
    lines=text.splitlines(keepends=True);lines[node.lineno-1:node.end_lineno]=[method]
    text=''.join(lines);ast.parse(text);bridge.write_text(text)
    criteria=package/'arrival_criteria.py';text=criteria.read_text();(before/'arrival_criteria.py').write_text(text)
    first=text.index("    if status.get('arrival_basis') == 'upstream_goal_distance_with_body_clearance':")
    last=text.index('    status_confirmation = status.get("confirmation")',first)
    replacement='''    if status.get('arrival_basis') == 'upstream_goal_distance_3d':
        try:
            position = status['vehicle_position']
            goal = status['object_position']
            observed = confirmation['object_position']
            if any(len(point) != 3 for point in (position, goal, observed)):
                return False
            goal_distance = math.dist(position, goal)
            observed_distance = math.dist(position, observed)
            limit = float(status['object_arrival_tolerance_m'])
            reported = float(status['object_distance_m'])
        except (KeyError, ValueError, TypeError, OverflowError):
            return False
        return (confirmation.get('query') == query and limit == 3.0
                and math.isfinite(reported) and abs(goal_distance-reported) <= 1e-3
                and goal_distance < limit and observed_distance < limit
                and math.dist(goal, observed) < limit)
'''
    text=text[:first]+replacement+text[last:];ast.parse(text);criteria.write_text(text)
    contract=package/'runtime_contract.py';text=contract.read_text()
    old="'object_arrival_distance': 2.0, 'traffic_light_arrival_distance': 2.0"
    assert old in text;contract.write_text(text.replace(old,"'object_arrival_distance': 3.0, 'traffic_light_arrival_distance': 3.0"))
    manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
    for name in manifest['members']:
        manifest['members'][name]=hashlib.sha256((candidate/name).read_bytes()).hexdigest()
    (root/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    scope=json.loads((root/'candidate_scope.json').read_text())
    scope['runtime_delta'].append('nav2_adapter/wildos_nav2_adapter/runtime_contract.py')
    scope['arrival']='Exact upstream criterion: XYZ Euclidean distance strictly <3m; no body offset, standoff, or new confirmation needed for navigation completion.'
    scope['user_replaced_prior_2m_body_rule']=True
    (root/'candidate_scope.json').write_text(json.dumps(scope,indent=2));print(json.dumps(scope,indent=2))
