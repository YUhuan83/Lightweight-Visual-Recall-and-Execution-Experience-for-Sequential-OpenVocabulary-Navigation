"""Restore direct object goals and distance completion in the isolated v62 copy."""
import ast
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent
BRIDGE = ROOT / 'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py'
if not (ROOT / 'before/graph_path_bridge.py').exists():
    shutil.copy2(BRIDGE, ROOT / 'before/graph_path_bridge.py')
text = (ROOT / 'before/graph_path_bridge.py').read_text(encoding='utf-8-sig')
text = text.replace('        self.object_found = False\n',
                    '        self.object_found = False\n        self.object_destination = None\n')
old = '''        if (self.upstream_target_approach and self.active_object_confirmation is not None
                and not confirmed_observation):
            return  # An unlabelled raw point cannot replace a confirmed target.
'''
assert old in text
text = text.replace(old, '')
old = '''        if message.header.frame_id != "spot1/odom" or not all(map(math.isfinite, point)):
            return
        if self.memory_goal is not None:
'''
new = '''        if message.header.frame_id != "spot1/odom" or not all(map(math.isfinite, point)):
            return
        if self.upstream_target_approach:
            first = self.object_destination is None
            self.object_destination = deepcopy(message)
            self.expected = point
            self.memory_goal = None
            self.memory_arrival_heading = None
            self.query_grace_until = 0.0
            # Updating the object goal must not cancel a healthy running path.
            self.relay_destination(message)
            if first:
                self.status("awaiting_graph_route", goal=list(point))
            self.check_object_goal_distance()
            return
        if self.memory_goal is not None:
'''
assert old in text
text = text.replace(old, new)
old = '''    def on_object_confirmation(self, message):
'''
assert old in text
text = text.replace(old, old + '''        if self.upstream_target_approach:
            return  # Confirmation feeds the recall stores, not navigation.
''')
old = '''        self.remember_driven_pose(message)

    def on_map(self, message):
'''
new = '''        self.remember_driven_pose(message)
        self.check_object_goal_distance()

    def check_object_goal_distance(self):
        if (not self.upstream_target_approach or self.object_found
                or self.object_destination is None or self.odom is None
                or self.odom_received_at is None
                or time.monotonic() - self.odom_received_at > self.odom_timeout):
            return
        goal = self.object_destination.pose.position
        position = xy(self.odom.pose.pose)
        heading = yaw(self.odom.pose.pose)
        object_position = [goal.x, goal.y, goal.z]
        clearance = vehicle_body_distance(position, heading, object_position,
                                          self.carrier_model)
        tolerance = min(float(self.get_parameter('object_arrival_distance').value),
                        BODY_OBJECT_ARRIVAL_DISTANCE_M)
        if not within_body_arrival(clearance, tolerance):
            return
        # Original strategy: the localized destination persists while odometry
        # closes its distance. The 2 m body clearance is the vehicle adaptation.
        self.object_found = True
        self.reset()
        self.memory_selecting = False
        self.pending_destination = None
        self.request_graph_reset()
        self.retry_graph_reset()
        self.status('object_found', query=self.query,
                    object_position=object_position,
                    arrival_basis='upstream_goal_distance_with_body_clearance',
                    vehicle_position=list(position), vehicle_heading=heading,
                    object_distance_m=clearance,
                    reference_point_distance_m=distance(position, (goal.x, goal.y)),
                    object_arrival_tolerance_m=tolerance,
                    memory_view_id=(self.memory_active.get('id')
                                    if self.memory_active else None))

    def on_map(self, message):
'''
assert old in text
text = text.replace(old, new)
ast.parse(text)
BRIDGE.write_text(text, encoding='utf8')

print('Restored direct goal/odometry completion; existing localization and recall unchanged.')
