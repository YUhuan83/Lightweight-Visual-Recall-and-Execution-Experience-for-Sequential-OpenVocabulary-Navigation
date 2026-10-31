"""Actual Humble CDR and geometric admission of the frozen section; no engine."""
import io
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from nav_msgs.msg import Path as Route,OccupancyGrid
from nav2_msgs.action import FollowPath
from rosidl_runtime_py.set_message import set_message_fields
from rclpy.serialization import serialize_message,deserialize_message
from rclpy.parameter import Parameter,parameter_value_to_python
from unique_identifier_msgs.msg import UUID
from execute_recorded import action_uuid_values
from wildos_nav2_adapter.graph_route import validate_vehicle_path,validate_local_prefix
from wildos_nav2_adapter.motion_path import diagnose_maneuver_join,motion_sections
from wildos_nav2_adapter.graph_path_bridge import xy,yaw

root=Path(__file__).resolve().parent
fixture=json.loads((root/'recorded_section_fixture.json').read_text())


def inputs():
    route=Route();set_message_fields(route,deepcopy(fixture['path']))
    points=[xy(p.pose) for p in route.poses];headings=[yaw(p.pose) for p in route.poses]
    start=fixture['original_planned_status']['actual_start_pose']
    grid=OccupancyGrid();grid.header.frame_id='spot1/odom'
    grid.info.resolution=.25;grid.info.width=grid.info.height=160
    grid.info.origin.position.x=720.;grid.info.origin.position.y=-95.;grid.info.origin.orientation.w=1.
    grid.data=[0]*(160*160)
    return route,points,headings,start,grid


class RecordedRosInputTests(unittest.TestCase):
    def test_actual_action_uuid_is_json_serializable(self):
        identifier=UUID(uuid=list(range(16)))
        values=action_uuid_values(identifier)
        self.assertEqual(values,list(range(16)))
        self.assertTrue(all(type(v) is int for v in values))
        self.assertEqual(json.loads(json.dumps(values)),values)

    def test_humble_parameter_decode_includes_geometry_arrays(self):
        for value in ([1.252,0.,1.56],True,False,72,8.,'ackermann',['CarLike','GridBased']):
            parameter=Parameter('test',value=value).get_parameter_value()
            self.assertEqual(parameter_value_to_python(parameter),value)

    def test_repeated_decodes_preserve_frozen_input(self):
        original=json.dumps(fixture,sort_keys=True)
        first=inputs()[0];second=inputs()[0]
        self.assertEqual(serialize_message(first),serialize_message(second))
        self.assertEqual(json.dumps(fixture,sort_keys=True),original)

    def test_actual_cdr_round_trip_and_original_checker(self):
        route,points,headings,start,grid=inputs()
        restored=deserialize_message(serialize_message(route),Route)
        self.assertEqual(len(restored.poses),33);self.assertEqual(restored.header.frame_id,'spot1/odom')
        self.assertEqual(xy(restored.poses[-1].pose),points[-1])
        goal=FollowPath.Goal(path=restored,controller_id='FollowPath',goal_checker_id=fixture['original_planned_status']['goal_checker_id'])
        decoded=deserialize_message(serialize_message(goal),FollowPath.Goal)
        self.assertEqual(decoded.goal_checker_id,'cusp_goal_checker')

    def test_recorded_actual_pose_and_single_section_are_admissible_on_free_grid(self):
        route,points,headings,start,grid=inputs()
        self.assertEqual(len(motion_sections(points)),1)
        self.assertIsNone(validate_vehicle_path(points,headings,points,grid,points[-1],start=start[:2],minimum_turning_radius=8.,heading_quantization_bins=72))
        self.assertIsNone(validate_local_prefix(points,headings,grid,start[:2],start[2]))
        self.assertIsNotNone(diagnose_maneuver_join(points,headings,start[:2],start[2],8.,endpoint_position_tolerance=.2)[0])

    def test_wrong_spawn_position_is_rejected(self):
        route,points,headings,start,grid=inputs()
        reason=validate_vehicle_path(points,headings,points,grid,points[-1],start=(start[0]+5.,start[1]),minimum_turning_radius=8.,heading_quantization_bins=72)
        self.assertEqual(reason,'wrong_start')

    def test_new_physical_obstacle_is_rejected_before_arm(self):
        route,points,headings,start,grid=inputs()
        x,y=points[len(points)//2]
        mx=math.floor((x-grid.info.origin.position.x)/grid.info.resolution)
        my=math.floor((y-grid.info.origin.position.y)/grid.info.resolution)
        grid.data[my*grid.info.width+mx]=100
        self.assertIn(validate_vehicle_path(points,headings,points,grid,points[-1],start=start[:2],minimum_turning_radius=8.,heading_quantization_bins=72),('collision_cell','footprint_collision'))


if __name__=='__main__':
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RecordedRosInputTests))
    (root/'recorded_ros_input_tests.log').write_text(stream.getvalue())
    report=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful(),scope=__doc__)
    (root/'recorded_ros_input_tests.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));raise SystemExit(not result.wasSuccessful())
