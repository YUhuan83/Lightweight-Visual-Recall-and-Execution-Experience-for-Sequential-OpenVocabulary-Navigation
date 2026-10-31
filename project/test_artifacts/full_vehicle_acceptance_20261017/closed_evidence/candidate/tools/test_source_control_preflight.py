"""Preflight checks actuator ownership while allowing passive ROS readers."""
from types import SimpleNamespace as S
import unittest
import server_test_preflight as preflight

def endpoint(name,kind='std_msgs/msg/String',namespace='/'):
    return S(node_name=name,node_namespace=namespace,topic_type=kind)

class SourceEndpointTests(unittest.TestCase):
    def setUp(self):
        self.owner=endpoint('wildos_cmd_vel_to_control')
        self.native=endpoint('wildos_native_vehicle_control')

    def test_closed_field_observer_and_native_both_subscribe(self):
        # session_20261017_151218 stopped before unlock on exactly this shape.
        self.assertEqual(preflight.source_endpoint_errors([self.owner],
            [endpoint('execution_contract_observer'),self.native]),[])

    def test_rviz_recorders_and_readers_do_not_become_extra_actuator_owners(self):
        self.assertEqual(preflight.source_endpoint_errors([self.owner],
            [self.native,endpoint('rosbag2_recorder'),endpoint('rviz'),endpoint('diagnostic_reader')]),[])

    def test_missing_duplicate_wrong_type_or_wrong_namespace_actuator_is_blocked(self):
        for readers in ([],[endpoint('rviz')],[self.native,self.native],
                        [endpoint('wildos_native_vehicle_control','beamng_msgs/msg/VehicleControl')],
                        [endpoint('wildos_native_vehicle_control',namespace='/other')]):
            with self.subTest(readers=readers):
                self.assertTrue(preflight.source_endpoint_errors([self.owner],readers))

    def test_extra_or_wrong_source_producers_remain_blocked(self):
        for owners in ([],[self.owner,self.owner],[self.owner,endpoint('unexpected')],
                       [endpoint('unexpected')],[endpoint('wildos_cmd_vel_to_control','beamng_msgs/msg/VehicleControl')],
                       [endpoint('wildos_cmd_vel_to_control',namespace='/other')]):
            with self.subTest(owners=owners):
                self.assertTrue(preflight.source_endpoint_errors(owners,[self.native,endpoint('rosbag2_recorder')]))

if __name__=='__main__':unittest.main()
