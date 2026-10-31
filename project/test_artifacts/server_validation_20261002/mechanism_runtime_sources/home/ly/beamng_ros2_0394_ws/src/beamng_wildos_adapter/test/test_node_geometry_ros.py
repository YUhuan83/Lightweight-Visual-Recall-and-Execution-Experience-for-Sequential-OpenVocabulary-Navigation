"""Real ROS message/callback geometry checks, in an isolated ROS domain."""
import math
import unittest
from unittest.mock import Mock, patch
import numpy as np
import rclpy
from beamng_msgs.msg import StateSensor
from sensor_msgs.msg import PointCloud2
from beamng_wildos_adapter.node import Adapter
from beamng_wildos_adapter.pointcloud import decode_xyz
from beamng_wildos_adapter.geometry import quaternion_matrix


class NodeGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def test_planar_navigation_frame_prevents_altitude_from_reversing_carrot(self):
        with patch('beamng_wildos_adapter.node.StaticTransformBroadcaster'):
            node=Adapter()
        try:
            node.odom_pub=Mock();node.tf_pub=Mock()
            state=StateSensor();state.header.stamp.sec=100
            state.position.x=734.294;state.position.y=-11.946;state.position.z=52.154
            state.dir.x=math.cos(.0175);state.dir.z=-math.sin(.0175);state.up.z=1.
            node.state_cb(state)
            transforms={call.args[0].child_frame_id:call.args[0] for call in node.tf_pub.sendTransform.call_args_list}
            physical=transforms['spot1/base_link'];planar=transforms['spot1/nav_base_link']
            self.assertEqual(planar.transform.translation.z,0.)
            self.assertEqual(planar.transform.rotation.x,0.)
            self.assertEqual(planar.transform.rotation.y,0.)
            self.assertEqual(planar.transform.translation.x,physical.transform.translation.x)
            self.assertEqual(planar.transform.translation.y,physical.transform.translation.y)
            # Real Town-scale altitude, mild pitch, goal still 0.6 m behind.
            t=physical.transform.translation;q=physical.transform.rotation
            goal=np.array([t.x-.6,t.y,0.])
            old_carrot=(goal-[t.x,t.y,t.z])@quaternion_matrix([q.x,q.y,q.z,q.w])
            self.assertGreater(old_carrot[0],0.)  # Old frame would request Drive.
            t=planar.transform.translation;q=planar.transform.rotation
            new_carrot=(goal-[t.x,t.y,t.z])@quaternion_matrix([q.x,q.y,q.z,q.w])
            self.assertLess(new_carrot[0],0.)  # Correct frame keeps Reverse.
        finally:node.destroy_node()

    def test_odom_cloud_and_static_tf_use_consistent_origins(self):
        with patch('beamng_wildos_adapter.node.StaticTransformBroadcaster'):
            node=Adapter()
        try:
            node.odom_pub=Mock(); node.lidar_pub=Mock(); node.map_pub=Mock(); node.tf_pub=Mock()
            offset=node.base_offset
            static=node.static_tf_pub.sendTransform.call_args.args[0]
            t=static.transform.translation
            np.testing.assert_allclose([t.x,t.y,t.z],-offset)
            state=StateSensor(); state.header.stamp.sec=100
            state.position.x=734.294; state.position.y=-11.946; state.position.z=52.154
            state.dir.x=1.; state.up.z=1.
            node.state_cb(state)
            pose=node.odom_pub.publish.call_args.args[0].pose.pose
            np.testing.assert_allclose([pose.position.x,pose.position.y,pose.position.z],
                                       np.array([734.294,-11.946,52.154])+offset)
            heading=.02; rotation=np.array([[math.cos(heading),-math.sin(heading),0.],
                                            [math.sin(heading),math.cos(heading),0.],[0.,0.,1.]])
            state.header.stamp.nanosec=100_000_000
            state.dir.x=math.cos(heading); state.dir.y=math.sin(heading)
            velocity=rotation@np.array([1.2,0.,0.])-np.cross([0.,0.,.2],rotation@offset)
            state.velocity.x,state.velocity.y,state.velocity.z=map(float,velocity)
            node.state_cb(state)
            odom=node.odom_pub.publish.call_args.args[0]
            self.assertAlmostEqual(odom.twist.twist.linear.x,1.2)
            self.assertAlmostEqual(odom.twist.twist.linear.y,0.)
            self.assertAlmostEqual(odom.twist.twist.angular.z,.2)
            cloud=PointCloud2(); cloud.header.stamp=state.header.stamp
            world=np.array([[740.,-8.,53.],[730.,-13.,52.]])
            with patch('beamng_wildos_adapter.node.decode_xyz',return_value=world):
                node.lidar_cb(cloud)
            points=decode_xyz(node.lidar_pub.publish.call_args.args[0])
            p=odom.pose.pose.position
            reconstructed=(points-offset)@rotation.T+np.array([p.x,p.y,p.z])
            np.testing.assert_allclose(reconstructed,world,atol=1e-5)
            self.assertEqual(node.map_pub.publish.call_args.args[0].header.frame_id,'spot1/base_link')
        finally: node.destroy_node()


if __name__=='__main__': unittest.main()
