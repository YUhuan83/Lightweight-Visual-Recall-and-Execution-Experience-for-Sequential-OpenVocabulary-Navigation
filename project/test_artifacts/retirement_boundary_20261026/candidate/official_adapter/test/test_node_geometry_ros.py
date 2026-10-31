"""Real ROS message/callback geometry checks, in an isolated ROS domain."""
import math
import json
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

    def test_both_map_chains_forward_the_same_ground_continuity_limit(self):
        from rclpy.parameter import Parameter
        from beamng_wildos_adapter.terrain import terrain_layers
        with patch('beamng_wildos_adapter.node.StaticTransformBroadcaster'):
            node = Adapter()
        try:
            node.set_parameters([Parameter('ground_max_slope', value=.23)])
            node.map_pub = Mock(); node.terrain_pub = Mock()
            points = np.array([[5., 0., -.4], [5.01, 0., -.4]])
            state = StateSensor()
            with patch('beamng_wildos_adapter.node.occupancy_from_points',
                       return_value=np.zeros((80, 80), dtype=np.int8)) as mapping:
                node.publish_occupancy(points, state.header.stamp)
                self.assertEqual(mapping.call_args.kwargs['ground_max_slope'], .23)
            with patch('beamng_wildos_adapter.node.terrain_layers', wraps=terrain_layers) as terrain:
                node.publish_terrain(points, (1.252, 0.), state.header.stamp)
                self.assertEqual(terrain.call_args.kwargs['max_slope'], .23)
        finally:
            node.destroy_node()

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

    def test_measurement_timing_keeps_sensor_and_ros_clocks_separate(self):
        with patch('beamng_wildos_adapter.node.StaticTransformBroadcaster'):
            node=Adapter()
        try:
            node.timing_pub=Mock()
            node.publish_measurement_timing(
                "camera", source_clock="beamng", output_clock="ros",
                source_stamp_ns=12_000_000_000,
                mapped_stamp_ns=1_800_000_012_000_000_000,
                status="published")
            payload=json.loads(node.timing_pub.publish.call_args.args[0].data)
            self.assertEqual(payload["schema_version"],1)
            self.assertEqual(payload["sensor"],"camera")
            self.assertEqual(payload["source_stamp_ns"],12_000_000_000)
            self.assertEqual(payload["mapped_stamp_ns"],1_800_000_012_000_000_000)
            self.assertEqual(payload["source_clock"],"beamng")
            self.assertEqual(payload["output_clock"],"ros")
            self.assertGreater(payload["adapter_rx_ros_ns"],0)
        finally:node.destroy_node()

    def test_odom_cloud_and_static_tf_use_consistent_origins(self):
        with patch('beamng_wildos_adapter.node.StaticTransformBroadcaster'):
            node=Adapter()
        try:
            node.odom_pub=Mock(); node.lidar_pub=Mock(); node.map_pub=Mock(); node.tf_pub=Mock(); node.terrain_pub=Mock()
            offset=node.base_offset
            transforms=node.static_tf_pub.sendTransform.call_args.args[0]
            static=next(t for t in transforms if t.child_frame_id=='spot1/ouster/front/os_lidar')
            ray=next(t for t in transforms if t.child_frame_id=='spot1/lidar_ray_origin')
            v=ray.transform.translation
            np.testing.assert_allclose([v.x,v.y,v.z],node.lidar_offset)
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
                node.flush_pending_sensors()
            points=decode_xyz(node.lidar_pub.publish.call_args.args[0])
            p=odom.pose.pose.position
            reconstructed=(points-offset)@rotation.T+np.array([p.x,p.y,p.z])
            np.testing.assert_allclose(reconstructed,world,atol=1e-5)
            self.assertEqual(node.map_pub.publish.call_args.args[0].header.frame_id,'spot1/base_link')
            terrain=node.terrain_pub.publish.call_args.args[0]
            self.assertEqual(terrain.header.frame_id,'spot1/terrain_base_link')
            self.assertIn('elevation_valid',terrain.layers)
            self.assertEqual(terrain.header.stamp,node.state_ros_stamp)
        finally: node.destroy_node()

    def test_pitched_vehicle_does_not_flatten_measured_world_slope(self):
        with patch('beamng_wildos_adapter.node.StaticTransformBroadcaster'):
            node=Adapter()
        try:
            node.odom_pub=Mock();node.tf_pub=Mock();node.map_pub=Mock()
            node.lidar_pub=Mock();node.terrain_pub=Mock()
            angle=.3;state=StateSensor();state.header.stamp.sec=100
            state.position.x=734.;state.position.y=-12.;state.position.z=52.
            state.dir.x=math.cos(angle);state.dir.z=math.sin(angle)
            state.up.x=-math.sin(angle);state.up.z=math.cos(angle)
            node.state_cb(state)
            transform=[c.args[0] for c in node.tf_pub.sendTransform.call_args_list
                       if c.args[0].child_frame_id=='spot1/terrain_base_link'][0]
            self.assertEqual(transform.transform.rotation.x,0.)
            self.assertEqual(transform.transform.rotation.y,0.)
            t=transform.transform.translation
            world=np.array([[t.x+x,t.y+y,t.z+.2*x-1.]
                            for x in np.arange(4.05,8.,.1)
                            for y in np.arange(2.05,5.,.1)])
            cloud=PointCloud2();cloud.header.stamp=state.header.stamp
            with patch('beamng_wildos_adapter.node.decode_xyz',return_value=world):
                node.lidar_cb(cloud)
                node.flush_pending_sensors()
            message=node.terrain_pub.publish.call_args.args[0]
            layers={name:np.array(data.data).reshape(80,80)[::-1,::-1]
                    for name,data in zip(message.layers,message.data)}
            rows,cols=np.nonzero(layers['elevation_valid']>.5)
            self.assertGreater(len(rows),30)
            design=np.c_[(cols+.5-40)*.5,(rows+.5-40)*.5,np.ones(len(rows))]
            plane=np.linalg.lstsq(design,layers['elevation'][rows,cols],rcond=None)[0]
            self.assertAlmostEqual(plane[0],.2,places=2)
        finally:node.destroy_node()

    def test_occupancy_and_level_terrain_use_the_actual_aperture(self):
        from beamng_wildos_adapter.geometry import body_rotation, shifted_body_position
        with patch('beamng_wildos_adapter.node.StaticTransformBroadcaster'):
            node=Adapter()
        try:
            node.map_pub=Mock();node.publish_terrain=Mock()
            state=StateSensor();state.header.stamp.sec=100
            state.position.x=734.;state.position.y=-12.;state.position.z=52.
            angle=.2;state.dir.x=math.cos(angle);state.dir.z=math.sin(angle)
            state.up.x=-math.sin(angle);state.up.z=math.cos(angle)
            node.state_cb(state)
            with patch('beamng_wildos_adapter.node.occupancy_from_points',return_value=np.zeros((80,80),dtype=np.int8)) as mapping:
                node.publish_occupancy(np.zeros((1,3)),node.state_ros_stamp)
                np.testing.assert_allclose(mapping.call_args.kwargs['sensor_origin_xy'],node.lidar_offset[:2])
            cloud=PointCloud2();cloud.header.stamp=state.header.stamp
            with patch('beamng_wildos_adapter.node.decode_xyz',return_value=np.array([[740.,-10.,52.]])):
                node.lidar_cb(cloud);node.flush_pending_sensors()
            rotation=body_rotation(state.dir,state.up)
            origin=(rotation@node.lidar_offset)[:2]
            np.testing.assert_allclose(node.publish_terrain.call_args.args[1],origin,atol=1e-8)
        finally:node.destroy_node()


if __name__=='__main__': unittest.main()
