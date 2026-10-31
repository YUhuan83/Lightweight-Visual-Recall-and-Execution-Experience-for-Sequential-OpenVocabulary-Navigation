"""Read-only overlay correctness with real ROS messages, isolated domain."""
import math
import unittest
from unittest.mock import Mock
import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from wildos_nav2_adapter.visualization_bridge import VisualizationBridge


class VisualizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def setUp(self):
        self.node=VisualizationBridge();odom=Odometry();odom.header.frame_id='spot1/odom'
        odom.pose.pose.position.x=734.;odom.pose.pose.position.y=-11.;odom.pose.pose.position.z=52.
        odom.pose.pose.orientation.z=math.sin(math.pi/4);odom.pose.pose.orientation.w=math.cos(math.pi/4)
        self.node.pose=odom

    def tearDown(self):self.node.destroy_node()

    def test_planar_local_path_is_visible_at_correct_world_xy(self):
        source=Path();source.header.frame_id='spot1/nav_base_link'
        ps=PoseStamped();ps.header=source.header;ps.pose.position.x=1.;ps.pose.orientation.w=1.
        source.poses=[ps];pub=Mock();self.node.overlays={'received_plan':(pub,source)}
        self.node.publish_overlays();result=pub.publish.call_args.args[0]
        self.assertEqual(result.header.frame_id,'spot1/odom')
        p=result.poses[0].pose.position
        self.assertAlmostEqual(p.x,734.);self.assertAlmostEqual(p.y,-10.);self.assertAlmostEqual(p.z,52.05)
        self.assertAlmostEqual(result.poses[0].pose.orientation.z,math.sin(math.pi/4))
        self.assertEqual(source.header.frame_id,'spot1/nav_base_link')
        self.assertEqual(source.poses[0].pose.position.x,1.)
        self.assertEqual(source.poses[0].pose.position.z,0.)

    def test_costmap_overlay_preserves_original_navigation_grid(self):
        source=OccupancyGrid();source.header.frame_id='spot1/odom'
        source.info.origin.position.z=0.;source.data=[0,100,99,-1]
        pub=Mock();self.node.overlays={'global_costmap':(pub,source)}
        self.node.publish_overlays();result=pub.publish.call_args.args[0]
        self.assertAlmostEqual(result.info.origin.position.z,51.9)
        self.assertEqual(list(result.data),[0,100,99,-1])
        self.assertEqual(source.info.origin.position.z,0.)


if __name__=='__main__':unittest.main()
