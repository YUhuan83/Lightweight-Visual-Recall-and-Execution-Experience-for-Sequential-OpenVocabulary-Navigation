"""Read-only overlay correctness with real ROS messages, isolated domain."""
import math
import unittest
from unittest.mock import Mock
import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String, MultiArrayDimension
from object_search_msgs.msg import ObjectMaskWithTf
from visualization_msgs.msg import Marker
from types import SimpleNamespace as NS
from wildos_nav2_adapter.model_visualization import rgb_target_image
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

    def test_rgb_stream_preserves_motion_when_mask_is_missing_or_stale(self):
        image=Image();image.height=1;image.width=2;image.step=6;image.encoding='rgb8'
        image.data=bytes([100,100,100,50,50,50]);image.header.stamp.sec=10
        mask=ObjectMaskWithTf();mask.query='traffic light';mask.header.stamp.sec=10
        mask.object_mask.layout.dim=[MultiArrayDimension(size=n) for n in (1,1,1,2)]
        mask.object_mask.data=[1,0]
        rendered=rgb_target_image(image,mask)
        self.assertEqual(list(rendered.data),[60,162,60,50,50,50])
        self.assertEqual(list(image.data),[100,100,100,50,50,50])
        mask.header.stamp.sec=9
        self.assertEqual(list(rgb_target_image(image,mask).data),list(image.data))
        self.assertEqual(list(rgb_target_image(image,mask,max_mask_age_sec=2.).data),[60,162,60,50,50,50])
        self.assertEqual(list(rgb_target_image(image).data),list(image.data))

    def test_preview_reduces_display_payload_without_changing_camera_message(self):
        image=Image();image.height=640;image.width=960;image.step=2880;image.encoding='rgb8'
        image.data=bytes([50])*(640*2880);image.header.stamp.sec=10
        rendered=rgb_target_image(image,preview_width=640)
        self.assertEqual((rendered.width,rendered.height),(640,427))
        self.assertEqual(len(rendered.data),640*427*3)
        self.assertEqual(rendered.header.stamp.sec,10)
        self.assertEqual((image.width,image.height),(960,640))

    def test_query_switch_clears_previous_target_mask(self):
        self.node.on_query(String(data='traffic light'))
        mask=ObjectMaskWithTf();mask.query='traffic light';self.node.on_mask(mask)
        self.assertIs(self.node.target_mask,mask)
        self.node.on_query(String(data='gas station'))
        self.node.on_mask(mask)
        self.assertIsNone(self.node.target_mask)

    def test_local_path_uses_its_recorded_pose_not_latest_vehicle_position(self):
        older=Odometry();older.header.frame_id='spot1/odom';older.header.stamp.sec=10
        older.pose.pose.position.x=100.;older.pose.pose.orientation.w=1.
        self.node.odom(older)
        latest=Odometry();latest.header.frame_id='spot1/odom';latest.header.stamp.sec=12
        latest.pose.pose.position.x=110.;latest.pose.pose.orientation.w=1.
        self.node.odom(latest)
        path=Path();path.header.frame_id='spot1/nav_base_link';path.header.stamp.sec=10
        ps=PoseStamped();ps.pose.position.x=1.;ps.pose.orientation.w=1.;path.poses=[ps]
        pub=Mock();self.node.overlays={'received_plan':(pub,path)}
        self.node.publish_overlays();result=pub.publish.call_args.args[0]
        self.assertEqual(result.poses[0].pose.position.x,101.)
        path.header.stamp.sec=9;pub.reset_mock();self.node.publish_overlays()
        pub.publish.assert_not_called()

    def test_topology_snapshot_replacement_clears_removed_nodes(self):
        from graphnav_msgs.msg import NavigationGraph
        self.node.scene_pub=Mock()
        graph=NavigationGraph();graph.header.frame_id='spot1/odom'
        graph.nodes=[NS(pose=NS(position=NS(x=734.,y=-11.,z=55.)),
            trav_properties=[NS(is_frontier=True,frontier_points=[])])]
        graph.edges=[];self.node.graph=graph
        self.node.draw_scene(self.node.get_clock().now().nanoseconds)
        markers=self.node.scene_pub.publish.call_args.args[0].markers
        self.assertEqual(markers[0].action,Marker.DELETEALL)
        nodes=next(m for m in markers if m.ns=='topology/nodes')
        self.assertEqual(nodes.points[0].x,734.)
        self.assertEqual(nodes.points[0].z,52.18)
        graph.nodes=[];self.node.draw_scene(self.node.get_clock().now().nanoseconds)
        markers=self.node.scene_pub.publish.call_args.args[0].markers
        self.assertEqual(next(m for m in markers if m.ns=='topology/nodes').points,[])

    def test_live_camera_publishes_each_frame_without_waiting_for_model_or_graph(self):
        self.node.model_pub=Mock()
        for stamp,color in ((10,50),(11,90)):
            image=Image();image.header.stamp.sec=stamp;image.encoding='rgb8'
            image.width=image.height=1;image.step=3;image.data=bytes([color]*3)
            self.node.on_camera(image)
        messages=[call.args[0] for call in self.node.model_pub.publish.call_args_list]
        self.assertEqual([m.header.stamp.sec for m in messages],[10,11])
        self.assertEqual([list(m.data) for m in messages],[[50]*3,[90]*3])


if __name__=='__main__':unittest.main()
