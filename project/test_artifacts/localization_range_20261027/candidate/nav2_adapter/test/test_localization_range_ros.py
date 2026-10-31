"""Range and acquisition contracts for localization versus local navigation."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import math
import time
import unittest
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from builtin_interfaces.msg import Time
from beamng_wildos_adapter import pointcloud
from beamng_wildos_adapter.geometry import body_rotation, shifted_body_position, xyz

ROOT=Path(__file__).resolve().parents[2]


class LocalizationRangeTests(unittest.TestCase):
    def setUp(self):
        source=ROOT/'official_adapter/beamng_wildos_adapter/node.py'
        tree=ast.parse(source.read_text(encoding='utf8'))
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Adapter')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='process_lidar')
        env=dict(np=np,time=time,math=math,decode_xyz=pointcloud.decode_xyz,
            limit_points=pointcloud.limit_points,split_lidar_ranges=getattr(pointcloud,'split_lidar_ranges',None),
            body_rotation=body_rotation,shifted_body_position=shifted_body_position,xyz=xyz,
            PointCloud2=PointCloud2,PointField=PointField)
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),env)
        self.process=env['process_lidar']
        self.node=S(get_parameter=lambda name:S(value=.1),last_lidar_skew_warning=0.,
            get_logger=Mock(),publish_measurement_timing=Mock(),measurement_output_clock=lambda:'ros',
            lidar_pub=Mock(),localization_lidar_pub=Mock(),publish_occupancy=Mock(),publish_terrain=Mock(),
            base_offset=np.array([-1.252,0.,.14]),lidar_offset=np.array([1.252,0.,1.56]),
            last_terrain_stamp_ns=None,lidar_frames=0)
        self.origin=np.array([100.,200.,5.])
        self.stamp=Time(sec=100,nanosec=123)
        self.state=S(header=S(stamp=S(sec=10,nanosec=0)),position=S(x=100.,y=200.,z=5.),
            dir=S(x=1.,y=0.,z=0.),up=S(x=0.,y=0.,z=1.))

    def cloud(self,points):
        m=PointCloud2();m.header.stamp.sec=10;m.header.frame_id='world';m.height=1
        m.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate('xyz')]
        m.width=len(points);m.point_step=12;m.row_step=12*len(points);m.is_bigendian=False
        m.data=np.asarray(points,dtype='<f4').tobytes()
        return m

    def run_cloud(self,points):
        self.process(self.node,self.cloud(points),self.state,self.stamp)

    def test_far_object_returns_reach_localization_but_not_navigation(self):
        self.run_cloud(self.origin+np.array([[5.,0.,0.],[55.,0.,0.],[90.,0.,0.]]))
        near=pointcloud.decode_xyz(self.node.lidar_pub.publish.call_args.args[0])
        self.node.localization_lidar_pub.publish.assert_called_once()
        far=pointcloud.decode_xyz(self.node.localization_lidar_pub.publish.call_args.args[0])
        np.testing.assert_allclose(near,[[5.,0.,0.]])
        np.testing.assert_allclose(far,[[5.,0.,0.],[55.,0.,0.]])
        np.testing.assert_allclose(self.node.publish_occupancy.call_args.args[0],near-self.node.base_offset)
        self.assertEqual(len(self.node.publish_terrain.call_args.args[0]),1)

    def test_far_returns_cannot_consume_the_near_field_sampling_budget(self):
        near=np.c_[np.linspace(2.,30.,40000),np.zeros((40000,2))]
        far=np.c_[np.linspace(50.,70.,40000),np.zeros((40000,2))]
        self.run_cloud(self.origin+np.vstack((near,far)))
        n=self.node.lidar_pub.publish.call_args.args[0]
        self.assertEqual(n.width,30000)
        self.node.localization_lidar_pub.publish.assert_called_once()
        d=self.node.localization_lidar_pub.publish.call_args.args[0]
        self.assertLessEqual(d.width,30000)
        self.assertTrue(np.any(pointcloud.decode_xyz(d)[:,0]>40.))
        self.assertEqual(len(self.node.publish_terrain.call_args.args[0]),n.width)

    def test_both_streams_keep_the_same_virtual_frame_and_pose_time(self):
        self.run_cloud(self.origin+np.array([[5.,0.,0.],[55.,0.,0.]]))
        self.node.localization_lidar_pub.publish.assert_called_once()
        messages=[self.node.lidar_pub.publish.call_args.args[0],self.node.localization_lidar_pub.publish.call_args.args[0]]
        for m in messages:
            self.assertEqual(m.header.frame_id,'spot1/ouster/front/os_lidar')
            self.assertEqual((m.header.stamp.sec,m.header.stamp.nanosec),(100,123))

    def test_pose_skew_still_drops_both_streams(self):
        self.state.header.stamp.nanosec=200_000_000
        self.run_cloud(self.origin+np.array([[55.,0.,0.]]))
        self.node.lidar_pub.publish.assert_not_called()
        self.node.localization_lidar_pub.publish.assert_not_called()
        self.node.publish_terrain.assert_not_called()

    def test_empty_scan_cannot_republish_an_old_localization(self):
        self.run_cloud(np.empty((0,3)))
        self.node.localization_lidar_pub.publish.assert_called_once()
        self.assertEqual(self.node.localization_lidar_pub.publish.call_args.args[0].width,0)

    def test_localization_consumer_is_not_wired_to_the_near_map_cloud(self):
        import yaml
        c=yaml.safe_load((ROOT/'official_adapter/config/beamng_triangulation.yaml').read_text(encoding='utf8'))
        self.assertEqual(c['lidar_topic'],'/spot1/ouster/front/points_for_localization')
        self.assertEqual(c['lidar_frame'],'spot1/ouster/front/os_lidar')

    def test_physical_profile_changes_only_the_measured_range(self):
        c=json.loads((ROOT/'config/sbr_physical_sensor_rig.json').read_text(encoding='utf8'))
        lidar=next(s for s in c['vehicles'][0]['sensors'] if s['name']=='front_lidar')
        self.assertEqual(lidar['max_distance'],80.)
        self.assertEqual((lidar['vertical_resolution'],lidar['density'],lidar['requested_update_time']),(16,30,.2))
