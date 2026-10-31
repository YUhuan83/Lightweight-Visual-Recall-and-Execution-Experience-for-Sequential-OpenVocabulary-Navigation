"""Sensor-ray origin is independent of the coordinates used for cloud points."""
import ast
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import unittest
import yaml
import numpy as np
from wildos_nav2_adapter.runtime_contract import navigation_runtime_contract
from wildos_nav2_adapter.sensor_geometry import geometry_restart_reason

ROOT = Path(__file__).resolve().parents[2]


class LidarRayOriginTests(unittest.TestCase):
    def test_both_costmaps_require_actual_sensor_origin_for_marking_and_clearing(self):
        config = yaml.safe_load((ROOT/'nav2_adapter/config/nav2_ackermann_control.yaml').read_text())
        required = navigation_runtime_contract(config, 'ackermann')
        for name in ('local_costmap', 'global_costmap'):
            for source in ('cloud', 'clearing_cloud'):
                key = 'obstacle_layer.'+source+'.sensor_frame'
                self.assertEqual(required['/'+name+'/'+name][key], 'spot1/lidar_ray_origin')
                for frame in ('', 'spot1/base_link', 'spot1/ouster/front/os_lidar'):
                    with self.subTest(costmap=name, source=source, frame=frame):
                        changed = yaml.safe_load(yaml.safe_dump(config))
                        changed[name][name]['ros__parameters']['obstacle_layer'][source]['sensor_frame'] = frame
                        with self.assertRaises(ValueError):
                            navigation_runtime_contract(changed, 'ackermann')

    def test_static_ray_origin_does_not_move_existing_cloud_frame(self):
        path = ROOT/'official_adapter/beamng_wildos_adapter/node.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Adapter')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'publish_static_lidar_tf')
        def transform():
            return S(header=S(), transform=S(translation=S(), rotation=S()))
        namespace = dict(TransformStamped=transform, np=np)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        adapter = S(base_offset=np.array([-2.172, .330, 0.]),
                    lidar_offset=np.array([1.252, 0., 1.56]), static_tf_pub=Mock(),
                    get_clock=lambda: S(now=lambda: S(to_msg=lambda: 'stamp')))
        namespace['publish_static_lidar_tf'](adapter)
        sent = adapter.static_tf_pub.sendTransform.call_args.args[0]
        self.assertIsInstance(sent, list)
        frames = {tf.child_frame_id: tf for tf in sent}
        virtual = frames['spot1/ouster/front/os_lidar'].transform.translation
        np.testing.assert_allclose([virtual.x, virtual.y, virtual.z], [2.172, -.330, 0.])
        ray = frames['spot1/lidar_ray_origin']
        self.assertEqual(ray.header.frame_id, 'spot1/base_link')
        v = ray.transform.translation
        np.testing.assert_allclose([v.x, v.y, v.z], adapter.lidar_offset)

    def test_ray_origin_change_requires_coordinated_restart(self):
        get = lambda name: S(value=[1.252, 0., 1.56])
        self.assertIsNone(geometry_restart_reason([S(name='lidar_offset', value=get('').value)], get))
        self.assertIn('restart', geometry_restart_reason([S(name='lidar_offset', value=[0., 0., 0.])], get))


if __name__ == '__main__':
    unittest.main()
