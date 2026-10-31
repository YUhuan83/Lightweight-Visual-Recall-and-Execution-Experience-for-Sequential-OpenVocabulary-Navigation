"""Physical sensor configuration contracts; no simulator or motion commands."""
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PhysicalSensorRigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.document = json.loads((ROOT / 'config/sbr_physical_sensor_rig.json').read_text())
        cls.vehicle, = cls.document['vehicles']
        cls.sensors = {s['name']: s for s in cls.vehicle['sensors']}

    def test_sensor_attachment_cannot_respawn_or_disable_vehicle_physics(self):
        self.assertEqual(set(self.document), {'vehicles'})
        self.assertEqual(set(self.vehicle), {'name', 'sensors'})
        self.assertEqual(self.vehicle['name'], 'ego')
        self.assertEqual(len(self.sensors), len(self.vehicle['sensors']))
        self.assertEqual(set(self.sensors), {'electrics', 'front_cam', 'front_lidar'})

    def test_camera_and_source_identity_remain_unchanged(self):
        self.assertEqual(self.sensors['front_cam'], {
            'name': 'front_cam', 'type': 'Camera.default', 'pos': [0., -1.5, 2.35],
            'dir': [0., -1., 0.], 'up': [0., 0., 1.], 'resolution': [960, 640],
            'field_of_view_y': 60., 'requested_update_time': .2,
            'is_using_shared_memory': False, 'is_streaming': False,
            'is_visualised': False})
        self.assertEqual(self.sensors['electrics'], {'name': 'electrics', 'type': 'Electrics'})

    def test_full_circle_lidar_has_level_vehicle_basis(self):
        sensor = self.sensors['front_lidar']
        self.assertEqual(sensor['type'], 'Lidar.default')
        self.assertIs(sensor['is_360_mode'], True)
        self.assertFalse(sensor.get('is_rotate_mode', False))
        self.assertFalse(sensor.get('is_static', False))
        self.assertFalse(sensor.get('is_dir_world_space', False))
        self.assertEqual(sensor['dir'], [0., -1., 0.])
        self.assertEqual(sensor['up'], [0., 0., 1.])
        self.assertEqual(sum(a*b for a, b in zip(sensor['dir'], sensor['up'])), 0.)

    def test_near_field_mount_preserves_sampling_and_range_contract(self):
        sensor = self.sensors['front_lidar']
        self.assertEqual(sensor['pos'], [0., 0., 1.7])
        self.assertEqual((sensor['vertical_resolution'], sensor['vertical_angle'],
                          sensor['max_distance'], sensor['density'],
                          sensor['requested_update_time']), (16, 60., 40., 30, .2))
        self.assertFalse(sensor['is_using_shared_memory'])
        self.assertFalse(sensor['is_visualised'])


if __name__ == '__main__':
    unittest.main()
