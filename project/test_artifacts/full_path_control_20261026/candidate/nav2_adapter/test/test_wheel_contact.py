import json
from pathlib import Path
import unittest
from wildos_nav2_adapter.wheel_contact import classify_wheel_contact


class WheelContactTests(unittest.TestCase):
    def test_recorded_trial18_distinguishes_airborne_spin_from_vehicle_motion(self):
        data = json.loads((Path(__file__).parent / 'fixtures/trial18_wheel_contact.json').read_text(encoding='utf-8'))
        result = classify_wheel_contact(data['wheels'], data['speed'])
        self.assertEqual(result['classification'], 'wheel_airborne_spin')
        self.assertEqual(result['spinning_airborne_wheels'], ['FR'])
        self.assertEqual(result['measured_wheels'], 4)

    def test_ground_contact_and_corner_unloading_are_not_airborne_spin(self):
        wheel = dict(name='FR', wheelSpeed=2., downForceRaw=0.,
                     contactMaterialID1=10, contactMaterialID2=4)
        self.assertEqual(classify_wheel_contact([wheel], 0.)['classification'], 'wheel_support_present')

    def test_missing_and_nonfinite_telemetry_are_not_ground_support_evidence(self):
        self.assertIsNone(classify_wheel_contact([{}], 0.))
        self.assertIsNone(classify_wheel_contact([], float('nan')))
