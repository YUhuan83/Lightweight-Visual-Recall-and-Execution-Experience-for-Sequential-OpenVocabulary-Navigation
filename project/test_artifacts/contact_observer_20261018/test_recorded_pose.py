"""Numerical spawn convention tests; actual scene admission remains separate."""
from pathlib import Path
import hashlib
import io
import json
import math
import sys
import unittest
import zipfile
import numpy as np
import recorded_pose

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from test_beamng_kinematic_math import Quat


def measured(rotation, position=(12., 20., 3.)):
    return dict(position=list(position), direction=rotation[:, 0].tolist(), up=rotation[:, 2].tolist())


class RecordedPoseTests(unittest.TestCase):
    def test_yaw_round_trip_against_shipped_passive_algebra(self):
        for yaw in (0., .8, -1.1489, math.pi, -math.pi/2):
            rotation=np.array([[math.cos(yaw),-math.sin(yaw),0.],
                               [math.sin(yaw),math.cos(yaw),0.],[0.,0.,1.]])
            result=recorded_pose.spawn_from_measurement(measured(rotation))
            q=Quat(*result['rot_quat'])
            np.testing.assert_allclose(q.rotate((0.,-1.,0.)),rotation[:,0],atol=1e-12)
            np.testing.assert_allclose(q.rotate((0.,0.,1.)),rotation[:,2],atol=1e-12)

    def test_pitch_roll_round_trip_and_quaternion_order(self):
        yaw, pitch, roll=-1.15,.07,-.04
        rz=np.array([[math.cos(yaw),-math.sin(yaw),0.],[math.sin(yaw),math.cos(yaw),0.],[0.,0.,1.]])
        ry=np.array([[math.cos(pitch),0.,math.sin(pitch)],[0.,1.,0.],[-math.sin(pitch),0.,math.cos(pitch)]])
        rx=np.array([[1.,0.,0.],[0.,math.cos(roll),-math.sin(roll)],[0.,math.sin(roll),math.cos(roll)]])
        rotation=rz@ry@rx
        result=recorded_pose.spawn_from_measurement(measured(rotation))
        q=Quat(*result['rot_quat'])
        np.testing.assert_allclose(q.rotate((0.,-1.,0.)),rotation[:,0],atol=1e-12)
        np.testing.assert_allclose(q.rotate((0.,0.,1.)),rotation[:,2],atol=1e-12)

    def test_vehicle_origin_is_not_shifted_twice(self):
        result=recorded_pose.spawn_from_measurement(measured(np.eye(3)))
        self.assertEqual(result['position'],[12.,20.,3.])
        np.testing.assert_allclose(result['rear_axle'],np.array([12.,20.,3.])+recorded_pose.SBR_BASE_OFFSET)

    def test_invalid_basis_cannot_become_spawn(self):
        for name,value in [('position',[float('nan'),0.,0.]),('direction',[0.,0.,0.]),
                           ('up',[1.,0.,0.]),('up',[False,0.,1.])]:
            data=measured(np.eye(3));data[name]=value
            with self.assertRaises(ValueError):recorded_pose.spawn_from_measurement(data)

    def test_known_town_component_agrees_with_measured_heading(self):
        archive=Path(__file__).parent/'contact_component_evidence.zip'
        with zipfile.ZipFile(archive) as zf: rows=json.loads(zf.read('run/traffic.json'))['native']
        native=next(v for v in rows if v.get('position') and v.get('direction'))
        result=recorded_pose.spawn_from_measurement(native)
        expected=Quat(0.,0.,.08679061247481905,.9962265754266173).rotate((0.,-1.,0.))
        error=math.atan2(math.sin(result['yaw']-math.atan2(expected[1],expected[0])),
                         math.cos(result['yaw']-math.atan2(expected[1],expected[0])))
        self.assertLess(abs(error),.03)

    def test_historical_fixture_has_same_axle_near_admitted_path_start(self):
        fixture=json.loads((Path(__file__).parent/'recorded_section_fixture.json').read_text())
        result=recorded_pose.spawn_from_measurement(fixture['nearby_measured_native_state'])
        start=fixture['original_planned_status']['actual_start_pose']
        self.assertLess(math.dist(result['rear_axle'][:2],start[:2]),.3)
        self.assertLess(abs(math.remainder(result['yaw']-start[2],2*math.pi)),.03)


if __name__=='__main__':
    stream=io.StringIO()
    result=unittest.TextTestRunner(stream=stream,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RecordedPoseTests))
    out=Path(__file__).parent
    (out/'recorded_pose_tests.log').write_text(stream.getvalue(),encoding='utf8')
    report=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
                passed=result.wasSuccessful(),engine_scene_validated=False,moving_replay_run=False)
    if result.wasSuccessful():
        fixture=json.loads((out/'recorded_section_fixture.json').read_text())
        report['spawn']=recorded_pose.spawn_from_measurement(fixture['nearby_measured_native_state'])
        (out/'recorded_spawn.json').write_text(json.dumps(report['spawn'],indent=2)+'\n')
    (out/'recorded_pose_tests.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    raise SystemExit(not result.wasSuccessful())
