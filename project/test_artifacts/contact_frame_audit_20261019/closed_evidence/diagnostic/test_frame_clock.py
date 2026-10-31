"""Actual bridge clock conversion plus recorded-path admission tests; no engine."""
import io
import json
from pathlib import Path
import unittest
from rclpy.time import Time
from beamng_ros2.utils import float_to_duration,float_to_time
from execute_recorded import post_enable_owner_ready
import test_recorded_ros_inputs as original

class FrameClockTests(unittest.TestCase):
    def test_pre_enable_coherent_cached_owner_cannot_admit_action(self):
        control=dict(enabled=False,source_ack_ready=True,source_control=dict(sender_id='one',generation=0,issued_wall_ns=900))
        native=dict(source_receipt=dict(sender_id='one',generation=0,applied=True))
        self.assertFalse(post_enable_owner_ready(control,native,'one',1,1000))

    def test_expected_new_generation_ack_admits_and_old_native_does_not(self):
        control=dict(enabled=True,source_ack_ready=True,source_control=dict(sender_id='one',generation=1,issued_wall_ns=1100))
        native=dict(source_receipt=dict(sender_id='one',generation=1,applied=True))
        self.assertTrue(post_enable_owner_ready(control,native,'one',1,1000))
        native['source_receipt']['generation']=0
        self.assertFalse(post_enable_owner_ready(control,native,'one',1,1000))

    def test_unexpected_generation_sender_or_unacknowledged_source_is_rejected(self):
        from copy import deepcopy
        control=dict(enabled=True,source_ack_ready=True,source_control=dict(sender_id='one',generation=1,issued_wall_ns=1100))
        native=dict(source_receipt=dict(sender_id='one',generation=1,applied=True))
        for field,value in [('sender_id','other'),('generation',2),('issued_wall_ns',999)]:
            changed=deepcopy(control);changed['source_control'][field]=value
            self.assertFalse(post_enable_owner_ready(changed,native,'one',1,1000))
        changed=deepcopy(control);changed['source_ack_ready']=False
        self.assertFalse(post_enable_owner_ready(changed,native,'one',1,1000))
        native['source_receipt']['applied']=False
        self.assertFalse(post_enable_owner_ready(control,native,'one',1,1000))

    def test_bridge_anchor_round_trip_preserves_engine_elapsed_time(self):
        anchor=1792331000000123456
        for seconds in (0.,4.3650002073264,8.2935003939201,13.872000658885):
            source=float_to_time(Time(nanoseconds=anchor),seconds).nanoseconds
            self.assertLessEqual(abs(source-anchor-round(seconds*1e9)),1)

    def test_ros_stamp_requires_anchor_subtraction_not_direct_sim_comparison(self):
        anchor=1792331000000123456
        source=float_to_time(Time(nanoseconds=anchor),8.2935003939201).nanoseconds
        self.assertGreater(abs(source*1e-9-8.2935003939201),1e9)
        self.assertLess(abs((source-anchor)*1e-9-8.2935003939201),1e-9)

    def test_adapter_offset_is_distinct_from_bridge_anchor(self):
        anchor=1792331000000123456;adapter_offset=10_200_000_000
        source=float_to_time(Time(nanoseconds=anchor),8.2935003939201).nanoseconds
        mapped=source+adapter_offset
        self.assertEqual(mapped-source,adapter_offset)
        self.assertLess(abs((mapped-adapter_offset-anchor)*1e-9-8.2935003939201),1e-9)

if __name__=='__main__':
    suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(original.RecordedRosInputTests),unittest.defaultTestLoader.loadTestsFromTestCase(FrameClockTests)])
    stream=io.StringIO();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    root=Path(__file__).resolve().parent
    (root/'frame_clock_tests.log').write_text(stream.getvalue())
    report=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful(),scope=__doc__,beamng_verified=False)
    (root/'frame_clock_tests.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));raise SystemExit(not result.wasSuccessful())
