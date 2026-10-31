"""Source-to-actuator lifetime and ownership; no ROS/vehicle required."""
import copy,json,unittest
from wildos_nav2_adapter.source_control import (
    SourceControlSender,SourceControlReceiver,receipt_issue,PROTOCOL,COMMAND_TOPIC)

BRAKE=dict(throttle=0.,brake=1.,steering=0.,parkingbrake=0.,clutch=0.,gear=2)
DRIVE=dict(BRAKE,throttle=.1,brake=0.)
W=1_700_000_000_000_000_000; M=100_000_000_000

class SourceControlTests(unittest.TestCase):
    def setUp(self):
        self.sender=SourceControlSender('a'*32)
        self.receiver=SourceControlReceiver()

    def frame(self,control=BRAKE,wall=W,mono=M):
        return self.sender.issue(control,wall,mono)

    def accept(self,frame,wall=W,mono=M,stopped=True):
        return self.receiver.accept(json.dumps(frame),wall,mono,stopped=stopped)

    def arm(self):
        self.assertIsNone(self.accept(self.frame()))

    def test_boot_requires_brake_and_motion_then_uses_same_source_identity(self):
        self.assertIsNotNone(self.accept(self.frame(DRIVE)))
        self.arm()
        f=self.frame(DRIVE)
        self.assertIsNone(self.accept(f))
        self.assertEqual(self.receiver.current(W,M)['sequence'],f['sequence'])

    def test_delay_is_charged_before_and_after_callback_not_restarted(self):
        f=self.frame()
        self.assertIsNone(self.accept(f,W+400_000_000,M+400_000_000))
        self.assertIsNotNone(self.receiver.current(W+499_000_000,M+499_000_000))
        self.assertIsNone(self.receiver.current(W+500_000_000,M+500_000_000))

    def test_expired_future_and_malformed_packets_do_not_renew_current(self):
        self.arm();expiry=self.receiver.deadline_mono_ns
        f=self.frame()
        for packet,wall in ((f,W+600_000_000),(dict(f,issued_wall_ns=W+1),W),
                            (dict(f,sequence=True),W),(dict(f,version='old'),W),
                            (dict(f,control=dict(DRIVE,throttle=float('nan'))),W)):
            self.assertIsNotNone(self.accept(packet,wall,M+400_000_000))
            self.assertEqual(self.receiver.deadline_mono_ns,expiry)

    def test_duplicate_and_reordered_packets_never_extend_deadline_or_restore_throttle(self):
        self.arm();old=self.frame(DRIVE);self.accept(old)
        stop=self.frame(BRAKE,W+50_000_000,M+50_000_000)
        self.accept(stop,W+50_000_000,M+50_000_000)
        deadline=self.receiver.deadline_mono_ns
        self.assertIsNotNone(self.accept(old,W+100_000_000,M+100_000_000))
        self.assertIsNotNone(self.accept(stop,W+100_000_000,M+100_000_000))
        self.assertEqual(self.receiver.deadline_mono_ns,deadline)
        self.assertEqual(self.receiver.current(W+100_000_000,M+100_000_000)['control']['brake'],1.)

    def test_generation_change_requires_brake_and_fences_old_mission(self):
        self.arm();old=self.frame(DRIVE);self.accept(old)
        self.sender.new_generation()
        self.assertIsNotNone(self.accept(self.frame(DRIVE)))
        self.assertIsNone(self.accept(self.frame()))
        self.assertIsNotNone(self.accept(old))

    def test_missing_demand_does_not_block_a_fresh_higher_sequence(self):
        self.arm()
        lost = self.frame(DRIVE, W+50_000_000, M+50_000_000)
        fresh = self.frame(DRIVE, W+100_000_000, M+100_000_000)
        self.assertIsNone(self.accept(fresh, W+100_000_000, M+100_000_000))
        deadline = self.receiver.deadline_mono_ns
        self.assertEqual(self.receiver.current(W+100_000_000, M+100_000_000)['sequence'], fresh['sequence'])
        self.assertIsNotNone(self.accept(lost, W+150_000_000, M+150_000_000))
        self.assertEqual(self.receiver.deadline_mono_ns, deadline)

    def test_lost_generation_brake_cannot_be_replaced_by_a_drive_packet(self):
        self.arm()
        old = self.frame(DRIVE)
        self.accept(old)
        self.sender.new_generation()
        self.frame(BRAKE, W+50_000_000, M+50_000_000)  # Dropped in transport.
        self.assertIsNotNone(self.accept(self.frame(DRIVE, W+100_000_000, M+100_000_000),
                                        W+100_000_000, M+100_000_000))
        brake = self.frame(BRAKE, W+150_000_000, M+150_000_000)
        self.assertIsNone(self.accept(brake, W+150_000_000, M+150_000_000))
        self.assertIsNone(self.accept(self.frame(DRIVE, W+200_000_000, M+200_000_000),
                                     W+200_000_000, M+200_000_000))
        self.assertIsNotNone(self.accept(old, W+250_000_000, M+250_000_000))

    def test_expired_stream_requires_brake_before_motion_can_resume(self):
        self.arm()
        self.assertIsNotNone(self.accept(self.frame(DRIVE,W+600_000_000,M+600_000_000),W+600_000_000,M+600_000_000))
        self.assertIsNone(self.accept(self.frame(BRAKE,W+600_000_000,M+600_000_000),W+600_000_000,M+600_000_000))
        self.assertIsNone(self.accept(self.frame(DRIVE,W+650_000_000,M+650_000_000),W+650_000_000,M+650_000_000))

    def test_foreign_process_cannot_take_live_stream_or_takeover_while_moving(self):
        self.arm();other=SourceControlSender('b'*32)
        self.assertIsNotNone(self.accept(other.issue(BRAKE,W,M)))
        f=other.issue(BRAKE,W+600_000_000,M+600_000_000)
        self.assertIsNotNone(self.accept(f,W+600_000_000,M+600_000_000,False))
        self.assertIsNone(self.accept(f,W+600_000_000,M+600_000_000,True))
        self.assertIsNotNone(self.accept(self.frame(BRAKE,W+650_000_000,M+650_000_000),W+650_000_000,M+650_000_000))

    def test_wall_clock_rollback_cannot_extend_steady_deadline(self):
        self.arm()
        self.assertIsNone(self.receiver.current(W-1_000_000,M+600_000_000))

    def test_receipt_must_match_issued_source_and_actual_control(self):
        self.arm();f=self.frame(DRIVE);self.accept(f)
        receipt=self.receiver.receipt(f,DRIVE,True,W+10_000_000,M+10_000_000)
        self.assertIsNone(receipt_issue(receipt,W+20_000_000))
        self.assertIsNone(self.sender.ack_issue(receipt,W+20_000_000,M+20_000_000))
        for changes in ({'sequence':999},{'sender_id':'b'*32},{'control':BRAKE},{'applied':False}):
            self.assertIsNotNone(self.sender.ack_issue(dict(receipt,**changes),W+20_000_000,M+20_000_000))

    def test_rpc_ack_after_source_expiry_is_not_a_valid_source_receipt(self):
        self.arm();f=self.frame(DRIVE);self.accept(f)
        receipt=self.receiver.receipt(f,DRIVE,True,W+600_000_000,M+600_000_000)
        self.assertFalse(receipt['applied'])
        self.assertIsNotNone(self.sender.ack_issue(receipt,W+600_000_000,M+600_000_000))

    def test_old_generation_ack_cannot_rearm_after_stop(self):
        self.arm();f=self.frame(DRIVE);self.accept(f)
        receipt=self.receiver.receipt(f,DRIVE,True,W,M)
        self.sender.new_generation()
        self.assertIsNotNone(self.sender.ack_issue(receipt,W,M))

    def test_histories_and_packet_size_are_bounded(self):
        self.arm()
        for i in range(300):self.frame(DRIVE)
        self.assertLessEqual(len(self.sender.sent),64)
        self.assertIsNotNone(self.receiver.accept(' '*4097,W,M,stopped=True))

    def test_snapshots_and_returned_frames_cannot_mutate_ownership_history(self):
        f=self.frame();self.accept(f)
        f['control']['throttle']=.8
        saved=self.receiver.current(W,M);saved['control']['brake']=0.
        self.assertEqual(self.receiver.current(W,M)['control'],BRAKE)
        receipt=self.receiver.receipt(self.receiver.current(W,M),BRAKE,True,W,M)
        self.assertIsNone(self.sender.ack_issue(receipt,W,M))

    def test_replayed_older_ack_cannot_replace_newer_ack_and_forward_clock_jump_expires(self):
        first=self.frame();self.accept(first)
        r1=self.receiver.receipt(first,BRAKE,True,W,M)
        second=self.frame(DRIVE);self.accept(second)
        r2=self.receiver.receipt(second,DRIVE,True,W,M)
        self.assertIsNone(self.sender.ack_issue(r2,W,M))
        self.assertIsNotNone(self.sender.ack_issue(r1,W,M))
        self.assertIsNone(self.receiver.current(W+600_000_000,M+1))

    def test_rpc_snapshot_cannot_claim_ack_after_new_generation_takes_ownership(self):
        self.arm();old=self.frame(DRIVE);self.accept(old)
        self.sender.new_generation();self.accept(self.frame())
        receipt=self.receiver.receipt(old,DRIVE,True,W,M)
        self.assertFalse(receipt['applied'])

    def test_pruned_and_malformed_receipts_fail_closed(self):
        f=self.frame();self.accept(f)
        receipt=self.receiver.receipt(f,BRAKE,True,W,M)
        for _ in range(65):self.frame()
        self.assertIsNotNone(self.sender.ack_issue(receipt,W,M))
        for bad in (None,[],{},dict(receipt,applied_wall_ns=True),dict(receipt,control={}),
                    dict(receipt,sequence=True),dict(receipt,applied_wall_ns=W+1)):
            self.assertIsNotNone(self.sender.ack_issue(bad,W,M))

    def test_clock_step_cannot_resurrect_a_demand_already_observed_expired(self):
        self.arm();self.accept(self.frame(DRIVE))
        self.assertIsNone(self.receiver.current(W+600_000_000,M+100_000_000))
        self.assertIsNone(self.receiver.current(W+200_000_000,M+200_000_000))
        self.assertIsNotNone(self.accept(self.frame(DRIVE,W+200_000_000,M+200_000_000),
                                        W+200_000_000,M+200_000_000))
        self.assertIsNone(self.accept(self.frame(BRAKE,W+200_000_000,M+200_000_000),
                                     W+200_000_000,M+200_000_000))

if __name__=='__main__':unittest.main()
