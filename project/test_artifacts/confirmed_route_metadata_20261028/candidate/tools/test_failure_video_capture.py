"""Actual JPEG/MP4 encoding contracts; no simulator or screen required."""
from pathlib import Path
import tempfile
import time
import unittest
import io
import subprocess
from types import SimpleNamespace
from unittest.mock import Mock
import cv2
import numpy as np
from failure_video_capture import JpegSpool,ScreenCapture

ROOT=Path(__file__).resolve().parents[1]


class FailureVideoContracts(unittest.TestCase):
    def setUp(self):
        root=ROOT/'test_artifacts/offline_navigation/video_tmp'
        root.mkdir(parents=True,exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(dir=root)
        self.directory=Path(self.temp.name)
        assert self.directory.resolve().is_relative_to(root.resolve())
        self.spools=[]

    def spool(self, **kwargs):
        spool=JpegSpool(self.directory,'front_camera',time.monotonic(),**kwargs)
        self.spools.append(spool)
        return spool

    def tearDown(self):
        for s in self.spools:
            if not s.closed: s.file.close()
        self.temp.cleanup()

    def image(self, value, shape=(32,48,3)):
        return np.full(shape,value,np.uint8)

    def test_success_discards_frames_without_generating_mp4(self):
        s=self.spool();s.add(self.image(70),elapsed=0.,source_stamp_ns=1000000001)
        result=s.finish(False,fps=5)
        self.assertFalse(result['saved']);self.assertFalse(s.path.exists())
        self.assertFalse(list(self.directory.glob('*.mp4')))
        self.assertEqual(result['source_frames'][0]['source_stamp_ns'],1000000001)

    def test_failure_preserves_elapsed_gaps_and_exact_source_ns(self):
        s=self.spool()
        for elapsed,value,stamp in [(0.,20,1000000001),(.6,100,1000000002),(1.8,190,1000000003)]:
            s.add(self.image(value),elapsed=elapsed,source_stamp_ns=stamp)
        result=s.finish(True,fps=5,end_elapsed=2.)
        self.assertTrue(result['saved']);self.assertEqual(len(result['frame_index']),11)
        self.assertEqual(result['frame_index'][2]['source_stamp_ns'],1000000001)
        self.assertEqual(result['frame_index'][3]['source_stamp_ns'],1000000002)
        self.assertEqual(result['frame_index'][9]['source_stamp_ns'],1000000003)
        self.assertFalse(s.path.exists())
        cap=cv2.VideoCapture(result['path']);frames=[]
        while True:
            ok,frame=cap.read()
            if not ok:break
            frames.append(frame)
        cap.release();self.assertEqual(len(frames),11)
        self.assertLess(float(frames[2].mean()),40)
        self.assertGreater(float(frames[9].mean()),170)

    def test_initial_delay_is_reported_instead_of_inventing_early_footage(self):
        s=self.spool();s.add(self.image(30),elapsed=2.)
        r=s.finish(True,fps=5,end_elapsed=2.4)
        self.assertTrue(r['saved']);self.assertEqual(r['video_start_elapsed'],2.)
        self.assertEqual(r['frame_index'][0]['elapsed'],2.)
        self.assertEqual(len(r['frame_index']),3)

    def test_visual_review_defers_mp4_then_exports_only_failure(self):
        s=self.spool(); s.add(self.image(90),elapsed=.2,source_stamp_ns=1000000001)
        pending=s.finish(None,fps=5)
        self.assertTrue(pending['review_pending']); self.assertTrue(s.path.exists())
        self.assertFalse(list(self.directory.glob('*.mp4')))
        loaded,meta=JpegSpool.load_pending(pending['metadata_path'])
        r=loaded.finish(True,fps=meta['fps'])
        self.assertTrue(r['saved']); self.assertEqual(r['frame_index'][0]['source_stamp_ns'],1000000001)

    def test_reviewed_success_discards_pending_spool(self):
        s=self.spool();s.add(self.image(60),elapsed=0)
        pending=s.finish(None,fps=5)
        loaded,meta=JpegSpool.load_pending(pending['metadata_path'])
        r=loaded.finish(False,fps=meta['fps'])
        self.assertTrue(r['discarded_temporary_frames'])
        self.assertFalse(s.path.exists());self.assertFalse(list(self.directory.glob('*.mp4')))

    def test_size_and_count_are_bounded(self):
        s=self.spool(max_bytes=10)
        with self.assertRaisesRegex(RuntimeError,'size bound'):
            s.add(self.image(0),elapsed=0.)
        self.assertFalse(s.frames)
        s.file.close();s.closed=True;s.path.unlink()
        t=self.spool(max_frames=1);t.add(self.image(20),elapsed=0.)
        with self.assertRaisesRegex(RuntimeError,'frame bound'):
            t.add(self.image(30),elapsed=.1)
        self.assertEqual(len(t.frames),1)

    def test_time_order_and_timestamp_units_are_checked(self):
        s=self.spool();s.add(self.image(20),elapsed=1.,source_stamp_ns=123)
        with self.assertRaisesRegex(ValueError,'backwards'):
            s.add(self.image(30),elapsed=.9)
        with self.assertRaisesRegex(ValueError,'integer nanoseconds'):
            s.add(self.image(30),elapsed=1.1,source_stamp_ns=123.)
        self.assertEqual(len(s.frames),1)

    def test_no_frames_reports_missing_video(self):
        s=self.spool();r=s.finish(True,fps=5)
        self.assertFalse(r['saved']);self.assertIn('No frames',r['error'])

    def test_resolution_change_retains_recoverable_evidence(self):
        s=self.spool();s.add(self.image(20),elapsed=0.)
        s.add(self.image(30,shape=(48,48,3)),elapsed=.2)
        r=s.finish(True,fps=5)
        self.assertFalse(r['saved']);self.assertIn('resolution changed',r['error'])
        self.assertTrue(Path(r['retained_temporary_frames']).exists())

    def test_screen_pipe_keeps_draining_after_recording_bound(self):
        spool=self.spool(max_frames=1)
        encoded=cv2.imencode('.jpg',self.image(50))[1].tobytes()
        pipe=io.BytesIO(encoded*4)
        screen=ScreenCapture.__new__(ScreenCapture)
        screen.spool=spool;screen.error=None;screen.dropped_after_error=0
        screen.process=SimpleNamespace(stdout=pipe)
        screen.read()
        self.assertEqual(len(spool.frames),1)
        self.assertEqual(screen.dropped_after_error,3)
        self.assertEqual(pipe.tell(),len(encoded)*4)
        self.assertIn('frame bound',screen.error)

    def test_screen_shutdown_reaps_even_an_unresponsive_owned_producer(self):
        screen=ScreenCapture.__new__(ScreenCapture)
        screen.process=Mock();screen.process.poll.return_value=None
        screen.process.wait.side_effect=[subprocess.TimeoutExpired('ffmpeg',5),
            subprocess.TimeoutExpired('ffmpeg',5),0]
        screen.reader=Mock();screen.reader.is_alive.return_value=False;screen.log=Mock()
        screen.stop()
        screen.process.send_signal.assert_called_once()
        screen.process.terminate.assert_called_once();screen.process.kill.assert_called_once()
        self.assertEqual(screen.process.wait.call_count,3)
        screen.process.stdout.close.assert_called_once();screen.log.close.assert_called_once()


if __name__=='__main__':unittest.main(verbosity=2)
