import unittest
from image_stream_health import ImageStreamHealth


class ImageFlowContracts(unittest.TestCase):
    def test_camera_can_be_healthy_without_a_display_consumer(self):
        s=ImageStreamHealth()
        for stamp in (10_000_000_000,10_500_000_000):
            s.observe('camera',stamp,'camera',stamp+50_000_000)
        self.assertTrue(s.source_report(11_000_000_000)['passed'])
        self.assertFalse(s.report(11_000_000_000)['passed'])

    def test_raw_camera_missing_frozen_and_stale_still_fail(self):
        s=ImageStreamHealth()
        self.assertFalse(s.source_report(11_000_000_000)['passed'])
        for _ in range(4):s.observe('camera',10_000_000_000,'camera',10_000_000_000)
        self.assertFalse(s.source_report(10_500_000_000)['passed'])
        s.observe('camera',10_500_000_000,'camera',10_500_000_000)
        self.assertFalse(s.source_report(13_000_000_000)['passed'])

    def test_late_raw_frame_cannot_refresh_a_frozen_stream(self):
        s=ImageStreamHealth()
        for stamp in (10_000_000_000,10_500_000_000):
            s.observe('camera',stamp,'camera',stamp+50_000_000)
        s.observe('camera',10_000_000_000,'camera',10_750_000_000)
        self.assertEqual(s.last_source_ns,10_500_000_000)
        self.assertEqual(s.advancing_sources,2)

    def feed(self, stream, stamp, source_frame='camera', preview_frame='camera'):
        stream.observe('preview', stamp, preview_frame, stamp+100_000_000)
        stream.observe('camera', stamp, source_frame, stamp+50_000_000)

    def test_normal_preview_before_raw_and_best_effort_loss(self):
        s=ImageStreamHealth()
        for stamp in (10_000_000_000,10_500_000_000):self.feed(s,stamp)
        s.observe('camera',10_750_000_000,'camera',10_800_000_000)
        self.assertTrue(s.report(11_000_000_000)['passed'])

    def test_frozen_preview_repeated_heartbeat_is_not_live(self):
        s=ImageStreamHealth()
        for _ in range(4):self.feed(s,10_000_000_000)
        self.assertFalse(s.report(10_500_000_000)['passed'])
        self.assertEqual(s.advancing,1)

    def test_missing_output_stale_output_and_wrong_identity_rejected(self):
        s=ImageStreamHealth();s.observe('camera',10_000_000_000,'camera',10_000_000_000)
        self.assertFalse(s.report(10_500_000_000)['passed'])
        for stamp in (10_000_000_000,10_500_000_000):self.feed(s,stamp,preview_frame='other')
        self.assertFalse(s.report(11_000_000_000)['passed'])
        s=ImageStreamHealth()
        for stamp in (10_000_000_000,10_500_000_000):self.feed(s,stamp)
        self.assertFalse(s.report(13_000_000_000)['passed'])

    def test_out_of_order_does_not_replace_latest_and_memory_is_bounded(self):
        s=ImageStreamHealth()
        for stamp in range(10_000_000_000,12_000_000_000,10_000_000):self.feed(s,stamp)
        last=s.last_preview_ns;count=s.advancing
        self.feed(s,10_000_000_000)
        self.assertEqual((s.last_preview_ns,s.advancing),(last,count))
        self.assertEqual(len(s.sources),128)
        self.assertEqual(len(s.previews),128)
        s.observe('preview',20_000_000_000,'camera',12_000_000_000)
        self.assertEqual(s.last_preview_ns,last)


if __name__=='__main__':unittest.main()
