import unittest
from recording_window import select_rviz_window

class RecordingWindowTests(unittest.TestCase):
    def test_recorded_qt_main_and_two_render_children(self):
        tree='''0x400092 "/path/beamng_navigation.rviz* - RViz": ("rviz2" "rviz2") 1920x1080+0+0
          0x400094 "rviz2": ("rviz2" "rviz2") 939x949+972+93
          0x400013 "rviz2": ("rviz2" "rviz2") 907x971+25+71'''
        self.assertEqual(select_rviz_window(tree),'0x400092')

    def test_camera_panel_alone_is_not_the_requested_layout(self):
        with self.assertRaises(ValueError):
            select_rviz_window('0x400094 "rviz2": ("rviz2" "rviz2") 939x949+0+0')

    def test_two_main_windows_require_disambiguation(self):
        with self.assertRaises(ValueError):
            select_rviz_window('0x1 "RViz": ("rviz2" "rviz2") 1920x1080+0+0\n0x2 "RViz": ("rviz2" "rviz2") 1280x720+0+0')

if __name__=='__main__':unittest.main(verbosity=2)
