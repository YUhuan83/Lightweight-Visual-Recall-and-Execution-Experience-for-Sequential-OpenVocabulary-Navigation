import unittest
import numpy as np
from PIL import Image
from preview import sample_ready


class EmptyFrameTests(unittest.TestCase):
    def test_missing_camera(self):
        for reading in (None, {}, {'colour':None}, {'colour':b'not decoded'}):
            self.assertFalse(sample_ready('mounted',reading))
        self.assertTrue(sample_ready('mounted',{'colour':Image.new('RGB',(2,2))}))

    def test_missing_cloud(self):
        for cloud in (None, [], np.zeros((3,)),np.zeros((2,4))):
            self.assertFalse(sample_ready('lidar',{'pointCloud':cloud}))
        self.assertTrue(sample_ready('lidar',{'pointCloud':np.zeros((2,3))}))


if __name__ == '__main__':
    unittest.main()
