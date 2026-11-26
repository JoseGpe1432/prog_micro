import unittest
from unittest.mock import patch
import numpy as np
from visual_odometry import VisualOdometry
class PhaseRestTest(unittest.TestCase):
    def test_phase_without_points(self):
        f=np.random.default_rng(3).integers(0,256,(240,320,3),dtype=np.uint8)
        with patch('visual_odometry.time.monotonic') as clock:
            clock.return_value=0; v=VisualOdometry()
            v._points=lambda *args: None
            for i in range(40):
                clock.return_value=i*.2; v.update(f,0)
            self.assertTrue(v.calibrated)
            self.assertEqual(v.quality,'REPOSO_CONFIRMADO')
            self.assertEqual((v.x,v.y,v.th),(0,0,0))
    def test_blank_or_missing_imu(self):
        for f,yaw in [(np.zeros((240,320,3),np.uint8),0),(np.random.default_rng(3).integers(0,256,(240,320,3),dtype=np.uint8),None)]:
            with patch('visual_odometry.time.monotonic') as clock:
                clock.return_value=0; v=VisualOdometry()
                for i in range(85):
                    clock.return_value=i*.2; v.update(f,yaw)
                self.assertFalse(v.calibrated)
                self.assertGreater(v.cal_retries,0)
