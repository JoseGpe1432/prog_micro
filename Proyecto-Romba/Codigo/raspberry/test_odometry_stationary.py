import unittest
from unittest.mock import patch
import numpy as np
import cv2
from visual_odometry import VisualOdometry
class StationaryTest(unittest.TestCase):
    def test_yaw_drift(self):
        f=np.random.default_rng(2).integers(0,256,(240,320,3),dtype=np.uint8)
        with patch('visual_odometry.time.monotonic') as clock:
            clock.return_value=0; v=VisualOdometry()
            for i in range(40):
                clock.return_value=i*.2; v.update(f,i*.003,'MOVIMIENTO_DETECTADO')
            self.assertTrue(v.calibrated)
            self.assertEqual((v.x,v.y,v.th),(0,0,0))
            self.assertEqual(v.quality,'REPOSO_CONFIRMADO')
            self.assertAlmostEqual(v.yaw_bias,.015,places=5)
    def test_motion_after_calibration(self):
        f=np.random.default_rng(2).integers(0,256,(480,640,3),dtype=np.uint8)
        with patch('visual_odometry.time.monotonic') as clock:
            clock.return_value=0; v=VisualOdometry()
            for i in range(30):
                clock.return_value=i*.2; v.update(f,0,'APARENTEMENTE_ESTABLE')
            for i in range(1,5):
                clock.return_value=(29+i)*.2
                moved=cv2.warpAffine(f,np.float32([[1,0,i*5],[0,1,0]]),(640,480))
                v.update(moved,0,'MOVIMIENTO_DETECTADO')
            self.assertGreater(abs(v.x)+abs(v.y),0)
            before=(v.x,v.y,v.th)
            for i in range(5,10):
                clock.return_value=(29+i)*.2; v.update(moved,0,'MOVIMIENTO_DETECTADO')
            self.assertEqual(before,(v.x,v.y,v.th))
            self.assertEqual(v.quality,'REPOSO_CONFIRMADO')
