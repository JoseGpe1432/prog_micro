import unittest
import cv2
import numpy as np
from camera_motion import CameraMotion

class TestCameraMotion(unittest.TestCase):
    def frame(self):
        rng=np.random.default_rng(123)
        a=rng.integers(0,255,(240,320),dtype=np.uint8)
        return cv2.cvtColor(a,cv2.COLOR_GRAY2BGR)
    def test_motion_invalidates_reference(self):
        m=CameraMotion(); f=self.frame()
        m.update(f,1.0)
        for i in range(1,5):
            matrix=np.float32([[1,0,i*5],[0,1,0]])
            moved=cv2.warpAffine(f,matrix,(320,240))
            result=m.update(moved,1+i*.2)
        self.assertTrue(result['reference_invalidated'])
        self.assertFalse(result['movement_authorized'])
        self.assertFalse(result['free_space_verified'])
    def test_static_scene(self):
        m=CameraMotion();f=self.frame()
        for i in range(8): result=m.update(f,1+i*.2)
        self.assertEqual(result['state'],'APARENTEMENTE_ESTABLE')
        self.assertFalse(result['reference_invalidated'])
    def test_no_texture(self):
        m=CameraMotion();f=np.zeros((240,320,3),dtype=np.uint8)
        m.update(f,1.0);r=m.update(f,1.2)
        self.assertEqual(r['state'],'SEGUIMIENTO_INSUFICIENTE')
        self.assertFalse(r['movement_authorized'])

if __name__=='__main__':unittest.main()
