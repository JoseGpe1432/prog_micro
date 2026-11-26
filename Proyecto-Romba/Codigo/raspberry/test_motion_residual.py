import unittest
import cv2
import numpy as np
from motion_residual import MotionResidual

class TestResidual(unittest.TestCase):
    def frame(self):
        rng=np.random.default_rng(7)
        img=rng.integers(20,230,(240,320),dtype=np.uint8)
        return cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    def test_static_has_low_residual(self):
        m=MotionResidual();f=self.frame();m.update(f,1.0)
        d=m.update(f,1.2)
        self.assertEqual(d['state'],'RESIDUOS_CALCULADOS')
        self.assertLess(d['residual_ratio'],.01)
        self.assertFalse(d['movement_authorized'])
    def test_translation_compensation(self):
        m=MotionResidual();f=self.frame();m.update(f,1.0)
        moved=cv2.warpAffine(f,np.float32([[1,0,6],[0,1,2]]),(320,240))
        d=m.update(moved,1.2)
        self.assertEqual(d['state'],'RESIDUOS_CALCULADOS')
        self.assertLess(d['residual_ratio'],.06)
    def test_blank_scene_unknown(self):
        m=MotionResidual();f=np.zeros((240,320,3),dtype=np.uint8)
        m.update(f,1.0);d=m.update(f,1.2)
        self.assertEqual(d['state'],'SEGUIMIENTO_INSUFICIENTE')
        self.assertFalse(d['free_space_verified'])
    def test_local_change_not_obstacle_confirmation(self):
        m=MotionResidual();f=self.frame();m.update(f,1.0)
        changed=f.copy();cv2.rectangle(changed,(180,160),(230,220),(0,0,0),-1)
        d=m.update(changed,1.2)
        self.assertEqual(d['state'],'RESIDUOS_CALCULADOS')
        self.assertGreater(d['residual_ratio'],.01)
        self.assertFalse(d['obstacle_confirmed'])

if __name__=='__main__':unittest.main()
