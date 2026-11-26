import unittest,time
import numpy as np
from floor_segmentation import FloorSegmentation
class TestFloor(unittest.TestCase):
    def test_static_object(self):
        f=np.full((480,640,3),(70,130,190),np.uint8)
        s=FloorSegmentation(); self.assertTrue(s.calibrate(f)[0]); now=time.monotonic()
        s.update(f,now); self.assertEqual(s.status()['candidate_boxes'],[])
        f[240:420,30:170]=0
        for i in range(3): s.update(f,now+.2*(i+1))
        r=s.status(); self.assertGreater(r['zones']['izquierda']['non_floor_ratio'],.3); self.assertEqual(r['evaluation_suggested'],'EVALUAR_DERECHA'); self.assertGreater(r['tracking']['persistent_count'],0); self.assertFalse(r['movement_authorized'])
    def test_uncalibrated(self):
        s=FloorSegmentation(); self.assertFalse(s.status()['calibrated']); self.assertFalse(s.calibrate(None)[0])
