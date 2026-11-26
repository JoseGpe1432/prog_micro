import unittest
from camera_forward import command, evaluate
from proximity import assess
from vision import GroundProjector
import numpy as np
import config as cfg
class CameraForwardTest(unittest.TestCase):
    def seg(self):
        return dict(calibrated=True,state='SEGMENTACION_ACTIVA',frame_age_s=.1,proximity=dict(coverage=10,near_ratio=0,near_blocked=False,nearest_m=None,corridor_visible=False),orange_barriers=[])
    def test_no_history(self): self.assertEqual(command(self.seg())[:2],(.03,0.))
    def test_threshold(self):
        s=self.seg();s['proximity']['nearest_m']=.25
        self.assertEqual(command(s)[:2],(0.,0.))
    def test_slow(self):
        s=self.seg();s['proximity']['nearest_m']=.35
        self.assertEqual(command(s)[:2],(.015,0.))
    def test_stale(self):
        s=self.seg();s['frame_age_s']=.5
        self.assertTrue(evaluate(s)['blocked'])
    def test_orange_unknown(self):
        s=self.seg();s['orange_barriers']=[{'front_m_est':None}]
        self.assertTrue(evaluate(s)['blocked'])
    def test_projected_obstacle(self):
        m=np.zeros((480,640),np.uint8)
        u,v,ok=GroundProjector(640,480).floor_to_pixel(np.array([cfg.ROBOT_RADIUS+.23]),np.array([0.]))
        x,y=int(u[0]),int(v[0]);m[y-5:y+6,x-5:x+6]=255
        self.assertTrue(assess(m)['near_blocked'])
    def test_no_coverage(self):
        s=self.seg();s['proximity']['coverage']=0
        self.assertTrue(evaluate(s)['blocked'])
if __name__=='__main__': unittest.main()
