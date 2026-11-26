import unittest
from camera_avoid import CameraAvoid
from test_camera_forward import CameraForwardTest

class AvoidTest(unittest.TestCase):
    def seg(self,d=None,l=.0,r=.02):
        s=CameraForwardTest().seg();s['proximity']['nearest_m']=d
        s['zones']={'izquierda':{'non_floor_ratio':l},'derecha':{'non_floor_ratio':r}}
        return s
    def test_left(self):
        a=CameraAvoid();s=self.seg(.25)
        self.assertEqual(a.step(s,[],True,0)[:2],(0,0))
        self.assertEqual(a.step(s,[],True,.5)[:2],(0,.18))
        self.assertEqual(a.step(self.seg(),[],True,1.4)[:2],(0,0))
        self.assertEqual(a.step(self.seg(),[],True,1.9)[:2],(.03,0))
    def test_right(self):
        a=CameraAvoid();s=self.seg(.25,.04,0)
        a.step(s,[],True,0);self.assertEqual(a.step(s,[],True,.5)[:2],(0,-.18))
    def test_uncertain(self):
        a=CameraAvoid();s=self.seg(.25);a.step(s,[],True,0)
        self.assertEqual(a.step(s,[],False,.5)[:2],(0,0));self.assertEqual(a.phase,'HALT')
    def test_both_blocked(self):
        a=CameraAvoid();s=self.seg(.25,.1,.1);a.step(s,[],True,0)
        self.assertEqual(a.step(s,[],True,.5)[:2],(0,0));self.assertEqual(a.phase,'HALT')
    def test_close(self):
        a=CameraAvoid();self.assertEqual(a.step(self.seg(.1),[],True,0)[:2],(0,0));self.assertEqual(a.phase,'HALT')
    def test_side_changes(self):
        a=CameraAvoid();s=self.seg(.25);a.step(s,[],True,0);a.step(s,[],True,.5)
        self.assertEqual(a.step(self.seg(.25,.2,0),[],True,.6)[:2],(0,0));self.assertEqual(a.phase,'HALT')
