import unittest
import numpy as np
from proximity import assess
from supervised_auto import command
class ProximityTest(unittest.TestCase):
    def test_clear_and_near(self):
        m=np.zeros((480,640),np.uint8)
        p=assess(m); self.assertFalse(p['corridor_visible']); self.assertFalse(p['near_blocked'])
        m[350:420,290:350]=255
        self.assertGreater(assess(m)['mid_ratio'],0)
    def test_decision(self):
        s={'state':'SEGMENTACION_ACTIVA','frame_age_s':.1,'zones':{k:{'non_floor_ratio':.1} for k in ('centro','izquierda','derecha')},'proximity':{'coverage':100,'corridor_visible':True,'near_ratio':0,'near_blocked':False,'nearest_m':.6}}
        self.assertEqual(command(s,'REPOSO_CONFIRMADO',(0,0,0))[0],.015)
        s['proximity']['near_blocked']=True
        self.assertEqual(command(s,'SEGUIMIENTO',(0,0,0))[:2],(0,0))
