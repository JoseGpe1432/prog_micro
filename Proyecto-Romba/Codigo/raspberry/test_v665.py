import unittest
import numpy as np
import config as cfg
from vision import GroundProjector
from proximity import assess
from supervised_auto import command
from planning_preview import Preview, ALGORITHMS

class V665(unittest.TestCase):
    def test_geometry(self):
        x,y=GroundProjector(640,480).pixel_to_floor(320,240)
        self.assertAlmostEqual(x-cfg.ROBOT_RADIUS,.55,places=2)
    def test_blind_corridor_stops(self):
        p=assess(np.zeros((480,640),np.uint8))
        self.assertFalse(p['corridor_visible'])
        self.assertGreater(p['blind_front_m_est'],.18)
        s={'state':'SEGMENTACION_ACTIVA','frame_age_s':.1,'zones':{k:{'non_floor_ratio':0} for k in ('centro','izquierda','derecha')},'proximity':p}
        self.assertEqual(command(s,'REPOSO_CONFIRMADO',(0,0,0))[:2],(0,0))
    def test_five_routes_demo(self):
        p=Preview();p.set_demo(True);p.set_goal(p.grid.origin,p.grid.origin+5)
        for alg in ALGORITHMS:
            self.assertTrue(p.plan((0,0,0),alg)[0],alg)
