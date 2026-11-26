import unittest
import numpy as np
from corridor_history import CorridorHistory
from grid_map import OccupancyGrid
from proximity import assess
from supervised_auto import command
from visual_safety import evaluate
from types import SimpleNamespace

class V68(unittest.TestCase):
    def test_unknown_and_stale(self):
        g=OccupancyGrid(); h=CorridorHistory(); mask=np.zeros((480,640),np.uint8)
        h.update(g,mask,(0,0,0),100)
        self.assertGreater(h.status(100)['unknown_cells'],0)
        self.assertFalse(h.status(100)['complete'])
        self.assertFalse(h.status(101)['complete'])
        self.assertGreater(h.status(100)['current_cells'],0)
        h.invalidate(); self.assertFalse(h.status(100)['complete']); self.assertFalse(h.seen)
    def test_history_from_observations(self):
        g=OccupancyGrid(); h=CorridorHistory(); mask=np.zeros((480,640),np.uint8)
        # Synthetic camera views from behind, including side offsets to cover corners.
        for i,(x,y) in enumerate([(-.40,-.15),(-.40,.15),(-.30,0),(-.20,0),(0,0)]):
            h.update(g,mask,(x,y,0),100+i*.2)
        s=h.status(100.8)
        self.assertGreater(s['historical_cells'],0)
        self.assertTrue(s['complete'],s)
        h.update(g,np.full_like(mask,255),(0,0,0),101)
        self.assertFalse(h.status(101)['complete'])
    def test_decision(self):
        p=assess(np.zeros((480,640),np.uint8)); s={'state':'SEGMENTACION_ACTIVA','frame_age_s':.1,'zones':{k:{'non_floor_ratio':0} for k in ('centro','izquierda','derecha')},'proximity':p}
        self.assertEqual(command(s,'REPOSO_CONFIRMADO',(0,0,0))[0],0)
        hist={'complete':True}
        self.assertEqual(command(s,'REPOSO_CONFIRMADO',(0,0,0),hist)[0],.015)
        self.assertFalse(evaluate(s,[],True,hist)['blocked'])
        self.assertTrue(evaluate(s,[SimpleNamespace(x=.2)],True,hist)['blocked'])
        self.assertTrue(evaluate(s,[],False,hist)['blocked'])
        p['near_blocked']=True
        self.assertEqual(command(s,'SEGUIMIENTO',(0,0,0),hist)[0],0)
        self.assertTrue(evaluate(s,[],True,hist)['blocked'])
