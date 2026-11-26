import unittest
from types import SimpleNamespace
from route_execution import simulate
from planning_preview import ALGORITHMS
from visual_safety import evaluate
from dashboard import make_app
class V67(unittest.TestCase):
    def test_all_followers(self):
        for a in ALGORITHMS:
            r=simulate(a); self.assertEqual(r['state'],'META_ALCANZADA',a); self.assertGreater(r['steps'],0)
    def test_guard(self):
        s={'state':'SEGMENTACION_ACTIVA','frame_age_s':.1,'proximity':{'corridor_visible':True,'near_blocked':False}}
        self.assertFalse(evaluate(s,[],True)['blocked'])
        self.assertTrue(evaluate(s,[SimpleNamespace(x=.2)],True)['blocked'])
        s['proximity']['corridor_visible']=False
        self.assertTrue(evaluate(s,[],True)['blocked'])
        self.assertFalse(evaluate(s,[],True)['turn_authorized'])
    def test_endpoint(self):
        c=make_app(None).test_client()
        self.assertEqual(c.post('/api/simulate',json={'algorithm':'BFS'}).json['state'],'META_ALCANZADA')
        self.assertEqual(c.post('/api/simulate',json={'algorithm':'bad'}).status_code,400)
