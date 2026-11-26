import unittest
from visual_planner import evaluate

class TestVisualPlanner(unittest.TestCase):
    def ref(self, left=False, center=False, right=False):
        return {'calibrated': True, 'frames': 5, 'zones': {
            'izquierda': {'confirmed_change': left, 'changed_ratio': .9 if left else 0},
            'centro': {'confirmed_change': center, 'changed_ratio': .9 if center else 0},
            'derecha': {'confirmed_change': right, 'changed_ratio': .9 if right else 0},
        }}
    def test_right_object(self):
        for planner in ('BFS','DFS','UCS','GREEDY','ASTAR'):
            r=evaluate(self.ref(right=True),planner,age_seconds=.1)
            self.assertEqual(r['evaluation_suggested'],'EVALUAR_IZQUIERDA')
            self.assertFalse(r['movement_authorized'])
            self.assertFalse(r['map_updated_from_camera'])
    def test_left_object(self):
        for planner in ('BFS','DFS','UCS','GREEDY','ASTAR'):
            self.assertEqual(evaluate(self.ref(left=True),planner)['evaluation_suggested'],'EVALUAR_DERECHA')
    def test_both_blocked(self):
        self.assertEqual(evaluate(self.ref(left=True,right=True))['evaluation_suggested'],'DETENER_Y_REEVALUAR')
    def test_missing_or_stale(self):
        self.assertEqual(evaluate({})['zones']['centro'],'DESCONOCIDO')
        self.assertEqual(evaluate(self.ref(right=True),age_seconds=2)['inspection_order'],[])
    def test_no_change_is_not_free(self):
        self.assertEqual(evaluate(self.ref())['evaluation_suggested'],'DETENER_Y_REEVALUAR')

if __name__=='__main__': unittest.main()
