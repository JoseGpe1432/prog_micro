import unittest
from supervised_auto import command
class AutoTest(unittest.TestCase):
    def test_commands(self):
        s={'state':'SEGMENTACION_ACTIVA','frame_age_s':.1,'zones':{k:{'non_floor_ratio':0} for k in ('centro','izquierda','derecha')}}
        self.assertEqual(command(s,'REPOSO_CONFIRMADO',(0,0,0))[0],.03)
        self.assertEqual(command(s,'DEBIL',(0,0,0))[:2],(0,0))
        s['zones']['centro']['non_floor_ratio']=.2
        self.assertEqual(command(s,'SEGUIMIENTO',(0,0,0))[0],0)
        s['frame_age_s']=1
        self.assertEqual(command(s,'SEGUIMIENTO',(0,0,0))[:2],(0,0))
