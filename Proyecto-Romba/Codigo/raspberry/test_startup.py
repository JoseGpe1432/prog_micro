import unittest
from supervised_auto import startup_command
class Startup(unittest.TestCase):
    def test_start(self):
        s={'calibrated':True,'state':'SEGMENTACION_ACTIVA','proximity':{'coverage':100,'near_ratio':0,'near_blocked':False}}
        h={'current_cells':17}
        q='REPOSO_CONFIRMADO'; pose=(0,0,0)
        self.assertEqual(startup_command(s,q,pose,{'reasons':['CORREDOR SIN OBSERVACIONES SUFICIENTES']},h)[0],.015)
        self.assertEqual(startup_command(s,q,pose,{'reasons':['CONTRASTE CERCANO']},h)[0],0)
        self.assertEqual(startup_command(s,'INVALIDO',pose,{'reasons':[]},h)[0],0)
        s['proximity']['near_blocked']=True
        self.assertEqual(startup_command(s,q,pose,{'reasons':[]},h)[0],0)
if __name__=='__main__': unittest.main()
