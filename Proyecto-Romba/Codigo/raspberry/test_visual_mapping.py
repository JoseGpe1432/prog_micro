import unittest
import numpy as np
from visual_mapping import VisualMapping
from grid_map import OccupancyGrid
class MappingTest(unittest.TestCase):
    def test_stationary(self):
        m=VisualMapping(); g=OccupancyGrid(); mask=np.zeros((480,640),np.uint8)
        m.update(g,mask,(0,0,0),[]); before=g.lo.copy()
        mask[240:400,250:390]=255
        for _ in range(150): m.update(g,mask,(0,0,0),[dict(persistent=True,misses=0,box=[250,240,140,160])])
        np.testing.assert_array_equal(before,g.lo); self.assertEqual(m.frames,1)
        self.assertFalse(m.status()['movement_authorized'])
    def test_two_views(self):
        m=VisualMapping(); g=OccupancyGrid(); mask=np.zeros((480,640),np.uint8)
        m.update(g,mask,(0,0,0),[]); m.update(g,mask,(.03,0,0),[])
        self.assertGreater(np.count_nonzero(g.lo<-.7),0)
