import unittest
from region_tracker import RegionTracker

class RegionTrackerTests(unittest.TestCase):
    def test_persistence_and_id(self):
        t = RegionTracker(min_hits=3)
        a = t.update([[20, 80, 30, 40]], timestamp=1.0)
        self.assertEqual(a['tracks'][0]['state'], 'CANDIDATO_NUEVO')
        first = a['tracks'][0]['id']
        t.update([[23, 80, 30, 40]], timestamp=1.2, global_shift_px=(3, 0))
        a = t.update([[26, 80, 30, 40]], timestamp=1.4, global_shift_px=(3, 0))
        self.assertEqual(a['tracks'][0]['id'], first)
        self.assertEqual(a['tracks'][0]['state'], 'CANDIDATO_PERSISTENTE')
        self.assertFalse(a['movement_authorized'])
        self.assertFalse(a['obstacle_confirmed'])

    def test_disappears(self):
        t = RegionTracker(max_misses=1)
        t.update([[20, 80, 30, 40]], timestamp=1.0)
        a = t.update([], timestamp=1.2)
        self.assertEqual(a['tracks'][0]['state'], 'SEGUIMIENTO_PERDIDO')
        a = t.update([], timestamp=1.4)
        self.assertEqual(a['tracks'], [])
        self.assertEqual(a['lost_ids'], [1])

    def test_invalid_alignment_clears(self):
        t = RegionTracker()
        t.update([[20, 80, 30, 40]], timestamp=1.0)
        a = t.update([], valid=False, timestamp=1.2)
        self.assertEqual(a['state'], 'SEGUIMIENTO_PERDIDO')
        self.assertEqual(a['tracks'], [])

    def test_long_gap_clears(self):
        t = RegionTracker()
        t.update([[20, 80, 30, 40]], timestamp=1.0)
        a = t.update([[20, 80, 30, 40]], timestamp=4.0)
        self.assertEqual(a['tracks'][0]['hits'], 1)

if __name__ == '__main__': unittest.main()
