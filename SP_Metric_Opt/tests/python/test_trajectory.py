import unittest
import os
import sys

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.trajectory import (
    generate_stops_in_map,
    generate_path_only
)

class TestTrajectory(unittest.TestCase):

    def setUp(self):
        self.cfgs = {
            "D1_RANGE": [0, 100],
            "D2_RANGE": [0, 360],
            "ROBOT_STEP_SIZE": 10.0
        }

    def test_generate_stops_in_map(self):
        stops = generate_stops_in_map(self.cfgs, x_step_ratio=0.5, y_step_ratio=0.5)
        # Should generate stops at combination of:
        # X: 0, 50, 100
        # Y: 0, 50, 100
        # Total = 9 stops
        self.assertEqual(len(stops), 9)
        for stop in stops:
            self.assertTrue(0 <= stop[0] <= 100)
            self.assertTrue(0 <= stop[1] <= 100)

    def test_generate_path_only(self):
        stops = [(0, 0), (100, 100)]
        path = generate_path_only(self.cfgs, stops, n_steps=50)
        self.assertEqual(len(path), 50)
        for point in path:
            self.assertTrue(0 <= point[0] <= 100)
            self.assertTrue(0 <= point[1] <= 100)

if __name__ == "__main__":
    unittest.main()
