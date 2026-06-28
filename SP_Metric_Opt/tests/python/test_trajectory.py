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
        # D1_RANGE=[0,100] => x step = 50 => stops at 0, 50, 100 (3)
        # D2_RANGE=[0,360] => y step = 180 => stops at 0, 180, 360 (3)
        # Total = 9 stops
        self.assertEqual(len(stops), 9)
        for stop in stops:
            self.assertTrue(0 <= stop[0] <= 100)
            self.assertTrue(0 <= stop[1] <= 360)

    def test_generate_path_only(self):
        stops = [(0, 0), (100, 100)]
        path = generate_path_only(self.cfgs, stops, n_steps=50)
        self.assertEqual(len(path), 50)
        for point in path:
            self.assertTrue(0 <= point[0] <= 100)
            self.assertTrue(0 <= point[1] <= 100)

    def test_generate_path_only_physical_units(self):
        """With step_size = 1.0, each move should change coordinate by at most 1."""
        cfgs = {
            "D1_RANGE": [0, 20],
            "D2_RANGE": [0, 360],
            "ROBOT_STEP_SIZE": 1.0
        }
        stops = [(0, 0), (20, 20)]
        path = generate_path_only(cfgs, stops, n_steps=30)
        self.assertEqual(len(path), 30)
        for i in range(1, len(path)):
            prev = path[i - 1]
            curr = path[i]
            self.assertLessEqual(abs(curr[0] - prev[0]), 1)
            self.assertLessEqual(abs(curr[1] - prev[1]), 1)

    def test_generate_path_only_step_size_fallback(self):
        """When ROBOT_STEP_SIZE is not in cfgs, should default to 5.0."""
        cfgs = {
            "D1_RANGE": [0, 100],
            "D2_RANGE": [0, 360],
        }
        stops = [(0, 0), (100, 100)]
        path = generate_path_only(cfgs, stops, n_steps=10)
        self.assertEqual(len(path), 10)
        # Because default step size is 5.0, bounding box should still hold
        for point in path:
            self.assertTrue(0 <= point[0] <= 100)
            self.assertTrue(0 <= point[1] <= 100)

    def test_generate_stops_in_map_physical_params(self):
        """Stops should be generated with rectangular map params."""
        cfgs = {
            "D1_RANGE": [-150, 150],
            "D2_RANGE": [0, 360],
        }
        stops = generate_stops_in_map(cfgs, x_step_ratio=0.5, y_step_ratio=0.5)
        # x: -150, 0, 150 -> 3 stops
        # y: 0, 180, 360 -> 3 stops
        # total = 9
        self.assertEqual(len(stops), 9)
        for stop in stops:
            self.assertTrue(-150 <= stop[0] <= 150)
            self.assertTrue(0 <= stop[1] <= 360)

if __name__ == "__main__":
    unittest.main()
