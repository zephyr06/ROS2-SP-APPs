"""Tests for trajectory generation physical unit correctness.

These tests specifically target the int()-truncation bug in generate_path_only
that causes the robot to get stuck when step_size < 1.
"""
import os
import sys
import math
import tempfile
import unittest
import shutil

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.trajectory import (
    generate_stops_in_map,
    generate_path_only,
)


class TestTrajectoryPhysicalUnits(unittest.TestCase):
    """Tests for physical-unit-aware trajectory generation."""

    def _compute_bounding_box(self, path):
        """Return (min_x, max_x, min_y, max_y) for a path."""
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        return min(xs), max(xs), min(ys), max(ys)

    def _total_path_length(self, path):
        """Return total euclidean distance accumulated along path."""
        total = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            total += math.hypot(dx, dy)
        return total

    # ------------------------------------------------------------------
    # Bug reproduction: int() truncation causes stuck robot
    # ------------------------------------------------------------------

    def test_small_step_size_robot_should_not_be_stuck(self):
        """With step_size=0.2, the robot must make *some* progress over 1000 steps.

        Because int(0.2) = 0, the buggy code gets stuck at every integer
        boundary. After the fix the robot should move ~200m total.
        """
        cfgs = {
            "D1_RANGE": [-50, 50],
            "D2_RANGE": [0, 360],
            "ROBOT_STEP_SIZE": 0.2,
        }
        stops = [(-50, -50), (-50, 50), (50, -50), (50, 50)]
        path = generate_path_only(cfgs, stops, n_steps=1000)

        total_len = self._total_path_length(path)
        # Even with random reversals, we should move at least 10m in 1000 steps.
        # The buggy code produces total_len ≈ 0 because int(0.2)=0.
        self.assertGreater(total_len, 10.0,
            f"Robot appears stuck: total path length = {total_len}")

    def test_step_size_one_should_move_euclidean_distance(self):
        """With step_size=1.0, 1000 steps should cover at least 50m.

        The buggy code rounds every step via int(), so at near-zero coords
        int(0 ± 1) = 0 → stuck.  The total distance is usually < 5m.
        """
        cfgs = {
            "D1_RANGE": [-50, 50],
            "D2_RANGE": [0, 360],
            "ROBOT_STEP_SIZE": 1.0,
        }
        stops = [(-50, -50), (50, 50)]
        path = generate_path_only(cfgs, stops, n_steps=1000)

        total_len = self._total_path_length(path)
        self.assertGreater(total_len, 20.0,
            f"Robot appears stuck: total path length = {total_len}")

    def test_coordinates_should_be_floats_after_fix(self):
        """After fixing int() truncation, coordinates should be floats."""
        cfgs = {
            "D1_RANGE": [-50, 50],
            "D2_RANGE": [0, 360],
            "ROBOT_STEP_SIZE": 0.5,
        }
        stops = [(-50, -50), (50, 50)]
        path = generate_path_only(cfgs, stops, n_steps=100)

        # At least one coordinate must be a float (not int).
        has_float = any(
            isinstance(p[0], float) or isinstance(p[1], float)
            for p in path
        )
        self.assertTrue(has_float,
            "All coordinates are integers; likely int() truncation persists")

    # ------------------------------------------------------------------
    # Boundary behaviour (these pass before and after the fix)
    # ------------------------------------------------------------------

    def test_path_respects_map_bounds_negative_origin(self):
        """Path should stay inside D1_RANGE even with negative coordinates."""
        cfgs = {
            "D1_RANGE": [-100, 100],
            "D2_RANGE": [0, 360],
            "ROBOT_STEP_SIZE": 5.0,
        }
        stops = [(-100, -100), (100, 100)]
        path = generate_path_only(cfgs, stops, n_steps=200)
        for p in path:
            self.assertGreaterEqual(p[0], -100)
            self.assertLessEqual(p[0], 100)
            self.assertGreaterEqual(p[1], -100)
            self.assertLessEqual(p[1], 100)

    def test_path_respects_map_bounds_positive_origin(self):
        """Path should stay inside D1_RANGE with a positive-only map."""
        cfgs = {
            "D1_RANGE": [0, 100],
            "D2_RANGE": [0, 360],
            "ROBOT_STEP_SIZE": 10.0,
        }
        stops = [(0, 0), (100, 100)]
        path = generate_path_only(cfgs, stops, n_steps=50)
        for p in path:
            self.assertGreaterEqual(p[0], 0)
            self.assertLessEqual(p[0], 100)
            self.assertGreaterEqual(p[1], 0)
            self.assertLessEqual(p[1], 100)

    # ------------------------------------------------------------------
    # Coverage / distance tests
    # ------------------------------------------------------------------

    def test_100x100_map_1mps_1000s_should_span_most_of_map(self):
        """On a 100×100m map with 1.0 m/s for 1000 simulated seconds,
        the bounding box of the path should span > 20 m in at least one axis.
        The buggy code gets stuck near the origin, giving a span < 5 m.
        """
        cfgs = {
            "D1_RANGE": [-50, 50],   # 100m wide
            "D2_RANGE": [0, 360],
            "ROBOT_STEP_SIZE": 1.0,  # 1 m/step assuming 1000ms period
        }
        stops = [(-50, -50), (-50, 50), (50, -50), (50, 50)]
        path = generate_path_only(cfgs, stops, n_steps=1000)

        min_x, max_x, min_y, max_y = self._compute_bounding_box(path)
        span_x = max_x - min_x
        span_y = max_y - min_y
        self.assertGreater(max(span_x, span_y), 20.0,
            f"Path too confined: span_x={span_x}, span_y={span_y}")

    def test_100x100_map_2mps_should_span_even_more(self):
        """Double speed → larger step → wider span."""
        cfgs = {
            "D1_RANGE": [-50, 50],
            "D2_RANGE": [0, 360],
            "ROBOT_STEP_SIZE": 2.0,
        }
        stops = [(-50, -50), (-50, 50), (50, -50), (50, 50)]
        path = generate_path_only(cfgs, stops, n_steps=500)

        min_x, max_x, min_y, max_y = self._compute_bounding_box(path)
        span_x = max_x - min_x
        span_y = max_y - min_y
        self.assertGreater(max(span_x, span_y), 20.0,
            f"Path too confined: span_x={span_x}, span_y={span_y}")

    # ------------------------------------------------------------------
    # Stops generation unchanged
    # ------------------------------------------------------------------

    def test_generate_stops_in_map(self):
        stops = generate_stops_in_map({"D1_RANGE": [0, 100], "D2_RANGE": [0, 100]}, x_step_ratio=0.5, y_step_ratio=0.5)
        self.assertEqual(len(stops), 9)


class TestTrajectoryRegression(unittest.TestCase):
    """Regression tests preserved from the original test_trajectory.py."""

    def setUp(self):
        self.cfgs = {
            "D1_RANGE": [0, 100],
            "D2_RANGE": [0, 360],
            "ROBOT_STEP_SIZE": 10.0
        }

    def test_generate_path_only(self):
        stops = [(0, 0), (100, 100)]
        path = generate_path_only(self.cfgs, stops, n_steps=50)
        self.assertEqual(len(path), 50)
        for point in path:
            self.assertTrue(0 <= point[0] <= 100)
            self.assertTrue(0 <= point[1] <= 100)

    def test_generate_path_only_step_size_fallback(self):
        cfgs = {
            "D1_RANGE": [0, 100],
            "D2_RANGE": [0, 360],
        }
        stops = [(0, 0), (100, 100)]
        path = generate_path_only(cfgs, stops, n_steps=10)
        self.assertEqual(len(path), 10)
        for point in path:
            self.assertTrue(0 <= point[0] <= 100)
            self.assertTrue(0 <= point[1] <= 100)


if __name__ == "__main__":
    unittest.main()
