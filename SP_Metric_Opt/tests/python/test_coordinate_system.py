"""Tests for Cartesian GMM coordinate system migration.

These tests enforce that the taskset generator and trace sampler operate
exclusively in Cartesian coordinates, with no polar conversion creep.
"""
import unittest
import os
import sys
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.generation_config_parser import standardize_config
from Gen_Taskset.lib.taskset_generator import generate_taskset_parameters
from Gen_Taskset.lib.gmm_model import GaussianComponent, GMMTaskModel


class TestCoordinateSystem(unittest.TestCase):
    """Phase-1 TDD tests for Cartesian-only GMM operation."""

    def test_d_ranges_are_centered_when_map_provided(self):
        """Parser must center D1_RANGE and D2_RANGE at 0 when map dims are given."""
        config = {
            "MAP_WIDTH_M": 200,
            "MAP_HEIGHT_M": 200,
            "CPU_UTIL_RANDOM_RANGE": [0.5, 0.5],
            "D2_RANGE": [0, 360],  # legacy; should be overridden
        }
        std = standardize_config(config)
        self.assertEqual(std["D1_RANGE"], [-100, 100])
        self.assertEqual(std["D2_RANGE"], [-100, 100])

    def test_variance_factor_table_removed(self):
        """D1_VARIANCE_FACTOR_TABLE must NOT be injected into the config."""
        config = {
            "MAP_WIDTH_M": 200,
            "MAP_HEIGHT_M": 200,
            "CPU_UTIL_RANDOM_RANGE": [0.5, 0.5],
        }
        std = standardize_config(config)
        self.assertNotIn("D1_VARIANCE_FACTOR_TABLE", std)

    def test_env_task_et_at_map_center_is_mean(self):
        """At (x=0, y=0) the conditional mean should equal Et_mean (mean-only mode)."""
        config = {
            "MAP_WIDTH_M": 200,
            "MAP_HEIGHT_M": 200,
            "CPU_UTIL_RANDOM_RANGE": [0.5, 0.5],
            "SIGMA_OVER_Et_RANGE": [0.5, 0.6],
            "RO_1_Et_RANGE": [0.5, 0.7],
            "RO_2_Et_RANGE": [0.0, 0.1],
            "N_GMM_COMPONENTS_PER_TASK": 1,
            "PERIODS_MS": [1000],
            "N_TASKS": 1,
            "RANDOM_SEED": 42,
            "N_CORES": 2,
            "MAX_UTIL_PER_TASK": 0.95,
            "MIN_PERIOD_ENV_DEPENDENT": 0,
            "PERF_RECORD_TASK_PROBABILITY": 0.5,
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "SP_THRESHOLD_RANGE": [0.5, 0.9],
            "FIXED_TASK_SIGMA_RATIO": 0.001,
            "MAX_TIME_LIMIT_OPTIONS": 10,
            "SP_WEIGHTS_SUM": 5.0,
        }
        std = standardize_config(config)
        params = generate_taskset_parameters(std)
        env_task = [t for t in params["tasks"] if t["env_dependent"]][0]

        # Reconstruct a single component to exercise the GMM directly
        comp_dict = env_task["tasks"][0]
        component = GaussianComponent(None, None, coeffs=comp_dict["coeffs"])

        # Mean-only mode should return exactly the component mean
        et_center = component.sample_conditional_execution_time(d1=0.0, d2=0.0, mean_only=True)
        self.assertAlmostEqual(et_center, comp_dict["Et_mean"], places=5)

    def test_env_task_et_scales_with_positive_x_correlation(self):
        """With positive x-correlation, ET at positive x should exceed ET at negative x."""
        config = {
            "MAP_WIDTH_M": 200,
            "MAP_HEIGHT_M": 200,
            "CPU_UTIL_RANDOM_RANGE": [0.5, 0.5],
            "SIGMA_OVER_Et_RANGE": [0.5, 0.6],
            "RO_1_Et_RANGE": [0.7, 0.9],   # strong positive x correlation
            "RO_2_Et_RANGE": [0.0, 0.0],
            "N_GMM_COMPONENTS_PER_TASK": 1,
            "PERIODS_MS": [1000],
            "N_TASKS": 1,
            "RANDOM_SEED": 42,
            "N_CORES": 2,
            "MAX_UTIL_PER_TASK": 0.95,
            "MIN_PERIOD_ENV_DEPENDENT": 0,
            "PERF_RECORD_TASK_PROBABILITY": 0.5,
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "SP_THRESHOLD_RANGE": [0.5, 0.9],
            "FIXED_TASK_SIGMA_RATIO": 0.001,
            "MAX_TIME_LIMIT_OPTIONS": 10,
            "SP_WEIGHTS_SUM": 5.0,
        }
        std = standardize_config(config)
        params = generate_taskset_parameters(std)
        env_task = [t for t in params["tasks"] if t["env_dependent"]][0]

        comp_dict = env_task["tasks"][0]
        component = GaussianComponent(None, None, coeffs=comp_dict["coeffs"])

        et_left = component.sample_conditional_execution_time(d1=-80.0, d2=0.0, mean_only=True)
        et_right = component.sample_conditional_execution_time(d1=80.0, d2=0.0, mean_only=True)

        self.assertGreater(et_right, et_left,
                           "With positive x-correlation, ET should increase as x increases")

    def test_trajectory_y_bounds_use_d2_range(self):
        """Trajectory generator must clip y to D2_RANGE, not mirror D1_RANGE."""
        from Gen_Taskset.lib.trajectory import _apply_step

        # Test that Y clips to D2_RANGE independently
        y = _apply_step(coord=90.0, delta=20.0, min_bound=-50.0, max_bound=50.0)
        self.assertEqual(y, 50.0)

        # And similarly for lower bound
        y = _apply_step(coord=-90.0, delta=-20.0, min_bound=-50.0, max_bound=50.0)
        self.assertEqual(y, -50.0)


if __name__ == "__main__":
    unittest.main()
