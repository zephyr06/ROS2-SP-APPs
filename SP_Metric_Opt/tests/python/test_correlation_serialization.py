"""Tests for correlation coefficient (ro_1_Et, ro_2_Et) serialization correctness.

These tests specifically target the bug in generate_taskset_parameters where
ro_1_Et and ro_2_Et are both set to c.mean_vector[2] (the Et mean, e.g. 180.0)
instead of the actual correlation coefficients (which must lie within [-1, 1]).

The bug causes:
  - Correlations >|1| → invalid covariance matrices.
  - np.sqrt(max(0, sigma_22 - ...)) returns 0 for conditional variance.
  - Execution time sampling loses all variance and becomes deterministic once reloaded.
"""
import os
import sys
import tempfile
import unittest
import yaml

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.taskset_generator import generate_taskset_parameters, load_and_fill_taskset_param_file
from Gen_Taskset.lib.generation_config_parser import standardize_config


class TestCorrelationSerialization(unittest.TestCase):
    """TDD tests for the ro_1_Et / ro_2_Et serialization bug."""

    def _make_config(self):
        """A minimal config that triggers the generation pipeline."""
        return standardize_config({
            "PERIODS_MS": [20, 1000],
            "N_TASKS": 2,
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
            "SIGMA_OVER_Et_RANGE": [0.5, 0.6],
            "RO_1_Et_RANGE": [-0.9, -0.7],
            "RO_2_Et_RANGE": [-0.1, 0.1],
            "N_GMM_COMPONENTS_PER_TASK": 2,
            "SP_THRESHOLD_RANGE": [0.5, 0.9],
            "CPU_UTIL_RANDOM_RANGE": [0.5, 0.5],
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "N_CORES": 1,
            "RANDOM_SEED": 42,
            "D1_RANGE": [-100, 100],
            "D2_RANGE": [0, 360],
            "MAX_UTIL_PER_TASK": 0.95,
            "MIN_PERIOD_ENV_DEPENDENT": 0,
            "PERF_RECORD_TASK_PROBABILITY": 0.5,
            "FIXED_TASK_SIGMA_RATIO": 0.001,
            "MAX_TIME_LIMIT_OPTIONS": 10,
            "SP_WEIGHTS_SUM": 5.0,
        })

    def test_correlations_within_bounds_after_generation(self):
        """Bug: ro_1_Et and ro_2_Et are set to c.mean_vector[2] ~ 180.

        All correlation coefficients MUST lie in [-1.0, 1.0].
        """
        cfg = self._make_config()
        taskset_params = generate_taskset_parameters(cfg, n_sec=100)
        for task in taskset_params['tasks']:
            for comp in task['tasks']:
                ro_1 = comp['ro_1_Et']
                ro_2 = comp['ro_2_Et']
                self.assertGreaterEqual(
                    ro_1, -1.0,
                    f"ro_1_Et={ro_1} out of bounds; should be in [-1,1]"
                )
                self.assertLessEqual(
                    ro_1, 1.0,
                    f"ro_1_Et={ro_1} out of bounds; should be in [-1,1]"
                )
                self.assertGreaterEqual(
                    ro_2, -1.0,
                    f"ro_2_Et={ro_2} out of bounds; should be in [-1,1]"
                )
                self.assertLessEqual(
                    ro_2, 1.0,
                    f"ro_2_Et={ro_2} out of bounds; should be in [-1,1]"
                )

    def test_reloaded_taskset_has_valid_covariance_variance(self):
        """Buggy ro_1_Et/ro_2_Et (e.g. 180) makes covariance matrix have enormous off-diagonal
        values. When we invert it and compute conditional variance, the sqrt often becomes
        exactly 0.0 because of floating-point cancellation.

        After serialization + reload, sqrt_cond_var must be > 0 (there must be some variance).
        """
        cfg = self._make_config()
        taskset_params = generate_taskset_parameters(cfg, n_sec=100)

        # Write to temp YAML and reload (simulating the C++ loader path)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(taskset_params, f)
            fpath = f.name

        try:
            reloaded = load_and_fill_taskset_param_file(fpath)
        finally:
            os.unlink(fpath)

        zero_var_count = 0
        for task in reloaded['tasks']:
            for comp in task['tasks']:
                # After load_and_fill, coeffs dict is populated
                coeffs = comp['coeffs']
                sqrt_cond_var = coeffs['sqrt_cond_var']
                if sqrt_cond_var == 0.0:
                    zero_var_count += 1

        # With valid correlations in [-1,1], zero variance should never happen.
        self.assertEqual(
            zero_var_count, 0,
            f"{zero_var_count} components have zero conditional variance after reload; "
            "this indicates an invalid covariance matrix (likely from |ro| > 1)"
        )

    def test_correlations_preserved_after_reload(self):
        """ro_1_Et/ro_2_Et should remain identical after YAML round-trip."""
        cfg = self._make_config()
        taskset_params = generate_taskset_parameters(cfg, n_sec=100)

        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(taskset_params, f)
            fpath = f.name

        try:
            reloaded = load_and_fill_taskset_param_file(fpath)
        finally:
            os.unlink(fpath)

        for i, task in enumerate(taskset_params['tasks']):
            for k, comp in enumerate(task['tasks']):
                reloaded_comp = reloaded['tasks'][i]['tasks'][k]
                self.assertAlmostEqual(
                    comp['ro_1_Et'], reloaded_comp['ro_1_Et'],
                    msg=f"Task {i} component {k}: ro_1_Et changed after reload"
                )
                self.assertAlmostEqual(
                    comp['ro_2_Et'], reloaded_comp['ro_2_Et'],
                    msg=f"Task {i} component {k}: ro_2_Et changed after reload"
                )

    def test_ro_1_and_ro_2_are_different_when_ranges_differ(self):
        """When RO_1_Et_RANGE and RO_2_Et_RANGE differ, at least some component should
        have different ro_1_Et and ro_2_Et.

        The buggy code sets both to c.mean_vector[2], so they'd always be identical.
        """
        cfg = self._make_config()
        taskset_params = generate_taskset_parameters(cfg, n_sec=100)

        any_different = False
        for task in taskset_params['tasks']:
            for comp in task['tasks']:
                if abs(comp['ro_1_Et'] - comp['ro_2_Et']) > 1e-6:
                    any_different = True
                    break
            if any_different:
                break

        self.assertTrue(any_different,
            "All components have ro_1_Et == ro_2_Et; "
            "this is a strong signal they were both set to c.mean_vector[2]"
        )


if __name__ == "__main__":
    unittest.main()
