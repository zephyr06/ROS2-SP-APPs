"""TDD tests for UUniFast-based taskset generation.

Design:
- Most tasks get fixed execution times from UUniFast (deterministic, no env variation).
- N_ENV_DEPENDENT_TASKS tasks get env-dependent execution times: their *mean* ET follows
  the UUniFast allocation, but spatial variation (via GMM + D1_VARIANCE_FACTOR_TABLE) is
  added on top.
- The total utilization must exactly match target MEAN_CPU_UTIL * N_CORES when
  USE_UUNIFAST is enabled.
"""
import os
import sys
import unittest

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.taskset_generator import (
    uunifast_distribution,
    generate_taskset_parameters,
)
from Gen_Taskset.lib.generation_config_parser import standardize_config


class TestUUniFast(unittest.TestCase):
    """Tests for UUniFast execution time generation."""

    def _make_cfg(self, use_uunifast=True):
        return standardize_config({
            "SMALL_PERIOD_HZ": [50],
            "BIG_PERIOD_HZ": [1],
            "N_BIG_PERIOD_TASKS": 1,
            "N_SMALL_PERIOD_TASKS": 1,
            "N_ENV_DEPENDENT_TASKS": 1,
            "MEAN_CPU_UTIL": 0.5,
            "N_CORES": 1,
            "RANDOM_SEED": 42,
            "USE_UUNIFAST": use_uunifast,
            # Minimal GMM params (only env-dependent tasks will use them)
            "RO_1_Et_RANGE": [-0.5, 0.5],
            "RO_2_Et_RANGE": [-0.5, 0.5],
            "D1_RANGE": [-100, 100],
            "D2_RANGE": [0, 360],
            "SIGMA_OVER_Et_RANGE": [0.1, 0.2],
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "SP_THRESHOLDS_SET": [0.2, 0.4, 0.6, 0.8, 1.0],
        })

    def test_uunifast_respects_max_util_cap(self):
        """No single utilization should exceed the per-task cap."""
        vect = uunifast_distribution(n=5, target_util=2.5, max_util_cap=0.95)
        for u in vect:
            self.assertLess(u, 0.95, f"Utilization {u} exceeds cap 0.95")

    def test_uunifast_sum_equals_target(self):
        """UUniFast vector must sum to the target utilization exactly."""
        target = 0.75
        vect = uunifast_distribution(n=5, target_util=target)
        self.assertEqual(len(vect), 5)
        self.assertAlmostEqual(
            sum(vect), target, places=10,
            msg=f"UUniFast sum {sum(vect)} != target {target}"
        )

        # Every individual utilization must be in [0, target]
        for u in vect:
            self.assertGreaterEqual(u, 0.0)
            self.assertLessEqual(u, target)

    def test_uunifast_reproducible_with_seed(self):
        """With same seed, UUniFast must return identical vectors."""
        import random
        random.seed(7)
        v1 = uunifast_distribution(n=4, target_util=1.0)
        random.seed(7)
        v2 = uunifast_distribution(n=4, target_util=1.0)
        self.assertListEqual(v1, v2)

    def test_env_dependent_tasks_have_spatial_params(self):
        """Env-dependent tasks must have non-zero spatial correlation parameters."""
        cfg = self._make_cfg(use_uunifast=True)
        params = generate_taskset_parameters(cfg)

        env_tasks = [t for t in params['tasks'] if t.get('env_dependent')]
        self.assertEqual(
            len(env_tasks), cfg["N_ENV_DEPENDENT_TASKS"],
            "Expected exactly N_ENV_DEPENDENT_TASKS env-dependent tasks"
        )

        for t in env_tasks:
            for comp in t['tasks']:
                ro_1 = abs(comp['ro_1_Et'])
                ro_2 = abs(comp['ro_2_Et'])
                self.assertGreater(
                    ro_1 + ro_2, 0.0,
                    "Env-dependent task should have at least one non-zero spatial correlation"
                )

    def test_non_env_tasks_have_zero_spatial_correlation(self):
        """Non-env tasks must have ro_1_Et = ro_2_Et = 0 (no spatial variation)."""
        cfg = self._make_cfg(use_uunifast=True)
        params = generate_taskset_parameters(cfg)

        non_env = [t for t in params['tasks'] if not t.get('env_dependent')]
        self.assertGreater(len(non_env), 0, "Should have at least one non-env task")

        for t in non_env:
            for comp in t['tasks']:
                self.assertEqual(comp['ro_1_Et'], 0.0)
                self.assertEqual(comp['ro_2_Et'], 0.0)

    def test_total_utilization_matches_target(self):
        """Total utilization of generated taskset must equal MEAN_CPU_UTIL * N_CORES.

        When USE_UUNIFAST=True, the sum of (Et_mean / period) over all tasks must exactly
        match the target utilization.
        """
        cfg = self._make_cfg(use_uunifast=True)
        target_util = cfg["MEAN_CPU_UTIL"] * cfg["N_CORES"]
        params = generate_taskset_parameters(cfg)

        total_util = sum(
            t['Et_mean'] / t['period'] for t in params['tasks']
        )
        self.assertAlmostEqual(
            total_util, target_util, places=6,
            msg=f"Total utilization {total_util} != target {target_util}"
        )

    def test_backward_compat_without_uunifast(self):
        """When USE_UUNIFAST is explicitly False, old generation logic still works."""
        cfg = self._make_cfg(use_uunifast=False)
        params = generate_taskset_parameters(cfg)

        # Should produce valid tasks; legacy mode may have env_dependent=False
        self.assertEqual(len(params['tasks']), cfg["N_BIG_PERIOD_TASKS"] + cfg["N_SMALL_PERIOD_TASKS"])
        for t in params['tasks']:
            self.assertFalse(t.get('env_dependent', False))


if __name__ == "__main__":
    unittest.main()
