"""TDD tests for UUniFast-based taskset generation.

Design:
- Most tasks get fixed execution times from UUniFast (deterministic, no env variation).
- Env-dependent tasks (count set by ENV_DEPENDENT_TASKS_RATIO, ceil-rounded and
  clamped to [1, N_TASKS]) get env-dependent execution times: their *mean* ET
  follows the UUniFast allocation, but spatial variation (via GMM) is added on top.
- The total utilization must exactly match the sampled per-core utilization *
  N_CORES when USE_UUNIFAST is enabled.
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
            "PERIODS_MS": [20, 1000],
            "N_TASKS": 2,
            # Pin the ratio to [0.5, 0.5] so n_env = ceil(0.5*2) = 1
            # deterministically (the ratio is the sole source of the count now).
            "ENV_DEPENDENT_TASKS_RATIO": [0.5, 0.5],
            "CPU_UTIL_RANDOM_RANGE": [0.5, 0.5],
            "N_CORES": 1,
            "RANDOM_SEED": 42,
            "USE_UUNIFAST": use_uunifast,
            "MIN_PERIOD_ENV_DEPENDENT": 0,  # no filter, so exact env counts hold
            # Minimal GMM params (only env-dependent tasks will use them)
            "RO_1_Et_RANGE": [-0.5, 0.5],
            "RO_2_Et_RANGE": [-0.5, 0.5],
            "D1_RANGE": [-100, 100],
            "D2_RANGE": [0, 360],
            "SIGMA_OVER_Et_RANGE": [0.1, 0.2],
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "SP_THRESHOLDS_SET": [0.2, 0.4, 0.6, 0.8, 1.0],
            "MAX_UTIL_PER_TASK": 0.95,
            "PERF_RECORD_TASK_PROBABILITY": 0.5,
            "N_GMM_COMPONENTS_PER_TASK": 2,
            "SP_THRESHOLD_RANGE": [0.5, 0.9],
            "FIXED_TASK_SIGMA_RATIO": 0.001,
            "MAX_TIME_LIMIT_OPTIONS": 10,
            "SP_WEIGHTS_SUM": 5.0,
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
            len(env_tasks), 1,
            "Expected exactly 1 env-dependent task (pinned via ratio [0.5, 0.5])"
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
        """Total utilization of generated taskset must equal the sampled per-core
        utilization * N_CORES.

        When USE_UUNIFAST=True, the sum of (Et_mean / period) over all tasks must
        exactly match the target utilization. The per-core utilization is sampled
        from CPU_UTIL_RANDOM_RANGE (a degenerate [0.5, 0.5] here, so the sampled
        value is exactly 0.5 and the target is deterministic).
        """
        cfg = self._make_cfg(use_uunifast=True)
        params = generate_taskset_parameters(cfg)
        target_util = params["per_core_cpu_util"] * cfg["N_CORES"]

        total_util = sum(
            t['Et_mean'] / t['period'] for t in params['tasks']
        )
        self.assertAlmostEqual(
            total_util, target_util, places=6,
            msg=f"Total utilization {total_util} != target {target_util}"
        )

    def test_env_and_time_limit_tasks_are_disjoint(self):
        """No task may be both env_dependent and time_limit_task."""
        cfg = self._make_cfg(use_uunifast=True)
        params = generate_taskset_parameters(cfg)

        for t in params['tasks']:
            env = t.get('env_dependent', False)
            tl = t.get('time_limit_task', False)
            self.assertFalse(
                env and tl,
                f"Task must not be both env_dependent and time_limit_task: {t}"
            )

if __name__ == "__main__":
    unittest.main()
