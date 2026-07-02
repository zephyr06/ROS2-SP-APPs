import unittest
import os
import sys
import numpy as np

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.taskset_generator import (
    generate_single_gaussian_task,
    generate_taskset_parameters
)

class TestTasksetGenerator(unittest.TestCase):

    def setUp(self):
        self.cfgs = {
            "PERIODS_MS": [1000, 500, 200, 100, 50, 20],
            "N_TASKS": 4,
            "D1_RANGE": [-10.0, 10.0],
            "D2_RANGE": [0.0, 360.0],
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
            "SIGMA_OVER_Et_RANGE": [0.5, 0.6],
            "RO_1_Et_RANGE": [-0.9, -0.7],
            "RO_2_Et_RANGE": [-0.1, 0.1],
            "MEAN_CPU_UTIL": 1.2,
            "Et_SCALE_FACTOR": 2.0,
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "N_ENV_DEPENDENT_TASKS": 1,
            "SP_THRESHOLDS_SET": [0.2, 0.4, 0.6, 0.8, 1.0],
            "N_CORES": 2,
            "RANDOM_SEED": 42
        }

    def test_generate_single_gaussian_task(self):
        task = generate_single_gaussian_task(self.cfgs, period=100.0, et_mean=20.0, et_sigma=2.0)
        self.assertEqual(task["period"], 100.0)
        self.assertEqual(task["Et_mean"], 20.0)
        self.assertEqual(task["Et_sigma"], 2.0)
        self.assertGreater(task["D1_MAX"], task["D1_MIN"])
        self.assertAlmostEqual(task["D1_sigma"], (task["D1_MAX"] - task["D1_MIN"]) / 4.0)
        self.assertGreater(task["D2_MAX"], task["D2_MIN"])
        self.assertAlmostEqual(task["D2_sigma"], (task["D2_MAX"] - task["D2_MIN"]) / 4.0)
        self.assertIn("component", task)
        self.assertIn("coeffs", task)

    def test_generate_taskset_parameters_utilization_and_cores(self):
        import math
        res = generate_taskset_parameters(self.cfgs)
        self.assertEqual(res["n_tasks"], len(res["tasks"]))
        
        # Verify internal mathematical and structural consistency of generated taskset parameters
        for t in res["tasks"]:
            # 1. Periods & counts consistency
            self.assertTrue(isinstance(t["period"], int))
            self.assertGreater(t["period"], 0)
            
            # 2. Grid limits and spatial sigma consistency
            self.assertGreater(t["D1_MAX"], t["D1_MIN"])
            self.assertAlmostEqual(t["D1_sigma"], (t["D1_MAX"] - t["D1_MIN"]) / 4.0)
            self.assertGreater(t["D2_MAX"], t["D2_MIN"])
            self.assertAlmostEqual(t["D2_sigma"], (t["D2_MAX"] - t["D2_MIN"]) / 4.0)
            
            # 3. Component list length and weights normalization consistency
            self.assertEqual(t["n_weights"], len(t["tasks"]))
            self.assertEqual(t["n_weights"], len(t["weights"]))
            self.assertAlmostEqual(sum(t["weights"]), 1.0)
            self.assertTrue(all(w > 0.0 for w in t["weights"]))
            
            # 4. GMM spatial component means midpoint consistency
            for c in t["tasks"]:
                self.assertAlmostEqual(c["coeffs"]["mu1"][0], (t["D1_MAX"] + t["D1_MIN"]) / 2.0)
                self.assertAlmostEqual(c["coeffs"]["mu1"][1], (t["D2_MAX"] + t["D2_MIN"]) / 2.0)
                
                # GMM component execution time mean and sigma bounds
                self.assertGreater(c["Et_mean"], 0)
                self.assertGreater(c["Et_sigma"], 0)
                
            # 5. SP constraints boundaries consistency
            self.assertTrue(0.0 <= t["sp_threshold"] <= 1.0)
            self.assertGreater(t["sp_weight"], 0.0)

            # 6. GMM Mixture Execution Time mean consistency
            expected_mean = sum(c["Et_mean"] * w for c, w in zip(t["tasks"], t["weights"]))
            self.assertAlmostEqual(t["Et_mean"], expected_mean)
            
            # 7. GMM Mixture Execution Time standard deviation consistency
            var_sum = 0.0
            for c, w in zip(t["tasks"], t["weights"]):
                var_sum += (c["Et_sigma"] ** 2) * w
                var_sum += ((c["Et_mean"] - t["Et_mean"]) ** 2) * w
            expected_sigma = math.sqrt(max(0.0, var_sum))
            self.assertAlmostEqual(t["Et_sigma"], expected_sigma)

        # 8. Processor assignments and cores range consistency
        processor_ids = [t["processorId"] for t in res["tasks"]]
        for pid in processor_ids:
            self.assertTrue(isinstance(pid, int))
        max_pid = max(processor_ids)
        for pid in processor_ids:
            self.assertTrue(0 <= pid <= max_pid)
        
        # 5b. Total weight sum consistency
        expected_weight_sum = self.cfgs.get("SP_WEIGHTS_SUM", 5.0)
        self.assertAlmostEqual(
            sum(t["sp_weight"] for t in res["tasks"]), expected_weight_sum
        )

        # 9. Total taskset utilization consistency with reported cpu_util
        total_scaled_util = sum(t["Et_mean"] / t["period"] for t in res["tasks"])
        self.assertAlmostEqual(total_scaled_util, res["cpu_util"])

    def test_normal_tasks_have_random_sigma_and_gaussian_bounds(self):
        """Normal tasks (non-env, non-perf) have natural random sigma and bounds = mean ± 2σ."""
        cfgs = self.cfgs.copy()
        cfgs["PERF_RECORD_TASK_PROBABILITY"] = 0.0  # no perf tasks
        cfgs["N_ENV_DEPENDENT_TASKS"] = 1
        cfgs["RANDOM_SEED"] = 42
        res = generate_taskset_parameters(cfgs)

        normal_tasks = [t for t in res["tasks"] if not t["env_dependent"] and not t["time_limit_task"]]
        self.assertGreater(len(normal_tasks), 0, "Should have at least one normal task")

        for t in normal_tasks:
            # Zero spatial correlations
            for c in t["tasks"]:
                self.assertEqual(c["ro_1_Et"], 0.0)
                self.assertEqual(c["ro_2_Et"], 0.0)
            # Natural sigma drawn from SIGMA_OVER_Et_RANGE [0.5, 0.6]
            sigma_ratio = t["Et_sigma"] / t["Et_mean"]
            self.assertGreater(sigma_ratio, 0.45,
                               "Normal tasks should have random sigma from SIGMA_OVER_Et_RANGE")
            # Bounds = mean ± 2*sigma (clamped at minimum 1.0)
            expected_min = max(1.0, t["Et_mean"] - 2.0 * t["Et_sigma"])
            expected_max = max(1.0, t["Et_mean"] + 2.0 * t["Et_sigma"])
            self.assertAlmostEqual(t["execution_time_min"], expected_min)
            self.assertAlmostEqual(t["execution_time_max"], expected_max)
            # No performance records
            self.assertEqual(t.get("performance_records_time", ""), "")
            self.assertEqual(t.get("performance_records_perf", ""), "")

    def test_perf_tasks_have_tiny_sigma_and_full_range_bounds(self):
        """Perf tasks have tiny sigma, no correlations, full range bounds, and performance records."""
        cfgs = self.cfgs.copy()
        cfgs["PERF_RECORD_TASK_PROBABILITY"] = 1.0  # all eligible non-env tasks become perf
        cfgs["N_ENV_DEPENDENT_TASKS"] = 1
        cfgs["RANDOM_SEED"] = 42
        res = generate_taskset_parameters(cfgs)

        perf_tasks = [t for t in res["tasks"] if t["time_limit_task"]]
        self.assertGreater(len(perf_tasks), 0, "Should have at least one perf task")

        for t in perf_tasks:
            self.assertFalse(t["env_dependent"], "Perf tasks must not be env-dependent")
            # Tiny sigma (< 5% of mean)
            sigma_ratio = t["Et_sigma"] / t["Et_mean"]
            self.assertLess(sigma_ratio, 0.05, "Perf tasks should have tiny sigma")
            # Zero spatial correlations
            for c in t["tasks"]:
                self.assertEqual(c["ro_1_Et"], 0.0)
                self.assertEqual(c["ro_2_Et"], 0.0)
            # Full range bounds = period × FINAL_Et_OVER_PERIOD_RANGE
            expected_min = t["period"] * cfgs["FINAL_Et_OVER_PERIOD_RANGE"][0]
            expected_max = t["period"] * cfgs["FINAL_Et_OVER_PERIOD_RANGE"][1]
            self.assertAlmostEqual(t["execution_time_min"], expected_min, places=1)
            self.assertAlmostEqual(t["execution_time_max"], expected_max, places=1)
            # Has performance record strings
            self.assertGreater(len(t.get("performance_records_time", "")), 0)
            self.assertGreater(len(t.get("performance_records_perf", "")), 0)

    def test_env_tasks_have_spatial_correlations(self):
        """Env tasks have non-zero spatial correlations and natural random sigma."""
        cfgs = self.cfgs.copy()
        cfgs["N_ENV_DEPENDENT_TASKS"] = 2
        cfgs["RANDOM_SEED"] = 42
        res = generate_taskset_parameters(cfgs)

        env_tasks = [t for t in res["tasks"] if t["env_dependent"]]
        self.assertGreaterEqual(len(env_tasks), 2, "Should have at least 2 env tasks")

        for t in env_tasks:
            self.assertFalse(t["time_limit_task"], "Env tasks must not be perf tasks")
            # At least one component has non-zero spatial correlation
            has_corr = any(
                abs(c["ro_1_Et"]) > 0.0 or abs(c["ro_2_Et"]) > 0.0
                for c in t["tasks"]
            )
            self.assertTrue(has_corr, "Env tasks should have spatial correlations")
            # Natural sigma from SIGMA_OVER_Et_RANGE
            sigma_ratio = t["Et_sigma"] / t["Et_mean"]
            self.assertGreater(sigma_ratio, 0.45,
                               "Env tasks should have random natural sigma")

class TestP19UnifiedPoolTasksets(unittest.TestCase):
    """P19: the former big/small period split (``N_BIG_PERIOD_TASKS`` +
    ``N_SMALL_PERIOD_TASKS`` drawing from ``BIG_PERIODS_MS`` /
    ``SMALL_PERIODS_MS``) is collapsed into a single ``N_TASKS`` count drawing
    from one unified ``PERIODS_MS`` pool.

    These tests cover the P19-native behavior that subsumes the old P17
    single-rate cases (``N_BIG=0`` / ``N_SMALL=0``): with one pool there is no
    "all-big vs all-small" distinction to test, so we instead assert (a) every
    generated period comes from ``PERIODS_MS``, (b) the pool exhausts
    gracefully (duplicates allowed) when ``N_TASKS`` exceeds the pool size, and
    (c) the minimal single-task taskset (the former ``N=1`` corner case) still
    generates a schedulable task.
    """

    def setUp(self):
        # Canonical P19 shape: PERIODS_MS + N_TASKS, no big/small split.
        self.cfgs = {
            "PERIODS_MS": [1000, 500, 200, 100, 50, 33, 20],
            "D1_RANGE": [-10.0, 10.0],
            "D2_RANGE": [0.0, 360.0],
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
            "SIGMA_OVER_Et_RANGE": [0.5, 0.6],
            "RO_1_Et_RANGE": [-0.9, -0.7],
            "RO_2_Et_RANGE": [-0.1, 0.1],
            "MEAN_CPU_UTIL": 0.9,
            "Et_SCALE_FACTOR": 2.0,
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "SP_THRESHOLDS_SET": [0.2, 0.4, 0.6, 0.8, 1.0],
            "N_CORES": 2,
            "RANDOM_SEED": 42,
        }
        # The single-task case must stay schedulable: with N_CORES=2 the lone
        # task would carry cpu_util = 0.9 x 2 = 1.8 (> MAX_UTIL_PER_TASK=0.95),
        # so uunifast_distribution cannot realize it. Drop to 1 core there so
        # cpu_util = 0.9 < 1.0. (Mirrors resolve_taskset_config_path's N==1
        # handling -- see P17/P19.)
        self.single_task_cfgs = dict(self.cfgs, N_CORES=1)

    def test_all_periods_from_unified_pool(self):
        """Every generated period is a member of the configured PERIODS_MS."""
        cfgs = dict(self.cfgs, N_TASKS=3, N_ENV_DEPENDENT_TASKS=1)
        res = generate_taskset_parameters(cfgs)
        self.assertEqual(len(res["tasks"]), 3)
        pool = set(cfgs["PERIODS_MS"])
        for t in res["tasks"]:
            self.assertIn(t["period"], pool,
                          f"period {t['period']} not in unified PERIODS_MS pool")

    def test_pool_exhaustion_allows_duplicates(self):
        """N_TASKS > len(PERIODS_MS): pick_period exhausts the pool and falls
        back to allowing duplicate periods (acceptable per the documented
        design -- the period pool is unchanged at large N)."""
        pool = self.cfgs["PERIODS_MS"]
        n_tasks = len(pool) + 3  # strictly more tasks than distinct periods
        cfgs = dict(self.cfgs, N_TASKS=n_tasks, N_ENV_DEPENDENT_TASKS=1)
        res = generate_taskset_parameters(cfgs)
        self.assertEqual(len(res["tasks"]), n_tasks)
        # Every period is still a valid pool member; duplicates are expected.
        pool_set = set(pool)
        for t in res["tasks"]:
            self.assertIn(t["period"], pool_set)
        # Sanity: with more tasks than distinct periods, at least one period
        # must recur (the pool is genuinely exhausted, not silently truncated).
        periods = [t["period"] for t in res["tasks"]]
        self.assertLess(len(set(periods)), len(periods),
                        "expected at least one duplicate period at large N")

    def test_single_task_single_rate(self):
        """N_TASKS=1: the minimal single-rate taskset (the former N=1 corner
        case). The lone task is schedulable -- its utilization stays < 1.0."""
        cfgs = dict(self.single_task_cfgs, N_TASKS=1, N_ENV_DEPENDENT_TASKS=0)
        res = generate_taskset_parameters(cfgs)
        self.assertEqual(len(res["tasks"]), 1)
        self.assertIn(res["tasks"][0]["period"], set(cfgs["PERIODS_MS"]))
        # Utilization vector sums to the configured per-core total (0.9).
        self.assertAlmostEqual(
            sum(t["Et_mean"] / t["period"] for t in res["tasks"]),
            res["cpu_util"],
        )
        # Schedulability: the lone task's utilization is below the per-task cap.
        t = res["tasks"][0]
        self.assertLess(t["Et_mean"] / t["period"], 1.0)

    def test_legacy_big_small_keys_rejected(self):
        """A config still carrying the old paired keys (SMALL_PERIOD_HZ /
        BIG_PERIOD_HZ + N_BIG / N_SMALL counts) is now rejected -- the
        backward-compat alias was removed; configs must use PERIODS_MS +
        N_TASKS directly. generate_taskset_parameters runs standardize_config,
        so the legacy keys surface as a ValueError before generation."""
        legacy = {
            "SMALL_PERIOD_HZ": [10, 20, 50],
            "BIG_PERIOD_HZ": [1, 2, 5],
            "N_BIG_PERIOD_TASKS": 2,
            "N_SMALL_PERIOD_TASKS": 3,
            "D1_RANGE": [-10.0, 10.0],
            "D2_RANGE": [0.0, 360.0],
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
            "SIGMA_OVER_Et_RANGE": [0.5, 0.6],
            "RO_1_Et_RANGE": [-0.9, -0.7],
            "RO_2_Et_RANGE": [-0.1, 0.1],
            "MEAN_CPU_UTIL": 0.9,
            "Et_SCALE_FACTOR": 2.0,
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "SP_THRESHOLDS_SET": [0.2, 0.4, 0.6, 0.8, 1.0],
            "N_CORES": 2,
            "RANDOM_SEED": 42,
        }
        with self.assertRaises(ValueError):
            generate_taskset_parameters(dict(legacy))

    def test_n_tasks_below_one_rejected(self):
        """N_TASKS < 1 -> standardize_config rejects it (the degenerate
        all-zero config from P17 is now simply N_TASKS < 1 under P19)."""
        from Gen_Taskset.lib.generation_config_parser import standardize_config
        # standardize_config validates on first call, so a directly-bad N_TASKS
        # raises immediately -- no two-step setup needed.
        for bad in (0, -1):
            with self.assertRaises(ValueError):
                standardize_config(dict(self.cfgs, N_TASKS=bad))


if __name__ == "__main__":
    unittest.main()
