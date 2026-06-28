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
            "SMALL_PERIOD_HZ": [10, 20, 50],
            "BIG_PERIOD_HZ": [1, 2, 5],
            "D1_RANGE": [-10.0, 10.0],
            "D2_RANGE": [0.0, 360.0],
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
            "SIGMA_OVER_Et_RANGE": [0.5, 0.6],
            "RO_1_Et_RANGE": [-0.9, -0.7],
            "RO_2_Et_RANGE": [-0.1, 0.1],
            "MEAN_CPU_UTIL": 1.2,
            "Et_SCALE_FACTOR": 2.0,
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "N_BIG_PERIOD_TASKS": 2,
            "N_SMALL_PERIOD_TASKS": 2,
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
        cfgs["MIN_PERIOD_WITH_PERFORMANCE_RECORDS"] = 33
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
        cfgs["MIN_PERIOD_WITH_PERFORMANCE_RECORDS"] = 33
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

if __name__ == "__main__":
    unittest.main()
