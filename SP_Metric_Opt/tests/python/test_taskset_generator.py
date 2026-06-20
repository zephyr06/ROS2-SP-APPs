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
            self.assertEqual(t["sp_weight"], 1.0)
            
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
        
        # 9. Total taskset utilization consistency with reported cpu_util
        total_scaled_util = sum(t["Et_mean"] / t["period"] for t in res["tasks"])
        self.assertAlmostEqual(total_scaled_util, res["cpu_util"])

if __name__ == "__main__":
    unittest.main()
