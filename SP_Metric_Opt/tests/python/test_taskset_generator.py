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
        self.assertEqual(task["D1_MIN"], -10.0)
        self.assertEqual(task["D1_MAX"], 10.0)
        self.assertEqual(task["D2_MIN"], 0.0)
        self.assertEqual(task["D2_MAX"], 360.0)
        self.assertIn("component", task)
        self.assertIn("coeffs", task)

    def test_generate_taskset_parameters_utilization_and_cores(self):
        res = generate_taskset_parameters(self.cfgs)
        self.assertEqual(res["n_tasks"], 4)
        
        # Verify cpu util is scaled close to target
        self.assertAlmostEqual(res["cpu_util"], 1.2)
        
        calculated_util = sum(t["Et_mean"] / t["period"] for t in res["tasks"])
        self.assertAlmostEqual(calculated_util, 1.2)

        # Check core assignment
        core_ids = [t["processorId"] for t in res["tasks"]]
        for cid in core_ids:
            self.assertTrue(cid in [0, 1])

        # Verify GMM component counts
        for t in res["tasks"]:
            self.assertEqual(t["n_weights"], len(t["tasks"]))

if __name__ == "__main__":
    unittest.main()
