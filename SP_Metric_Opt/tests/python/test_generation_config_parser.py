import unittest
import os
import sys

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.generation_config_parser import (
    standardize_config,
    validate_generation_config
)

class TestGenerationConfigParser(unittest.TestCase):

    def test_standardize_config_hz_conversion(self):
        config = {
            "SMALL_PERIOD_HZ": [10, 20, 50],
            "BIG_PERIOD_HZ": [1, 2, 5],
            "D1_RANGE": [-10, 10],
            "Et_SCALE_FACTOR": 2.0
        }
        res = standardize_config(config)
        # 1000 / 10 = 100, 1000 / 20 = 50, 1000 / 50 = 20
        self.assertEqual(res["SMALL_PERIODS_MS"], [100, 50, 20])
        # 1000 / 1 = 1000, 1000 / 2 = 500, 1000 / 5 = 200
        self.assertEqual(res["BIG_PERIODS_MS"], [1000, 500, 200])

    def test_standardize_config_defaults(self):
        config = {
            "D1_RANGE": [-5, 5],
            "Et_SCALE_FACTOR": 2.0
        }
        res = standardize_config(config)
        self.assertEqual(res["SMALL_PERIODS_MS"], [100, 50, 33, 20])
        self.assertEqual(res["BIG_PERIODS_MS"], [4000, 2000, 1000])
        self.assertEqual(res["N_BIG_PERIOD_TASKS"], 2)
        self.assertEqual(res["N_SMALL_PERIOD_TASKS"], 8)
        self.assertEqual(res["N_TASKS"], 10)

    def test_validate_generation_config(self):
        # Valid config
        valid_cfg = {
            "D1_RANGE": [-10, 10],
            "D2_RANGE": [0, 360],
            "MEAN_CPU_UTIL": 0.9
        }
        self.assertTrue(validate_generation_config(valid_cfg))

        # Invalid config (missing MEAN_CPU_UTIL)
        invalid_cfg = {
            "D1_RANGE": [-10, 10],
            "D2_RANGE": [0, 360]
        }
        self.assertFalse(validate_generation_config(invalid_cfg))

if __name__ == "__main__":
    unittest.main()
