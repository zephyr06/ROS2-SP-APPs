import os
import sys
import tempfile
import shutil
import unittest
import numpy as np
import yaml

# Ensure project root and its parent are in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.dirname(PROJECT_ROOT))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "Visualize_SP_Metric"))

from Visualize_SP_Metric.visualize_ET_distribution import (
    average_execution_time_with_intervals,
    read_period,
    read_ET_data_from_file
)

class TestVisualizeUtils(unittest.TestCase):

    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def test_average_execution_time_with_intervals_success(self):
        # Timestamps: 1.0, 5.0 in interval 1 [0, 10)
        # Timestamps: 12.0, 15.0 in interval 2 [10, 20)
        time_stamps = np.array([1.0, 5.0, 12.0, 15.0])
        execution_times = np.array([10.0, 20.0, 30.0, 40.0])
        T = 10.0

        res = average_execution_time_with_intervals(time_stamps, execution_times, T)
        
        # Expected midpoints: 5.0, 15.0
        # Expected averages: (10+20)/2 = 15.0, (30+40)/2 = 35.0
        expected = np.array([
            [5.0, 15.0],
            [15.0, 35.0]
        ])
        np.testing.assert_allclose(res, expected)

    def test_average_execution_time_with_intervals_mismatch_length(self):
        time_stamps = np.array([1.0, 5.0])
        execution_times = np.array([10.0])
        with self.assertRaises(ValueError):
            average_execution_time_with_intervals(time_stamps, execution_times, 10.0)

    def test_read_period_valid(self):
        # Create a dummy yaml file
        yaml_content = {
            "tasks": [
                {"name": "MPC", "period": 500},
                {"name": "SLAM", "period": 1000}
            ]
        }
        yaml_path = os.path.join(self.temp_dir, "tasks.yaml")
        with open(yaml_path, "w") as f:
            yaml.dump(yaml_content, f)

        # Retrieve periods
        self.assertEqual(read_period(yaml_path, "MPC"), 500)
        self.assertEqual(read_period(yaml_path, "SLAM"), 1000)
        # Non-existent task
        self.assertIsNone(read_period(yaml_path, "TSP"))

    def test_read_period_file_not_found(self):
        # Non-existent file
        res = read_period(os.path.join(self.temp_dir, "missing.yaml"), "MPC")
        self.assertIsNone(res)

    def test_read_ET_data_from_file_parsing(self):
        # Create a dummy execution time file
        et_content = (
            "task::MPC::25.5\n"
            "task::MPC::invalid\n"
            "task::MPC::12.3\n"
        )
        et_path = os.path.join(self.temp_dir, "MPC_execution_time.txt")
        with open(et_path, "w") as f:
            f.write(et_content)

        # Parse without sampling
        res = read_ET_data_from_file(et_path, sample_threshold=100)
        np.testing.assert_allclose(res, np.array([25.5, 12.3]))

    def test_read_ET_data_from_file_sampling(self):
        # Create a file with 20 entries
        et_path = os.path.join(self.temp_dir, "MPC_execution_time.txt")
        with open(et_path, "w") as f:
            for i in range(20):
                f.write(f"task::MPC::{float(i)}\n")

        # Parse with low sample threshold to force sampling
        res = read_ET_data_from_file(et_path, sample_threshold=10, sample_rate=0.5)
        # Should sample approximately 10 elements (20 * 0.5)
        self.assertTrue(len(res) > 0)
        self.assertTrue(len(res) <= 20)
