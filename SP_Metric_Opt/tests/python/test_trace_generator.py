import unittest
import os
import tempfile
import sys
import numpy as np

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.trace_generator import generate_execution_time_trace
from Gen_Taskset.lib.gmm_model import GaussianComponent

class TestTraceGenerator(unittest.TestCase):

    def setUp(self):
        # Setup configs
        self.cfgs = {
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "D1_VARIANCE_FACTOR_TABLE": [1.0] * 50
        }
        
        # Setup mock task parameter
        mean_vector = np.array([5.0, 180.0, 50.0])
        cov_matrix = np.array([
            [4.0, 0.0, 2.0],
            [0.0, 100.0, 0.0],
            [2.0, 0.0, 16.0]
        ])
        comp = GaussianComponent(mean_vector, cov_matrix)
        
        self.task_param = {
            'weights': [1.0],
            'D1_MIN': 0.0,
            'D1_MAX': 100.0,
            'D1_sigma': 25.0,
            'D2_MIN': 0.0,
            'D2_MAX': 360.0,
            'D2_sigma': 90.0,
            'Et_mean': 50.0,
            'Et_sigma': 4.0,
            'tasks': [
                {
                    'coeffs': comp.get_coeffs_dict()
                }
            ]
        }

    def test_generate_execution_time_trace(self):
        path_xys = [(10, 10), (20, 20), (30, 30)]
        period = 100
        ms_per_move = 1000
        n_steps = 10
        
        fd, temp_path = tempfile.mkstemp()
        try:
            os.close(fd) # Close immediately, we will write to it in trace generator
            steps, bounds = generate_execution_time_trace(
                path_xys=path_xys,
                period=period,
                ms_per_move=ms_per_move,
                n_steps=n_steps,
                task_param=self.task_param,
                cfgs=self.cfgs,
                dump_path=temp_path
            )
            
            self.assertEqual(len(steps), 10)
            self.assertTrue(bounds[0] >= 5.0) # 100 * 0.05
            self.assertTrue(bounds[1] <= 90.0) # 100 * 0.9
            
            # Verify file contents
            self.assertTrue(os.path.exists(temp_path))
            with open(temp_path, 'r') as f:
                lines = f.readlines()
            self.assertEqual(len(lines), 10)
            for line in lines:
                parts = line.strip().split(',')
                self.assertEqual(len(parts), 3)
                # verify coordinates correspond to path_xys
                x = int(parts[0])
                y = int(parts[1])
                self.assertTrue(x in [10, 20, 30])
                self.assertTrue(y in [10, 20, 30])
                
        finally:
            if os.path.exists(temp_path):
                os.remove(temp_path)

if __name__ == "__main__":
    unittest.main()
