import os
import sys
import tempfile
import shutil
import unittest
import numpy as np

# Ensure project root and parent are in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.dirname(PROJECT_ROOT))

from Gen_Taskset.lib.gmm_model import GaussianComponent
from Gen_Taskset.lib.visualizer import plot_3d_execution_time_surface, plot_moving_trajectory

class TestVisualizer(unittest.TestCase):

    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        
        # Construct standard configs
        self.cfgs = {
            "D1_RANGE": [-10, 10],
            "D2_RANGE": [0, 360],
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9]
        }
        
        # Generate mock GMM coeffs
        mean_vector = np.array([5.0, 180.0, 50.0])
        cov_matrix = np.array([
            [4.0, 0.0, 2.0],
            [0.0, 100.0, 0.0],
            [2.0, 0.0, 16.0]
        ])
        self.comp = GaussianComponent(mean_vector, cov_matrix)
        self.coeffs = self.comp.get_coeffs_dict()

        # Task parameters for single Gaussian component
        self.task_param_single = {
            'D1_MAX': 10.0,
            'D1_MIN': -10.0,
            'D2_MAX': 360.0,
            'D2_MIN': 0.0,
            'period': 100.0,
            'Et_mean': 50.0,
            'Et_sigma': 4.0,
            'coeffs': self.coeffs
        }

        # Task parameters for GMM mixture
        self.task_param_mix = {
            'D1_MAX': 10.0,
            'D1_MIN': -10.0,
            'D2_MAX': 360.0,
            'D2_MIN': 0.0,
            'D1_sigma': 2.5,
            'D2_sigma': 90.0,
            'period': 100.0,
            'Et_mean': 50.0,
            'Et_sigma': 10.0,
            'weights': [0.5, 0.5],
            'tasks': [
                {'coeffs': self.coeffs},
                {'coeffs': self.coeffs}
            ]
        }

    def tearDown(self):
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def test_plot_3d_execution_time_surface_single(self):
        output_file = os.path.join(self.temp_dir, "surface_single.png")
        
        # Test visualizer run
        res = plot_3d_execution_time_surface(
            cfgs=self.cfgs,
            task_param=self.task_param_single,
            output_path=output_file,
            draw=False,
            is_mixture=False
        )
        
        # X range: [-10, 10], size 21. Y range: [-10, 10], size 21.
        self.assertEqual(res.shape, (21, 21))
        self.assertTrue(os.path.exists(output_file))
        self.assertTrue(os.path.getsize(output_file) > 0)

    def test_plot_3d_execution_time_surface_mixture(self):
        output_file = os.path.join(self.temp_dir, "surface_mixture.png")
        
        # Test visualizer run
        res = plot_3d_execution_time_surface(
            cfgs=self.cfgs,
            task_param=self.task_param_mix,
            output_path=output_file,
            draw=False,
            is_mixture=True
        )
        
        self.assertEqual(res.shape, (21, 21))
        self.assertTrue(os.path.exists(output_file))
        self.assertTrue(os.path.getsize(output_file) > 0)

    def test_plot_moving_trajectory(self):
        output_file = os.path.join(self.temp_dir, "trajectory.png")
        steps = [(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]
        stops = [(0.0, 0.0), (2.0, 2.0)]
        
        # Test trajectory plotter
        plot_moving_trajectory(
            steps=steps,
            stops=stops,
            cfgs=self.cfgs,
            output_path=output_file,
            draw=False
        )
        
        self.assertTrue(os.path.exists(output_file))
        self.assertTrue(os.path.getsize(output_file) > 0)
