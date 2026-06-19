import unittest
import os
import sys
import numpy as np

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.gmm_model import GaussianComponent, GMMTaskModel, calc_mix_Et_sigma

class TestGMMModel(unittest.TestCase):

    def setUp(self):
        # 3D mean: [D1_mean, D2_mean, Et_mean]
        self.mean_vector = np.array([5.0, 180.0, 50.0])
        # Covariance matrix for (D1, D2, Et)
        self.cov_matrix = np.array([
            [4.0, 0.0, 2.0],
            [0.0, 100.0, 0.0],
            [2.0, 0.0, 16.0]
        ])

    def test_gaussian_component_coeffs(self):
        comp = GaussianComponent(self.mean_vector, self.cov_matrix)
        coeffs = comp.get_coeffs_dict()
        
        self.assertEqual(coeffs['mu2'], 50.0)
        self.assertEqual(coeffs['mu1'], [5.0, 180.0])
        self.assertAlmostEqual(coeffs['Et_mean'], 50.0)
        self.assertAlmostEqual(coeffs['Et_sigma'], 4.0)  # sqrt(16)
        
        # Verify conditional mean formula
        # p(Et | D1=7.0, D2=180.0) -> cond_mean = mu_et + sigma_21 @ inv_sigma_11 @ (x - mu_xy)
        # sigma_21 = [2, 0], inv_sigma_11 = [[1/4, 0], [0, 1/100]]
        # sigma_21 @ inv_sigma_11 = [1/2, 0]
        # For x = [7, 180], x - mu_xy = [2, 0]
        # cond_mean = 50 + 0.5 * 2 = 51.0
        sampled_mean = comp.sample_conditional_execution_time(7.0, 180.0, mean_only=True)
        self.assertAlmostEqual(sampled_mean, 51.0)

    def test_calc_mix_Et_sigma(self):
        comp1 = GaussianComponent(np.array([0, 0, 40.0]), self.cov_matrix)
        comp2 = GaussianComponent(np.array([0, 0, 60.0]), self.cov_matrix)
        
        weights = [0.5, 0.5]
        mix_mean = 50.0  # (40 + 60) / 2
        # mix_variance = sum(w * (c_variance + (c_mean - mix_mean)^2))
        # c_variance = 16.0 for both.
        # mix_variance = 0.5 * (16 + 10^2) + 0.5 * (16 + 10^2) = 116.0
        # mix_sigma = sqrt(116) = 10.7703296
        mix_sigma = calc_mix_Et_sigma(mix_mean, weights, [comp1, comp2])
        self.assertAlmostEqual(mix_sigma, np.sqrt(116.0))

    def test_gmm_task_model_sampling(self):
        comp1 = GaussianComponent(np.array([5, 180, 40.0]), self.cov_matrix)
        comp2 = GaussianComponent(np.array([5, 180, 60.0]), self.cov_matrix)
        
        weights = [0.5, 0.5]
        gmm_model = GMMTaskModel(
            components=[comp1, comp2],
            weights=weights,
            period=100.0,
            d1_min=0.0,
            d1_max=10.0,
            d1_sigma=2.5,
            d2_min=0.0,
            d2_max=360.0,
            d2_sigma=90.0,
            et_mean=50.0,
            et_sigma=10.0
        )
        
        variance_factor_table = [1.0] * 20
        final_et_range = [0.05, 0.9]
        
        # Sample 100 times and check bounds
        for _ in range(100):
            sample = gmm_model.sample_execution_time(
                d1=5.0,
                d2=180.0,
                variance_factor_table=variance_factor_table,
                final_et_range=final_et_range
            )
            self.assertTrue(5.0 <= sample <= 90.0) # period * 0.05 <= sample <= period * 0.9

if __name__ == "__main__":
    unittest.main()
