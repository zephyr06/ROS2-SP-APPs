import pytest
import numpy as np
from Gen_Taskset.lib.gmm_model import GaussianComponent, GMMTaskModel, calc_mix_Et_sigma

def test_gaussian_component_precomputations():
    # Set up basic 3D mean and covariance
    mean_vec = np.array([0.0, 0.0, 10.0])
    cov_matrix = np.array([
        [4.0, 0.0, 0.5],
        [0.0, 9.0, 0.2],
        [0.5, 0.2, 1.0]
    ])
    
    comp = GaussianComponent(mean_vec, cov_matrix)
    
    # Conditional mean should equal mu_et at the center coordinate
    sample_mean_center = comp.sample_conditional_execution_time(0.0, 0.0, mean_only=True)
    assert pytest.approx(sample_mean_center) == 10.0
    
    # Coefficients dictionary export
    coeffs = comp.get_coeffs_dict()
    assert coeffs['Et_mean'] == 10.0
    assert coeffs['Et_sigma'] == 1.0

def test_gmm_task_model_sampling():
    comp1 = GaussianComponent(np.array([0.0, 0.0, 10.0]), np.eye(3))
    comp2 = GaussianComponent(np.array([0.0, 0.0, 20.0]), np.eye(3))
    
    task_model = GMMTaskModel(
        components=[comp1, comp2],
        weights=[0.5, 0.5],
        period=100.0,
        d1_min=-10.0, d1_max=10.0, d1_sigma=5.0,
        d2_min=0.0, d2_max=360.0, d2_sigma=90.0,
        et_mean=15.0, et_sigma=5.0
    )
    
    variance_factor_table = [1.0] * 100
    final_et_range = [0.05, 0.9]
    
    # Sample a set of execution times
    samples = [task_model.sample_execution_time(0.0, 0.0, variance_factor_table, final_et_range) for _ in range(50)]
    for s in samples:
        assert 1.0 <= s <= 100.0  # Assert bounded by period and min bounds
        
def test_calc_mix_Et_sigma():
    comp1 = GaussianComponent(np.array([0.0, 0.0, 10.0]), np.eye(3))
    comp2 = GaussianComponent(np.array([0.0, 0.0, 20.0]), np.eye(3))
    
    mix_sigma = calc_mix_Et_sigma(15.0, [0.5, 0.5], [comp1, comp2])
    # mix_sigma^2 = (1.0^2 * 0.5 + (10 - 15)^2 * 0.5) + (1.0^2 * 0.5 + (20 - 15)^2 * 0.5)
    #             = (0.5 + 25 * 0.5) + (0.5 + 25 * 0.5)
    #             = 13 + 13 = 26
    # mix_sigma   = sqrt(26) = 5.099
    assert pytest.approx(mix_sigma, 0.01) == 5.099
