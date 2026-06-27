import numpy as np

class GaussianComponent:
    """Represents a single 3D Normal distribution component over (X, Y, Et)."""
    
    def __init__(self, mean_vector: np.ndarray, cov_matrix: np.ndarray, coeffs: dict = None):
        self.mean_vector = mean_vector
        self.cov_matrix = cov_matrix
        
        # Parse or precompute coefficients
        if coeffs is not None:
            self.mu_et = coeffs['mu2']
            self.mu_xy = coeffs['mu1']
            self.sigma_21_inv_sigma_11 = coeffs['Sigma21_mul_inv_Sigma11']
            self.sqrt_cond_var = coeffs['sqrt_cond_var']
            self.et_mean = coeffs['Et_mean']
            self.et_sigma = coeffs['Et_sigma']
        else:
            self.mu_et = mean_vector[2]
            self.mu_xy = mean_vector[:2]
            
            sigma_11 = cov_matrix[:2, :2]
            sigma_22 = cov_matrix[2, 2]
            sigma_12 = cov_matrix[:2, 2]
            sigma_21 = cov_matrix[2, :2]
            
            self.inv_sigma_11 = np.linalg.inv(sigma_11)
            self.sigma_21_inv_sigma_11 = sigma_21 @ self.inv_sigma_11
            self.sqrt_cond_var = np.sqrt(max(0.0, sigma_22 - self.sigma_21_inv_sigma_11 @ sigma_12))
            self.et_mean = self.mu_et
            self.et_sigma = np.sqrt(sigma_22)

    def sample_conditional_execution_time(self, d1: float, d2: float, mean_only: bool = False) -> float:
        """Samples execution time from conditional 1D normal distribution p(Et | D1=d1, D2=d2)."""
        x_observed = np.array([d1, d2])
        cond_mean = self.mu_et + self.sigma_21_inv_sigma_11 @ (x_observed - self.mu_xy)
        
        if mean_only:
            return float(cond_mean)
        return float(np.random.normal(cond_mean, self.sqrt_cond_var))

    def get_coeffs_dict(self) -> dict:
        """Converts the internal coefficients to dictionary format (for compatibility)."""
        return {
            'mu2': float(self.mu_et),
            'mu1': self.mu_xy.tolist() if isinstance(self.mu_xy, np.ndarray) else list(self.mu_xy),
            'Sigma21_mul_inv_Sigma11': self.sigma_21_inv_sigma_11.tolist() if isinstance(self.sigma_21_inv_sigma_11, np.ndarray) else list(self.sigma_21_inv_sigma_11),
            'sqrt_cond_var': float(self.sqrt_cond_var),
            'Et_mean': float(self.et_mean),
            'Et_sigma': float(self.et_sigma),
            'ro_1_Et': float(getattr(self, 'ro_1_Et', 0.0)),
            'ro_2_Et': float(getattr(self, 'ro_2_Et', 0.0)),
        }


class GMMTaskModel:
    """Represents a Mixture Gaussian Task whose execution time follows a 3D GMM model."""
    
    def __init__(self, components: list[GaussianComponent], weights: list[float], period: float, 
                 d1_min: float, d1_max: float, d1_sigma: float, 
                 d2_min: float, d2_max: float, d2_sigma: float,
                 et_mean: float, et_sigma: float):
        self.components = components
        self.weights = weights
        self.period = period
        self.deadline = period
        
        self.d1_min = d1_min
        self.d1_max = d1_max
        self.d1_sigma = d1_sigma
        
        self.d2_min = d2_min
        self.d2_max = d2_max
        self.d2_sigma = d2_sigma
        
        self.et_mean = et_mean
        self.et_sigma = et_sigma
        
        # Scheduling parameters (will be overwritten/normalized later)
        self.sp_threshold = 0.5
        self.sp_weight = 1.0
        
        # Performance record attributes
        self.performance_records_time = ""
        self.performance_records_perf = ""

    def sample_execution_time(self, d1: float, d2: float, 
                              variance_factor_table: list,
                              final_et_range: list,
                              et_min_2sigma: bool = True, 
                              mean_only: bool = False) -> float:
        """Selects a GMM component based on weights and samples execution time, enforcing bounds."""
        # 1. Choose component
        comp_idx = np.random.choice(len(self.components), p=self.weights)
        comp = self.components[comp_idx]
        
        sample = comp.sample_conditional_execution_time(d1, d2, mean_only=mean_only)
        
        # 2. Scale by spatial variance factor
        r_i = int(d1)
        if r_i >= len(variance_factor_table):
            r_i = len(variance_factor_table) - 1
        elif r_i < 0:
            r_i = 0
        
        sample *= variance_factor_table[r_i]
        
        # 3. Apply min_Et restriction
        if et_min_2sigma:
            min_Et = comp.et_mean - 2 * comp.et_sigma
            if sample < min_Et:
                sample = min_Et
        
        # 4. Limit to period/deadline bounds if period is specified
        if self.period is not None:
            et_min = self.period * final_et_range[0]
            et_max = self.period * final_et_range[1]
            
            if sample > self.period:
                sample = self.period
            if sample < et_min:
                sample = et_min
            if sample > et_max:
                sample = et_max
        
        if sample < 1.0:
            sample = 1.0
            
        return float(sample)


def calc_mix_Et_sigma(Et_mean: float, weights: list, components_params: list) -> float:
    """Computes the standard deviation of a GMM mixture distribution."""
    Et_sigma = 0.0
    n_weights = len(weights)
    for i in range(n_weights):
        # components_params can be either dicts or GaussianComponent instances
        c_sigma = components_params[i].et_sigma if hasattr(components_params[i], 'et_sigma') else components_params[i]['Et_sigma']
        c_mean = components_params[i].et_mean if hasattr(components_params[i], 'et_mean') else components_params[i]['Et_mean']
        
        Et_sigma += (c_sigma ** 2) * weights[i]
        Et_sigma += ((c_mean - Et_mean) ** 2) * weights[i]
        
    if Et_sigma < 0:
        Et_sigma = 0.0
    else:
        Et_sigma = np.sqrt(Et_sigma)
    return float(Et_sigma)
