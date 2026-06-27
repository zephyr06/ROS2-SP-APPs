import numpy as np
import random
import yaml
from .gmm_model import GaussianComponent, GMMTaskModel, calc_mix_Et_sigma
from .generation_config_parser import standardize_config

# Keep common parameters that are shared across gaussian tasks in a GMM task
SHARED_TASK_PARAMS = ["period", "D1_MIN", "D1_MAX", "D1_sigma", "D2_MIN", "D2_MAX", "D2_sigma"]

def generate_single_gaussian_task(
    cfgs: dict,
    period: float = None,
    et_mean: float = None,
    et_sigma: float = None,
    ro_1_Et: float = None,
    ro_2_Et: float = None,
    prd_sel: str = 'big',
    picked_periods: list = None
) -> dict:
    """Generates components and coefficients for a single Gaussian task."""
    rt = {}
    
    # 1. Determine period (ms)
    if period is not None:
        rt["period"] = period
    else:
        # Standardize: check for periods list in ms, fall back to converting HZ to ms
        periods_key = "BIG_PERIODS_MS" if prd_sel == 'big' else "SMALL_PERIODS_MS"
        periods_list = cfgs.get(periods_key)
        
        if picked_periods is None:
            picked_periods = []
            
        selected_period = None
        for _ in range(10):
            selected_period = int(np.random.choice(periods_list))
            if selected_period not in picked_periods:
                picked_periods.append(selected_period)
                break
        if selected_period is None:
            selected_period = int(np.random.choice(periods_list))
        rt["period"] = selected_period

    # 2. Determine Et_mean
    if et_mean is not None:
        rt["Et_mean"] = et_mean
    else:
        et_over_period = np.random.uniform(cfgs["Et_OVER_PERIOD_RANGE"][0], cfgs["Et_OVER_PERIOD_RANGE"][1])
        rt["Et_mean"] = max(1.0, rt["period"] * et_over_period)

    # 3. Determine Et_sigma
    if et_sigma is not None:
        rt["Et_sigma"] = et_sigma
    else:
        sigma_over_et = np.random.uniform(cfgs["SIGMA_OVER_Et_RANGE"][0], cfgs["SIGMA_OVER_Et_RANGE"][1])
        rt["Et_sigma"] = max(0.001, rt["Et_mean"] * sigma_over_et)

    # 4. Grid space limits
    rt['D1_MIN'] = cfgs["D1_RANGE"][0]
    rt['D1_MAX'] = cfgs["D1_RANGE"][1]
    D1_RANGE = rt['D1_MAX'] - rt['D1_MIN']
    rt["D1_sigma"] = D1_RANGE / 4.0

    rt['D2_MIN'] = cfgs["D2_RANGE"][0]
    rt['D2_MAX'] = cfgs["D2_RANGE"][1]
    D2_RANGE = rt['D2_MAX'] - rt['D2_MIN']
    rt["D2_sigma"] = D2_RANGE / 4.0

    # 5. Correlations
    if ro_1_Et is not None:
        rt["ro_1_Et"] = ro_1_Et
    else:
        ro_1_et = np.random.uniform(cfgs["RO_1_Et_RANGE"][0], cfgs["RO_1_Et_RANGE"][1])
        rt["ro_1_Et"] = np.clip(ro_1_et, -1.0, 1.0)

    if ro_2_Et is not None:
        rt["ro_2_Et"] = ro_2_Et
    else:
        ro_2_et = np.random.uniform(cfgs["RO_2_Et_RANGE"][0], cfgs["RO_2_Et_RANGE"][1])
        rt["ro_2_Et"] = np.clip(ro_2_et, -1.0, 1.0)

    # 6. Covariance matrix and mean vector
    STD_R = rt["D1_sigma"]
    STD_THETA = rt["D2_sigma"]
    STD_Et = rt["Et_sigma"]
    ro_R_Et = rt["ro_1_Et"]
    ro_THETA_Et = rt["ro_2_Et"]
    Et_mean_val = rt["Et_mean"]

    cov_matrix = np.array([
        [STD_R**2,                 0,                                ro_R_Et * STD_R * STD_Et        ],
        [0,                        STD_THETA**2,                     ro_THETA_Et * STD_THETA * STD_Et],
        [ro_R_Et * STD_R * STD_Et, ro_THETA_Et * STD_THETA * STD_Et, STD_Et**2                       ]
    ])
    
    D1_mean = (rt['D1_MAX'] + rt['D1_MIN']) / 2.0
    D2_mean = (rt['D2_MAX'] + rt['D2_MIN']) / 2.0
    mean_vec = np.array([D1_mean, D2_mean, Et_mean_val])    

    # 7. Precompute coefficients and build GaussianComponent
    component = GaussianComponent(mean_vec, cov_matrix)
    component.ro_1_Et = rt["ro_1_Et"]
    component.ro_2_Et = rt["ro_2_Et"]
    rt['component'] = component
    rt['coeffs'] = component.get_coeffs_dict()

    return rt

def generate_mix_gaussian_task(cfgs: dict, prd_sel: str = 'big', picked_periods: list = None) -> GMMTaskModel:
    """Generates GMMTaskModel containing multiple components and weights."""
    gaussian_task_params = []
    weights = []    
    n_weights = cfgs.get("N_GMM_COMPONENTS_PER_TASK", 4)

    total = 0.0
    period = None
    for i in range(n_weights):
        task_param = generate_single_gaussian_task(cfgs, period=period, prd_sel=prd_sel, picked_periods=picked_periods)
        period = task_param['period']
        gaussian_task_params.append(task_param)
        
        value = random.random()
        if value < 0.1:
            value = 0.1
        total += value
        weights.append(value)

    for i in range(n_weights):
        weights[i] /= total

    # Compute GMM mix mean and sigma
    Et_mean = sum(gaussian_task_params[i]['Et_mean'] * weights[i] for i in range(n_weights))
    Et_sigma = calc_mix_Et_sigma(Et_mean, weights, gaussian_task_params)

    # Instantiate the GMMTaskModel
    g_params = gaussian_task_params[0]
    components = [item['component'] for item in gaussian_task_params]

    task_model = GMMTaskModel(
        components=components,
        weights=weights,
        period=period,
        d1_min=g_params['D1_MIN'],
        d1_max=g_params['D1_MAX'],
        d1_sigma=g_params['D1_sigma'],
        d2_min=g_params['D2_MIN'],
        d2_max=g_params['D2_MAX'],
        d2_sigma=g_params['D2_sigma'],
        et_mean=Et_mean,
        et_sigma=Et_sigma
    )
    return task_model

def generate_taskset_parameters(cfgs: dict, dump_dir: str = None, save_plots: bool = False, n_sec: int = None) -> dict:
    """Orchestrates generation of all GMMTaskModels scaled to target MEAN_CPU_UTIL."""
    cfgs = standardize_config(cfgs)
    
    # Seeding for reproducibility
    seed = cfgs.get("RANDOM_SEED")
    if seed is not None:
        np.random.seed(seed)
        random.seed(seed)

    n_cores = cfgs["N_CORES"]
    cpu_util = cfgs['MEAN_CPU_UTIL'] * n_cores
    total_util = 0.0
    taskset_param = []
    picked_periods = []

    g_n_big_periods = cfgs.get("N_BIG_PERIOD_TASKS", 2)
    g_n_small_periods = cfgs.get("N_SMALL_PERIOD_TASKS", 8)

    # 1. Generate task GMM parameters
    for _ in range(g_n_big_periods):
        task_param = generate_mix_gaussian_task(cfgs, prd_sel='big', picked_periods=picked_periods)
        taskset_param.append(task_param)
        total_util += task_param.et_mean / task_param.period
    
    for _ in range(g_n_small_periods):
        task_param = generate_mix_gaussian_task(cfgs, prd_sel='small', picked_periods=picked_periods)
        taskset_param.append(task_param)
        total_util += task_param.et_mean / task_param.period
        
    # 2. Scale execution times to target MEAN_CPU_UTIL
    util_scale_factor = cpu_util / total_util
    n_tasks = len(taskset_param)
    
    for i in range(n_tasks):
        n_weights = len(taskset_param[i].components)
        period = taskset_param[i].period
        
        # We need to scale components of the mixture
        Et_mean_new = 0.0
        new_components = []
        for k in range(n_weights):
            old_comp = taskset_param[i].components[k]
            scaled_mean = old_comp.et_mean * util_scale_factor
            scaled_sigma = old_comp.et_sigma * util_scale_factor

            # Preserve correlations from the original component
            old_ro_1 = getattr(old_comp, 'ro_1_Et', None)
            old_ro_2 = getattr(old_comp, 'ro_2_Et', None)

            scaled_component_dict = generate_single_gaussian_task(
                cfgs,
                period=period,
                et_mean=scaled_mean,
                et_sigma=scaled_sigma,
                ro_1_Et=old_ro_1,
                ro_2_Et=old_ro_2,
            )
            new_components.append(scaled_component_dict['component'])
            Et_mean_new += scaled_component_dict['Et_mean'] * taskset_param[i].weights[k]
        
        taskset_param[i].components = new_components
        taskset_param[i].et_mean = Et_mean_new
        taskset_param[i].et_sigma = calc_mix_Et_sigma(Et_mean_new, taskset_param[i].weights, new_components)

    # 3. Generate SP constraints
    trd_min = cfgs.get('SP_THRESHOLD_RANGE', [0.5, 0.9])[0]
    trd_max = cfgs.get('SP_THRESHOLD_RANGE', [0.5, 0.9])[1]
    
    # Check if we should select from discrete SP thresholds set
    sp_thresholds_set = cfgs.get("SP_THRESHOLDS_SET", [0.2, 0.4, 0.6, 0.8, 1.0])
    
    for i in range(n_tasks):
        taskset_param[i].sp_weight = 1.0
        if sp_thresholds_set:
            # Random selection from discrete set (to match paper)
            taskset_param[i].sp_threshold = float(np.random.choice(sp_thresholds_set))
        else:
            # Continuous uniform distribution
            taskset_param[i].sp_threshold = random.uniform(trd_min, trd_max)

    # 4. Core Allocation (Processor ID assignment)
    # n_cores is already defined at the beginning of the function
    
    # Sort tasks by utilization (et_mean / period) descending for First-Fit bin packing
    indexed_tasks = [(i, taskset_param[i]) for i in range(n_tasks)]
    indexed_tasks.sort(key=lambda item: item[1].et_mean / item[1].period, reverse=True)
    
    core_utilizations = [0.0] * n_cores
    for original_idx, t in indexed_tasks:
        min_core_idx = int(np.argmin(core_utilizations))
        t.processorId = min_core_idx
        core_utilizations[min_core_idx] += (t.et_mean / t.period)

    # Convert GMMTaskModel objects to dictionary structure for serialization
    tasks_dict_list = []
    for i in range(n_tasks):
        t = taskset_param[i]
        
        # Serialize components parameters
        serialized_components = []
        for c in t.components:
            serialized_components.append({
                'Et_mean': float(c.et_mean),
                'Et_sigma': float(c.et_sigma),
                'ro_1_Et': float(getattr(c, 'ro_1_Et', 0.0)),
                'ro_2_Et': float(getattr(c, 'ro_2_Et', 0.0)),
                'coeffs': c.get_coeffs_dict()
            })
        
        tasks_dict_list.append({
            'weights': t.weights,
            'n_weights': len(t.components),
            'period': int(t.period),
            'D1_MIN': float(t.d1_min),
            'D1_MAX': float(t.d1_max),
            'D1_sigma': float(t.d1_sigma),
            'D2_MIN': float(t.d2_min),
            'D2_MAX': float(t.d2_max),
            'D2_sigma': float(t.d2_sigma),
            'Et_mean': float(t.et_mean),
            'Et_sigma': float(t.et_sigma),
            'sp_weight': float(t.sp_weight),
            'sp_threshold': float(t.sp_threshold),
            'processorId': int(getattr(t, 'processorId', 0)),
            'tasks': serialized_components
        })
        
    rt = {
        'n_tasks': n_tasks, 
        'cpu_util': cpu_util, 
        'tasks': tasks_dict_list
    }
    return rt

def load_and_fill_taskset_param_file(taskset_param_fpath: str) -> dict:
    """Loads a taskset parameters YAML file and reinstantiates GMMTaskModels and coefficients."""
    with open(taskset_param_fpath, "r") as f:
        taskset_data = yaml.safe_load(f)
        
    n_tasks = taskset_data['n_tasks']
    for i in range(n_tasks):
        task_dict = taskset_data['tasks'][i]
        n_weights = task_dict['n_weights']
        
        # Precompute and populate coefficients dictionary for each component
        for k in range(n_weights):
            comp_dict = task_dict['tasks'][k]
            
            STD_R = task_dict["D1_sigma"]
            STD_THETA = task_dict["D2_sigma"]
            STD_Et = comp_dict["Et_sigma"]
            
            # Extract correlations if present, otherwise default to 0.0
            ro_R_Et = comp_dict.get("ro_1_Et", 0.0)
            ro_THETA_Et = comp_dict.get("ro_2_Et", 0.0)
            Et_mean = comp_dict["Et_mean"]
            
            cov_matrix = np.array([
                [STD_R**2,                 0,                                ro_R_Et * STD_R * STD_Et        ],
                [0,                        STD_THETA**2,                     ro_THETA_Et * STD_THETA * STD_Et],
                [ro_R_Et * STD_R * STD_Et, ro_THETA_Et * STD_THETA * STD_Et, STD_Et**2                       ]
            ])
            
            D1_mean = (task_dict['D1_MAX'] + task_dict['D1_MIN']) / 2.0
            D2_mean = (task_dict['D2_MAX'] + task_dict['D2_MIN']) / 2.0
            mean_vec = np.array([D1_mean, D2_mean, Et_mean])
            
            component = GaussianComponent(mean_vec, cov_matrix)
            comp_dict['coeffs'] = component.get_coeffs_dict()
            
    return taskset_data
