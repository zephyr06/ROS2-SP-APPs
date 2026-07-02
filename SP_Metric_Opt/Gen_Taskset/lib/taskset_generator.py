import numpy as np
import random
import yaml
from .gmm_model import GaussianComponent, GMMTaskModel, calc_mix_Et_sigma
from .generation_config_parser import standardize_config

# Keep common parameters that are shared across gaussian tasks in a GMM task
SHARED_TASK_PARAMS = ["period", "D1_MIN", "D1_MAX", "D1_sigma", "D2_MIN", "D2_MAX", "D2_sigma"]

def uunifast_distribution(n: int, target_util: float, max_util_cap: float = 0.95) -> list[float]:
    """Classic UUniFast algorithm for generating n utilization values that sum to target_util.

    Each individual utilization is capped below max_util_cap (default 0.95 for single-core
    feasibility).  Retries the whole vector up to 100 times; if still failing, falls back
    to capping and re-normalizing.

    Reference: Bini, Enrico, and Giorgio C. Buttazzo.
    "Measuring the performance of schedulability tests."
    Real-Time Systems 30.1-2 (2005): 129-154.

    Args:
        n: Number of tasks.
        target_util: Total utilization to distribute.
        max_util_cap: Maximum per-task utilization (exclusive).

    Returns:
        List of n utilization values summing to target_util, each < max_util_cap.
    """
    # Ensure a feasible cap: if the target is higher than what the default
    # cap allows, raise it just enough to be mathematically possible.
    required_min_cap = target_util / n
    if required_min_cap >= max_util_cap:
        max_util_cap = min(0.9999, required_min_cap + 0.001)

    for _ in range(100):
        sum_u = target_util
        vect_u = [0.0] * n
        for i in range(n - 1):
            next_sum_u = sum_u * (random.random() ** (1.0 / (n - i)))
            vect_u[i] = sum_u - next_sum_u
            sum_u = next_sum_u
        vect_u[n - 1] = sum_u

        # Numerical safety clip
        for i in range(n):
            vect_u[i] = max(0.0, min(target_util, vect_u[i]))

        if all(u < max_util_cap for u in vect_u):
            return vect_u

    # Fallback: iterative cap-and-redistribute to preserve exact total
    # without re-inflating capped values above the limit.
    for _ in range(1000):
        if all(u < max_util_cap for u in vect_u):
            break
        excess = 0.0
        free_indices = []
        for i in range(n):
            if vect_u[i] >= max_util_cap:
                excess += vect_u[i] - (max_util_cap * 0.9999)
                vect_u[i] = max_util_cap * 0.9999
            else:
                free_indices.append(i)
        if not free_indices:
            vect_u = [target_util / n] * n
            break
        free_sum = sum(vect_u[i] for i in free_indices)
        if free_sum > 0:
            for i in free_indices:
                vect_u[i] += excess * (vect_u[i] / free_sum)
        else:
            share = excess / len(free_indices)
            for i in free_indices:
                vect_u[i] += share
    return vect_u

def pick_period(cfgs: dict, prd_sel: str, picked_periods: list) -> int:
    """Select a random period from the configured list, avoiding recent duplicates."""
    periods_key = "BIG_PERIODS_MS" if prd_sel == 'big' else "SMALL_PERIODS_MS"
    periods_list = cfgs.get(periods_key)

    selected_period = None
    for _ in range(10):
        selected_period = int(np.random.choice(periods_list))
        if selected_period not in picked_periods:
            picked_periods.append(selected_period)
            break
    if selected_period is None:
        selected_period = int(np.random.choice(periods_list))
    return selected_period

def generate_single_gaussian_task(
    cfgs: dict,
    period: float,
    et_mean: float = None,
    et_sigma: float = None,
    ro_1_Et: float = None,
    ro_2_Et: float = None,
) -> dict:
    """Generates components and coefficients for a single Gaussian task."""
    rt = {"period": period}

    # 1. Determine Et_mean
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

def generate_mix_gaussian_task(
    cfgs: dict,
    period: float,
    et_mean: float = None,
    et_sigma: float = None,
    ro_1_Et: float = None,
    ro_2_Et: float = None,
) -> GMMTaskModel:
    """Generates GMMTaskModel containing multiple components and weights.

    When et_mean is provided (e.g. from UUniFast), all components share that Et_mean.
    When et_sigma is provided, all components share that Et_sigma.
    When ro_1_Et/ro_2_Et are provided, they override per-component random correlations.
    All components share the provided period.
    """
    gaussian_task_params = []
    weights = []
    n_weights = cfgs.get("N_GMM_COMPONENTS_PER_TASK", 4)

    total = 0.0
    for _ in range(n_weights):
        task_param = generate_single_gaussian_task(
            cfgs,
            period=period,
            et_mean=et_mean,
            et_sigma=et_sigma,
            ro_1_Et=ro_1_Et,
            ro_2_Et=ro_2_Et,
        )
        gaussian_task_params.append(task_param)

        value = random.random()
        if value < 0.1:
            value = 0.1
        total += value
        weights.append(value)

    for i in range(n_weights):
        weights[i] /= total

    # Compute GMM mix mean and sigma
    mix_Et_mean = sum(gaussian_task_params[i]['Et_mean'] * weights[i] for i in range(n_weights))
    mix_Et_sigma = calc_mix_Et_sigma(mix_Et_mean, weights, gaussian_task_params)

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
        et_mean=mix_Et_mean,
        et_sigma=mix_Et_sigma
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
    taskset_param = []
    picked_periods = []

    # P17: N_BIG_PERIOD_TASKS and N_SMALL_PERIOD_TASKS may each be 0 (single-rate
    # tasksets: all-big or all-small). The period-pick loops below are no-ops for
    # a 0 count, and standardize_config() guarantees n_tasks >= 1. The defaults
    # (2 / 8) are kept for backward compatibility with configs that omit them.
    g_n_big_periods = cfgs.get("N_BIG_PERIOD_TASKS", 2)
    g_n_small_periods = cfgs.get("N_SMALL_PERIOD_TASKS", 8)
    n_tasks = g_n_big_periods + g_n_small_periods

    # Determine number of env-dependent tasks.
    # If N_ENV_DEPENDENT_TASKS is specified in config, use it.
    # Otherwise default to ALL tasks being env-dependent for maximum
    # per-interval utilization variance (100-200% per-core swings).
    n_env_dependent_cfg = cfgs.get("N_ENV_DEPENDENT_TASKS", None)
    if n_env_dependent_cfg is None:
        n_env_dependent = random.randint(1, n_tasks)
    else:
        n_env_dependent = min(int(n_env_dependent_cfg), n_tasks)

    # Small sigma base for perf tasks so ET is effectively deterministic
    FIXED_TASK_SIGMA_RATIO = cfgs.get("FIXED_TASK_SIGMA_RATIO", 0.001)

    # 1. Generate periods
    periods = []
    for _ in range(g_n_big_periods):
        periods.append(pick_period(cfgs, prd_sel='big', picked_periods=picked_periods))
    for _ in range(g_n_small_periods):
        periods.append(pick_period(cfgs, prd_sel='small', picked_periods=picked_periods))

    # ------------------------------------------------------------------
    # UUniFast mode: generate exact utilization vector, then derive Et_mean
    # Legacy random-Et-then-scale mode has been removed.
    # ------------------------------------------------------------------
    util_vector = uunifast_distribution(n_tasks, cpu_util, max_util_cap=cfgs["MAX_UTIL_PER_TASK"])

    # Pick env-dependent tasks weighted by utilization from tasks whose
    # period is >= MIN_PERIOD_ENV_DEPENDENT (avoids short-period blow-ups).
    min_period_env = cfgs.get("MIN_PERIOD_ENV_DEPENDENT", 0)
    env_candidates = [i for i in range(n_tasks) if periods[i] >= min_period_env]

    if n_env_dependent == n_tasks or len(env_candidates) <= n_env_dependent:
        env_task_indices = set(env_candidates)
    else:
        candidate_utils = np.array([util_vector[i] for i in env_candidates])
        candidate_probs = candidate_utils / candidate_utils.sum()
        env_task_indices = set(
            np.random.choice(
                len(env_candidates),
                size=n_env_dependent,
                replace=False,
                p=candidate_probs
            )
        )
        env_task_indices = {env_candidates[i] for i in env_task_indices}

    # 2. Optionally tighten utilization cap ONLY for env-dependent tasks.
    #    This leaves headroom so spatial variation from strong negative
    #    correlations doesn't push observed ET above period at map edges.
    max_util_env = cfgs.get("MAX_UTIL_PER_ENV_TASK")
    if max_util_env is not None:
        freed = 0.0
        non_env_indices = []
        for i in range(n_tasks):
            if i in env_task_indices:
                if util_vector[i] > max_util_env:
                    freed += util_vector[i] - max_util_env
                    util_vector[i] = max_util_env
            else:
                non_env_indices.append(i)
        # Redistribute freed utilization to non-env tasks proportionally
        if freed > 0 and non_env_indices:
            non_env_sum = sum(util_vector[i] for i in non_env_indices)
            if non_env_sum > 0:
                for i in non_env_indices:
                    util_vector[i] += freed * (util_vector[i] / non_env_sum)
            else:
                for i in non_env_indices:
                    util_vector[i] += freed / len(non_env_indices)

    # 3. Select time-limit (performance-record) tasks from non-env candidates.
    # All non-env tasks are eligible regardless of period (the former
    # MIN_PERIOD_WITH_PERFORMANCE_RECORDS period floor was removed in P16);
    # the legacy key is still accepted by the config loader but is now a no-op.
    perf_prob = cfgs.get("PERF_RECORD_TASK_PROBABILITY", 0.5)
    perf_candidates = []
    for i in range(n_tasks):
        if i in env_task_indices:
            continue  # perf tasks must be disjoint from env tasks
        perf_candidates.append(i)

    time_limit_task_indices = set()
    for i in perf_candidates:
        if random.random() < perf_prob:
            time_limit_task_indices.add(i)

    # 3. Build task models with appropriate sigma / correlations per type
    task_idx = 0
    for i in range(n_tasks):
        period = periods[task_idx]
        u_i = util_vector[task_idx]
        et_mean = max(1.0, u_i * period)
        is_env = task_idx in env_task_indices
        is_perf = task_idx in time_limit_task_indices

        if is_env:
            # Env tasks: natural large sigma for spatial variation, env correlations
            et_sigma_for_components = None  # random from SIGMA_OVER_Et_RANGE per component
            ro_1 = np.random.uniform(cfgs["RO_1_Et_RANGE"][0], cfgs["RO_1_Et_RANGE"][1])
            ro_2 = np.random.uniform(cfgs["RO_2_Et_RANGE"][0], cfgs["RO_2_Et_RANGE"][1])
        elif is_perf:
            # Perf tasks: tiny sigma so ET is effectively deterministic (no spatial variation)
            et_sigma_for_components = max(0.001, et_mean * FIXED_TASK_SIGMA_RATIO)
            ro_1 = 0.0
            ro_2 = 0.0
        else:
            # Normal tasks: random sigma following Gaussian distribution, no correlations
            et_sigma_for_components = None  # random from SIGMA_OVER_Et_RANGE per component
            ro_1 = 0.0
            ro_2 = 0.0

        task_model = generate_mix_gaussian_task(
            cfgs,
            period=period,
            et_mean=et_mean,
            et_sigma=et_sigma_for_components,
            ro_1_Et=ro_1,
            ro_2_Et=ro_2,
        )
        # Override to exact UUniFast target (numerical safety after GMM mixing)
        task_model.et_mean = et_mean
        task_model.env_dependent = is_env
        task_model.time_limit_task = is_perf
        taskset_param.append(task_model)
        task_idx += 1

    # 3. Generate static task properties (deadline, SP constraints)
    for i in range(n_tasks):
        taskset_param[i].deadline = int(round(taskset_param[i].period * random.uniform(0.5, 1.0)))

    trd_min = cfgs.get('SP_THRESHOLD_RANGE', [0.5, 0.9])[0]
    trd_max = cfgs.get('SP_THRESHOLD_RANGE', [0.5, 0.9])[1]
    sp_thresholds_set = cfgs.get("SP_THRESHOLDS_SET", [0.2, 0.4, 0.6, 0.8, 1.0])

    for i in range(n_tasks):
        taskset_param[i].sp_weight = 1.0
        if sp_thresholds_set:
            taskset_param[i].sp_threshold = float(np.random.choice(sp_thresholds_set))
        else:
            taskset_param[i].sp_threshold = random.uniform(trd_min, trd_max)

    # 4. Core Allocation (Processor ID assignment)
    indexed_tasks = [(i, taskset_param[i]) for i in range(n_tasks)]
    indexed_tasks.sort(key=lambda item: item[1].et_mean / item[1].period, reverse=True)

    core_utilizations = [0.0] * n_cores
    for _, t in indexed_tasks:
        min_core_idx = int(np.argmin(core_utilizations))
        t.processorId = min_core_idx
        core_utilizations[min_core_idx] += (t.et_mean / t.period)

    # 5. Compute C++-facing derived fields (deadline, weights, execution bounds,
    #    performance records, total running time) before serialization.
    g_final_et_range = cfgs.get("FINAL_Et_OVER_PERIOD_RANGE", [0.05, 0.9])
    n_ms = (n_sec * 1000) if n_sec is not None else 100000
    max_time_limit_options = cfgs.get("MAX_TIME_LIMIT_OPTIONS", 10)

    tasks_dict_list = []
    for i in range(n_tasks):
        t = taskset_param[i]

        serialized_components = []
        for c in t.components:
            serialized_components.append({
                'Et_mean': float(c.et_mean),
                'Et_sigma': float(c.et_sigma),
                'ro_1_Et': float(getattr(c, 'ro_1_Et', 0.0)),
                'ro_2_Et': float(getattr(c, 'ro_2_Et', 0.0)),
                'coeffs': c.get_coeffs_dict()
            })

        # Performance records for time-limit tasks
        perf_records_time_str = ""
        perf_records_perf_str = ""

        if getattr(t, 'time_limit_task', False):
            # Perf tasks: full range bounds so TL options span the config range
            execution_time_min = t.period * g_final_et_range[0]
            execution_time_max = t.period * g_final_et_range[1]
            n_steps = max_time_limit_options - 1
            step = (execution_time_max - execution_time_min) / n_steps
            t_s = execution_time_min
            perf_time = []
            perf_perf = []
            for _ in range(max_time_limit_options):
                perf_time.append(t_s)
                perf_perf.append(len(perf_perf) * 0.1 + 0.1)
                t_s += step
            perf_records_time_str = " ".join(f"{x:.3f}" for x in perf_time)
            perf_records_perf_str = " ".join(f"{x:.1f}" for x in perf_perf)
            sp_weight_base = 2.0
        else:
            # Normal and env tasks: min/max = mean ± 2*sigma (Gaussian distribution bounds)
            execution_time_min = max(1.0, t.et_mean - 2.0 * t.et_sigma)
            execution_time_max = max(1.0, t.et_mean + 2.0 * t.et_sigma)
            sp_weight_base = 1.0

        tasks_dict_list.append({
            'weights': t.weights,
            'n_weights': len(t.components),
            'period': int(t.period),
            'deadline': int(t.deadline),
            'D1_MIN': float(t.d1_min),
            'D1_MAX': float(t.d1_max),
            'D1_sigma': float(t.d1_sigma),
            'D2_MIN': float(t.d2_min),
            'D2_MAX': float(t.d2_max),
            'D2_sigma': float(t.d2_sigma),
            'Et_mean': float(t.et_mean),
            'Et_sigma': float(t.et_sigma),
            'sp_weight': float(sp_weight_base),
            'sp_threshold': float(t.sp_threshold),
            'processorId': int(getattr(t, 'processorId', 0)),
            'env_dependent': bool(getattr(t, 'env_dependent', False)),
            'time_limit_task': bool(getattr(t, 'time_limit_task', False)),
            'execution_time_min': float(execution_time_min),
            'execution_time_max': float(execution_time_max),
            'performance_records_time': perf_records_time_str,
            'performance_records_perf': perf_records_perf_str,
            'total_running_time': int(n_ms),
            'name': f'task_{i+1}',
            'tasks': serialized_components
        })

    # Normalize weights to SP_WEIGHTS_SUM
    g_total_weights = cfgs.get("SP_WEIGHTS_SUM", 5.0)
    total_weights = sum(t['sp_weight'] for t in tasks_dict_list)
    if total_weights > 0.0:
        for t in tasks_dict_list:
            t['sp_weight'] *= g_total_weights / total_weights

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
