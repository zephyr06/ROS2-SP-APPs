import json
import math
import sys
import numpy as np
import random
import yaml
from .gmm_model import GaussianComponent, GMMTaskModel, calc_mix_Et_sigma
from .generation_config_parser import standardize_config

# Keep common parameters that are shared across gaussian tasks in a GMM task
SHARED_TASK_PARAMS = ["period", "D1_MIN", "D1_MAX", "D1_sigma", "D2_MIN", "D2_MAX", "D2_sigma"]

# ---------------------------------------------------------------------------
# Config integrity: every generation parameter must be set explicitly.
# ---------------------------------------------------------------------------
# A forgotten parameter previously fell through to a hard-coded default in
# standardize_config / generate_taskset_parameters, so two runs that looked
# identical could silently diverge because one forgot a key. These registries
# make that impossible: REQUIRED keys must be present, and if one is missing
# the user is prompted (interactive) or the run aborts with a clear list
# (non-interactive). The ``suggest`` value is the former silent default,
# surfaced as a *suggestion* the user can accept with Enter -- it is never
# applied without the user's consent.
REQUIRED_CONFIG_PARAMS = [
    {"key": "N_TASKS",                    "suggest": 10,                       "desc": "total number of tasks"},
    {"key": "PERIODS_MS",                 "suggest": [1000, 500, 200, 100, 50, 33, 20], "desc": "period pool (ms) every task draws from"},
    {"key": "N_CORES",                    "suggest": 2,                        "desc": "number of processor cores"},
    {"key": "MAX_UTIL_PER_TASK",          "suggest": 0.95,                     "desc": "per-task utilization cap"},
    {"key": "MIN_PERIOD_ENV_DEPENDENT",   "suggest": 0,                        "desc": "min period (ms) for env-dependent tasks"},
    {"key": "PERF_RECORD_TASK_PROBABILITY", "suggest": 0.5,                    "desc": "probability a non-env task becomes a perf-record task"},
    {"key": "N_GMM_COMPONENTS_PER_TASK",  "suggest": 4,                        "desc": "GMM components per task"},
    {"key": "FINAL_Et_OVER_PERIOD_RANGE", "suggest": [0.05, 0.9],              "desc": "final ET/period range for perf-task time-limit options"},
    {"key": "SP_THRESHOLD_RANGE",         "suggest": [0.001, 0.9],             "desc": "SP threshold range (continuous uniform sampling per task)"},
    {"key": "FIXED_TASK_SIGMA_RATIO",     "suggest": 0.001,                    "desc": "sigma/mean ratio for perf (near-deterministic) tasks"},
    {"key": "MAX_TIME_LIMIT_OPTIONS",     "suggest": 10,                       "desc": "number of time-limit options generated for perf tasks"},
    {"key": "SP_WEIGHT_RANGE",            "suggest": [0.1, 1.0],               "desc": "SP weight range (continuous uniform sampling per task, pre-normalization)"},
    {"key": "SP_WEIGHTS_SUM",             "suggest": 5.0,                      "desc": "total SP weight sum tasks are normalized to"},
    {"key": "IMPORTANT_TASK_RATIO",       "suggest": 0.5,                     "desc": "fraction of tasks (by sp_weight) labeled important; persisted as Task::is_important (P0.6/P0.7/P0.8)"},
    {"key": "Et_OVER_PERIOD_RANGE",       "suggest": [0.1, 0.3],               "desc": "ET/period sampling range"},
    {"key": "SIGMA_OVER_Et_RANGE",        "suggest": [0.5, 0.6],               "desc": "sigma/ET sampling range"},
    {"key": "RO_1_Et_RANGE",              "suggest": [-0.9, -0.7],             "desc": "ro_1_Et correlation sampling range"},
    {"key": "RO_2_Et_RANGE",              "suggest": [-0.1, 0.1],              "desc": "ro_2_Et correlation sampling range"},
    {"key": "D1_RANGE",                   "suggest": [-100, 100],              "desc": "map x range (derived from MAP_WIDTH_M if set)"},
    {"key": "D2_RANGE",                   "suggest": [-100, 100],              "desc": "map y range (derived from MAP_HEIGHT_M if set)"},
    # Note: CPU_UTIL_RANDOM_RANGE is also required, but it is enforced by a hard
    # raise inside standardize_config (it predates this integrity gate and keeps
    # its own error message/tests), so it is intentionally NOT listed here.
]
# Keys whose absence is a documented, meaningful choice rather than a forgotten
# value. These are never prompted and never cause a raise; the generator reads
# them with cfgs.get(key) and treats None as the opt-out signal.
OPTIONAL_CONFIG_PARAMS = [
    {"key": "RANDOM_SEED",            "desc": "absent = OS entropy (non-reproducible run)"},
    {"key": "MAX_UTIL_PER_ENV_TASK",  "desc": "absent = env tasks use MAX_UTIL_PER_TASK"},
    {"key": "DEADLINE_MODE",          "desc": "absent/'constrained' = deadline=period*U(0.5,1.0) (D<T, "
                                              "constrained deadlines); 'implicit' = deadline=period (D=T, "
                                              "the gmm_model default). D=T makes RM≡DM (P0.9's DM-over-RM "
                                              "distinction collapses); the important-first group lock is "
                                              "deadline-independent and still meaningful."},
]


def _parse_prompted_value(raw: str, suggest):
    """Parse a value typed at the integrity prompt.

    ``json.loads`` handles ints, floats, bools, lists, and null; if the raw
    string is not valid JSON it is kept as a plain string.
    """
    raw = raw.strip()
    if raw == "":
        return suggest
    try:
        return json.loads(raw)
    except (json.JSONDecodeError, ValueError):
        return raw


def _write_back_resolved_keys(config_path: str, resolved: dict) -> None:
    """Persist interactively-resolved keys into the top-level config file.

    Only the keys the user just resolved are merged in; existing keys are
    left untouched. The INCLUDE base template is never written to -- only the
    top-level config_path the user actually passed in.
    """
    try:
        with open(config_path, "r") as f:
            on_disk = json.load(f)
        if not isinstance(on_disk, dict):
            return
        on_disk.update(resolved)
        with open(config_path, "w") as f:
            json.dump(on_disk, f, indent=4)
    except OSError as e:
        print(f"  Warning: could not write resolved values back to "
              f"{config_path} ({e}); using them for this run only.")


def validate_config_integrity(cfgs: dict, config_path: str = None) -> dict:
    """Ensure every required taskset-generation parameter is explicitly set.

    Missing required keys trigger an interactive prompt (offering the suggested
    default, accepted by pressing Enter) when stdin is a TTY; resolved values
    are written back to ``config_path`` so the user is not re-prompted next
    run. In a non-interactive session (parallel e2e/compare_optimizers worker,
    CI, piped stdin) -- or when no ``config_path`` is available -- raises
    ``ValueError`` listing every missing key with its suggestion and
    description. Idempotent: a complete config returns unchanged.
    """
    missing = [e for e in REQUIRED_CONFIG_PARAMS if e["key"] not in cfgs]
    if not missing:
        return cfgs

    # Non-interactive: cannot prompt. Fail loudly with the full list so the
    # user knows exactly what to add. Mirrors the sys.stdin.isatty() guard in
    # simulation_experiments/run_sim_experiments.py (the parallel
    # compare_optimizers harness runs workers without a TTY).
    if not sys.stdin.isatty() or config_path is None:
        lines = ["Taskset generation config is missing required parameters:"]
        for e in missing:
            lines.append(f"  - {e['key']} (suggested: {e['suggest']!r}) -- {e['desc']}")
        where = f" {config_path}" if config_path else " your config file"
        lines.append(f"Add the missing keys to{where} (or run interactively to be prompted).")
        raise ValueError("\n".join(lines))

    # Interactive: prompt per key with the suggestion, write resolved values back.
    print("\nTaskset generation config is missing required parameters.")
    print("Suggested values are the former silent defaults -- press Enter to accept,")
    print("or type a value (JSON: int/float/list/bool/null).\n")
    resolved = {}
    for e in missing:
        key = e["key"]
        desc = e["desc"]
        suggest = e["suggest"]
        print(f"{key} -- {desc}")
        print(f"  suggested: {suggest!r}")
        try:
            raw = input(f"  {key} [Enter to accept]: ")
        except EOFError:
            raw = ""
        value = _parse_prompted_value(raw, suggest)
        cfgs[key] = value
        resolved[key] = value
        print()
    _write_back_resolved_keys(config_path, resolved)
    return cfgs

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

def pick_period(cfgs: dict, picked_periods: list) -> int:
    """Select a random period from the unified ``PERIODS_MS`` pool, avoiding duplicates.

    Tries up to 10 times to draw a period not already picked (so distinct
    periods are preferred while the pool has variety); falls back to an
    unconditional draw once the pool is exhausted at large N (duplicate periods
    are acceptable per the user). The big/small distinction was removed in P19
    -- every task draws from the same pool.
    """
    periods_list = cfgs["PERIODS_MS"]

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
    n_weights = cfgs["N_GMM_COMPONENTS_PER_TASK"]  # presence enforced by validate_config_integrity

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
    """Orchestrates generation of all GMMTaskModels scaled to a sampled per-core utilization."""
    cfgs = standardize_config(cfgs)
    # Defense-in-depth: if a caller bypassed load_generation_config (e.g. a test
    # or direct call) and the config is incomplete, surface it here rather than
    # silently falling back to a default. config_path is unknown at this layer,
    # so a missing key raises (non-interactive) rather than prompting.
    validate_config_integrity(cfgs, config_path=None)

    # Seeding for reproducibility
    seed = cfgs.get("RANDOM_SEED")
    if seed is not None:
        np.random.seed(seed)
        random.seed(seed)

    n_cores = cfgs["N_CORES"]
    # P14: per-core CPU utilization is sampled uniformly from
    # CPU_UTIL_RANDOM_RANGE [low, high] per task set (the first draw off the
    # seeded RNG, so the sampled value is reproducible under a fixed
    # RANDOM_SEED). The range is required -- standardize_config raises if it is
    # absent -- so there is no fixed-scalar fallback here. The total cpu_util
    # is per_core * N_CORES. Recording per_core_cpu_util in the returned dict
    # surfaces the realized load in taskset_param.yaml so each task set's
    # utilization is inspectable.
    cpu_util_range = cfgs["CPU_UTIL_RANDOM_RANGE"]
    per_core_cpu_util = random.uniform(cpu_util_range[0], cpu_util_range[1])
    cpu_util = per_core_cpu_util * n_cores
    taskset_param = []
    picked_periods = []

    # P19: the big/small period split is gone -- N_TASKS is the sole task-count
    # input and every task draws its period from the unified PERIODS_MS pool.
    # standardize_config() guarantees n_tasks >= 1 and a non-empty PERIODS_MS;
    # pick_period handles pool exhaustion (duplicate periods) at large N.
    n_tasks = cfgs["N_TASKS"]

    # Determine the number of env-dependent tasks from the per-taskset ratio.
    # ENV_DEPENDENT_TASKS_RATIO [low, high] (set/validated by
    # standardize_config; default [0.1, 0.3]) is sampled uniformly AFTER
    # per_core_cpu_util above (so P14's asserted per-core value is unchanged
    # for existing seeds) and BEFORE uunifast below. n_env = ceil(ratio*N)
    # clamped to [1, N] -- every task set has >= 1 env-dependent task. The
    # legacy literal-count knob N_ENV_DEPENDENT_TASKS is no longer read
    # (full removal); standardize_config hard-rejects it if a config still
    # carries it.
    ratio_low, ratio_high = cfgs["ENV_DEPENDENT_TASKS_RATIO"]
    ratio = random.uniform(ratio_low, ratio_high)
    n_env_dependent = max(1, min(math.ceil(ratio * n_tasks), n_tasks))

    # Small sigma base for perf tasks so ET is effectively deterministic
    FIXED_TASK_SIGMA_RATIO = cfgs["FIXED_TASK_SIGMA_RATIO"]  # presence enforced by validate_config_integrity

    # 1. Generate periods (all drawn from the unified PERIODS_MS pool)
    periods = []
    for _ in range(n_tasks):
        periods.append(pick_period(cfgs, picked_periods=picked_periods))

    # ------------------------------------------------------------------
    # UUniFast mode: generate exact utilization vector, then derive Et_mean
    # Legacy random-Et-then-scale mode has been removed.
    # ------------------------------------------------------------------
    util_vector = uunifast_distribution(n_tasks, cpu_util, max_util_cap=cfgs["MAX_UTIL_PER_TASK"])

    # Pick env-dependent tasks weighted by utilization from tasks whose
    # period is >= MIN_PERIOD_ENV_DEPENDENT (avoids short-period blow-ups).
    min_period_env = cfgs["MIN_PERIOD_ENV_DEPENDENT"]  # presence enforced by validate_config_integrity
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
    #
    #    (3a) no-inflation: freed env-util is DROPPED, NOT redistributed. A task's
    #    utilization must never be raised above its UUniFast-drawn u_i to meet the
    #    total target — the prior proportional redistribution did exactly that
    #    (and it would raise PERF u_i -> raise perf execution_time_mu = perf WCET
    #    under the P0.8 et_mean rule -> spurious gate rejections). Dropping only
    #    LOWERS the realized load, which the gate never penalizes (it certifies
    #    schedulability, not a utilization target), so this is strictly safe.
    #    Side effect (cosmetic, see the `cpu_util` note on the returned dict):
    #    the realized total load falls below the sampled target by the freed
    #    amount.
    max_util_env = cfgs.get("MAX_UTIL_PER_ENV_TASK")
    if max_util_env is not None:
        for i in range(n_tasks):
            if i in env_task_indices and util_vector[i] > max_util_env:
                util_vector[i] = max_util_env

    # 3. Select time-limit (performance-record) tasks from non-env candidates.
    # All non-env tasks are eligible regardless of period (the former
    # MIN_PERIOD_WITH_PERFORMANCE_RECORDS period floor was removed in P16);
    # the legacy key is still accepted by the config loader but is now a no-op.
    perf_prob = cfgs["PERF_RECORD_TASK_PROBABILITY"]  # presence enforced by validate_config_integrity
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
    #
    # Deadline mode is config-driven (DEADLINE_MODE). The GMMTaskModel default
    # is deadline=period (gmm_model.py); the constrained draw below is the ONLY
    # place that overrides it to deadline < period. 'implicit' (D=T) leaves the
    # default in place -- the loosest feasible deadline, which removes the
    # tight-deadline-on-isolated-heavy-task failure mode (Mode 1 of the P0.8
    # gate rejection). Note D=T makes RM≡DM, so P0.9's DM-over-RM ordering
    # argument collapses (the important-first group lock is unaffected).
    deadline_mode = cfgs.get("DEADLINE_MODE", "constrained")
    if deadline_mode != "implicit":
        for i in range(n_tasks):
            taskset_param[i].deadline = int(round(taskset_param[i].period * random.uniform(0.5, 1.0)))

    # P2.14: SP_THRESHOLDS_SET removed -- sp_threshold is now sampled continuously
    # and uniformly from SP_THRESHOLD_RANGE for every task. The former discrete
    # option set (which also carried 1.0 = 100% DDL-miss tolerated as "safe")
    # is gone, so no task can be marked unpenalizable.
    trd_min = cfgs['SP_THRESHOLD_RANGE'][0]  # presence enforced by validate_config_integrity
    trd_max = cfgs['SP_THRESHOLD_RANGE'][1]

    # P2.15: sp_weight is sampled continuously and uniformly from SP_WEIGHT_RANGE
    # for every task, replacing the former hardcoded 2:1 perf/non-perf base split
    # (sp_weight_base = 2.0 for time_limit_task, 1.0 otherwise). "Importance" is
    # now a random per-task property, decoupled from task type. SP_WEIGHTS_SUM
    # normalization below preserves the scale contract; only the ratios change.
    wt_min = cfgs['SP_WEIGHT_RANGE'][0]  # presence enforced by validate_config_integrity
    wt_max = cfgs['SP_WEIGHT_RANGE'][1]

    for i in range(n_tasks):
        taskset_param[i].sp_weight = random.uniform(wt_min, wt_max)
        taskset_param[i].sp_threshold = random.uniform(trd_min, trd_max)

    # P0.6/P0.7/P0.8: important-task labeling. The top IMPORTANT_TASK_RATIO
    # fraction of tasks by sp_weight (pre-normalization -- ratios are invariant
    # under the SP_WEIGHTS_SUM scaling below) are marked important. The label is
    # a generation-time property persisted to YAML and read back by C++ Task::
    # is_important, so every consumer (static-solution priority-lock, online
    # fall-back safety check, generation-time RTA, miss-rate analysis) reads one
    # source of truth instead of each recomputing a top-X% cut. sp_weight is
    # sampled continuously uniform -> ties are measure-zero; if a tie ever lands
    # exactly on the boundary, break by task index (deterministic). Count is
    # ceil(N * ratio) = (N * ratio) rounded up, mirroring the old analysis-side
    # int(N * pct + 0.9999) ceil; for the default ratio=0.5 this is (N+1)//2.
    important_ratio = cfgs["IMPORTANT_TASK_RATIO"]
    n_important = max(1, math.ceil(n_tasks * important_ratio))
    order = sorted(range(n_tasks), key=lambda i: (taskset_param[i].sp_weight, i), reverse=True)
    for rank, i in enumerate(order):
        taskset_param[i].is_important = (rank < n_important)

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
    g_final_et_range = cfgs["FINAL_Et_OVER_PERIOD_RANGE"]  # presence enforced by validate_config_integrity
    n_ms = (n_sec * 1000) if n_sec is not None else 100000
    max_time_limit_options = cfgs["MAX_TIME_LIMIT_OPTIONS"]

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
        else:
            # Normal and env tasks: min/max = mean ± 2*sigma (Gaussian distribution bounds)
            execution_time_min = max(1.0, t.et_mean - 2.0 * t.et_sigma)
            execution_time_max = max(1.0, t.et_mean + 2.0 * t.et_sigma)

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
            'sp_weight': float(t.sp_weight),
            'sp_threshold': float(t.sp_threshold),
            'is_important': bool(getattr(t, 'is_important', False)),
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
    g_total_weights = cfgs["SP_WEIGHTS_SUM"]  # presence enforced by validate_config_integrity
    total_weights = sum(t['sp_weight'] for t in tasks_dict_list)
    if total_weights > 0.0:
        for t in tasks_dict_list:
            t['sp_weight'] *= g_total_weights / total_weights

    rt = {
        'n_tasks': n_tasks,
        # NOTE (3a): this is the SAMPLED target utilization (per_core * n_cores),
        # NOT the realized load. When MAX_UTIL_PER_ENV_TASK caps env tasks, the
        # env-cap block above DROPS the freed env-util (no redistribution), so
        # the realized total load (sum of u_i) is LOWER than this field by the
        # freed amount. This over-report is cosmetic: the P0.8 gate does NOT read
        # cpu_util (it derives WCET from execution_time_mu / execution_time_max
        # and the RTA's utilization guard reads sum(WCET/period) per core). The
        # one contract consumer (test_specifications.py: assert sum(Et_mean/
        # period) == cpu_util) uses a config WITHOUT MAX_UTIL_PER_ENV_TASK, so
        # the env-cap block never fires there and the assertion holds.
        'cpu_util': cpu_util,
        # P14: realized per-core utilization sampled from CPU_UTIL_RANDOM_RANGE
        # that produced cpu_util above. Same over-report caveat as cpu_util when
        # env-capping fires.
        'per_core_cpu_util': per_core_cpu_util,
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
