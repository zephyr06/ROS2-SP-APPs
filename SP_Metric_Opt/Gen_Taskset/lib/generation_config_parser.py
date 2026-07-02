import json
import math
import os
import tempfile

# Directory holding the on-disk paper_* taskset config files (this file lives at
# Gen_Taskset/lib/, the configs live at Gen_Taskset/task_sets_config/).
_TASK_SETS_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "task_sets_config",
)
# Name of the shared base template that per-N paper configs INCLUDE.
_PAPER_BASE_TEMPLATE_NAME = "taskset_cfg_paper_base.json"

# Per-core CPU utilization held constant across all task counts (see P13). The
# synthesized configs below use this value so dynamic N (10/12/14/...) matches
# the on-disk paper_4/6/8 semantics. Kept in sync with those files.
_DEFAULT_PER_CORE_CPU_UTIL = 0.9


def resolve_taskset_config_path(num_tasks, config_dir=None, temp_dir=None):
    """Return the path to the taskset config for ``num_tasks`` tasks.

    If an on-disk ``taskset_cfg_paper_{N}.json`` exists in ``config_dir``
    (default: the repo's ``Gen_Taskset/task_sets_config/``), its path is
    returned unchanged -- this is the existing behavior for N=4/6/8.

    For any other N (e.g. 10/12/14/16/18), a thin override config is
    **synthesized** in ``temp_dir`` (default: a process-wide
    ``tempfile.mkdtemp``) with the same shape as the on-disk paper files:
    it INCLUDEs ``taskset_cfg_paper_base.json`` and sets only ``N_BIG_PERIOD_TASKS``
    (=2), ``N_SMALL_PERIOD_TASKS`` (=N-2), ``MEAN_CPU_UTIL``
    (=0.9 per-core, see P13), ``N_CORES`` (=2) and ``RANDOM_SEED`` (=42; callers
    override the seed per-taskset anyway).

    The synthesized ``INCLUDE`` is written as an **absolute** path to the real
    base template, so ``load_generation_config`` resolves it correctly
    regardless of where the temp file lives. This reuses the existing
    INCLUDE-resolution path verbatim -- no generator changes.

    Parameters
    ----------
    num_tasks : int
        Total number of tasks. Must be >= 1 (P17 relaxed the former >= 2
        floor that was tied to a fixed N_BIG=2; a single-task taskset is now
        valid, e.g. a single-rate small-period taskset with N_BIG=0).
    config_dir : str, optional
        Directory to look for an existing ``taskset_cfg_paper_{N}.json``.
        Defaults to the repo config dir.
    temp_dir : str, optional
        Directory to write the synthesized config into when no on-disk file
        exists. If None, ``tempfile.mkdtemp`` is used (caller is responsible
        for cleanup; the synthesized files are throwaway).

    Returns
    -------
    str
        Absolute path to the config file to feed into
        ``load_generation_config``.
    """
    # Reject bool (a subclass of int) and float outright; accept plain ints and
    # int-valued strings. bool/float inputs are almost certainly caller bugs.
    if isinstance(num_tasks, bool) or isinstance(num_tasks, float):
        raise ValueError(f"num_tasks must be an integer, got {num_tasks!r}")
    try:
        n = int(num_tasks)
    except (TypeError, ValueError):
        raise ValueError(f"num_tasks must be an integer, got {num_tasks!r}")
    if n < 1:
        raise ValueError(
            f"num_tasks must be >= 1 (P17: the former N>=2 floor tied to a "
            f"fixed N_BIG=2 was relaxed; N_BIG=0 / N_SMALL=0 are now allowed), "
            f"got {n}"
        )

    cfg_dir = config_dir or _TASK_SETS_CONFIG_DIR
    on_disk = os.path.join(cfg_dir, f"taskset_cfg_paper_{n}.json")
    if os.path.exists(on_disk):
        return os.path.abspath(on_disk)

    # Synthesize a thin override file equivalent to the on-disk paper configs.
    # INCLUDE is an absolute path so load_generation_config (which resolves
    # INCLUDE relative to the config file's directory) finds the real base
    # template regardless of where the temp file was written.
    base_template_abs = os.path.join(cfg_dir, "templates", _PAPER_BASE_TEMPLATE_NAME)
    if not os.path.exists(base_template_abs):
        raise FileNotFoundError(
            f"Cannot synthesize taskset config for N={n}: base template not "
            f"found at {base_template_abs}"
        )

    # Synthesized split (P17): keep the historical 2 big / (N-2) small default
    # for N >= 2 so existing on-disk paper_4/6/8 and the cross-task sweep
    # [4,6,8,10,12,14,16,18] are unaffected. For N == 1, emit a single
    # small-period task (N_BIG=0) -- the minimal single-rate taskset, which the
    # former N_BIG=2 hardcode could not represent (it would have made
    # N_SMALL = -1).
    if n >= 2:
        n_big, n_small = 2, n - 2
        n_cores = 2
        desc = f"Paper parameters for {n} tasks (2 big, {n_small} small) [synthesized]"
    else:  # n == 1
        # Single-task taskset: drop to 1 core so cpu_util = 0.9 x 1 = 0.9 stays
        # feasible (util < 1.0). With N_CORES=2 the lone task would carry
        # cpu_util = 1.8, which uunifast_distribution can only realize as a
        # single 1.8 utilization -- overloaded / unschedulable. N=1 is a
        # single-rate corner case, never on the cross-task sweep.
        n_big, n_small = 0, 1
        n_cores = 1
        desc = "Paper parameters for 1 task (0 big, 1 small) [synthesized, single-rate]"
    synthesized = {
        "INCLUDE": base_template_abs,
        "DESC": desc,
        "N_BIG_PERIOD_TASKS": n_big,
        "N_SMALL_PERIOD_TASKS": n_small,
        "MEAN_CPU_UTIL": _DEFAULT_PER_CORE_CPU_UTIL,
        "N_CORES": n_cores,
        "RANDOM_SEED": 42,
    }

    write_dir = temp_dir or tempfile.mkdtemp(prefix="synthesized_taskset_cfg_")
    os.makedirs(write_dir, exist_ok=True)
    synth_path = os.path.join(write_dir, f"taskset_cfg_paper_{n}.json")
    with open(synth_path, "w") as f:
        json.dump(synthesized, f, indent=4)
    return synth_path


def load_generation_config(config_path: str) -> dict:
    """Reads the JSON configuration file for task set generation.

    Supports an optional ``INCLUDE`` key with a path (relative to the config
    file's directory or the ``templates/`` subdirectory) pointing to a base
    JSON file whose values are merged in.  The top-level file always wins on
    key collisions.
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    config_dir = os.path.dirname(os.path.abspath(config_path))
    with open(config_path, 'r') as f:
        config = json.load(f)

    if "INCLUDE" in config:
        include_path = config.pop("INCLUDE")
        # Resolve relative to config dir, then templates dir
        candidates = [
            os.path.join(config_dir, include_path),
            os.path.join(config_dir, "templates", include_path),
        ]
        resolved = None
        for cand in candidates:
            if os.path.exists(cand):
                resolved = cand
                break
        if resolved is None:
            raise FileNotFoundError(
                f"Include file not found for config {config_path}: "
                f"tried {candidates}"
            )
        with open(resolved, 'r') as f:
            base = json.load(f)
        # Remove INCLUDE from base if present (nested includes not supported)
        base.pop("INCLUDE", None)
        merged = dict(base)
        merged.update(config)
        config = merged

    # Standardize/convert parameters
    config = standardize_config(config)
    return config

def standardize_config(config: dict) -> dict:
    """Validates configuration keys, sets defaults, and converts Hz to periods (ms) if needed."""
    # Convert HZ to periods in ms if HZ parameters are used
    if "SMALL_PERIODS_MS" not in config:
        if "SMALL_PERIOD_HZ" in config:
            config["SMALL_PERIODS_MS"] = [int(1000.0 / hz) for hz in config["SMALL_PERIOD_HZ"]]
        elif "HZ" in config:
            config["SMALL_PERIODS_MS"] = [int(1000.0 / hz) for hz in config["HZ"] if hz >= 10]
        else:
            config["SMALL_PERIODS_MS"] = [100, 50, 33, 20] # Default
            
    if "BIG_PERIODS_MS" not in config:
        if "BIG_PERIOD_HZ" in config:
            config["BIG_PERIODS_MS"] = [int(1000.0 / hz) for hz in config["BIG_PERIOD_HZ"]]
        elif "HZ" in config:
            config["BIG_PERIODS_MS"] = [int(1000.0 / hz) for hz in config["HZ"] if hz < 10]
        else:
            config["BIG_PERIODS_MS"] = [4000, 2000, 1000] # Default

    # Legacy key check
    _LEGACY_KEYS = {
        "N_PERFORMANCE_RECORD_TASKS",
        "N_MIX_WEIGHTS_PER_TASK",
        "N_PROCESSORS",
        "MIN_PERIOID_WITH_PERFORMANCE_RECORDS",
    }
    found_legacy = _LEGACY_KEYS & config.keys()
    if found_legacy:
        raise KeyError(
            f"Legacy config keys detected: {sorted(found_legacy)}. "
            "Update the config to use the current field names instead."
        )

    config["N_BIG_PERIOD_TASKS"] = config.get("N_BIG_PERIOD_TASKS", 2)
    config["N_SMALL_PERIOD_TASKS"] = config.get("N_SMALL_PERIOD_TASKS", 8)
    config["N_TASKS"] = config["N_BIG_PERIOD_TASKS"] + config["N_SMALL_PERIOD_TASKS"]

    # P17: N_BIG=0 and N_SMALL=0 are now allowed (single-rate tasksets), so the
    # only hard requirement is at least one task total. Explicitly reject a
    # degenerate all-zero config (which would otherwise divide by zero in
    # uunifast_distribution) with a clear message instead of a cryptic crash.
    if config["N_BIG_PERIOD_TASKS"] < 0 or config["N_SMALL_PERIOD_TASKS"] < 0:
        raise ValueError(
            "N_BIG_PERIOD_TASKS and N_SMALL_PERIOD_TASKS must be >= 0, got "
            f"N_BIG={config['N_BIG_PERIOD_TASKS']}, "
            f"N_SMALL={config['N_SMALL_PERIOD_TASKS']}"
        )
    if config["N_TASKS"] < 1:
        raise ValueError(
            "N_TASKS must be >= 1 (P17 allows N_BIG=0 or N_SMALL=0 but not both); "
            f"got N_BIG={config['N_BIG_PERIOD_TASKS']}, "
            f"N_SMALL={config['N_SMALL_PERIOD_TASKS']}"
        )

    # Per-task utilization caps
    config["MAX_UTIL_PER_TASK"] = config.get("MAX_UTIL_PER_TASK", 0.95)
    # Optional tighter cap applied only to env-dependent tasks.
    # If None, env tasks use the same MAX_UTIL_PER_TASK cap as everyone else.
    config["MAX_UTIL_PER_ENV_TASK"] = config.get("MAX_UTIL_PER_ENV_TASK", None)
    # MIN_PERIOD_WITH_PERFORMANCE_RECORDS was a period floor that gated which
    # non-env tasks could become time-limit (performance-record) tasks. It was
    # removed in P16 so all non-env tasks are eligible regardless of period.
    # The key is still accepted here (kept as a no-op) so existing configs and
    # templates that set it continue to load without error; it has no effect.
    config["MIN_PERIOD_WITH_PERFORMANCE_RECORDS"] = config.get(
        "MIN_PERIOD_WITH_PERFORMANCE_RECORDS", 0
    )

    # Minimum period for tasks that may be marked env-dependent.
    # (Short-period env tasks are prone to ET > period with strong spatial correlations.)
    config["MIN_PERIOD_ENV_DEPENDENT"] = config.get("MIN_PERIOD_ENV_DEPENDENT", 0)

    # Probability of selecting a non-env-dependent task as a performance-record task
    config["PERF_RECORD_TASK_PROBABILITY"] = config.get(
        "PERF_RECORD_TASK_PROBABILITY", 0.5
    )

    # GMM properties
    config["N_GMM_COMPONENTS_PER_TASK"] = config.get("N_GMM_COMPONENTS_PER_TASK", 4)

    # Scale factor
    config["Et_SCALE_FACTOR"] = config.get("Et_SCALE_FACTOR", 2.0)
    config["FINAL_Et_OVER_PERIOD_RANGE"] = config.get("FINAL_Et_OVER_PERIOD_RANGE", [0.05, 0.9])
    
    # Backward-compat alias
    if "N_CORES" not in config and "N_PROCESSORS" in config:
        config["N_CORES"] = config["N_PROCESSORS"]

    # N_CORES default logic
    if "N_CORES" not in config:
        config["N_CORES"] = 2 if config.get("MEAN_CPU_UTIL", 0.5) > 1.0 else 1
    
    # SP threshold option set
    config["SP_THRESHOLDS_SET"] = config.get("SP_THRESHOLDS_SET", [0.2, 0.4, 0.6, 0.8, 1.0])

    # Physical map dimensions: center both D1_RANGE (x) and D2_RANGE (y) at origin.
    # Cartesian coordinates: D1 = x, D2 = y.
    map_w = config.get("MAP_WIDTH_M", None)
    map_h = config.get("MAP_HEIGHT_M", None)
    if map_w is not None and map_h is not None:
        config["D1_RANGE"] = [-int(map_w / 2.0), int(map_w / 2.0)]
        config["D2_RANGE"] = [-int(map_h / 2.0), int(map_h / 2.0)]

    # Legacy D1_VARIANCE_FACTOR_TABLE removed: spatial variance is fully captured
    # by the GMM covariance matrix in Cartesian coordinates.
    return config

def validate_generation_config(config: dict) -> bool:
    """Validates required keys are present and correct."""
    required_keys = ["D2_RANGE", "MEAN_CPU_UTIL"]
    for key in required_keys:
        if key not in config:
            return False
    has_map = "MAP_WIDTH_M" in config and "MAP_HEIGHT_M" in config
    has_d1 = "D1_RANGE" in config
    return has_map or has_d1
