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


def resolve_taskset_config_path(num_tasks, config_dir=None, temp_dir=None):
    """Return the path to the taskset config for ``num_tasks`` tasks.

    If an on-disk ``taskset_cfg_paper_{N}.json`` exists in ``config_dir``
    (default: the repo's ``Gen_Taskset/task_sets_config/``), its path is
    returned unchanged -- this is the existing behavior for N=4/6/8.

    For any other N (e.g. 10/12/14/16/18), a thin override config is
    **synthesized** in ``temp_dir`` (default: a process-wide
    ``tempfile.mkdtemp``) with the same shape as the on-disk paper files:
    it INCLUDEs ``taskset_cfg_paper_base.json`` and sets only ``N_TASKS``
    (=N), ``N_CORES`` (=2) and ``RANDOM_SEED`` (=42; callers override the seed
    per-taskset anyway). Every task draws its period from the unified
    ``PERIODS_MS`` pool and the per-core load from ``CPU_UTIL_RANDOM_RANGE``,
    both defined in the base template.

    The synthesized ``INCLUDE`` is written as an **absolute** path to the real
    base template, so ``load_generation_config`` resolves it correctly
    regardless of where the temp file lives. This reuses the existing
    INCLUDE-resolution path verbatim -- no generator changes.

    Parameters
    ----------
    num_tasks : int
        Total number of tasks (``N_TASKS``). Must be >= 1 (P17 relaxed the
        former >= 2 floor that was tied to a fixed N_BIG=2; a single-task
        taskset is now valid). A single ``N_TASKS`` count is the sole task-count
        input; every task draws from one ``PERIODS_MS`` pool, so any N >= 1 is
        representable.
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

    # Synthesized config: N_TASKS is the sole task-count input and every task
    # draws from the unified PERIODS_MS pool in the base template. This is
    # equivalent to the former "2 big / (N-2) small" synthesized split in
    # period composition (the base template's PERIODS_MS is the big-pool
    # periods followed by the small-pool periods), just expressed through one
    # pool + one count. The base template also supplies CPU_UTIL_RANDOM_RANGE
    # (P14), so the synthesized override does not need to set it.
    #
    # N=1 feasibility: drop to 1 core and cap the per-core range below 1.0
    # (override the base template's [0.5, 1.5] sweep) so the lone task's
    # utilization stays schedulable. The base range can sample >1.0 per core,
    # which on a single core / single task means utilization >1.0 --
    # unschedulable. With N_CORES=2 the single task would carry cpu_util up to
    # 3.0 (1.5 x 2), also unschedulable. N=1 is never on the cross-task sweep;
    # this just keeps the corner case schedulable under P14's required range.
    # (Adapted from P17, which assumed P13's fixed 0.9.)
    n_cores = 1 if n == 1 else 2
    desc = (
        f"Paper parameters for {n} tasks [synthesized]"
        if n >= 2
        else "Paper parameters for 1 task [synthesized, single-rate]"
    )
    synthesized = {
        "INCLUDE": base_template_abs,
        "DESC": desc,
        "N_TASKS": n,
        "N_CORES": n_cores,
        "RANDOM_SEED": 42,
    }
    if n == 1:
        # Keep the single task schedulable: sample per-core util below 1.0.
        synthesized["CPU_UTIL_RANDOM_RANGE"] = [0.5, 0.9]

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
    """Validates configuration keys, sets defaults, and unifies the period pool.

    P19 collapsed the former paired big/small period schema (``BIG_PERIODS_MS``
    + ``SMALL_PERIODS_MS`` / ``N_BIG_PERIOD_TASKS`` + ``N_SMALL_PERIOD_TASKS``)
    into a single unified pool + count:

    - ``PERIODS_MS``: list[int] -- the single period pool every task draws from.
    - ``N_TASKS``: int -- total task count (the sole count input; no split).

    These are now the **only** accepted period/count inputs. The former
    backward-compat alias (which built ``PERIODS_MS`` from ``HZ`` /
    ``*_PERIOD_HZ`` / ``*_PERIODS_MS`` and ``N_TASKS`` from ``N_BIG`` +
    ``N_SMALL``) has been removed: any of those legacy keys now raises a
    ``ValueError`` pointing at the canonical replacement. Configs must set
    ``PERIODS_MS`` and ``N_TASKS`` directly (all shipped configs already do).
    """
    # --- Period pool: canonical key only, hard-reject legacy Hz/pool keys. ----
    _LEGACY_PERIOD_KEYS = {
        "HZ", "SMALL_PERIOD_HZ", "BIG_PERIOD_HZ",
        "SMALL_PERIODS_MS", "BIG_PERIODS_MS",
    }
    found_period_legacy = _LEGACY_PERIOD_KEYS & config.keys()
    if found_period_legacy:
        raise ValueError(
            f"Hz-style period keys are no longer supported; set PERIODS_MS "
            f"(list of periods in ms) instead. Found: {sorted(found_period_legacy)}"
        )

    # Canonical default when no period info is given -- the base template's pool
    # (so a bare config still works and matches the on-disk paper configs).
    if "PERIODS_MS" not in config:
        config["PERIODS_MS"] = [1000, 500, 200, 100, 50, 33, 20]

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

    # --- Task count: canonical N_TASKS only, hard-reject the big/small split. --
    _LEGACY_COUNT_KEYS = {"N_BIG_PERIOD_TASKS", "N_SMALL_PERIOD_TASKS"}
    found_count_legacy = _LEGACY_COUNT_KEYS & config.keys()
    if found_count_legacy:
        raise ValueError(
            f"Big/small task-count split is no longer supported; set N_TASKS "
            f"(total task count) instead. Found: {sorted(found_count_legacy)}"
        )

    if "N_TASKS" not in config:
        config["N_TASKS"] = 10

    # Validation: the only hard requirements are at least one task total and a
    # non-empty period pool. Explicit rejection avoids a cryptic divide-by-zero
    # in uunifast_distribution.
    if not isinstance(config["N_TASKS"], int) or config["N_TASKS"] < 1:
        raise ValueError(
            f"N_TASKS must be a positive integer, got {config['N_TASKS']!r}"
        )
    if not isinstance(config["PERIODS_MS"], list) or len(config["PERIODS_MS"]) == 0:
        raise ValueError(
            "PERIODS_MS must be a non-empty list of periods (ms), "
            f"got {config['PERIODS_MS']!r}"
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

    # N_CORES default logic. P14 made CPU_UTIL_RANDOM_RANGE the sole load input
    # (a [low, high] per-core range, not a single scalar), so the old
    # "2 cores when MEAN_CPU_UTIL > 1.0" heuristic has nothing to test. Default
    # to 1 core; all shipped configs set N_CORES explicitly, so this only
    # applies to bare configs that omit it.
    if "N_CORES" not in config:
        config["N_CORES"] = 1

    # P14: the per-core CPU utilization range is now **required**. Each task set
    # samples its per-core utilization uniformly from [low, high] (the first draw
    # off the seeded RNG, so the realized value is reproducible under a fixed
    # RANDOM_SEED), sweeping under- to over-subscribed loads within one run.
    # The former fixed-MEAN_CPU_UTIL scalar was removed as the load source
    # (P13); a config that omits the range now raises instead of silently
    # falling back, since a forgotten range would otherwise lock every task set
    # to one load point and quietly defeat the experiment's purpose.
    if "CPU_UTIL_RANDOM_RANGE" not in config:
        raise ValueError(
            "CPU_UTIL_RANDOM_RANGE is required: set a [low, high] per-core "
            "utilization range (the generator samples one value per task set). "
            "The former fixed MEAN_CPU_UTIL scalar is no longer supported."
        )
    cpu_util_range = config["CPU_UTIL_RANDOM_RANGE"]
    if (not isinstance(cpu_util_range, (list, tuple))
            or len(cpu_util_range) != 2
            or any(not isinstance(v, (int, float)) or isinstance(v, bool)
                    for v in cpu_util_range)):
        raise ValueError(
            "CPU_UTIL_RANDOM_RANGE must be a [low, high] pair of numbers, "
            f"got {cpu_util_range!r}"
        )
    low, high = cpu_util_range
    if low < 0.0 or low > high:
        raise ValueError(
            "CPU_UTIL_RANDOM_RANGE requires 0 <= low <= high, "
            f"got [{low}, {high}]"
        )
    config["CPU_UTIL_RANDOM_RANGE"] = [float(low), float(high)]

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
    required_keys = ["D2_RANGE", "CPU_UTIL_RANDOM_RANGE"]
    for key in required_keys:
        if key not in config:
            return False
    has_map = "MAP_WIDTH_M" in config and "MAP_HEIGHT_M" in config
    has_d1 = "D1_RANGE" in config
    return has_map or has_d1
