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


def _resolve_period_pool_ms(config, ms_key, hz_key, hz_split, default_ms):
    """Resolve one old big/small period pool to a list of periods in ms.

    Used only by ``standardize_config``'s backward-compat alias to build the
    unified ``PERIODS_MS`` from old-shape configs. The precedence mirrors the
    pre-P19 behavior exactly:

    1. ``ms_key`` (e.g. ``BIG_PERIODS_MS``) used as-is if present;
    2. else ``hz_key`` (e.g. ``BIG_PERIOD_HZ``) converted to ms;
    3. else the legacy ``HZ`` list filtered through ``hz_split`` (the <10 /
       >=10 Hz split that separated big from small);
    4. else ``default_ms`` (the pool's pre-P19 default).

    Each pool defaults independently, so a config that specifies only one pool
    still gets the other pool's default -- matching pre-P19 composition.
    """
    if ms_key in config:
        return list(config[ms_key])
    if hz_key in config:
        return [int(1000.0 / hz) for hz in config[hz_key]]
    if "HZ" in config:
        return [int(1000.0 / hz) for hz in config["HZ"] if hz_split(hz)]
    return list(default_ms)


def resolve_taskset_config_path(num_tasks, config_dir=None, temp_dir=None):
    """Return the path to the taskset config for ``num_tasks`` tasks.

    If an on-disk ``taskset_cfg_paper_{N}.json`` exists in ``config_dir``
    (default: the repo's ``Gen_Taskset/task_sets_config/``), its path is
    returned unchanged -- this is the existing behavior for N=4/6/8.

    For any other N (e.g. 10/12/14/16/18), a thin override config is
    **synthesized** in ``temp_dir`` (default: a process-wide
    ``tempfile.mkdtemp``) with the same shape as the on-disk paper files:
    it INCLUDEs ``taskset_cfg_paper_base.json`` and sets only ``N_TASKS``
    (=N), ``MEAN_CPU_UTIL`` (=0.9 per-core, see P13), ``N_CORES`` (=2) and
    ``RANDOM_SEED`` (=42; callers override the seed per-taskset anyway).
    Every task draws its period from the unified ``PERIODS_MS`` pool defined
    in the base template (P19 collapsed the former big/small split into a
    single pool).

    The synthesized ``INCLUDE`` is written as an **absolute** path to the real
    base template, so ``load_generation_config`` resolves it correctly
    regardless of where the temp file lives. This reuses the existing
    INCLUDE-resolution path verbatim -- no generator changes.

    Parameters
    ----------
    num_tasks : int
        Total number of tasks (``N_TASKS``). Must be >= 1 (P17 relaxed the
        former >= 2 floor that was tied to a fixed N_BIG=2; a single-task
        taskset is now valid). P19 replaced the N_BIG/N_SMALL split with a
        single ``N_TASKS`` count; every task draws from one ``PERIODS_MS``
        pool, so any N >= 1 is representable.
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

    # Synthesized config (P19): the big/small split no longer exists in the
    # schema -- N_TASKS is the sole task-count input and every task draws from
    # the unified PERIODS_MS pool in the base template. This is equivalent to
    # the former "2 big / (N-2) small" synthesized split in period composition
    # (the base template's PERIODS_MS is the big-pool periods followed by the
    # small-pool periods), just expressed through one pool + one count.
    #
    # N=1 feasibility: drop to 1 core so cpu_util = 0.9 x 1 = 0.9 stays
    # schedulable (the lone task's utilization stays < 1.0). With N_CORES=2 the
    # single task would carry cpu_util = 1.8, which uunifast_distribution can
    # only realize as a single 1.8 utilization -- overloaded / unschedulable.
    # N=1 is never on the cross-task sweep; this just keeps the corner case
    # schedulable. (Carried over verbatim from P17.)
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
    """Validates configuration keys, sets defaults, and unifies the period pool.

    P19 collapsed the former paired big/small period schema (``BIG_PERIODS_MS``
    + ``SMALL_PERIODS_MS`` / ``N_BIG_PERIOD_TASKS`` + ``N_SMALL_PERIOD_TASKS``)
    into a single unified pool + count:

    - ``PERIODS_MS``: list[int] -- the single period pool every task draws from.
    - ``N_TASKS``: int -- total task count (the sole count input; no split).

    Old configs that still carry the paired keys (or the legacy ``HZ`` /
    ``*_PERIOD_HZ`` Hz lists) keep working: when ``PERIODS_MS`` is absent, it is
    built by concatenating the big-pool periods followed by the small-pool
    periods (preserving the historical composition), and ``N_TASKS`` is the sum
    of the two counts. The old keys are then deleted so the standardized config
    carries only the canonical new keys (keeps ``_should_generate`` baselines
    clean post-refactor).
    """
    # --- Period pool: backward-compat alias, then canonical default. ----------
    # Resolve the big/small period pools (in ms) from whatever the config
    # provides: explicit *_PERIODS_MS lists, *_PERIOD_HZ lists, or the legacy
    # single HZ list split at 10 Hz. These are only used to build PERIODS_MS
    # for old-shape configs; new canonical configs set PERIODS_MS directly.
    # Each pool defaults independently (pre-P19 behavior preserved).
    big_ms = _resolve_period_pool_ms(
        config, "BIG_PERIODS_MS", "BIG_PERIOD_HZ",
        hz_split=lambda hz: hz < 10, default_ms=[4000, 2000, 1000],
    )
    small_ms = _resolve_period_pool_ms(
        config, "SMALL_PERIODS_MS", "SMALL_PERIOD_HZ",
        hz_split=lambda hz: hz >= 10, default_ms=[100, 50, 33, 20],
    )

    if "PERIODS_MS" not in config:
        # Old-shape config (or no period info): big-pool periods first, then
        # small-pool periods, preserving the historical composition of existing
        # configs (a former "2 big + 4 small" still yields 2 long + 4 short
        # periods, just from one pool). When both pools fall to their defaults
        # this is exactly the merged big+small default.
        config["PERIODS_MS"] = list(big_ms) + list(small_ms)

    # Drop the old paired period keys so the standardized config is canonical.
    for old_key in (
        "SMALL_PERIODS_MS", "BIG_PERIODS_MS",
        "SMALL_PERIOD_HZ", "BIG_PERIOD_HZ",
    ):
        config.pop(old_key, None)
    # Legacy single HZ list is also superseded by PERIODS_MS.
    config.pop("HZ", None)

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

    # --- Task count: backward-compat alias (N_BIG + N_SMALL), then N_TASKS. ---
    if "N_TASKS" not in config:
        n_big = config.get("N_BIG_PERIOD_TASKS", 2)
        n_small = config.get("N_SMALL_PERIOD_TASKS", 8)
        config["N_TASKS"] = n_big + n_small

    # Drop the old paired count keys so the standardized config is canonical.
    # (They are no longer read by the generator -- see P19.)
    config.pop("N_BIG_PERIOD_TASKS", None)
    config.pop("N_SMALL_PERIOD_TASKS", None)

    # P19 validation: the only hard requirements are at least one task total
    # and a non-empty period pool. (P17's N_BIG/N_SMALL>=0 checks had no
    # referent once the split was removed; the degenerate all-zero config is
    # now simply N_TASKS < 1.) Explicit rejection avoids a cryptic divide-by-
    # zero in uunifast_distribution.
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
