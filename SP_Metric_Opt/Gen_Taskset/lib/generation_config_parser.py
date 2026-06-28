import json
import math
import os

def load_generation_config(config_path: str) -> dict:
    """Reads the JSON configuration file for task set generation."""
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found: {config_path}")
    
    with open(config_path, 'r') as f:
        config = json.load(f)
    
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

    # Per-task utilization caps
    config["MAX_UTIL_PER_TASK"] = config.get("MAX_UTIL_PER_TASK", 0.95)
    # Optional tighter cap applied only to env-dependent tasks.
    # If None, env tasks use the same MAX_UTIL_PER_TASK cap as everyone else.
    config["MAX_UTIL_PER_ENV_TASK"] = config.get("MAX_UTIL_PER_ENV_TASK", None)
    config["MIN_PERIOD_WITH_PERFORMANCE_RECORDS"] = config.get(
        "MIN_PERIOD_WITH_PERFORMANCE_RECORDS", 100
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
