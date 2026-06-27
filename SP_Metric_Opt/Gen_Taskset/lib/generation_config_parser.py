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

    # Map N_PERFORMANCE_RECORD_TASKS/N_ENV_DEPENDENT_TASKS
    if "N_ENV_DEPENDENT_TASKS" not in config:
        config["N_ENV_DEPENDENT_TASKS"] = config.get("N_PERFORMANCE_RECORD_TASKS", 2)

    # Set default task counts
    config["N_BIG_PERIOD_TASKS"] = config.get("N_BIG_PERIOD_TASKS", 2)
    config["N_SMALL_PERIOD_TASKS"] = config.get("N_SMALL_PERIOD_TASKS", 8)
    config["N_TASKS"] = config["N_BIG_PERIOD_TASKS"] + config["N_SMALL_PERIOD_TASKS"]

    # Minimum period for performance records / soft tasks
    config["MIN_PERIOD_WITH_PERFORMANCE_RECORDS"] = config.get(
        "MIN_PERIOD_WITH_PERFORMANCE_RECORDS", 
        config.get("MIN_PERIOID_WITH_PERFORMANCE_RECORDS", 100)
    )

    # GMM properties
    config["N_GMM_COMPONENTS_PER_TASK"] = config.get(
        "N_GMM_COMPONENTS_PER_TASK", 
        config.get("N_MIX_WEIGHTS_PER_TASK", 4)
    )

    # Scale factor
    config["Et_SCALE_FACTOR"] = config.get("Et_SCALE_FACTOR", 2.0)
    config["FINAL_Et_OVER_PERIOD_RANGE"] = config.get("FINAL_Et_OVER_PERIOD_RANGE", [0.05, 0.9])
    
    # N_CORES default logic
    if "N_CORES" not in config:
        config["N_CORES"] = 2 if config.get("MEAN_CPU_UTIL", 0.5) > 1.0 else 1
    
    # SP threshold option set
    config["SP_THRESHOLDS_SET"] = config.get("SP_THRESHOLDS_SET", [0.2, 0.4, 0.6, 0.8, 1.0])

    # Physical map dimensions override D1_RANGE
    map_w = config.get("MAP_WIDTH_M", None)
    map_h = config.get("MAP_HEIGHT_M", None)
    if map_w is not None and map_h is not None:
        d1_half = int(max(map_w, map_h) / 2.0)
        config["D1_RANGE"] = [-d1_half, d1_half]

    # Calculate D1 variance factor table
    d1_max = config.get("D1_RANGE", [-100, 100])[1]
    nn = math.ceil(d1_max)
    d1_variance_factor_tbl = [0.0] * nn
    nn_mid = int(nn * 0.75)
    
    if nn_mid > 0:
        d1_variance_factor_tbl[nn_mid] = 1.0
        dlt = config["Et_SCALE_FACTOR"] ** (1.0 / nn_mid)
        for i in range(1, nn_mid + 1):
            i1 = nn_mid - i
            if i1 >= 0:
                d1_variance_factor_tbl[i1] = dlt ** i
            i2 = nn_mid + i
            if i2 < nn:
                d1_variance_factor_tbl[i2] = 1.0
    else:
        d1_variance_factor_tbl = [1.0] * max(1, nn)

    config["D1_VARIANCE_FACTOR_TABLE"] = d1_variance_factor_tbl
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
