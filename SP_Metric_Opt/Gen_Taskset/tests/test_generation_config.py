import pytest
from Gen_Taskset.lib.generation_config_parser import standardize_config, validate_generation_config

def test_standardize_config_converts_hz():
    raw_cfg = {
        "SMALL_PERIOD_HZ": [10, 20, 50],
        "BIG_PERIOD_HZ": [0.5, 1],
        "MEAN_CPU_UTIL": 0.75,
        "D1_RANGE": [-10, 10],
        "D2_RANGE": [0, 360]
    }
    std_cfg = standardize_config(raw_cfg)
    assert std_cfg["SMALL_PERIODS_MS"] == [100, 50, 20]
    assert std_cfg["BIG_PERIODS_MS"] == [2000, 1000]
    assert std_cfg["N_TASKS"] == 10  # 2 + 8 default

def test_validate_config():
    valid_cfg = {
        "D1_RANGE": [-10, 10],
        "D2_RANGE": [0, 360],
        "MEAN_CPU_UTIL": 0.75
    }
    assert validate_generation_config(valid_cfg) is True
    
    invalid_cfg = {
        "MEAN_CPU_UTIL": 0.75
    }
    assert validate_generation_config(invalid_cfg) is False
