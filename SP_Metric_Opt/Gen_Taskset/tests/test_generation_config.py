import pytest
from Gen_Taskset.lib.generation_config_parser import standardize_config, validate_generation_config

def test_standardize_config_rejects_hz_keys():
    """Hz-style period keys are no longer supported -- standardize_config
    raises ValueError pointing at PERIODS_MS instead of aliasing."""
    raw_cfg = {
        "SMALL_PERIOD_HZ": [10, 20, 50],
        "BIG_PERIOD_HZ": [0.5, 1],
        "MEAN_CPU_UTIL": 0.75,
        "D1_RANGE": [-10, 10],
        "D2_RANGE": [0, 360]
    }
    with pytest.raises(ValueError) as exc:
        standardize_config(raw_cfg)
    assert "PERIODS_MS" in str(exc.value)

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
