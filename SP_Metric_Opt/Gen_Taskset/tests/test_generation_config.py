import pytest
from Gen_Taskset.lib.generation_config_parser import standardize_config, validate_generation_config

def test_standardize_config_converts_hz():
    """P19: old-shape Hz configs are aliased to the unified PERIODS_MS.

    The big-pool periods come first, then the small-pool periods, preserving
    the historical composition. Hz->ms conversion: big [0.5,1]->[2000,1000],
    small [10,20,50]->[100,50,20]. N_TASKS defaults to N_BIG(2)+N_SMALL(8)=10.
    """
    raw_cfg = {
        "SMALL_PERIOD_HZ": [10, 20, 50],
        "BIG_PERIOD_HZ": [0.5, 1],
        "MEAN_CPU_UTIL": 0.75,
        "D1_RANGE": [-10, 10],
        "D2_RANGE": [0, 360]
    }
    std_cfg = standardize_config(raw_cfg)
    # Big-pool periods first, then small-pool periods
    assert std_cfg["PERIODS_MS"] == [2000, 1000, 100, 50, 20]
    assert std_cfg["N_TASKS"] == 10  # 2 + 8 default
    # Old paired keys must be deleted from the standardized config.
    for old_key in (
        "SMALL_PERIODS_MS", "BIG_PERIODS_MS",
        "SMALL_PERIOD_HZ", "BIG_PERIOD_HZ",
        "N_BIG_PERIOD_TASKS", "N_SMALL_PERIOD_TASKS",
    ):
        assert old_key not in std_cfg

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
