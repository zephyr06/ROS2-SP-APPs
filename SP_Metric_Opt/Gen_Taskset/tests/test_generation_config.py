import pytest
from Gen_Taskset.lib.generation_config_parser import standardize_config, validate_generation_config

def test_standardize_config_rejects_hz_keys():
    """Hz-style period keys are no longer supported -- standardize_config
    raises ValueError pointing at PERIODS_MS instead of aliasing."""
    raw_cfg = {
        "SMALL_PERIOD_HZ": [10, 20, 50],
        "BIG_PERIOD_HZ": [0.5, 1],
        "CPU_UTIL_RANDOM_RANGE": [0.75, 0.75],
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
        "CPU_UTIL_RANDOM_RANGE": [0.75, 0.75]
    }
    assert validate_generation_config(valid_cfg) is True

    invalid_cfg = {
        "CPU_UTIL_RANDOM_RANGE": [0.75, 0.75]
    }
    assert validate_generation_config(invalid_cfg) is False


def test_standardize_config_defaults_important_task_ratio():
    """P0.6/P0.7/P0.8: IMPORTANT_TASK_RATIO defaults to 0.5 (the resolved
    design rule -- half the tasks are important) so legacy configs that predate
    the knob still label rather than silently marking zero tasks important."""
    raw_cfg = {
        "CPU_UTIL_RANDOM_RANGE": [0.75, 0.75],
        "D1_RANGE": [-10, 10],
        "D2_RANGE": [0, 360],
    }
    out = standardize_config(raw_cfg)
    assert out["IMPORTANT_TASK_RATIO"] == 0.5


def test_standardize_config_rejects_invalid_important_task_ratio():
    """Out-of-range ratios raise rather than silently producing zero or
    all-important tasksets."""
    base = {
        "CPU_UTIL_RANDOM_RANGE": [0.75, 0.75],
        "D1_RANGE": [-10, 10],
        "D2_RANGE": [0, 360],
    }
    for bad in (0.0, -0.1, 1.1, True):
        cfg = dict(base)
        cfg["IMPORTANT_TASK_RATIO"] = bad
        with pytest.raises(ValueError):
            standardize_config(cfg)

    # valid boundaries: just above 0 and 1.0 inclusive
    for good in (0.01, 0.5, 1.0):
        cfg = dict(base)
        cfg["IMPORTANT_TASK_RATIO"] = good
        out = standardize_config(cfg)
        assert out["IMPORTANT_TASK_RATIO"] == float(good)
