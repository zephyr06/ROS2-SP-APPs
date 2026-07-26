import pytest
import numpy as np
import random
import glob
import os
import json
import yaml
import tempfile
from Gen_Taskset.lib.generation_config_parser import standardize_config, load_generation_config
from Gen_Taskset.lib.taskset_generator import generate_taskset_parameters
from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline, _compute_hyper_period

def test_config_specifications_validation():
    # Setup config with exact specifications (canonical P19 keys: PERIODS_MS +
    # N_TASKS; Hz/split aliases were removed and now raise).
    cfgs = {
        "PERIODS_MS": [2000, 1000, 100, 50, 20],
        "N_TASKS": 6,
        "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
        "SIGMA_OVER_Et_RANGE": [0.2, 0.4],
        "RO_1_Et_RANGE": [-0.9, -0.7],
        "RO_2_Et_RANGE": [-0.1, 0.1],
        "D1_RANGE": [-20, 20],
        "D2_RANGE": [0, 360],
        "N_GMM_COMPONENTS_PER_TASK": 2,
        "CPU_UTIL_RANDOM_RANGE": [1.6, 1.6],
        "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
        "N_CORES": 2,
        "RANDOM_SEED": 42,
        "MAX_UTIL_PER_TASK": 0.95,
        "MIN_PERIOD_ENV_DEPENDENT": 0,
        "SP_THRESHOLD_RANGE": [0.5, 0.9],
        "PERF_RECORD_TASK_PROBABILITY": 0.5,
        "FIXED_TASK_SIGMA_RATIO": 0.001,
        "MAX_TIME_LIMIT_OPTIONS": 10,
        "SP_WEIGHTS_SUM": 5.0
    }

    # Canonical keys pass through standardize_config unchanged.
    cfgs_std = standardize_config(cfgs.copy())
    assert cfgs_std["PERIODS_MS"] == [2000, 1000, 100, 50, 20]
    assert cfgs_std["N_TASKS"] == 6
    
    # Test Seeding & Determinism (Generating twice with same seed yields identical parameters)
    res1 = generate_taskset_parameters(cfgs.copy())
    res2 = generate_taskset_parameters(cfgs.copy())
    
    assert len(res1["tasks"]) == res1["n_tasks"]
    assert len(res2["tasks"]) == res2["n_tasks"]
    
    # Assert identity of periods, thresholds and execution times between res1 and res2 (Determinism check)
    for t1, t2 in zip(res1["tasks"], res2["tasks"]):
        assert t1["period"] == t2["period"]
        assert t1["sp_threshold"] == t2["sp_threshold"]
        assert abs(t1["Et_mean"] - t2["Et_mean"]) < 1e-7
        assert abs(t1["Et_sigma"] - t2["Et_sigma"]) < 1e-7
        assert t1["processorId"] == t2["processorId"]
        
    # Verify internal mathematical and structural consistency of res1 (derived directly from the generated taskset)
    import math
    
    for t in res1["tasks"]:
        # 1. Periods & counts consistency
        assert isinstance(t["period"], int)
        assert t["period"] > 0
        
        # 2. Grid limits and spatial sigma consistency
        assert t["D1_MAX"] > t["D1_MIN"]
        assert abs(t["D1_sigma"] - (t["D1_MAX"] - t["D1_MIN"]) / 4.0) < 1e-7
        assert t["D2_MAX"] > t["D2_MIN"]
        assert abs(t["D2_sigma"] - (t["D2_MAX"] - t["D2_MIN"]) / 4.0) < 1e-7
        
        # 3. Component list length and weights normalization consistency
        assert t["n_weights"] == len(t["tasks"])
        assert t["n_weights"] == len(t["weights"])
        assert abs(sum(t["weights"]) - 1.0) < 1e-7
        assert all(w > 0.0 for w in t["weights"])
        
        # 4. GMM spatial component means midpoint consistency
        for c in t["tasks"]:
            assert abs(c["coeffs"]["mu1"][0] - (t["D1_MAX"] + t["D1_MIN"]) / 2.0) < 1e-7
            assert abs(c["coeffs"]["mu1"][1] - (t["D2_MAX"] + t["D2_MIN"]) / 2.0) < 1e-7
            
            # GMM component execution time mean and sigma bounds
            assert c["Et_mean"] > 0
            assert c["Et_sigma"] > 0
            
        # 5. SP constraints boundaries consistency
        assert 0.0 <= t["sp_threshold"] <= 1.0
        assert t["sp_weight"] > 0.0

        # 6. GMM Mixture Execution Time mean consistency
        expected_mean = sum(c["Et_mean"] * w for c, w in zip(t["tasks"], t["weights"]))
        assert abs(t["Et_mean"] - expected_mean) < 1e-7

        # 7. GMM Mixture Execution Time standard deviation consistency
        var_sum = 0.0
        for c, w in zip(t["tasks"], t["weights"]):
            var_sum += (c["Et_sigma"] ** 2) * w
            var_sum += ((c["Et_mean"] - t["Et_mean"]) ** 2) * w
        expected_sigma = math.sqrt(max(0.0, var_sum))
        assert abs(t["Et_sigma"] - expected_sigma) < 1e-7

    # 5b. Total weight sum consistency
    expected_weight_sum = cfgs["SP_WEIGHTS_SUM"]
    assert abs(sum(t["sp_weight"] for t in res1["tasks"]) - expected_weight_sum) < 1e-4

    # 8. Processor assignments and cores range consistency
    processor_ids = [t["processorId"] for t in res1["tasks"]]
    assert all(isinstance(pid, int) for pid in processor_ids)
    max_pid = max(processor_ids)
    assert all(0 <= pid <= max_pid for pid in processor_ids)
    
    # 9. Total taskset utilization consistency with reported cpu_util
    total_scaled_util = sum(t["Et_mean"] / t["period"] for t in res1["tasks"])
    assert abs(total_scaled_util - res1["cpu_util"]) < 1e-5

    # 11. Verify FINAL_Et_OVER_PERIOD_RANGE configuration is present
    assert cfgs_std["FINAL_Et_OVER_PERIOD_RANGE"] == cfgs["FINAL_Et_OVER_PERIOD_RANGE"]

def test_hz_key_rejected():
    # The legacy single HZ list is no longer supported -- it used to be split
    # at 10 Hz (<10 -> big, >=10 -> small) and aliased to PERIODS_MS. It now
    # raises ValueError pointing the user at PERIODS_MS.
    cfgs = {
        "HZ": [0.5, 2.0, 10, 25],
        "D1_RANGE": [-10, 10],
        "D2_RANGE": [0, 360],
        "CPU_UTIL_RANDOM_RANGE": [0.5, 0.5]
    }
    with pytest.raises(ValueError) as exc:
        standardize_config(cfgs)
    assert "PERIODS_MS" in str(exc.value)

# Parameterize over stable test configs (NOT paper configs, which change frequently)
CONFIG_FILES = glob.glob(os.path.join(os.path.dirname(__file__), "test_configs/*.json"))

@pytest.mark.parametrize("config_path", CONFIG_FILES)
def test_all_configurations_specifications(config_path):
    # Load config and verify it parses successfully
    raw_config = load_generation_config(config_path)
    cfgs = standardize_config(raw_config)
    
    # 1. Verify basic fields
    assert "D1_RANGE" in cfgs
    assert "D2_RANGE" in cfgs
    assert "CPU_UTIL_RANDOM_RANGE" in cfgs

    # P19: N_TASKS is the sole task-count input (the big/small split is gone).
    expected_tasks_count = cfgs["N_TASKS"]

    # 2. Run generation under a dedicated test_output folder to check serialization and traces
    test_output_root = os.path.join(os.path.dirname(__file__), "test_output")
    os.makedirs(test_output_root, exist_ok=True)
    config_name = os.path.splitext(os.path.basename(config_path))[0]
    config_output_dir = os.path.join(test_output_root, config_name)

    import shutil
    if os.path.exists(config_output_dir):
        shutil.rmtree(config_output_dir)
    os.makedirs(config_output_dir, exist_ok=True)

    # Run generation: n_sec must cover at least 2 hyper-periods.
    # Compute required time from periods in the already-standardized config
    # (P19: single unified PERIODS_MS pool).
    periods_ms = [int(p) for p in cfgs["PERIODS_MS"]]
    hp_ms = _compute_hyper_period(periods_ms)
    required_n_sec = max(2 * hp_ms // 1000, 10)  # at least 2× hyper-period, or 10s

    run_full_generation_pipeline(
        cfg_file=config_path,
        n_sec=required_n_sec,
        dir_path=config_output_dir,
        add_perf_records=True,
        interact=False,
        n_path_per_task=1,
        n_inst_per_path=1
    )
    
    # Load the generated characteristics to inspect outputs
    char_fpath = os.path.join(config_output_dir, "taskset_characteristics.yaml")
    assert os.path.exists(char_fpath)
    with open(char_fpath, "r") as f:
        char_data = yaml.safe_load(f)
        
    assert len(char_data["tasks"]) == expected_tasks_count
    
    # Verify weight normalization sum
    total_weights_sum = sum(t["sp_weight"] for t in char_data["tasks"])
    expected_weights_sum = cfgs["SP_WEIGHTS_SUM"]
    assert abs(total_weights_sum - expected_weights_sum) < 1e-4
    
    # Verify task parameters and core allocation bounds
    n_cores = cfgs["N_CORES"]
    for t in char_data["tasks"]:
        assert 0 <= t["processorId"] < n_cores
        assert 0.5 * t["period"] <= t["deadline"] <= t["period"]
        
        # Verify SP thresholds bounds (P2.14: SP_THRESHOLDS_SET removed;
        # sp_threshold is sampled uniformly from SP_THRESHOLD_RANGE).
        trd_range = cfgs.get("SP_THRESHOLD_RANGE", [0.5, 0.9])
        assert trd_range[0] <= t["sp_threshold"] <= trd_range[1]
            
    # Verify trace files constraints
    step_size = cfgs.get("ROBOT_STEP_SIZE", 5.0)
    for idx, t in enumerate(char_data["tasks"]):
        period = t["period"]
        trace_path = os.path.join(config_output_dir, f"path_Et_task_{idx}_0_0.txt")
        assert os.path.exists(trace_path)
        
        with open(trace_path, "r") as tf:
            lines = tf.readlines()
            
        expected_steps = int(required_n_sec * 1000 / period)
        assert len(lines) == expected_steps
        
        prev_x, prev_y = None, None
        for line in lines:
            parts = line.strip().split(",")
            assert len(parts) == 3
            x = int(parts[0])
            y = int(parts[1])
            et = float(parts[2])
            
            # Check spatial grid boundaries mapping
            assert cfgs["D1_RANGE"][0] <= x <= cfgs["D1_RANGE"][1]
            assert cfgs["D2_RANGE"][0] <= y <= cfgs["D2_RANGE"][1]
            
            # Verify that robot coordinates move by at most ROBOT_STEP_SIZE
            if prev_x is not None and prev_y is not None:
                assert abs(x - prev_x) <= step_size
                assert abs(y - prev_y) <= step_size
                
            prev_x, prev_y = x, y
