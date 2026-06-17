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
from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline

def test_config_specifications_validation():
    # Setup config with exact specifications
    cfgs = {
        "SMALL_PERIOD_HZ": [10, 20, 50],
        "BIG_PERIOD_HZ": [0.5, 1],
        "N_BIG_PERIOD_TASKS": 2,
        "N_SMALL_PERIOD_TASKS": 4,
        "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
        "SIGMA_OVER_Et_RANGE": [0.2, 0.4],
        "RO_1_Et_RANGE": [-0.9, -0.7],
        "RO_2_Et_RANGE": [-0.1, 0.1],
        "D1_RANGE": [-20, 20],
        "D2_RANGE": [0, 360],
        "N_MIX_WEIGHTS_PER_TASK": 2,
        "SP_THRESHOLDS_SET": [0.2, 0.4, 0.6, 0.8, 1.0],
        "MEAN_CPU_UTIL": 1.6,
        "Et_SCALE_FACTOR": 1.5,
        "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
        "N_CORES": 2,
        "RANDOM_SEED": 42
    }
    
    # Test standardization conversions
    cfgs_std = standardize_config(cfgs.copy())
    assert cfgs_std["SMALL_PERIODS_MS"] == [100, 50, 20]
    assert cfgs_std["BIG_PERIODS_MS"] == [2000, 1000]
    
    # Test Seeding & Determinism (Generating twice with same seed yields identical parameters)
    res1 = generate_taskset_parameters(cfgs.copy())
    res2 = generate_taskset_parameters(cfgs.copy())
    
    assert res1["n_tasks"] == 6
    assert res2["n_tasks"] == 6
    assert abs(res1["cpu_util"] - 1.6) < 1e-7
    
    # Assert identity of periods, thresholds and execution times between res1 and res2
    for t1, t2 in zip(res1["tasks"], res2["tasks"]):
        assert t1["period"] == t2["period"]
        assert t1["sp_threshold"] == t2["sp_threshold"]
        assert abs(t1["Et_mean"] - t2["Et_mean"]) < 1e-7
        assert abs(t1["Et_sigma"] - t2["Et_sigma"]) < 1e-7
        assert t1["processorId"] == t2["processorId"]
        
    # Test Task Count and Util Scaling specifications
    total_scaled_util = 0.0
    processor_ids = set()
    for t in res1["tasks"]:
        assert t["period"] in [20, 50, 100, 1000, 2000]
        assert t["sp_threshold"] in [0.2, 0.4, 0.6, 0.8, 1.0]
        total_scaled_util += t["Et_mean"] / t["period"]
        processor_ids.add(t["processorId"])
        
    assert abs(total_scaled_util - 1.6) < 1e-5
    assert processor_ids == {0, 1}

def test_hz_key_parsing_specification():
    # Test that universal HZ key parses correctly into big/small periods
    cfgs = {
        "HZ": [0.5, 2.0, 10, 25],
        "D1_RANGE": [-10, 10],
        "D2_RANGE": [0, 360],
        "MEAN_CPU_UTIL": 0.5
    }
    cfgs_std = standardize_config(cfgs)
    assert cfgs_std["SMALL_PERIODS_MS"] == [100, 40]
    assert cfgs_std["BIG_PERIODS_MS"] == [2000, 500]

# Parameterize over all configuration json files in task_sets_config
CONFIG_FILES = glob.glob(os.path.join(os.path.dirname(__file__), "../task_sets_config/*.json"))

@pytest.mark.parametrize("config_path", CONFIG_FILES)
def test_all_configurations_specifications(config_path):
    # Load config and verify it parses successfully
    raw_config = load_generation_config(config_path)
    cfgs = standardize_config(raw_config)
    
    # 1. Verify basic fields
    assert "D1_RANGE" in cfgs
    assert "D2_RANGE" in cfgs
    assert "MEAN_CPU_UTIL" in cfgs
    
    n_big = cfgs.get("N_BIG_PERIOD_TASKS", 2)
    n_small = cfgs.get("N_SMALL_PERIOD_TASKS", 8)
    expected_tasks_count = n_big + n_small
    
    # 2. Run generation under a dedicated test_output folder to check serialization and traces
    test_output_root = os.path.join(os.path.dirname(__file__), "test_output")
    os.makedirs(test_output_root, exist_ok=True)
    config_name = os.path.splitext(os.path.basename(config_path))[0]
    config_output_dir = os.path.join(test_output_root, config_name)
    
    import shutil
    if os.path.exists(config_output_dir):
        shutil.rmtree(config_output_dir)
    os.makedirs(config_output_dir, exist_ok=True)
    
    # Run short 2-second simulation trace generation to verify files and limits
    run_full_generation_pipeline(
        cfg_file=config_path,
        n_sec=10,
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
    expected_weights_sum = cfgs.get("SP_WEIGHTS_SUM", 5.0)
    assert abs(total_weights_sum - expected_weights_sum) < 1e-4
    
    # Verify task parameters and core allocation bounds
    n_cores = cfgs.get("N_CORES", 2 if cfgs["MEAN_CPU_UTIL"] > 1.0 else 1)
    for t in char_data["tasks"]:
        assert 0 <= t["processorId"] < n_cores
        assert t["deadline"] == t["period"]
        
        # Verify SP thresholds bounds
        sp_thresholds_set = cfgs.get("SP_THRESHOLDS_SET")
        if sp_thresholds_set:
            assert t["sp_threshold"] in sp_thresholds_set
        else:
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
            
        expected_steps = int(10000 / period)
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
            assert cfgs["D1_RANGE"][0] <= y <= cfgs["D1_RANGE"][1]
            
            # Check execution time limits
            final_et_range = cfgs.get("FINAL_Et_OVER_PERIOD_RANGE", [0.05, 0.9])
            assert period * final_et_range[0] - 1e-4 <= et <= period * final_et_range[1] + 1e-4
            assert et >= 1.0
            
            # Verify that robot coordinates move by at most ROBOT_STEP_SIZE
            if prev_x is not None and prev_y is not None:
                assert abs(x - prev_x) <= step_size
                assert abs(y - prev_y) <= step_size
                
            prev_x, prev_y = x, y
