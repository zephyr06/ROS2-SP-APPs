import pytest
import os
import tempfile
import json
import yaml
import numpy as np
from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline

def test_integration_pipeline():
    # Setup dedicated test_output directory
    test_output_root = os.path.join(os.path.dirname(__file__), "test_output")
    os.makedirs(test_output_root, exist_ok=True)
    output_dir = os.path.join(test_output_root, "integration_test_output")
    
    import shutil
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    
    config_data = {
        "SMALL_PERIOD_HZ": [10, 20, 50],
        "BIG_PERIOD_HZ": [0.5, 1],
        "N_BIG_PERIOD_TASKS": 1,
        "N_SMALL_PERIOD_TASKS": 2,
        "N_ENV_DEPENDENT_TASKS": 1,
        "MIN_PERIOD_WITH_PERFORMANCE_RECORDS": 33,
        "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
        "SIGMA_OVER_Et_RANGE": [0.2, 0.4],
        "RO_1_Et_RANGE": [-0.9, -0.7],
        "RO_2_Et_RANGE": [-0.1, 0.1],
        "D1_RANGE": [-10, 10],
        "D2_RANGE": [0, 360],
        "N_GMM_COMPONENTS_PER_TASK": 2,
        "SP_THRESHOLD_RANGE": [0.5, 0.9],
        "SP_THRESHOLDS_SET": [0.2, 0.4, 0.6, 0.8, 1.0],
        "MEAN_CPU_UTIL": 0.5,
        "Et_SCALE_FACTOR": 1.5,
        "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
        "SP_WEIGHTS_SUM": 10.0,      # Custom weight sum limit
        "ROBOT_STEP_SIZE": 3.0       # Custom step size
    }
    config_path = os.path.join(output_dir, "test_config.json")
    with open(config_path, "w") as f:
        json.dump(config_data, f)
        
    # Execute the orchestrator full pipeline
    run_full_generation_pipeline(
        cfg_file=config_path,
        n_sec=10, # Short duration for fast integration tests
        dir_path=output_dir,
        add_perf_records=True,
        interact=False,
        n_path_per_task=1,
        n_inst_per_path=1
    )
    
    # Verify that output files exist and are non-empty
    assert os.path.exists(os.path.join(output_dir, "taskset_param.yaml"))
    assert os.path.exists(os.path.join(output_dir, "taskset_characteristics.yaml"))
    assert os.path.exists(os.path.join(output_dir, "taskset_characteristics_0.yaml"))
    assert os.path.exists(os.path.join(output_dir, "path_0.png"))
    assert os.path.exists(os.path.join(output_dir, "cpu_util.png"))
    
    # Load and validate taskset_characteristics.yaml content
    with open(os.path.join(output_dir, "taskset_characteristics.yaml"), "r") as f:
        char_data = yaml.safe_load(f)
        
    assert len(char_data["tasks"]) == 3
    
    # 1. Verify weights normalization (sum of sp_weight should equal SP_WEIGHTS_SUM)
    total_weight = sum(t["sp_weight"] for t in char_data["tasks"])
    assert abs(total_weight - config_data["SP_WEIGHTS_SUM"]) < 1e-5
    
    # 2. Verify candidate selection and soft task properties
    soft_tasks = []
    for t in char_data["tasks"]:
        # All tasks must have valid execution bounds
        assert t["execution_time_min"] >= 1.0
        assert t["execution_time_max"] >= t["execution_time_min"]
        assert 0.5 * t["period"] <= t["deadline"] <= t["period"]
        
        if "performance_records_time" in t:
            soft_tasks.append(t)
            
    # We requested 1 performance record task
    assert len(soft_tasks) == 1
    soft_task = soft_tasks[0]
    
    # Soft task period must be >= MIN_PERIOD_WITH_PERFORMANCE_RECORDS
    assert soft_task["period"] >= config_data["MIN_PERIOD_WITH_PERFORMANCE_RECORDS"]
    
    # Verify format of performance records strings
    time_rec = [float(x) for x in soft_task["performance_records_time"].split()]
    perf_rec = [float(x) for x in soft_task["performance_records_perf"].split()]
    
    assert len(time_rec) == len(perf_rec)
    assert len(time_rec) >= 10
    # Time record should be strictly increasing
    for idx in range(1, len(time_rec)):
        assert time_rec[idx] > time_rec[idx - 1]
        
    # Verify trace files exist and satisfy constraints
    for task_idx, t in enumerate(char_data["tasks"]):
        period = t["period"]
        trace_path = os.path.join(output_dir, f"path_Et_task_{task_idx}_0_0.txt")
        assert os.path.exists(trace_path)
        
        with open(trace_path, "r") as tf:
            lines = tf.readlines()
            
        # Expected number of steps: 10,000 ms / period
        expected_steps = int(10000 / period)
        assert len(lines) == expected_steps
        
        prev_x, prev_y = None, None
        for line in lines:
            parts = line.strip().split(",")
            assert len(parts) == 3
            x = int(parts[0])
            y = int(parts[1])
            et = float(parts[2])
            
            # Assert coordinates are bounded by grid D1_RANGE
            assert config_data["D1_RANGE"][0] <= x <= config_data["D1_RANGE"][1]
            assert config_data["D1_RANGE"][0] <= y <= config_data["D1_RANGE"][1]
            
            assert et >= 1.0
            
            # Assert movement steps do not exceed ROBOT_STEP_SIZE
            if prev_x is not None and prev_y is not None:
                # The change can only be at most ROBOT_STEP_SIZE in X or Y
                assert abs(x - prev_x) <= config_data["ROBOT_STEP_SIZE"]
                assert abs(y - prev_y) <= config_data["ROBOT_STEP_SIZE"]
                
            prev_x, prev_y = x, y
