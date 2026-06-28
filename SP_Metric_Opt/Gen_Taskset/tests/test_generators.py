import pytest
from Gen_Taskset.lib.trajectory import generate_stops_in_map, generate_path_only
from Gen_Taskset.lib.taskset_generator import generate_taskset_parameters

def test_trajectory_generation():
    cfgs = {
        "D1_RANGE": [-10, 10],
        "D2_RANGE": [0, 360],
        "MEAN_CPU_UTIL": 0.75
    }
    stops = generate_stops_in_map(cfgs, x_step_ratio=0.5, y_step_ratio=0.5)
    # Stops should be generated on the grid boundary steps
    assert len(stops) > 0
    
    path = generate_path_only(cfgs, stops, n_steps=50, reverse_prob=0.1)
    assert len(path) == 50
    for x, y in path:
        assert -10 <= x <= 10
        assert 0 <= y <= 360
        
def test_taskset_parameters_generation():
    cfgs = {
        "D1_RANGE": [-100, 100],
        "D2_RANGE": [0, 360],
        "MEAN_CPU_UTIL": 0.75,
        "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
        "SIGMA_OVER_Et_RANGE": [0.1, 0.4],
        "RO_1_Et_RANGE": [-0.9, -0.7],
        "RO_2_Et_RANGE": [-0.2, 0.2],
        "N_GMM_COMPONENTS_PER_TASK": 4,
        "SP_THRESHOLD_RANGE": [0.5, 0.9],
        "SP_THRESHOLDS_SET": [0.2, 0.4, 0.6, 0.8, 1.0],
        "Et_SCALE_FACTOR": 2.0,
        "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
        "N_BIG_PERIOD_TASKS": 1,
        "N_SMALL_PERIOD_TASKS": 3
    }
    
    taskset = generate_taskset_parameters(cfgs)
    assert taskset["n_tasks"] == 4
    assert len(taskset["tasks"]) == 4
    for task in taskset["tasks"]:
        assert len(task["tasks"]) == 4  # 4 components
        assert task["sp_threshold"] in cfgs["SP_THRESHOLDS_SET"]
