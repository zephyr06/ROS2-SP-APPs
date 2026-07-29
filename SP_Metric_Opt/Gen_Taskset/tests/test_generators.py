import pytest
from Gen_Taskset.lib.trajectory import generate_stops_in_map, generate_path_only
from Gen_Taskset.lib.taskset_generator import generate_taskset_parameters

def test_trajectory_generation():
    cfgs = {
        "D1_RANGE": [-10, 10],
        "D2_RANGE": [0, 360],
        "CPU_UTIL_RANDOM_RANGE": [0.75, 0.75]
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
        "CPU_UTIL_RANDOM_RANGE": [0.75, 0.75],
        "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
        "SIGMA_OVER_Et_RANGE": [0.1, 0.4],
        "RO_1_Et_RANGE": [-0.9, -0.7],
        "RO_2_Et_RANGE": [-0.2, 0.2],
        "N_GMM_COMPONENTS_PER_TASK": 4,
        "SP_THRESHOLD_RANGE": [0.5, 0.9],
        "SP_WEIGHT_RANGE": [0.1, 1.0],
        "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
        "PERIODS_MS": [1000, 100, 50, 20],
        "N_TASKS": 4,
        "N_CORES": 2,
        "MAX_UTIL_PER_TASK": 0.95,
        "MIN_PERIOD_ENV_DEPENDENT": 0,
        "PERF_RECORD_TASK_PROBABILITY": 0.5,
        "FIXED_TASK_SIGMA_RATIO": 0.001,
        "MAX_TIME_LIMIT_OPTIONS": 10,
        "SP_WEIGHTS_SUM": 5.0,
        "IMPORTANT_TASK_RATIO": 0.5
    }

    taskset = generate_taskset_parameters(cfgs)
    assert taskset["n_tasks"] == 4
    assert len(taskset["tasks"]) == 4
    for task in taskset["tasks"]:
        assert len(task["tasks"]) == 4  # 4 components
        # P2.14: SP_THRESHOLDS_SET removed -- sp_threshold is sampled uniformly
        # from SP_THRESHOLD_RANGE, so assert range membership instead.
        trd = cfgs["SP_THRESHOLD_RANGE"]
        assert trd[0] <= task["sp_threshold"] <= trd[1]

    # P0.6/P0.7/P0.8: important-task labeling. The top IMPORTANT_TASK_RATIO
    # fraction by sp_weight (pre-normalization -- ratios are scale-invariant)
    # are marked important. For N=4, ratio=0.5 -> ceil(4*0.5)=2 important tasks.
    n_imp = sum(1 for t in taskset["tasks"] if t["is_important"])
    assert n_imp == 2
    # The marked tasks are exactly the top-2 by sp_weight. tasks_dict_list
    # preserves taskset_param order, so the list index is the stable identity.
    indexed = list(enumerate(taskset["tasks"]))
    by_weight = sorted(indexed, key=lambda kv: kv[1]["sp_weight"], reverse=True)
    top2_idx = {by_weight[0][0], by_weight[1][0]}
    imp_idx = {i for i, t in indexed if t["is_important"]}
    assert imp_idx == top2_idx
    # is_important is a bool on every task.
    for t in taskset["tasks"]:
        assert isinstance(t["is_important"], bool)
