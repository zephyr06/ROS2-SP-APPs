#!/usr/bin/env python3
"""Minimal reproduction: trace period selection exactly as generate_taskset_parameters does."""
import sys, os, random, numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.generation_config_parser import load_generation_config
from Gen_Taskset.lib.taskset_generator import generate_single_gaussian_task

cfgs = load_generation_config(os.path.join(PROJECT_ROOT, 'Gen_Taskset/task_sets_config/taskset_cfg_paper_6.json'))
seed = cfgs.get('RANDOM_SEED')
np.random.seed(seed)
random.seed(seed)

picked_periods = []
periods = []

print("--- Big tasks ---")
for _ in range(2):
    t = generate_single_gaussian_task(cfgs, prd_sel='big', picked_periods=picked_periods)
    periods.append(t['period'])
    print(f"period={t['period']}, picked={picked_periods}")

print("--- Small tasks ---")
for _ in range(4):
    t = generate_single_gaussian_task(cfgs, prd_sel='small', picked_periods=picked_periods)
    periods.append(t['period'])
    print(f"period={t['period']}, picked={picked_periods}")

print("\nFinal periods:", periods)
