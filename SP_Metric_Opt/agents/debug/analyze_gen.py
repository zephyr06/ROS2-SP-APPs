#!/usr/bin/env python3
import os, yaml, numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
DD = os.path.join(PROJECT_ROOT, 'test_taskset_6_gen_v2')

with open(os.path.join(DD, 'taskset_param.yaml')) as f:
    data = yaml.safe_load(f)

periods = [t['period'] for t in data['tasks']]
env_flags = [t.get('env_dependent', False) for t in data['tasks']]
target_means = [t['Et_mean'] for t in data['tasks']]

print(f"Env tasks: {sum(env_flags)} / {len(env_flags)}")

# Compute per-interval utilization
UPDATE_INTERVAL_S = 10
total_util = 0.0
interval_utils = None

for i in range(len(data['tasks'])):
    fp = os.path.join(DD, f'path_Et_task_{i}_0_0.txt')
    with open(fp) as f:
        vals = [float(line.strip().split(',')[-1]) for line in f]
    period = periods[i]
    n_steps = len(vals)
    n_intervals = n_steps * period // (1000 * UPDATE_INTERVAL_S)
    if interval_utils is None:
        interval_utils = [0.0] * max(1, n_intervals)
    for step_idx, et in enumerate(vals):
        interval_idx = step_idx * period // (1000 * UPDATE_INTERVAL_S)
        if interval_idx < len(interval_utils):
            interval_utils[interval_idx] += et

n_cores = 2
for idx in range(len(interval_utils)):
    interval_utils[idx] = (interval_utils[idx] / (n_cores * 1000 * UPDATE_INTERVAL_S)) * 100.0

print(f"Utilization range: [{min(interval_utils):.1f}%, {max(interval_utils):.1f}%]")
print(f"Mean utilization: {np.mean(interval_utils):.1f}%")
print(f"Std utilization: {np.std(interval_utils):.1f}%")

# Print each task
for i in range(len(data['tasks'])):
    fp = os.path.join(DD, f'path_Et_task_{i}_0_0.txt')
    with open(fp) as f:
        vals = [float(line.strip().split(',')[-1]) for line in f]
    mean_et = np.mean(vals)
    print(f"Task {i}: env={env_flags[i]}, period={periods[i]}, target={target_means[i]:.2f}, actual={mean_et:.2f}")
