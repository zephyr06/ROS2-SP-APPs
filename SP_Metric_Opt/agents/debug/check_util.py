#!/usr/bin/env python3
import os
import yaml

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
DD = os.path.join(PROJECT_ROOT, 'test_taskset_6_gen_v2')

total_util = 0.0
with open(os.path.join(DD, 'taskset_param.yaml')) as f:
    data = yaml.safe_load(f)
periods = [t['period'] for t in data['tasks']]
target_means = [t['Et_mean'] for t in data['tasks']]
env_flags = [t.get('env_dependent', False) for t in data['tasks']]

for i in [0, 1, 2, 3, 4, 5]:
    fp = os.path.join(DD, f'path_Et_task_{i}_0_0.txt')
    with open(fp) as f:
        vals = [float(line.strip().split(',')[-1]) for line in f]
    mean_et = sum(vals) / len(vals)
    period = periods[i]
    target = target_means[i]
    util = mean_et / period
    total_util += util
    print(f"Task {i}: env={env_flags[i]}, period={period}, target_mean={target:.2f}, actual_mean={mean_et:.2f}, util={util:.4f}")

print(f"\nTotal utilization: {total_util:.4f}")
print(f"Target utilization: 2.4")
