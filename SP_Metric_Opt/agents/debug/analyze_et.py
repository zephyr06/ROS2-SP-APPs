#!/usr/bin/env python3
import os
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
DD = os.path.join(PROJECT_ROOT, 'test_taskset_6_gen_v2')

for i in [0, 2]:
    fp = os.path.join(DD, f'path_Et_task_{i}_0_0.txt')
    with open(fp) as f:
        lines = [line.strip().split(',') for line in f]
    x = np.array([float(l[0]) for l in lines])
    y = np.array([float(l[1]) for l in lines])
    r = np.sqrt(x**2 + y**2)
    et = np.array([float(l[2]) for l in lines])

    print(f"\nTask {i}:")
    print(f"  r range: [{r.min():.1f}, {r.max():.1f}], mean={r.mean():.1f}")
    print(f"  Et range: [{et.min():.1f}, {et.max():.1f}], mean={et.mean():.1f}, std={et.std():.1f}")
    print(f"  Et unique values count: {len(np.unique(np.round(et, 2)))}")
    print(f"  Et value counts (top 5):")
    uniq, counts = np.unique(np.round(et, 2), return_counts=True)
    idx = np.argsort(counts)[::-1][:5]
    for ii in idx:
        print(f"    {uniq[ii]}: {counts[ii]} ({100*counts[ii]/len(et):.1f}%)")
