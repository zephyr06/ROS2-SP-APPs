#!/usr/bin/env python3
"""Debug script to trace UUniFast generation step by step."""
import sys, os, random, numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.generation_config_parser import standardize_config

def debug_uunifast(n, target_util):
    """UUniFast with full tracing."""
    print(f"UUniFast called: n={n}, target={target_util}")
    sum_u = target_util
    vect_u = [0.0] * n
    for i in range(n - 1):
        r = random.random()
        power = 1.0 / (n - i)
        next_sum_u = sum_u * (r ** power)
        vect_u[i] = sum_u - next_sum_u
        print(f"  i={i}: rand={r:.6f}, power={power:.4f}, next_sum={next_sum_u:.6f}, vect_i={vect_u[i]:.6f}")
        sum_u = next_sum_u
    vect_u[n - 1] = sum_u
    print(f"  i={n-1}: vect_i={vect_u[n-1]:.6f}")
    print(f"  Sum before clip: {sum(vect_u):.6f}")
    for i in range(n):
        vect_u[i] = max(0.0, min(target_util, vect_u[i]))
    print(f"  Sum after clip: {sum(vect_u):.6f}")
    return vect_u

def generate_periods(cfgs):
    """Mimic period generation from taskset_generator (unified PERIODS_MS pool)."""
    picked_periods = []
    periods = []
    n_tasks = cfgs["N_TASKS"]
    periods_pool = cfgs["PERIODS_MS"]

    for _ in range(n_tasks):
        p = int(np.random.choice(periods_pool))
        if p not in picked_periods:
            picked_periods.append(p)
        periods.append(p)

    return periods, picked_periods

def main():
    config = {
        "PERIODS_MS": [100, 50, 33, 20, 1000, 500, 200],
        "N_TASKS": 6,
        "MEAN_CPU_UTIL": 1.2,
        "N_CORES": 2,
        "RANDOM_SEED": 40,
    }
    cfgs = standardize_config(config)

    seed = cfgs.get("RANDOM_SEED")
    if seed is not None:
        np.random.seed(seed)
        random.seed(seed)

    n_tasks = cfgs["N_TASKS"]
    n_cores = cfgs["N_CORES"]
    cpu_util = cfgs["MEAN_CPU_UTIL"] * n_cores
    print(f"Config: n_tasks={n_tasks}, n_cores={n_cores}, MEAN_CPU_UTIL={cfgs['MEAN_CPU_UTIL']}, cpu_util target={cpu_util}")

    # Step 1: generate periods (this code exists in generate_taskset_parameters)
    periods, picked_periods = generate_periods(cfgs)
    print(f"Periods: {periods}")
    print(f"picked_periods: {picked_periods}")
    print(f"random.getstate after periods: consumed some random state?")

    # Step 2: UUniFast
    util_vector = debug_uunifast(n_tasks, cpu_util)
    print(f"Util vector: {[round(u, 4) for u in util_vector]}")
    print(f"Util sum: {sum(util_vector):.6f}")
    print(f"Et_means: {[round(u * p, 2) for u, p in zip(util_vector, periods)]}")
    print(f"Actual util from Et_means: {sum(u * p / p for u, p in zip(util_vector, periods)):.6f}")

    # Step 3: show what the file SHOULD have
    print("\n--- Comparison with actual generated file ---")
    print("If UUniFast is correct, the sum must be exactly 2.4")
    print("If sum != 2.4, there is either:")
    print("  a) A bug in uunifast_distribution (unlikely by telescoping)")
    print("  b) Random state consumed between random.seed() and uunifast call")

if __name__ == "__main__":
    main()
