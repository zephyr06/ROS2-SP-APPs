#!/usr/bin/env python3
"""Validate that env-dependent tasks show real ET variation after UUniFast fix."""
import sys, os, yaml, numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.gmm_model import GaussianComponent, GMMTaskModel

PARAM_FILE = os.path.join(PROJECT_ROOT, "test_taskset_6_gen_v2", "taskset_param.yaml")

def main():
    with open(PARAM_FILE, "r") as f:
        params = yaml.safe_load(f)

    variance_factor_table = [1.0] * 101
    final_et_range = [0.05, 0.9]
    np.random.seed(42)

    print("Task | Env | Period | Et_mean | Comp0_sigma | Sample_mean | Sample_std | Sample_range")
    print("-" * 90)
    for i, task in enumerate(params['tasks']):
        components = [GaussianComponent(None, None, coeffs=c['coeffs']) for c in task['tasks']]
        model = GMMTaskModel(
            components=components, weights=task['weights'], period=task['period'],
            d1_min=task['D1_MIN'], d1_max=task['D1_MAX'], d1_sigma=task['D1_sigma'],
            d2_min=task['D2_MIN'], d2_max=task['D2_MAX'], d2_sigma=task['D2_sigma'],
            et_mean=task['Et_mean'], et_sigma=task['Et_sigma']
        )
        env = task.get('env_dependent', False)
        samples = []
        for _ in range(500):
            if env:
                r = np.random.uniform(0, 100)
                a = np.random.uniform(0, 360)
                s = model.sample_execution_time(d1=r, d2=a, variance_factor_table=variance_factor_table,
                                                final_et_range=final_et_range, et_min_2sigma=True)
            else:
                s = task['Et_mean']
            samples.append(s)
        comp0_sigma = task['tasks'][0]['Et_sigma']
        print(f"  {i}  | {'Y' if env else 'N'}  | {task['period']:6d} | {task['Et_mean']:7.2f} | {comp0_sigma:11.2f} | "
              f"{np.mean(samples):11.2f} | {np.std(samples):10.2f} | [{np.min(samples):.1f}, {np.max(samples):.1f}]")

    print("\nInterpretation:")
    print("- Env=Y tasks should have large Sample_std (visible variation).")
    print("- Env=N tasks should have Sample_std=0 (deterministic).")
    print("- If Env=Y tasks have tiny std, the component sigma is still too small.")

if __name__ == "__main__":
    main()
