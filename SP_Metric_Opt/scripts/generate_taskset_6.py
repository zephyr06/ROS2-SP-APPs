#!/usr/bin/env python3
"""Generate a single taskset for the 6-task paper configuration."""
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline

CFG_FILE = os.path.join(PROJECT_ROOT, "Gen_Taskset/task_sets_config/taskset_cfg_paper_6.json")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "test_taskset_6_gen_v2")
N_SEC = 1000
N_PATH_PER_TASK = 1
N_INST_PER_PATH = 1

print(f"Generating 6-task paper config...")
print(f"  Config: {CFG_FILE}")
print(f"  Output: {OUTPUT_DIR}")
print(f"  Sim time: {N_SEC}s")

run_full_generation_pipeline(
    cfg_file=CFG_FILE,
    n_sec=N_SEC,
    dir_path=OUTPUT_DIR,
    add_perf_records=True,
    interact=False,
    n_path_per_task=N_PATH_PER_TASK,
    n_inst_per_path=N_INST_PER_PATH,
)

print(f"Done. Output saved to: {OUTPUT_DIR}")
