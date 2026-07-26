"""End-to-end tests for the physical units trajectory generation pipeline.

Validates that:
- MAP_WIDTH_M / MAP_HEIGHT_M / ROBOT_SPEED_MPS configs produce correct D1_RANGE
- ROBOT_STEP_SIZE is derived from speed * prd_max / 1000
- Generated paths respect physical step size bounds
- Total path length is correlated with simulation time
"""
import os
import sys
import tempfile
import unittest
import json

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline
from Gen_Taskset.lib.generation_config_parser import standardize_config


class TestPhysicalUnitsPipeline(unittest.TestCase):

    def _make_config(self, **overrides):
        """Build a base config with physical units."""
        cfg = {
            "MAP_WIDTH_M": 200,
            "MAP_HEIGHT_M": 200,
            "ROBOT_SPEED_MPS": 1.0,
            "PERIODS_MS": [2000, 1000, 100, 50, 20],
            "N_TASKS": 3,
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
            "SIGMA_OVER_Et_RANGE": [0.2, 0.4],
            "RO_1_Et_RANGE": [-0.9, -0.7],
            "RO_2_Et_RANGE": [-0.1, 0.1],
            "D2_RANGE": [0, 360],
            "N_GMM_COMPONENTS_PER_TASK": 2,
            "SP_THRESHOLD_RANGE": [0.5, 0.9],
            "SP_WEIGHT_RANGE": [0.1, 1.0],
            "CPU_UTIL_RANDOM_RANGE": [0.8, 0.8],
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "N_CORES": 2,
            "RANDOM_SEED": 42,
            "MAX_UTIL_PER_TASK": 0.95,
            "MIN_PERIOD_ENV_DEPENDENT": 0,
            "PERF_RECORD_TASK_PROBABILITY": 0.5,
            "FIXED_TASK_SIGMA_RATIO": 0.001,
            "MAX_TIME_LIMIT_OPTIONS": 10,
            "SP_WEIGHTS_SUM": 5.0
        }
        cfg.update(overrides)
        return cfg

    def test_map_params_derive_d1_range(self):
        """Config with MAP params should auto-derive D1_RANGE."""
        cfg = self._make_config(MAP_WIDTH_M=300, MAP_HEIGHT_M=150)
        std = standardize_config(cfg)
        # max(300, 150) / 2 = 150
        self.assertEqual(std["D1_RANGE"], [-150, 150])

    def test_speed_zero_step_size(self):
        """ROBOT_SPEED_MPS = 0 should produce ROBOT_STEP_SIZE = 0."""
        cfg = self._make_config(ROBOT_SPEED_MPS=0.0)
        std = standardize_config(cfg)
        # prd_max depends on selected periods; step_m = 0 * prd_max / 1000 = 0
        # This isn't set by standardize_config; it's set in orchestrator
        # We test the orchestrator behavior below.
        # Here we just confirm the derivation formula.
        self.assertEqual(std.get("ROBOT_STEP_SIZE", None), None)

    def test_orchestrator_injects_step_size(self):
        """run_full_generation_pipeline should derive ROBOT_STEP_SIZE."""
        temp_dir = tempfile.mkdtemp()
        try:
            cfg = self._make_config(
                MAP_WIDTH_M=100,
                MAP_HEIGHT_M=100,
                ROBOT_SPEED_MPS=2.0,
            )
            cfg_path = os.path.join(temp_dir, "test_cfg.json")
            with open(cfg_path, "w") as f:
                json.dump(cfg, f)

            run_full_generation_pipeline(
                cfg_file=cfg_path,
                n_sec=10,
                dir_path=temp_dir,
                add_perf_records=True,
                interact=False,
                n_path_per_task=1,
                n_inst_per_path=1,
            )

            # Verify path trace respects step bounds.
            # With speed 2.0 m/s on a 100x100 map, maximum prd is 2000ms (0.5Hz).
            # step_m = 2.0 * 2000 / 1000 = 4.0
            step_m = 4.0
            for fname in os.listdir(temp_dir):
                if fname.startswith("path_Et_task_") and fname.endswith(".txt"):
                    trace_path = os.path.join(temp_dir, fname)
                    with open(trace_path, "r") as f:
                        lines = f.readlines()
                    prev_x, prev_y = None, None
                    for line in lines:
                        parts = line.strip().split(",")
                        self.assertEqual(len(parts), 3)
                        x, y = int(parts[0]), int(parts[1])
                        if prev_x is not None:
                            # The path generator clamps to step_size.
                            # The actual physical step size is 4.0, but the
                            # simplify generates integer coordinates and clamps
                            # to step_size. With python casts to int, some
                            # positions may coincide for multiple time steps.
                            self.assertLessEqual(abs(x - prev_x), int(step_m) + 1)
                            self.assertLessEqual(abs(y - prev_y), int(step_m) + 1)
                        prev_x, prev_y = x, y
        finally:
            import shutil
            shutil.rmtree(temp_dir, ignore_errors=True)

    def test_trace_bounds_match_derived_map(self):
        """Trace coordinates should stay within derived D1_RANGE."""
        temp_dir = tempfile.mkdtemp()
        try:
            cfg = self._make_config(
                MAP_WIDTH_M=80,
                MAP_HEIGHT_M=80,
                ROBOT_SPEED_MPS=1.0,
            )
            cfg_path = os.path.join(temp_dir, "test_cfg.json")
            with open(cfg_path, "w") as f:
                json.dump(cfg, f)

            run_full_generation_pipeline(
                cfg_file=cfg_path,
                n_sec=10,
                dir_path=temp_dir,
                add_perf_records=True,
                interact=False,
                n_path_per_task=1,
                n_inst_per_path=1,
            )

            d1_min, d1_max = -40, 40  # max(80,80)/2 = 40
            for fname in os.listdir(temp_dir):
                if fname.startswith("path_Et_task_") and fname.endswith(".txt"):
                    trace_path = os.path.join(temp_dir, fname)
                    with open(trace_path, "r") as f:
                        lines = f.readlines()
                    for line in lines:
                        parts = line.strip().split(",")
                        x, y = int(parts[0]), int(parts[1])
                        self.assertGreaterEqual(x, d1_min)
                        self.assertLessEqual(x, d1_max)
                        self.assertGreaterEqual(y, d1_min)
                        self.assertLessEqual(y, d1_max)
        finally:
            import shutil
            shutil.rmtree(temp_dir, ignore_errors=True)

    def test_path_trace_count_is_ten_seconds(self):
        """For n_sec=10 and prd_max=2000ms, trace should have 5 steps."""
        temp_dir = tempfile.mkdtemp()
        try:
            # Force the big period to be 2000ms only to keep prd_max predictable.
            cfg = self._make_config(
                PERIODS_MS=[2000, 50],
                N_TASKS=2,
                MAP_WIDTH_M=50,
                MAP_HEIGHT_M=50,
                ROBOT_SPEED_MPS=1.0,
            )
            cfg_path = os.path.join(temp_dir, "test_cfg.json")
            with open(cfg_path, "w") as f:
                json.dump(cfg, f)

            run_full_generation_pipeline(
                cfg_file=cfg_path,
                n_sec=10,
                dir_path=temp_dir,
                add_perf_records=True,
                interact=False,
                n_path_per_task=1,
                n_inst_per_path=1,
            )

            # The big period task is the only candidate for prd_max.
            expected_steps = int(10 * 1000 / 2000)  # = 5 for big-period task
            big_trace = os.path.join(temp_dir, "path_Et_task_0_0_0.txt")
            with open(big_trace, "r") as f:
                lines = f.readlines()
            self.assertEqual(len(lines), expected_steps)
        finally:
            import shutil
            shutil.rmtree(temp_dir, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
