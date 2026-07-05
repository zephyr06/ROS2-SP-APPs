"""End-to-end tests for the simulation generation pipeline.

These tests exercise run_full_generation_pipeline with physical-unit configs
to verify that the generated paths and traces are realistic and well-formed.
"""
import os
import sys
import math
import json
import shutil
import tempfile
import unittest

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.orchestrator import run_full_generation_pipeline, _compute_hyper_period, validate_trajectory_config


class TestSimulationPipeline(unittest.TestCase):
    """High-level tests for the full taskset generation pipeline."""

    def _make_config(self, **overrides):
        """Build a base config with physical units."""
        cfg = {
            "MAP_WIDTH_M": 100,
            "MAP_HEIGHT_M": 100,
            "ROBOT_SPEED_MPS": 1.0,
            "PERIODS_MS": [20, 1000],
            "N_TASKS": 2,
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3],
            "SIGMA_OVER_Et_RANGE": [0.2, 0.4],
            "RO_1_Et_RANGE": [-0.9, -0.7],
            "RO_2_Et_RANGE": [-0.1, 0.1],
            "D2_RANGE": [0, 360],
            "N_GMM_COMPONENTS_PER_TASK": 2,
            "SP_THRESHOLD_RANGE": [0.5, 0.9],
            "SP_THRESHOLDS_SET": [0.2, 0.4, 0.6, 0.8, 1.0],
            "CPU_UTIL_RANDOM_RANGE": [0.8, 0.8],
            "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "N_CORES": 1,
            "RANDOM_SEED": 42,
            "MAX_UTIL_PER_TASK": 0.95,
            "MIN_PERIOD_ENV_DEPENDENT": 0,
            "PERF_RECORD_TASK_PROBABILITY": 0.5,
            "FIXED_TASK_SIGMA_RATIO": 0.001,
            "MAX_TIME_LIMIT_OPTIONS": 10,
            "SP_WEIGHTS_SUM": 5.0,
        }
        cfg.update(overrides)
        return cfg

    def _run_pipeline(self, cfg, n_sec=100, n_path=1, n_inst=1):
        """Helper that runs the pipeline in a temp dir and returns the dir."""
        temp_dir = tempfile.mkdtemp()
        cfg_path = os.path.join(temp_dir, "cfg.json")
        with open(cfg_path, "w") as f:
            json.dump(cfg, f)
        run_full_generation_pipeline(
            cfg_file=cfg_path,
            n_sec=n_sec,
            dir_path=temp_dir,
            add_perf_records=True,
            interact=False,
            n_path_per_task=n_path,
            n_inst_per_path=n_inst,
        )
        return temp_dir

    def test_generated_path_spans_large_fraction_of_map(self):
        """On a 100×100 m map with 1 m/s for 100 s, the path should span > 10 m."""
        cfg = self._make_config(MAP_WIDTH_M=100, MAP_HEIGHT_M=100, ROBOT_SPEED_MPS=1.0)
        temp_dir = self._run_pipeline(cfg, n_sec=100, n_path=1, n_inst=1)
        try:
            # Parse the path_0.png (image) – too heavy; instead read Et trace
            # which records (x, y, et) per step.
            et_files = [f for f in os.listdir(temp_dir) if f.startswith("path_Et_task_")]
            self.assertTrue(len(et_files) > 0, "No execution-time trace files generated")

            xs, ys = [], []
            for fname in et_files:
                with open(os.path.join(temp_dir, fname), "r") as f:
                    for line in f:
                        parts = line.strip().split(",")
                        self.assertEqual(len(parts), 3)
                        xs.append(int(parts[0]))
                        ys.append(int(parts[1]))

            span_x = max(xs) - min(xs)
            span_y = max(ys) - min(ys)
            self.assertGreater(max(span_x, span_y), 10,
                f"Path too confined: span_x={span_x}, span_y={span_y}")
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)

    def test_higher_speed_gives_larger_span(self):
        """Doubling speed should not decrease path span."""
        cfg_slow = self._make_config(MAP_WIDTH_M=100, MAP_HEIGHT_M=100, ROBOT_SPEED_MPS=0.5)
        cfg_fast = self._make_config(MAP_WIDTH_M=100, MAP_HEIGHT_M=100, ROBOT_SPEED_MPS=2.0)

        def get_span(cfg):
            temp_dir = self._run_pipeline(cfg, n_sec=100, n_path=1, n_inst=1)
            try:
                et_files = [f for f in os.listdir(temp_dir) if f.startswith("path_Et_task_")]
                xs, ys = [], []
                for fname in et_files:
                    with open(os.path.join(temp_dir, fname), "r") as f:
                        for line in f:
                            parts = line.strip().split(",")
                            xs.append(int(parts[0]))
                            ys.append(int(parts[1]))
                return max(xs) - min(xs), max(ys) - min(ys)
            finally:
                shutil.rmtree(temp_dir, ignore_errors=True)

        span_x_slow, span_y_slow = get_span(cfg_slow)
        span_x_fast, span_y_fast = get_span(cfg_fast)
        self.assertGreaterEqual(
            max(span_x_fast, span_y_fast),
            max(span_x_slow, span_y_slow),
            "Faster speed produced a smaller span than slower speed"
        )

    def test_trace_files_have_valid_format(self):
        """All path_Et files should be parseable int,int,float lines."""
        cfg = self._make_config()
        temp_dir = self._run_pipeline(cfg, n_sec=50, n_path=1, n_inst=1)
        try:
            et_files = [f for f in os.listdir(temp_dir) if f.startswith("path_Et_task_")]
            self.assertTrue(len(et_files) > 0)
            for fname in et_files:
                with open(os.path.join(temp_dir, fname), "r") as f:
                    for line in f:
                        parts = line.strip().split(",")
                        self.assertEqual(len(parts), 3)
                        int(parts[0])  # x
                        int(parts[1])  # y
                        float(parts[2])  # et
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)

    def test_taskset_characteristics_files_exist(self):
        """Expected YAML output files should exist after pipeline."""
        cfg = self._make_config()
        temp_dir = self._run_pipeline(cfg, n_sec=50, n_path=1, n_inst=1)
        try:
            self.assertTrue(os.path.exists(os.path.join(temp_dir, "taskset_param.yaml")))
            self.assertTrue(os.path.exists(os.path.join(temp_dir, "taskset_characteristics.yaml")))
            # At least one interval file
            interval_files = [f for f in os.listdir(temp_dir) if f.startswith("taskset_characteristics_interval_") and f.endswith(".yaml")]
            self.assertTrue(len(interval_files) > 0)
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)

    def test_multiple_paths_generate_multiple_images(self):
        """n_path=3 should yield path_0.png, path_1.png, path_2.png."""
        cfg = self._make_config()
        temp_dir = self._run_pipeline(cfg, n_sec=50, n_path=3, n_inst=1)
        try:
            for k in range(3):
                self.assertTrue(os.path.exists(os.path.join(temp_dir, f"path_{k}.png")))
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)

    def test_negative_origin_map_produces_traces(self):
        """Map centred at origin (e.g. -50..50) should still produce valid traces."""
        cfg = self._make_config(MAP_WIDTH_M=100, MAP_HEIGHT_M=100)
        temp_dir = self._run_pipeline(cfg, n_sec=100, n_path=1, n_inst=1)
        try:
            et_files = [f for f in os.listdir(temp_dir) if f.startswith("path_Et_task_")]
            self.assertTrue(len(et_files) > 0)
            for fname in et_files:
                with open(os.path.join(temp_dir, fname), "r") as f:
                    for line in f:
                        parts = line.strip().split(",")
                        x = int(parts[0])
                        y = int(parts[1])
                        self.assertGreaterEqual(x, -50)
                        self.assertLessEqual(x, 50)
                        self.assertGreaterEqual(y, -50)
                        self.assertLessEqual(y, 50)
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)


    # --- Hyper-period validation tests ---

    def test_hyper_period_computation(self):
        """LCM of periods must match expected values."""
        self.assertEqual(_compute_hyper_period([10, 20, 30]), 60)
        self.assertEqual(_compute_hyper_period([7, 13]), 91)
        self.assertEqual(_compute_hyper_period([100]), 100)
        self.assertEqual(_compute_hyper_period([100, 200, 50]), 200)

    def test_pipeline_rejects_too_short_sim_time(self):
        """n_sec < 2*hyper_period must raise ValueError before generation."""
        cfg = self._make_config(
            PERIODS_MS=[20, 1000],      # periods 20 ms + 1000 ms
            N_TASKS=2,
            CPU_UTIL_RANDOM_RANGE=[0.5, 0.5],
        )
        temp_dir = tempfile.mkdtemp()
        try:
            cfg_path = os.path.join(temp_dir, "cfg.json")
            with open(cfg_path, "w") as f:
                json.dump(cfg, f)

            # hyper_period = lcm(20, 1000) = 1000 ms
            # required = 2 * 1000 = 2000 ms = 2 s
            with self.assertRaises(ValueError) as ctx:
                run_full_generation_pipeline(
                    cfg_file=cfg_path,
                    n_sec=1,  # 1000 ms < 2000 ms
                    dir_path=temp_dir,
                    n_path_per_task=1,
                    n_inst_per_path=1,
                )
            msg = str(ctx.exception)
            self.assertIn("2× the hyper-period", msg)
            self.assertIn("1000ms", msg)
            self.assertIn("Increase n_sec to >= 2s", msg)
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)

    def test_pipeline_accepts_minimum_valid_sim_time(self):
        """n_sec exactly == 2*hyper_period (in seconds) must succeed."""
        cfg = self._make_config(
            PERIODS_MS=[20, 1000],      # periods 20 ms + 1000 ms
            N_TASKS=2,
            CPU_UTIL_RANDOM_RANGE=[0.5, 0.5],
        )
        temp_dir = tempfile.mkdtemp()
        try:
            cfg_path = os.path.join(temp_dir, "cfg.json")
            with open(cfg_path, "w") as f:
                json.dump(cfg, f)

            # required = 2 s; n_sec = 2 should succeed
            run_full_generation_pipeline(
                cfg_file=cfg_path,
                n_sec=2,
                dir_path=temp_dir,
                n_path_per_task=1,
                n_inst_per_path=1,
            )
            self.assertTrue(os.path.exists(os.path.join(temp_dir, "taskset_param.yaml")))
            self.assertTrue(os.path.exists(os.path.join(temp_dir, "taskset_characteristics.yaml")))
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)


class TestTrajectoryConfigGate(unittest.TestCase):
    """Tests for the trajectory-layer 2nd integrity gate (validate_trajectory_config).

    The generation gate (validate_config_integrity) covers GMM/ET/SP/period
    params read by generate_taskset_parameters. The orchestrator reads an
    additional, smaller set for path/trace generation (ROBOT_SPEED_MPS) that the
    generation gate does not cover; this 2nd gate ensures a forgotten
    ROBOT_SPEED_MPS cannot silently fall back to 1.0 m/s.
    """

    def setUp(self):
        # These tests run under pytest (no TTY stdin), so the gate takes the
        # non-interactive raise path -- exactly the behavior under test.
        self.base_cfg = {
            "MAP_WIDTH_M": 100, "MAP_HEIGHT_M": 100,
            "ROBOT_SPEED_MPS": 1.0,
            "PERIODS_MS": [20, 1000], "N_TASKS": 2,
            "Et_OVER_PERIOD_RANGE": [0.1, 0.3], "SIGMA_OVER_Et_RANGE": [0.2, 0.4],
            "RO_1_Et_RANGE": [-0.9, -0.7], "RO_2_Et_RANGE": [-0.1, 0.1],
            "D2_RANGE": [0, 360], "N_GMM_COMPONENTS_PER_TASK": 2,
            "SP_THRESHOLD_RANGE": [0.5, 0.9], "SP_THRESHOLDS_SET": [0.2, 0.4, 0.6, 0.8, 1.0],
            "CPU_UTIL_RANDOM_RANGE": [0.8, 0.8], "FINAL_Et_OVER_PERIOD_RANGE": [0.05, 0.9],
            "N_CORES": 1, "RANDOM_SEED": 42, "MAX_UTIL_PER_TASK": 0.95,
            "MIN_PERIOD_ENV_DEPENDENT": 0, "PERF_RECORD_TASK_PROBABILITY": 0.5,
            "FIXED_TASK_SIGMA_RATIO": 0.001, "MAX_TIME_LIMIT_OPTIONS": 10,
            "SP_WEIGHTS_SUM": 5.0,
        }

    def test_complete_config_passes_unchanged(self):
        """A config that sets ROBOT_SPEED_MPS passes the gate and is unmodified."""
        cfg = dict(self.base_cfg)
        result = validate_trajectory_config(cfg, config_path=None)
        self.assertIs(result, cfg)
        self.assertEqual(result["ROBOT_SPEED_MPS"], 1.0)

    def test_missing_robot_speed_raises_non_interactive(self):
        """A config omitting ROBOT_SPEED_MPS raises with a clear, actionable message."""
        cfg = dict(self.base_cfg)
        del cfg["ROBOT_SPEED_MPS"]
        with self.assertRaises(ValueError) as ctx:
            validate_trajectory_config(cfg, config_path=None)
        msg = str(ctx.exception)
        self.assertIn("Trajectory config is missing required parameters", msg)
        self.assertIn("ROBOT_SPEED_MPS", msg)
        self.assertIn("suggested: 1.0", msg)

    def test_pipeline_rejects_missing_robot_speed_before_generation(self):
        """run_full_generation_pipeline must fail fast on a missing ROBOT_SPEED_MPS,
        before any taskset is generated (so no partial output is left)."""
        cfg = dict(self.base_cfg)
        del cfg["ROBOT_SPEED_MPS"]
        temp_dir = tempfile.mkdtemp()
        try:
            cfg_path = os.path.join(temp_dir, "cfg.json")
            with open(cfg_path, "w") as f:
                json.dump(cfg, f)
            with self.assertRaises(ValueError) as ctx:
                run_full_generation_pipeline(
                    cfg_file=cfg_path, n_sec=2, dir_path=temp_dir,
                    n_path_per_task=1, n_inst_per_path=1,
                )
            self.assertIn("ROBOT_SPEED_MPS", str(ctx.exception))
            # Gate fires before generation, so no taskset_param.yaml is written.
            self.assertFalse(os.path.exists(os.path.join(temp_dir, "taskset_param.yaml")))
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
