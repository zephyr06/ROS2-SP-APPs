"""Tests for simulation_experiments/interval_sweep.py -- Fig 2 + normalization."""
import json
import os
import sys
import unittest
import unittest.mock

# Ensure project root is importable when run via pytest from repo root.
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import simulation_experiments.interval_sweep as sweep


def _data():
    """Two intervals, BF + INCR + RM. All SP well below the 5.0 ideal ceiling."""
    return [
        {"interval": 5, "scheduler": "BF", "mean_sp": 2.0, "std_sp": 0.2},
        {"interval": 10, "scheduler": "BF", "mean_sp": 2.0, "std_sp": 0.2},
        {"interval": 5, "scheduler": "INCR", "mean_sp": 1.6, "std_sp": 0.1},
        {"interval": 10, "scheduler": "INCR", "mean_sp": 1.4, "std_sp": 0.1},
        {"interval": 5, "scheduler": "RM", "mean_sp": 1.0, "std_sp": 0.05},
        {"interval": 10, "scheduler": "RM", "mean_sp": 1.0, "std_sp": 0.05},
    ]


class TestNormalizeSweepData(unittest.TestCase):
    def test_divides_by_ideal_sp_ceiling(self):
        """Each SP is divided by the constant ideal-SP ceiling (5.0).

        The sweep runs at one task count, so the ceiling is a single constant,
        not a per-interval reference mean. Every point stays <= 1.0.
        """
        norm = sweep._normalize_sweep_data(_data(), ideal_sp=5.0)
        by = {(d["interval"], d["scheduler"]): d["mean_sp"] for d in norm}
        # BF at 2.0 / 5.0 = 0.4 at both intervals
        self.assertAlmostEqual(by[(5, "BF")], 0.4)
        self.assertAlmostEqual(by[(10, "BF")], 0.4)
        # INCR at interval 5: 1.6 / 5.0 = 0.32
        self.assertAlmostEqual(by[(5, "INCR")], 0.32)
        self.assertAlmostEqual(by[(10, "INCR")], 0.28)
        # std scales the same way
        std_by = {(d["interval"], d["scheduler"]): d["std_sp"] for d in norm}
        self.assertAlmostEqual(std_by[(5, "INCR")], 0.02)  # 0.1 / 5.0
        # No point exceeds 1.0 (ceiling is the upper bound).
        self.assertTrue(all(0.0 <= v <= 1.0 + 1e-9 for v in by.values()))

    def test_returns_none_when_ceiling_missing(self):
        """No ideal-SP ceiling -> normalization skipped (None -> raw kept)."""
        data = [{"interval": 5, "scheduler": "INCR", "mean_sp": 1.6, "std_sp": 0.1}]
        self.assertIsNone(sweep._normalize_sweep_data(data, ideal_sp=None))
        self.assertIsNone(sweep._normalize_sweep_data(data, ideal_sp=0.0))

    def test_raw_records_not_mutated(self):
        data = _data()
        original = [dict(d) for d in data]
        sweep._normalize_sweep_data(data, ideal_sp=5.0)
        for orig, cur in zip(original, data):
            self.assertEqual(orig, cur)


class TestIntervalSweepFigure(unittest.TestCase):
    @unittest.mock.patch("simulation_experiments.interval_sweep.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.interval_sweep.save_figure")
    def test_raw_only_when_normalize_off(self, mock_save):
        cfg = {"main_scheduler_list": ["INCR", "BF", "RM", "CFS"]}
        with unittest.mock.patch.object(sweep, "FIGSIZE_SINGLE", (8, 6)):
            sweep.generate_interval_sweep_figure(_data(), cfg, "/tmp/fig2_test", ideal_sp=5.0)
        self.assertEqual(mock_save.call_count, 1)

    @unittest.mock.patch("simulation_experiments.interval_sweep.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.interval_sweep.save_figure")
    def test_emits_raw_and_normalized_when_on(self, mock_save):
        cfg = {
            "main_scheduler_list": ["INCR", "BF", "RM", "CFS"],
            "analysis": {"normalize_sp": True,
                         "sp_normalization_method": "upper_bound"},
        }
        with unittest.mock.patch.object(sweep, "FIGSIZE_SINGLE", (8, 6)):
            sweep.generate_interval_sweep_figure(_data(), cfg, "/tmp/fig2_test", ideal_sp=5.0)
        self.assertEqual(mock_save.call_count, 2)
        stems = [c.args[1] if len(c.args) > 1 else c.kwargs.get("output_path_stem")
                 for c in mock_save.call_args_list]
        self.assertTrue(any("normalized" in s for s in stems))

    @unittest.mock.patch("simulation_experiments.interval_sweep.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.interval_sweep.save_figure")
    def test_normalized_skipped_when_no_ceiling(self, mock_save):
        """With normalize_sp on but no ceiling, only the raw figure is drawn."""
        cfg = {
            "main_scheduler_list": ["INCR", "BF", "RM", "CFS"],
            "analysis": {"normalize_sp": True,
                         "sp_normalization_method": "upper_bound"},
        }
        with unittest.mock.patch.object(sweep, "FIGSIZE_SINGLE", (8, 6)):
            sweep.generate_interval_sweep_figure(_data(), cfg, "/tmp/fig2_test", ideal_sp=None)
        self.assertEqual(mock_save.call_count, 1)  # raw only


class TestSweepReuseMatchingInterval(unittest.TestCase):
    """P20: the sweep reuses the main step's tasksets when the interval matches.

    Covers three cases:
    - matching-interval main dir present + fresh -> compare_optimizers NOT run
      (the dir is returned directly).
    - main dir absent -> compare_optimizers IS run (regeneration allowed).
    - run_single_interval forwards --on_taskset_config_change to its child.
    """

    def _write_fresh_main_dir(self, tmp, num_tasks, n_tasksets, n_sec,
                              interval_sec, base_seed):
        """Create a main-step dir whose tasksets match what the sweep expects.

        Mirrors compare_optimizers' baseline exactly: the per-taskset
        generator_config.json is the *fully-resolved* config from
        load_generation_config(resolve_taskset_config_path(num_tasks)) with
        RANDOM_SEED = base_seed + idx and UPDATE_INTERVAL_S = interval_sec
        overlaid -- the same dict compare_optimizers writes when it generates.
        ``_should_generate`` against this baseline returns False (reuse) for
        every taskset, so _main_dir_is_fresh reports fresh.
        """
        from Gen_Taskset.lib.generation_config_parser import (
            load_generation_config, resolve_taskset_config_path,
        )
        from simulation_experiments.experiment_config_loader import (
            build_experiment_dir_name,
        )
        main_dir = os.path.join(
            tmp, build_experiment_dir_name(
                num_tasks, n_sec, interval_sec, base_seed)
        )
        os.makedirs(main_dir, exist_ok=True)
        with open(os.path.join(main_dir, "comparison_summary.csv"), "w") as f:
            f.write("Scheduler,Mean_SP_Metric,Std_SP_Metric,Mean_Miss_Rate,"
                    "Std_Miss_Rate,Mean_Scheduler_Execution_Time_s,"
                    "Important_Miss_Rate,Non_Important_Miss_Rate\n")
            f.write("INCR,1.5,0.1,0.0,0.0,0.01,0.0,0.0\n")
        base_config = load_generation_config(
            resolve_taskset_config_path(num_tasks))
        for idx in range(n_tasksets):
            ts_dir = os.path.join(main_dir, f"taskset_{idx}")
            os.makedirs(ts_dir, exist_ok=True)
            # Mark the taskset as generated (interval files present) so
            # _needs_generation returns False and the config comparison runs.
            with open(os.path.join(ts_dir,
                                   "taskset_characteristics_interval_0.yaml"), "w") as f:
                f.write("tasks: []\n")
            cfg_i = dict(base_config)
            cfg_i["RANDOM_SEED"] = base_seed + idx
            cfg_i["UPDATE_INTERVAL_S"] = interval_sec
            with open(os.path.join(ts_dir, "generator_config.json"), "w") as f:
                json.dump(cfg_i, f)
        return main_dir

    def test_matching_interval_reuses_main_dir_without_running(self):
        """Fresh main-step dir for the swept interval -> no subprocess.run."""
        import tempfile
        tmp = tempfile.mkdtemp()
        self._write_fresh_main_dir(tmp, num_tasks=6, n_tasksets=2, n_sec=30,
                                   interval_sec=10, base_seed=1000)
        with unittest.mock.patch.object(sweep.subprocess, "run") as mock_run:
            out = sweep.run_single_interval(
                num_tasks=6, n_tasksets=2, n_sec=30, interval_sec=10,
                base_seed=1000, schedulers=["INCR", "BF", "RM", "CFS"],
                bin_dir="release", output_parent=tmp, export_level=1,
                important_task_pct=0.1, num_workers=None, resume=False,
                verbose=0, reuse_matching_interval=True,
            )
        mock_run.assert_not_called()
        # Returned dir is the reused main-step dir, not the sweep's own.
        self.assertTrue(out.endswith("tasks6_dur30_interval10_seed1000"))

    def test_no_main_dir_runs_compare_optimizers(self):
        """No main-step dir for this interval -> compare_optimizers runs."""
        import tempfile
        tmp = tempfile.mkdtemp()
        with unittest.mock.patch.object(sweep.subprocess, "run") as mock_run:
            mock_run.return_value = unittest.mock.MagicMock(returncode=0)
            out = sweep.run_single_interval(
                num_tasks=6, n_tasksets=2, n_sec=30, interval_sec=5,
                base_seed=1000, schedulers=["INCR", "BF", "RM", "CFS"],
                bin_dir="release", output_parent=tmp, export_level=1,
                important_task_pct=0.1, num_workers=None, resume=False,
                verbose=0, reuse_matching_interval=True,
            )
        mock_run.assert_called_once()
        # No main dir for interval=5, so it wrote to the sweep's own dir.
        self.assertTrue(out.endswith("tasks6_sweep_interval5_seed1000"))

    def test_stale_main_dir_not_reused(self):
        """Main dir present but taskset config drifted -> not reused (regen)."""
        import tempfile
        tmp = tempfile.mkdtemp()
        main_dir = self._write_fresh_main_dir(
            tmp, num_tasks=6, n_tasksets=2, n_sec=30, interval_sec=10,
            base_seed=1000)
        # Corrupt one taskset's baseline so it no longer matches.
        with open(os.path.join(main_dir, "taskset_0", "generator_config.json"),
                  "w") as f:
            json.dump({"STALE": True}, f)
        with unittest.mock.patch.object(sweep.subprocess, "run") as mock_run:
            mock_run.return_value = unittest.mock.MagicMock(returncode=0)
            sweep.run_single_interval(
                num_tasks=6, n_tasksets=2, n_sec=30, interval_sec=10,
                base_seed=1000, schedulers=["INCR", "BF", "RM", "CFS"],
                bin_dir="release", output_parent=tmp, export_level=1,
                important_task_pct=0.1, num_workers=None, resume=False,
                verbose=0, reuse_matching_interval=True,
            )
        mock_run.assert_called_once()

    def test_forwards_on_taskset_config_change_to_child(self):
        """run_single_interval forwards the on_taskset_config_change policy
        verbatim to compare_optimizers (default 'prompt'; 'keep' used here
        only to exercise the forwarding path)."""
        import tempfile
        tmp = tempfile.mkdtemp()
        with unittest.mock.patch.object(sweep.subprocess, "run") as mock_run:
            mock_run.return_value = unittest.mock.MagicMock(returncode=0)
            sweep.run_single_interval(
                num_tasks=6, n_tasksets=2, n_sec=30, interval_sec=5,
                base_seed=1000, schedulers=["INCR", "BF", "RM", "CFS"],
                bin_dir="release", output_parent=tmp, export_level=1,
                important_task_pct=0.1, num_workers=None, resume=False,
                verbose=0, reuse_matching_interval=False,
                on_taskset_config_change="keep",
            )
        cmd = mock_run.call_args[0][0]
        self.assertIn("--on_taskset_config_change", cmd)
        self.assertEqual(
            cmd[cmd.index("--on_taskset_config_change") + 1], "keep"
        )

    def test_reuse_disabled_runs_even_when_main_dir_exists(self):
        """reuse_matching_interval=False always runs compare_optimizers."""
        import tempfile
        tmp = tempfile.mkdtemp()
        self._write_fresh_main_dir(tmp, num_tasks=6, n_tasksets=2, n_sec=30,
                                   interval_sec=10, base_seed=1000)
        with unittest.mock.patch.object(sweep.subprocess, "run") as mock_run:
            mock_run.return_value = unittest.mock.MagicMock(returncode=0)
            sweep.run_single_interval(
                num_tasks=6, n_tasksets=2, n_sec=30, interval_sec=10,
                base_seed=1000, schedulers=["INCR", "BF", "RM", "CFS"],
                bin_dir="release", output_parent=tmp, export_level=1,
                important_task_pct=0.1, num_workers=None, resume=False,
                verbose=0, reuse_matching_interval=False,
            )
        mock_run.assert_called_once()


if __name__ == "__main__":
    unittest.main()
