"""Tests for simulation_experiments.aggregate_across_tasks.

These tests mock the filesystem (temporary directories and synthetic CSV files)
to verify aggregation logic and figure generation without needing real
simulation outputs.
"""
import csv
import os
import sys
import tempfile
import shutil
import unittest
import unittest.mock

import yaml

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import simulation_experiments.aggregate_across_tasks as agg
from simulation_experiments.plotting_config import setup_publication_style


def _write_summary_csv(dir_path, rows):
    """Helper: write a comparison_summary.csv with the given rows."""
    os.makedirs(dir_path, exist_ok=True)
    csv_path = os.path.join(dir_path, "comparison_summary.csv")
    header = [
        "Scheduler", "Mean_SP_Metric", "Std_SP_Metric", "Mean_Miss_Rate",
        "Std_Miss_Rate", "Mean_Scheduler_Execution_Time_s",
        "Important_Miss_Rate", "Non_Important_Miss_Rate",
    ]
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for r in rows:
            writer.writerow([
                r["scheduler"],
                r["mean_sp"],
                r["std_sp"],
                r.get("mean_miss", 0.0),
                r.get("std_miss", 0.0),
                r["mean_sched_time"],
                r.get("important_miss", 0.0),
                r.get("non_important_miss", 0.0),
            ])


class TestParseTaskCount(unittest.TestCase):

    def test_valid(self):
        self.assertEqual(agg.parse_task_count_from_dir_name("tasks6_dur300_interval10_seed1000"), 6)
        self.assertEqual(agg.parse_task_count_from_dir_name("tasks8_foo"), 8)

    def test_invalid(self):
        self.assertIsNone(agg.parse_task_count_from_dir_name("foo_bar"))
        self.assertIsNone(agg.parse_task_count_from_dir_name(""))

    def test_edge_cases(self):
        self.assertEqual(agg.parse_task_count_from_dir_name("tasks4_"), 4)


class TestLoadExperimentSummaries(unittest.TestCase):

    def test_load_rows(self):
        temp_dir = tempfile.mkdtemp()
        try:
            rows = [
                {"scheduler": "INCR", "mean_sp": 0.9, "std_sp": 0.05,
                 "mean_sched_time": 0.01, "mean_miss": 0.1, "std_miss": 0.02,
                 "important_miss": 0.05, "non_important_miss": 0.12},
                {"scheduler": "BF", "mean_sp": 0.85, "std_sp": 0.04,
                 "mean_sched_time": 0.02, "mean_miss": 0.15, "std_miss": 0.03,
                 "important_miss": 0.08, "non_important_miss": 0.18},
            ]
            _write_summary_csv(temp_dir, rows)
            loaded = agg.load_experiment_summaries(temp_dir)
            self.assertEqual(len(loaded), 2)
            self.assertEqual(loaded[0]["Scheduler"], "INCR")
            self.assertAlmostEqual(float(loaded[0]["Mean_SP_Metric"]), 0.9)
            self.assertEqual(loaded[1]["Scheduler"], "BF")
        finally:
            shutil.rmtree(temp_dir)

    def test_missing_file(self):
        temp_dir = tempfile.mkdtemp()
        try:
            loaded = agg.load_experiment_summaries(temp_dir)
            self.assertEqual(loaded, [])
        finally:
            shutil.rmtree(temp_dir)


class TestAggregateData(unittest.TestCase):

    def test_aggregate_records(self):
        temp_base = tempfile.mkdtemp()
        try:
            # Temporarily redirect optimizer comparison dir
            orig_dir = agg.OPTIMIZER_COMPARISON_DIR
            agg.OPTIMIZER_COMPARISON_DIR = temp_base
            agg.FIGURES_OUTPUT_DIR = os.path.join(temp_base, "figures")

            # Create two experiment directories
            exp6 = os.path.join(temp_base, "tasks6_dur300_interval10_seed1000")
            exp8 = os.path.join(temp_base, "tasks8_dur300_interval10_seed2000")
            _write_summary_csv(exp6, [
                {"scheduler": "INCR", "mean_sp": 0.90, "std_sp": 0.05, "mean_sched_time": 0.01},
                {"scheduler": "BF", "mean_sp": 0.85, "std_sp": 0.04, "mean_sched_time": 0.02},
            ])
            _write_summary_csv(exp8, [
                {"scheduler": "INCR", "mean_sp": 0.88, "std_sp": 0.06, "mean_sched_time": 0.015},
                {"scheduler": "BF", "mean_sp": 0.82, "std_sp": 0.05, "mean_sched_time": 0.025},
            ])

            records = agg.aggregate_data_from_directories()
            self.assertEqual(len(records), 4)

            # Verify numeric parsing
            incr6 = [r for r in records if r["num_tasks"] == 6 and r["scheduler"] == "INCR"][0]
            self.assertAlmostEqual(incr6["mean_sp"], 0.90)
            self.assertAlmostEqual(incr6["std_sp"], 0.05)

            bf8 = [r for r in records if r["num_tasks"] == 8 and r["scheduler"] == "BF"][0]
            self.assertAlmostEqual(bf8["mean_sp"], 0.82)
        finally:
            shutil.rmtree(temp_base)
            agg.OPTIMIZER_COMPARISON_DIR = orig_dir

    def test_no_directory(self):
        temp_base = tempfile.mkdtemp()
        try:
            orig_dir = agg.OPTIMIZER_COMPARISON_DIR
            agg.OPTIMIZER_COMPARISON_DIR = os.path.join(temp_base, "nonexistent")
            records = agg.aggregate_data_from_directories()
            self.assertEqual(records, [])
        finally:
            shutil.rmtree(temp_base)
            agg.OPTIMIZER_COMPARISON_DIR = orig_dir

    def test_malformed_row_skipped(self):
        temp_base = tempfile.mkdtemp()
        try:
            orig_dir = agg.OPTIMIZER_COMPARISON_DIR
            agg.OPTIMIZER_COMPARISON_DIR = temp_base
            agg.FIGURES_OUTPUT_DIR = os.path.join(temp_base, "figures")

            exp_dir = os.path.join(temp_base, "tasks6_test")
            os.makedirs(exp_dir, exist_ok=True)
            csv_path = os.path.join(exp_dir, "comparison_summary.csv")
            with open(csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["Scheduler", "Mean_SP_Metric", "Std_SP_Metric",
                                 "Mean_Miss_Rate", "Std_Miss_Rate",
                                 "Mean_Scheduler_Execution_Time_s",
                                 "Important_Miss_Rate", "Non_Important_Miss_Rate"])
                writer.writerow(["INCR", "0.9", "0.05", "0", "0", "0.01", "0", "0"])
                writer.writerow(["BAD", "not_a_number", "0.05", "0", "0", "0.01", "0", "0"])

            records = agg.aggregate_data_from_directories()
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0]["scheduler"], "INCR")
        finally:
            shutil.rmtree(temp_base)
            agg.OPTIMIZER_COMPARISON_DIR = orig_dir

    def test_aggregation_excludes_stale_dirs(self):
        """Run-scoped ingestion must skip stale dirs from a different run.

        Two ``tasks6_*`` dirs coexist: one matching the current run
        (dur70) and one stale (dur300, from an older run). With a cfg,
        only the matching dir's records are ingested -- the
        "last-write-wins" pollution bug noted in the dev log must not
        recur. Without a cfg, the legacy path still ingests both.

        P23: when scoped (cfg given), aggregate scans ``<run_root>/sim/`` for
        the run's dirs, so the matching dir must live there. The legacy path
        still scans ``output_parent`` directly.
        """
        temp_base = tempfile.mkdtemp()
        try:
            cfg = {
                "num_tasks_for_cross_task_comparison": [6],
                "simulation_duration_seconds": 70,
                "scheduler_trigger_interval_seconds": 10,
                "base_random_seed": 1000,
            }
            # Scoped path: dirs live under <run_root>/sim/.
            run_root = agg.build_run_root(temp_base, cfg)
            sim_dir = os.path.join(run_root, "sim")
            current = os.path.join(sim_dir, "tasks6_dur70_interval10_seed1000")
            # Stale dir (dur300) also under sim/ -- scoped ingestion must skip
            # it because its prefix does not match the run's parameters.
            stale = os.path.join(sim_dir, "tasks6_dur300_interval10_seed1000")
            _write_summary_csv(current, [
                {"scheduler": "INCR", "mean_sp": 0.90, "std_sp": 0.05, "mean_sched_time": 0.01},
            ])
            _write_summary_csv(stale, [
                {"scheduler": "INCR", "mean_sp": 0.10, "std_sp": 0.02, "mean_sched_time": 0.01},
            ])

            scoped = agg.aggregate_data_from_directories(cfg=cfg, output_parent=temp_base)
            self.assertEqual(len(scoped), 1)
            self.assertEqual(scoped[0]["mean_sp"], 0.90)  # current run, not stale

            # Legacy path (no cfg): also drop a tasks6 dir at the top level so
            # the unscoped scan finds two there.
            legacy_current = os.path.join(temp_base, "tasks6_dur70_interval10_seed1000")
            legacy_stale = os.path.join(temp_base, "tasks6_dur300_interval10_seed1000")
            _write_summary_csv(legacy_current, [
                {"scheduler": "INCR", "mean_sp": 0.90, "std_sp": 0.05, "mean_sched_time": 0.01},
            ])
            _write_summary_csv(legacy_stale, [
                {"scheduler": "INCR", "mean_sp": 0.10, "std_sp": 0.02, "mean_sched_time": 0.01},
            ])
            legacy = agg.aggregate_data_from_directories(output_parent=temp_base)
            self.assertEqual(len(legacy), 2)
        finally:
            shutil.rmtree(temp_base)


class TestBuildLineChart(unittest.TestCase):

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.plt")
    def test_chart_generated(self, mock_plt):
        # Mock subplots to return a (fig, ax) tuple-like object
        mock_ax = unittest.mock.MagicMock()
        mock_fig = unittest.mock.MagicMock()
        mock_plt.subplots.return_value = (mock_fig, mock_ax)

        records = [
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 0.90, "std_sp": 0.05},
            {"num_tasks": 4, "scheduler": "BF", "mean_sp": 0.85, "std_sp": 0.04},
            {"num_tasks": 6, "scheduler": "INCR", "mean_sp": 0.88, "std_sp": 0.06},
            {"num_tasks": 6, "scheduler": "BF", "mean_sp": 0.82, "std_sp": 0.05},
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            output_stem = os.path.join(tmpdir, "test_fig")
            agg.build_line_chart(
                records,
                scheduler_list=["INCR", "BF"],
                metric_key="mean_sp",
                std_key="std_sp",
                ylabel="Mean SP",
                title="Test Chart",
                output_stem=output_stem,
            )
            # Should create a figure, then close it
            mock_plt.subplots.assert_called_once()
            mock_plt.close.assert_called_once()

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.plt")
    def test_log_y_sets_log_scale(self, mock_plt):
        """log_y=True must set log scale and label both major and minor ticks."""
        mock_ax = unittest.mock.MagicMock()
        mock_fig = unittest.mock.MagicMock()
        mock_plt.subplots.return_value = (mock_fig, mock_ax)

        records = [
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 0.9, "std_sp": 0.05},
            {"num_tasks": 6, "scheduler": "INCR", "mean_sp": 0.8, "std_sp": 0.04},
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            agg.build_line_chart(
                records, ["INCR"], "mean_sp", "std_sp", "Y", "T",
                os.path.join(tmpdir, "fig"), log_y=True,
            )
        mock_ax.set_yscale.assert_called_once_with("log")
        # Major decade ticks are labelled as plain numbers...
        mock_ax.yaxis.set_major_formatter.assert_called_once()
        # ...and the sub-decade (minor) ticks are labelled too (sparsely, 2x/5x
        # only) so a value between two decades (e.g. 0.062) is readable instead
        # of floating in an unlabelled gap up to the next decade tick.
        mock_ax.yaxis.set_minor_formatter.assert_called_once()

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.plt")
    def test_log_y_minor_ticks_are_sparse_2x_5x(self, mock_plt):
        """Minor ticks are labelled SPARSELY (2x, 5x only) regardless of span.

        Regression for the exec-time figure: a ~2.4-decade span
        (0.00025..0.062s) used to trip a "wide" branch that suppressed minor
        labels, leaving N>10 ET values in an unlabelled gap. The first fix
        labelled EVERY sub-decade (2x..9x), which overlapped into an unreadable
        wall of numbers. The contract is now: always label minor ticks, but only
        the well-spaced 2x and 5x multiples per decade -- e.g. 0.001 / 0.002 /
        0.005 / 0.01 / 0.02 / 0.05 / 0.1 -- so the in-between values are readable
        without overlap, at any decade span.
        """
        for mean_lo, mean_hi in [(2.5e-4, 6.2e-2), (5e-4, 1.0)]:
            mock_ax = unittest.mock.MagicMock()
            mock_fig = unittest.mock.MagicMock()
            mock_plt.subplots.return_value = (mock_fig, mock_ax)

            records = [
                {"num_tasks": 4, "scheduler": "INCR", "mean_sp": mean_lo, "std_sp": 0.0},
                {"num_tasks": 16, "scheduler": "INCR", "mean_sp": mean_hi, "std_sp": 0.0},
            ]
            with tempfile.TemporaryDirectory() as tmpdir:
                agg.build_line_chart(
                    records, ["INCR"], "mean_sp", "std_sp", "Y", "T",
                    os.path.join(tmpdir, "fig"), log_y=True,
                )
            loc = mock_ax.yaxis.set_minor_locator.call_args[0][0]
            # Sparse set: 0.2 and 0.5 only = 2 sub-decade positions per decade,
            # at BOTH narrow (~2.4-decade) and wide (~3.3-decade) spans.
            self.assertEqual(len(loc._subs), 2, f"span {mean_lo}..{mean_hi}")

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.plt")
    def test_log_y_false_default_no_log(self, mock_plt):
        """Without log_y, the y-axis must not be set to log scale."""
        mock_ax = unittest.mock.MagicMock()
        mock_fig = unittest.mock.MagicMock()
        mock_plt.subplots.return_value = (mock_fig, mock_ax)

        records = [
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 0.9, "std_sp": 0.05},
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            agg.build_line_chart(
                records, ["INCR"], "mean_sp", "std_sp", "Y", "T",
                os.path.join(tmpdir, "fig"),
            )
        mock_ax.set_yscale.assert_not_called()


def _write_taskset_characteristics(taskset_dir, weights, deadlines=None,
                                   with_perf_pairs=False):
    """Helper: write a taskset_characteristics_interval_0.yaml for ceiling tests."""
    os.makedirs(taskset_dir, exist_ok=True)
    path = os.path.join(taskset_dir, "taskset_characteristics_interval_0.yaml")
    tasks = []
    for i, w in enumerate(weights):
        t = {
            "id": i, "period": 100, "deadline": (deadlines[i] if deadlines else 50),
            "sp_threshold": 0.5, "sp_weight": w, "name": f"task_{i+1}",
        }
        if with_perf_pairs:
            t["timePerformancePairs"] = [{"time_limit": 1, "performance": 0.5}]
        tasks.append(t)
    with open(path, "w") as f:
        yaml.safe_dump({"tasks": tasks}, f)


class TestComputeSPUpperBound(unittest.TestCase):
    """Theoretical SP ceiling = sum(sp_weight * perf_coefficient) over tasks."""

    def test_sum_of_weights(self):
        with tempfile.TemporaryDirectory() as tmp:
            exp_dir = os.path.join(tmp, "tasks4_dur70_interval10_seed1000")
            _write_taskset_characteristics(os.path.join(exp_dir, "taskset_0"),
                                           weights=[1.25, 1.25, 1.25, 1.25])
            ub = agg.compute_sp_upper_bound(exp_dir)
            self.assertAlmostEqual(ub, 5.0)

    def test_constant_sum_across_n(self):
        """Generator normalizes weights so the ceiling is flat across N."""
        with tempfile.TemporaryDirectory() as tmp:
            exp4 = os.path.join(tmp, "tasks4_dur70_interval10_seed1000")
            exp6 = os.path.join(tmp, "tasks6_dur70_interval10_seed1000")
            _write_taskset_characteristics(os.path.join(exp4, "taskset_0"),
                                           weights=[1.25] * 4)
            _write_taskset_characteristics(os.path.join(exp6, "taskset_0"),
                                           weights=[0.7142857] * 5 + [1.4285714])
            ub4 = agg.compute_sp_upper_bound(exp4)
            ub6 = agg.compute_sp_upper_bound(exp6)
            self.assertAlmostEqual(ub4, 5.0, places=4)
            self.assertAlmostEqual(ub6, 5.0, places=4)

    def test_no_yaml_returns_none(self):
        with tempfile.TemporaryDirectory() as tmp:
            # experiment dir exists but has no taskset_*/characteristics.yaml
            exp_dir = os.path.join(tmp, "tasks4_test")
            os.makedirs(exp_dir, exist_ok=True)
            self.assertIsNone(agg.compute_sp_upper_bound(exp_dir))

    def test_missing_dir_returns_none(self):
        self.assertIsNone(agg.compute_sp_upper_bound("/nonexistent/path/xyz"))

    def test_missing_sp_weight_raises(self):
        """A task without sp_weight is a bug (the generator always writes one)
        -- compute_sp_upper_bound must raise, not silently default to 1.0."""
        with tempfile.TemporaryDirectory() as tmp:
            exp_dir = os.path.join(tmp, "tasks4_dur70_interval10_seed1000")
            ts_dir = os.path.join(exp_dir, "taskset_0")
            os.makedirs(ts_dir, exist_ok=True)
            path = os.path.join(ts_dir, "taskset_characteristics_interval_0.yaml")
            # One task has sp_weight, one does NOT -> must raise.
            with open(path, "w") as f:
                yaml.safe_dump({"tasks": [
                    {"id": 0, "sp_weight": 1.25},
                    {"id": 1},  # missing sp_weight
                ]}, f)
            with self.assertRaises(KeyError):
                agg.compute_sp_upper_bound(exp_dir)


class TestNormalizeRecordsSP(unittest.TestCase):
    """SP normalization: upper_bound (ideal-SP, default) / minmax per N.

    Semantics (by design): normalized SP = raw_SP / ideal_SP, where ideal_SP
    is the best possible with infinite computation (all deadlines met,
    SP_Func=1 for all tasks) = sum of sp_weight x perf_coefficient (~5.0,
    flat across N). Because SP_Func in [0,1] with non-negative weights,
    raw_SP <= ideal_SP always, so the ratio is in [0,1] with 1.0 = "all
    deadlines met perfectly". Independent of any scheduler's realized value.
    """

    def test_upper_bound_default_method(self):
        """Without an explicit method, upper_bound (ideal-SP) is the default.

        Each scheduler's SP is divided by the per-N ideal-SP ceiling; 1.0 =
        all deadlines met. Values are guaranteed <= 1.0 because raw_SP cannot
        exceed the ceiling.
        """
        records = [
            {"num_tasks": 4, "scheduler": "BF", "mean_sp": 4.0, "std_sp": 0.1,
             "_experiment_dir": "x"},
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 3.2, "std_sp": 0.2,
             "_experiment_dir": "x"},
            {"num_tasks": 4, "scheduler": "RM", "mean_sp": 2.0, "std_sp": 0.05,
             "_experiment_dir": "x"},
        ]
        cfg = {"analysis": {}}  # no method key -> default upper_bound
        norm = agg.normalize_records_sp(
            records, cfg, upper_bound_by_task_count={4: 5.0}
        )
        by = {r["scheduler"]: r["mean_sp"] for r in norm}
        self.assertAlmostEqual(by["BF"], 0.8)      # 4.0 / 5.0
        self.assertAlmostEqual(by["INCR"], 0.64)   # 3.2 / 5.0
        self.assertAlmostEqual(by["RM"], 0.4)      # 2.0 / 5.0
        # std scales the same way
        std_by = {r["scheduler"]: r["std_sp"] for r in norm}
        self.assertAlmostEqual(std_by["INCR"], 0.04)  # 0.2 / 5.0
        self.assertTrue(all(0.0 <= v <= 1.0 + 1e-9 for v in by.values()))

    def test_upper_bound_normalization(self):
        records = [
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 4.0, "std_sp": 0.1,
             "_experiment_dir": "unused"},
            {"num_tasks": 4, "scheduler": "BF", "mean_sp": 2.5, "std_sp": 0.2,
             "_experiment_dir": "unused"},
        ]
        # Precomputed ceiling = 5.0 for N=4.
        cfg = {"analysis": {"sp_normalization_method": "upper_bound"}}
        norm = agg.normalize_records_sp(
            records, cfg, upper_bound_by_task_count={4: 5.0}
        )
        self.assertAlmostEqual(norm[0]["mean_sp"], 0.8)   # 4.0 / 5.0
        self.assertAlmostEqual(norm[0]["std_sp"], 0.02)   # 0.1 / 5.0
        self.assertAlmostEqual(norm[1]["mean_sp"], 0.5)   # 2.5 / 5.0

    def test_upper_bound_clamps_to_unity(self):
        """Normalized values land in [0,1]; SP cannot exceed its ceiling."""
        records = [
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 5.0, "std_sp": 0.0,
             "_experiment_dir": "x"},
        ]
        cfg = {"analysis": {"sp_normalization_method": "upper_bound"}}
        norm = agg.normalize_records_sp(
            records, cfg, upper_bound_by_task_count={4: 5.0}
        )
        self.assertAlmostEqual(norm[0]["mean_sp"], 1.0)
        self.assertTrue(0.0 <= norm[0]["mean_sp"] <= 1.0)

    def test_no_ceiling_keeps_raw(self):
        """If no ceiling is available for a task count, raw values are kept."""
        records = [
            {"num_tasks": 8, "scheduler": "INCR", "mean_sp": 3.0, "std_sp": 0.1,
             "_experiment_dir": "x"},
        ]
        cfg = {"analysis": {"sp_normalization_method": "upper_bound"}}
        norm = agg.normalize_records_sp(records, cfg, upper_bound_by_task_count={8: None})
        self.assertAlmostEqual(norm[0]["mean_sp"], 3.0)  # unchanged
        self.assertAlmostEqual(norm[0]["std_sp"], 0.1)

    def test_minmax_per_task_count(self):
        records = [
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 4.0, "std_sp": 0.1,
             "_experiment_dir": "x"},
            {"num_tasks": 4, "scheduler": "BF", "mean_sp": 2.0, "std_sp": 0.2,
             "_experiment_dir": "x"},
            {"num_tasks": 4, "scheduler": "RM", "mean_sp": 3.0, "std_sp": 0.0,
             "_experiment_dir": "x"},
        ]
        cfg = {"analysis": {"sp_normalization_method": "minmax_per_task_count"}}
        norm = agg.normalize_records_sp(records, cfg)
        # min=2.0, max=4.0, span=2.0 -> INCR=1.0, BF=0.0, RM=0.5
        by = {r["scheduler"]: r["mean_sp"] for r in norm}
        self.assertAlmostEqual(by["INCR"], 1.0)
        self.assertAlmostEqual(by["BF"], 0.0)
        self.assertAlmostEqual(by["RM"], 0.5)

    def test_minmax_degenerate_all_equal(self):
        """All schedulers equal at a task count -> max maps to 1.0."""
        records = [
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 3.0, "std_sp": 0.0,
             "_experiment_dir": "x"},
            {"num_tasks": 4, "scheduler": "BF", "mean_sp": 3.0, "std_sp": 0.0,
             "_experiment_dir": "x"},
        ]
        cfg = {"analysis": {"sp_normalization_method": "minmax_per_task_count"}}
        norm = agg.normalize_records_sp(records, cfg)
        for r in norm:
            self.assertAlmostEqual(r["mean_sp"], 1.0)

    def test_raw_records_not_mutated(self):
        """Normalization must not alter the input records."""
        records = [
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 4.0, "std_sp": 0.1,
             "_experiment_dir": "x"},
        ]
        cfg = {"analysis": {"sp_normalization_method": "upper_bound"}}
        _ = agg.normalize_records_sp(records, cfg, upper_bound_by_task_count={4: 5.0})
        self.assertAlmostEqual(records[0]["mean_sp"], 4.0)  # untouched
        self.assertAlmostEqual(records[0]["std_sp"], 0.1)


    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", False)
    def test_skips_when_no_matplotlib(self):
        records = [{"num_tasks": 4, "scheduler": "INCR", "mean_sp": 0.9, "std_sp": 0.05}]
        with tempfile.TemporaryDirectory() as tmpdir:
            output_stem = os.path.join(tmpdir, "test_fig")
            # Should not crash, should just return early
            agg.build_line_chart(
                records,
                scheduler_list=["INCR"],
                metric_key="mean_sp",
                std_key="std_sp",
                ylabel="Y",
                title="T",
                output_stem=output_stem,
            )


class TestGenerateMainGroupFigures(unittest.TestCase):

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.build_line_chart")
    def test_figures_generated(self, mock_build):
        records = [
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 0.90, "std_sp": 0.05,
             "mean_miss_rate": 0.1, "std_miss_rate": 0.02, "mean_sched_time": 0.01, "std_sched_time": 0.001},
        ]
        cfg = {"main_scheduler_list": ["INCR", "BF", "RM", "CFS"]}
        with tempfile.TemporaryDirectory() as tmpdir:
            agg.FIGURES_OUTPUT_DIR = tmpdir
            agg.generate_main_group_figures(records, cfg)
            # Should call build_line_chart 5 times (1A, 1B, 1C, 1D, 1E)
            self.assertEqual(mock_build.call_count, 5)

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.build_line_chart")
    def test_figures_normalized_replaces_raw_1a_and_1b(self, mock_build):
        """With normalize_sp on, 1A/1B emit ONLY their normalized variants.

        The raw mean/std SP figures are the same type as the normalized ones
        (just an unscaled y-axis), so they are dropped to plot less. The
        non-SP figures (1C exec time, 1D/1E miss rate) are unaffected.
        """
        records = [
            {"num_tasks": 4, "scheduler": "BF", "mean_sp": 0.90, "std_sp": 0.05,
             "mean_miss_rate": 0.1, "std_miss_rate": 0.02, "mean_sched_time": 0.01, "std_sched_time": 0.001},
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 0.72, "std_sp": 0.04,
             "mean_miss_rate": 0.1, "std_miss_rate": 0.02, "mean_sched_time": 0.01, "std_sched_time": 0.001},
        ]
        cfg = {
            "main_scheduler_list": ["INCR", "BF", "RM", "CFS"],
            "analysis": {"normalize_sp": True,
                         "sp_normalization_method": "upper_bound"},
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            agg.FIGURES_OUTPUT_DIR = tmpdir
            agg.generate_main_group_figures(records, cfg)
            # 1A-norm, 1B-norm, 1C, 1D, 1E (raw 1A/1B dropped as redundant).
            self.assertEqual(mock_build.call_count, 5)
            stems = [c.args[-1] for c in mock_build.call_args_list]
            self.assertTrue(any("fig1a_mean_sp_normalized" in s for s in stems))
            self.assertTrue(any("fig1b_std_sp_normalized" in s for s in stems))
            self.assertFalse(any("fig1a_mean_sp_vs_tasks_main" in s for s in stems))
            self.assertFalse(any("fig1b_std_sp_vs_tasks_main" in s for s in stems))


class TestGenerateAblationFigures(unittest.TestCase):

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.build_line_chart")
    def test_ablation_figures(self, mock_build):
        records = [
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 0.90, "std_sp": 0.05,
             "mean_sched_time": 0.01, "std_sched_time": 0.001},
        ]
        cfg = {"ablation_scheduler_list": ["BF", "INCR", "INCR_NO_TL", "INCR_WCET"]}
        with tempfile.TemporaryDirectory() as tmpdir:
            agg.FIGURES_OUTPUT_DIR = tmpdir
            agg.generate_ablation_group_figures(records, cfg)
            self.assertEqual(mock_build.call_count, 2)

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.build_line_chart")
    def test_ablation_normalized_replaces_raw_sp(self, mock_build):
        """With normalize_sp on, the ablation SP figure emits ONLY normalized.

        Raw mean-SP is the same type as the normalized variant (unscaled
        y-axis), so it is dropped. Exec-time figure (different type) stays.
        """
        records = [
            {"num_tasks": 4, "scheduler": "INCR", "mean_sp": 0.90, "std_sp": 0.05,
             "mean_sched_time": 0.01, "std_sched_time": 0.001},
        ]
        cfg = {
            "ablation_scheduler_list": ["BF", "INCR", "INCR_NO_TL", "INCR_WCET"],
            "analysis": {"normalize_sp": True,
                         "sp_normalization_method": "upper_bound"},
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            agg.FIGURES_OUTPUT_DIR = tmpdir
            agg.generate_ablation_group_figures(records, cfg)
            # Only: ablation-SP-normalized + ablation-exec-time.
            self.assertEqual(mock_build.call_count, 2)
            stems = [c.args[-1] for c in mock_build.call_args_list]
            self.assertTrue(any("fig_ablation_mean_sp_normalized" in s for s in stems))
            self.assertFalse(any("fig_ablation_mean_sp_vs_tasks" in s for s in stems))


class TestImportantTaskMissRateFigure(unittest.TestCase):

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.save_figure")
    def test_figure_3_generated(self, mock_save):
        records = [
            {"num_tasks": 6, "scheduler": "INCR", "important_miss_rate": 0.05, "non_important_miss_rate": 0.12},
            {"num_tasks": 6, "scheduler": "BF", "important_miss_rate": 0.08, "non_important_miss_rate": 0.15},
            {"num_tasks": 6, "scheduler": "RM", "important_miss_rate": 0.10, "non_important_miss_rate": 0.18},
            {"num_tasks": 6, "scheduler": "CFS", "important_miss_rate": 0.12, "non_important_miss_rate": 0.20},
        ]
        cfg = {"num_tasks_for_single_task_figures": 6, "main_scheduler_list": ["INCR", "BF", "RM", "CFS"]}
        with tempfile.TemporaryDirectory() as tmpdir:
            agg.FIGURES_OUTPUT_DIR = tmpdir
            agg.generate_important_task_miss_rate_figure(records, cfg)
            # save_figure should be called twice (Fig 3 and Fig 3b)
            self.assertEqual(mock_save.call_count, 2)

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", True)
    def test_figure_3_no_data(self):
        records = [
            {"num_tasks": 8, "scheduler": "INCR", "important_miss_rate": 0.05, "non_important_miss_rate": 0.12},
        ]
        cfg = {"num_tasks_for_single_task_figures": 6, "main_scheduler_list": ["INCR", "BF", "RM", "CFS"]}
        with tempfile.TemporaryDirectory() as tmpdir:
            agg.FIGURES_OUTPUT_DIR = tmpdir
            # Should not crash when target task count not present
            agg.generate_important_task_miss_rate_figure(records, cfg)


class TestGenerateDistributionBoxplot(unittest.TestCase):

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.save_figure")
    def test_boxplot_generated(self, mock_save):
        temp_base = tempfile.mkdtemp()
        try:
            # Create fake experiment dir with interval_sp_metrics.txt
            orig_dir = agg.OPTIMIZER_COMPARISON_DIR
            agg.OPTIMIZER_COMPARISON_DIR = temp_base
            agg.FIGURES_OUTPUT_DIR = os.path.join(temp_base, "figures")

            exp_dir = os.path.join(temp_base, "tasks4_dur300_interval10_seed1000")
            taskset_dir = os.path.join(exp_dir, "taskset_0")
            sched_dir = os.path.join(taskset_dir, "INCR", "INCR")
            os.makedirs(sched_dir, exist_ok=True)

            metrics_file = os.path.join(sched_dir, "interval_sp_metrics.txt")
            with open(metrics_file, "w") as f:
                for i in range(10):
                    f.write(f"{i},0.{90 + i}\n")

            cfg = {"num_tasks_for_single_task_figures": 4, "main_scheduler_list": ["INCR"]}
            agg.generate_distribution_boxplot(cfg)
            mock_save.assert_called_once()
        finally:
            shutil.rmtree(temp_base)
            agg.OPTIMIZER_COMPARISON_DIR = orig_dir

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.save_figure")
    def test_boxplot_normalized_emitted(self, mock_save):
        """With normalize_sp on + a ceiling, ONLY the normalized boxplot is drawn.

        The raw boxplot is the same type as the normalized one (unscaled
        y-axis), so it is dropped. Normalization divides each SP point by the
        ideal-SP ceiling (sum of sp_weight from
        taskset_characteristics_interval_0.yaml), NOT by any scheduler's mean,
        so every point stays <= 1.0.
        """
        temp_base = tempfile.mkdtemp()
        try:
            orig_dir = agg.OPTIMIZER_COMPARISON_DIR
            agg.OPTIMIZER_COMPARISON_DIR = temp_base
            agg.FIGURES_OUTPUT_DIR = os.path.join(temp_base, "figures")

            exp_dir = os.path.join(temp_base, "tasks4_dur300_interval10_seed1000")
            taskset_dir = os.path.join(exp_dir, "taskset_0")
            os.makedirs(taskset_dir, exist_ok=True)
            # Provide a characteristics YAML so compute_sp_upper_bound can read
            # the ceiling. Two tasks, weights 3.0 + 2.0 = 5.0 ideal-SP ceiling.
            with open(os.path.join(taskset_dir, "taskset_characteristics_interval_0.yaml"), "w") as f:
                f.write("tasks:\n  - sp_weight: 3.0\n  - sp_weight: 2.0\n")
            # Raw SP points all below the 5.0 ceiling -> normalized < 1.0.
            for sched, base in [("BF", 90), ("INCR", 70)]:
                sched_dir = os.path.join(taskset_dir, sched, sched)
                os.makedirs(sched_dir, exist_ok=True)
                with open(os.path.join(sched_dir, "interval_sp_metrics.txt"), "w") as f:
                    for i in range(10):
                        f.write(f"{i},0.{base + i}\n")

            cfg = {
                "num_tasks_for_single_task_figures": 4,
                "main_scheduler_list": ["INCR", "BF"],
                "analysis": {"normalize_sp": True,
                             "sp_normalization_method": "upper_bound"},
            }
            agg.generate_distribution_boxplot(cfg)
            # Only the normalized boxplot is saved (raw dropped as redundant).
            self.assertEqual(mock_save.call_count, 1)
            stems = [c.args[1] if len(c.args) > 1 else c.kwargs.get("output_path_stem")
                     for c in mock_save.call_args_list]
            self.assertTrue(any("boxplot_normalized" in s for s in stems))
        finally:
            shutil.rmtree(temp_base)
            agg.OPTIMIZER_COMPARISON_DIR = orig_dir

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.save_figure")
    def test_boxplot_normalized_skipped_when_no_ceiling(self, mock_save):
        """If the ideal-SP ceiling can't be computed (no YAML), only raw is drawn."""
        temp_base = tempfile.mkdtemp()
        try:
            orig_dir = agg.OPTIMIZER_COMPARISON_DIR
            agg.OPTIMIZER_COMPARISON_DIR = temp_base
            agg.FIGURES_OUTPUT_DIR = os.path.join(temp_base, "figures")

            exp_dir = os.path.join(temp_base, "tasks4_dur300_interval10_seed1000")
            # No taskset_characteristics_interval_0.yaml -> ceiling is None.
            sched_dir = os.path.join(exp_dir, "taskset_0", "INCR", "INCR")
            os.makedirs(sched_dir, exist_ok=True)
            with open(os.path.join(sched_dir, "interval_sp_metrics.txt"), "w") as f:
                for i in range(10):
                    f.write(f"{i},0.{90 + i}\n")

            cfg = {
                "num_tasks_for_single_task_figures": 4,
                "main_scheduler_list": ["INCR"],
                "analysis": {"normalize_sp": True,
                             "sp_normalization_method": "upper_bound"},
            }
            agg.generate_distribution_boxplot(cfg)
            self.assertEqual(mock_save.call_count, 1)  # raw only
        finally:
            shutil.rmtree(temp_base)
            agg.OPTIMIZER_COMPARISON_DIR = orig_dir

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.save_figure")
    def test_boxplot_scans_run_root_sim(self, mock_save):
        """P23: when run_root is set, the boxplot finds the single-task dir
        under <run_root>/sim/ (not the module-global OPTIMIZER_COMPARISON_DIR).

        The global dir is intentionally left empty so a legacy-scan fallback
        would produce nothing; only the run_root-scoped scan should find data.
        """
        temp_base = tempfile.mkdtemp()
        try:
            # Global dir is empty -> any fallback to OPTIMIZER_COMPARISON_DIR
            # finds nothing. The data lives only under <run_root>/sim/.
            orig_dir = agg.OPTIMIZER_COMPARISON_DIR
            agg.OPTIMIZER_COMPARISON_DIR = temp_base
            agg.FIGURES_OUTPUT_DIR = os.path.join(temp_base, "figures")

            run_root = os.path.join(temp_base, "run_root")
            sim_dir = os.path.join(run_root, "sim")
            exp_dir = os.path.join(sim_dir, "tasks4_dur300_interval10_seed1000")
            sched_dir = os.path.join(exp_dir, "taskset_0", "INCR", "INCR")
            os.makedirs(sched_dir, exist_ok=True)
            with open(os.path.join(sched_dir, "interval_sp_metrics.txt"), "w") as f:
                for i in range(10):
                    f.write(f"{i},0.{90 + i}\n")

            cfg = {"num_tasks_for_single_task_figures": 4,
                   "main_scheduler_list": ["INCR"]}
            agg.generate_distribution_boxplot(cfg, run_root=run_root)
            mock_save.assert_called_once()
        finally:
            shutil.rmtree(temp_base)
            agg.OPTIMIZER_COMPARISON_DIR = orig_dir

    @unittest.mock.patch("simulation_experiments.aggregate_across_tasks.MATPLOTLIB_AVAILABLE", True)
    def test_boxplot_no_experiment_dir(self):
        temp_base = tempfile.mkdtemp()
        try:
            orig_dir = agg.OPTIMIZER_COMPARISON_DIR
            agg.OPTIMIZER_COMPARISON_DIR = temp_base
            cfg = {"num_tasks_for_single_task_figures": 6, "main_scheduler_list": ["INCR"]}
            # Should not crash when no experiment dir exists
            agg.generate_distribution_boxplot(cfg)
        finally:
            shutil.rmtree(temp_base)
            agg.OPTIMIZER_COMPARISON_DIR = orig_dir


class TestCopyConfigIntoRunRoot(unittest.TestCase):
    """P23: the driving config JSON is copied into the run root so a run is
    self-describing."""

    def test_copies_config_json_into_run_root(self):
        import json
        tmp = tempfile.mkdtemp()
        try:
            # Source config file outside the run root.
            src = os.path.join(tmp, "my_config.json")
            with open(src, "w") as f:
                json.dump({"mode": "test"}, f)
            run_root = os.path.join(tmp, "runs", "run_test_x")
            os.makedirs(run_root, exist_ok=True)

            agg._copy_config_into_run_root({"_config_source_path": src}, run_root)

            dst = os.path.join(run_root, "config.json")
            self.assertTrue(os.path.exists(dst))
            with open(dst) as f:
                self.assertEqual(json.load(f), {"mode": "test"})
        finally:
            shutil.rmtree(tmp)

    def test_skips_when_source_already_in_run_root(self):
        """If the config path already resolves to the run root's config.json,
        nothing is copied (no clobber / no error)."""
        tmp = tempfile.mkdtemp()
        try:
            run_root = os.path.join(tmp, "runs", "run_test_y")
            os.makedirs(run_root, exist_ok=True)
            dst = os.path.join(run_root, "config.json")
            with open(dst, "w") as f:
                f.write("ORIGINAL")
            mtime_before = os.path.getmtime(dst)

            # Source == destination (already in run root).
            agg._copy_config_into_run_root(
                {"_config_source_path": dst}, run_root)

            with open(dst) as f:
                self.assertEqual(f.read(), "ORIGINAL")  # untouched
            self.assertEqual(os.path.getmtime(dst), mtime_before)
        finally:
            shutil.rmtree(tmp)

    def test_skips_silently_when_no_source_path(self):
        """No _config_source_path -> no copy, no crash."""
        tmp = tempfile.mkdtemp()
        try:
            run_root = os.path.join(tmp, "runs", "run_test_z")
            os.makedirs(run_root, exist_ok=True)
            agg._copy_config_into_run_root({}, run_root)
            self.assertFalse(os.path.exists(os.path.join(run_root, "config.json")))
        finally:
            shutil.rmtree(tmp)


class TestIntegrationStyleFigureGeneration(unittest.TestCase):
    """End-to-end style test: create mock CSVs, set up matplotlib, generate figures."""

    def setUp(self):
        self.temp_base = tempfile.mkdtemp()
        self.orig_comparison_dir = agg.OPTIMIZER_COMPARISON_DIR
        self.orig_figures_dir = agg.FIGURES_OUTPUT_DIR
        agg.OPTIMIZER_COMPARISON_DIR = self.temp_base
        agg.FIGURES_OUTPUT_DIR = os.path.join(self.temp_base, "figures")
        setup_publication_style()

    def tearDown(self):
        shutil.rmtree(self.temp_base)
        agg.OPTIMIZER_COMPARISON_DIR = self.orig_comparison_dir
        agg.FIGURES_OUTPUT_DIR = self.orig_figures_dir

    def test_full_pipeline_with_mock_data(self):
        # Create experiment directories for 4 and 6 tasks
        for num_tasks in [4, 6]:
            exp_dir = os.path.join(self.temp_base, f"tasks{num_tasks}_test")
            rows = []
            for sched in ["INCR", "BF", "RM", "CFS"]:
                rows.append({
                    "scheduler": sched,
                    "mean_sp": 0.80 + (0.05 if sched == "INCR" else 0.0),
                    "std_sp": 0.03,
                    "mean_sched_time": 0.01 + (0.01 if sched == "BF" else 0.0),
                    "mean_miss": 0.1,
                    "std_miss": 0.02,
                    "important_miss": 0.05,
                    "non_important_miss": 0.12,
                })
            _write_summary_csv(exp_dir, rows)

        # Also create raw interval data for boxplot
        exp4 = os.path.join(self.temp_base, "tasks4_test")
        taskset_dir = os.path.join(exp4, "taskset_0")
        for sched in ["INCR", "BF", "RM", "CFS"]:
            sched_dir = os.path.join(taskset_dir, sched, sched)
            os.makedirs(sched_dir, exist_ok=True)
            with open(os.path.join(sched_dir, "interval_sp_metrics.txt"), "w") as f:
                for i in range(5):
                    f.write(f"{i},0.{85 + i}\n")

        records = agg.aggregate_data_from_directories()
        self.assertEqual(len(records), 8)  # 4 schedulers x 2 task counts

        cfg = {
            "num_tasks_for_single_task_figures": 4,
            "main_scheduler_list": ["INCR", "BF", "RM", "CFS"],
            "ablation_scheduler_list": ["BF", "INCR", "INCR_NO_TL", "INCR_WCET"],
        }

        agg.generate_main_group_figures(records, cfg)
        agg.generate_ablation_group_figures(records, cfg)
        agg.generate_important_task_miss_rate_figure(records, cfg)
        agg.generate_distribution_boxplot(cfg)

        # Verify figure files were created
        expected_files = [
            "fig1a_mean_sp_vs_tasks_main.png",
            "fig1c_mean_exec_time_vs_tasks_main.png",
            "fig_ablation_mean_sp_vs_tasks.png",
            "fig_ablation_mean_exec_time_vs_tasks.png",
            "fig3_important_task_miss_rate.png",
            "fig3b_non_important_task_miss_rate.png",
            "fig1f_sp_distribution_boxplot.png",
        ]

        # Note: debug figures (1B, 1D, 1E) are also generated
        debug_files = [
            "fig1b_std_sp_vs_tasks_main.png",
            "fig1d_mean_miss_rate_vs_tasks_main.png",
            "fig1e_std_miss_rate_vs_tasks_main.png",
        ]

        for fname in expected_files + debug_files:
            fpath = os.path.join(agg.FIGURES_OUTPUT_DIR, fname)
            self.assertTrue(os.path.exists(fpath), f"Missing figure: {fname}")
            self.assertGreater(os.path.getsize(fpath), 0, f"Empty figure: {fname}")


if __name__ == "__main__":
    unittest.main()
