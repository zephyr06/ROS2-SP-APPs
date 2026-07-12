import unittest
import os
import sys
import tempfile
import shutil
import unittest.mock

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import simulation_experiments.compare_optimizers as compare_optimizers
from simulation_experiments.compare_optimizers import (
    parse_interval_sp_metrics,
    plot_optimizer_sp_line,
    plot_optimizer_exec_time_line,
    plot_per_taskset_line,
    resolve_run_output_dir,
    ALL_SCHEDULERS,
)


class TestCompareOptimizers(unittest.TestCase):

    def test_resolve_run_output_dir_auto_name(self):
        """Auto-generated run name from parameters."""
        path = resolve_run_output_dir(
            "/base", None, num_tasks=6, n_sec=300,
            scheduler_trigger_interval=10, base_seed=42
        )
        self.assertEqual(path, "/base/tasks6_dur300_interval10_seed42")

    def test_resolve_run_output_dir_custom_name(self):
        """Custom run name overrides auto-generation."""
        path = resolve_run_output_dir(
            "/base", "my_exp_v2", num_tasks=8, n_sec=120,
            scheduler_trigger_interval=60, base_seed=99
        )
        self.assertEqual(path, "/base/my_exp_v2")

    def test_resolve_run_output_dir_run_root_nests_under_sim(self):
        """P23: when run_root is set, sims land under <run_root>/sim/<subfolder>
        so they co-locate with the figures (not at the top of output_dir)."""
        path = resolve_run_output_dir(
            "/base", None, num_tasks=6, n_sec=300,
            scheduler_trigger_interval=10, base_seed=42, run_root="/rr/runA"
        )
        self.assertEqual(path, "/rr/runA/sim/tasks6_dur300_interval10_seed42")

    def test_resolve_run_output_dir_run_root_with_custom_name(self):
        """P23: a custom run_name under run_root still nests under sim/."""
        path = resolve_run_output_dir(
            "/base", "custom_exp", num_tasks=8, n_sec=120,
            scheduler_trigger_interval=60, base_seed=99, run_root="/rr/runA"
        )
        self.assertEqual(path, "/rr/runA/sim/custom_exp")

    def test_resolve_run_output_dir_run_root_none_keeps_legacy_layout(self):
        """P23: run_root=None (standalone invocation) keeps the legacy
        <output_dir>/<subfolder> layout, so compare_optimizers stays usable
        on its own."""
        path = resolve_run_output_dir(
            "/base", None, num_tasks=6, n_sec=300,
            scheduler_trigger_interval=10, base_seed=42, run_root=None
        )
        self.assertEqual(path, "/base/tasks6_dur300_interval10_seed42")

    def test_all_schedulers_list(self):
        """Ensure the default scheduler list includes the expected modes.

        The canonical incremental arm is ``INCR_Reopt_10`` (reopt period 10) --
        the bare ``INCR`` arm was a redundant duplicate of it and was dropped
        from the default list, so the gates read the one arm actually run.
        """
        expected = {
            "INCR_Reopt_10", "BF",
            "INCR_NO_TL", "INCR_WCET",
            "RM", "RM_FAST", "RM_SLOW",
            "CFS",
        }
        self.assertEqual(set(ALL_SCHEDULERS), expected)

    def test_parse_interval_sp_metrics(self):
        """Parse correctly formatted interval SP metrics."""
        temp_dir = tempfile.mkdtemp()
        try:
            metrics_path = os.path.join(temp_dir, "interval_sp_metrics.txt")
            with open(metrics_path, "w") as f:
                f.write("0,0.95\n")
                f.write("1,0.88\n")
                f.write("2,0.92\n")
                f.write("\n")  # blank line
                f.write("bad_line\n")
            values = parse_interval_sp_metrics(metrics_path)
            self.assertEqual(values, [0.95, 0.88, 0.92])
        finally:
            shutil.rmtree(temp_dir)

    def test_parse_interval_sp_metrics_missing_file(self):
        """Gracefully handle missing metrics file."""
        values = parse_interval_sp_metrics("/nonexistent/path.txt")
        self.assertEqual(values, [])

    @unittest.mock.patch("simulation_experiments.compare_optimizers.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.compare_optimizers.save_figure")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.plt")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.np")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.matplotlib")
    def test_plot_optimizer_sp_line(self, mock_matplotlib, mock_np, mock_plt, mock_save):
        """SP line plot generation runs without error given valid data."""
        mock_np.array = lambda x: x
        mock_np.mean = lambda x: sum(x) / len(x) if x else 0.0
        mock_np.std = lambda x: 0.05
        mock_np.arange = lambda n: list(range(n))
        mock_matplotlib.colormaps = {"tab10": unittest.mock.Mock(
            side_effect=lambda i: f"color_{i}"
        )}
        mock_fig = unittest.mock.Mock()
        mock_ax = unittest.mock.Mock()
        mock_plt.subplots.return_value = (mock_fig, mock_ax)

        results = {
            "INCR": {"sp_values": [0.9, 0.95, 0.92]},
            "BF": {"sp_values": [0.85, 0.88]},
        }
        temp_dir = tempfile.mkdtemp()
        try:
            out_path = os.path.join(temp_dir, "sp_line.png")
            plot_optimizer_sp_line(results, ["INCR", "BF"], out_path)
            mock_plt.subplots.assert_called_once()
            # save_figure receives the stem (no extension); emits PNG + PDF.
            mock_save.assert_called_once_with(mock_fig, os.path.join(temp_dir, "sp_line"))
            mock_plt.close.assert_called_once_with(mock_fig)
        finally:
            shutil.rmtree(temp_dir)

    @unittest.mock.patch("simulation_experiments.compare_optimizers.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.compare_optimizers.save_figure")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.plt")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.np")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.matplotlib")
    def test_plot_optimizer_exec_time_line(self, mock_matplotlib, mock_np, mock_plt, mock_save):
        """Execution-time line plot generation runs without error."""
        mock_np.array = lambda x: x
        mock_np.mean = lambda x: sum(x) / len(x) if x else 0.0
        mock_np.std = lambda x: 0.01
        mock_np.arange = lambda n: list(range(n))
        mock_matplotlib.colormaps = {"tab10": unittest.mock.Mock(
            side_effect=lambda i: f"color_{i}"
        )}
        mock_fig = unittest.mock.Mock()
        mock_ax = unittest.mock.Mock()
        mock_plt.subplots.return_value = (mock_fig, mock_ax)

        results = {
            "INCR": {"sched_times": [1.2, 1.5, 1.3]},
            "BF": {"sched_times": [5.0, 5.5]},
        }
        temp_dir = tempfile.mkdtemp()
        try:
            out_path = os.path.join(temp_dir, "exec_line.png")
            plot_optimizer_exec_time_line(results, ["INCR", "BF"], out_path)
            mock_plt.subplots.assert_called_once()
            mock_save.assert_called_once_with(mock_fig, os.path.join(temp_dir, "exec_line"))
            mock_plt.close.assert_called_once_with(mock_fig)
        finally:
            shutil.rmtree(temp_dir)

    @unittest.mock.patch("simulation_experiments.compare_optimizers.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.compare_optimizers.save_figure")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.plt")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.np")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.matplotlib")
    def test_plot_per_taskset_line(self, mock_matplotlib, mock_np, mock_plt, mock_save):
        """Per-taskset line plot generation runs without error for multiple tasksets."""
        mock_np.array = lambda x: x
        mock_np.mean = lambda x: sum(x) / len(x) if x else 0.0

        class _MockArray(list):
            def __add__(self, other):
                return _MockArray([v + other for v in self])

        mock_np.arange = lambda n: _MockArray(range(n))
        mock_matplotlib.colormaps = {"tab10": unittest.mock.Mock(
            side_effect=lambda i: f"color_{i}"
        )}
        mock_fig = unittest.mock.Mock()
        mock_ax = unittest.mock.Mock()
        mock_plt.subplots.return_value = (mock_fig, mock_ax)

        results_by_taskset = [
            {"INCR": {"sp_values": [0.9, 0.95]}, "BF": {"sp_values": [0.85]}},
            {"INCR": {"sp_values": [0.92]}, "BF": {"sp_values": [0.88, 0.87]}},
        ]
        temp_dir = tempfile.mkdtemp()
        try:
            out_path = os.path.join(temp_dir, "per_taskset_line.png")
            plot_per_taskset_line(results_by_taskset, ["INCR", "BF"], out_path)
            mock_plt.subplots.assert_called_once()
            mock_save.assert_called_once_with(mock_fig, os.path.join(temp_dir, "per_taskset_line"))
            mock_plt.close.assert_called_once_with(mock_fig)
        finally:
            shutil.rmtree(temp_dir)


class TestNumTasksCliAcceptance(unittest.TestCase):
    """P13 Commit B: --num_tasks is no longer restricted to choices=[4,6,8],
    and any N >= 1 routes through resolve_taskset_config_path (P17 relaxed the
    former >= 2 floor to >= 1).

    We patch the resolver to raise a sentinel so we can assert that argparse
    accepted the value (no choices rejection) AND that main() routed it to the
    resolver -- without needing the C++ binary or taskset generation.
    """

    def _run_main_catching_resolver(self, cli_args):
        """Run compare_optimizers.main() with --num_tasks; expect the patched
        resolver to raise a sentinel. Returns the num_tasks the resolver saw."""
        seen = {}

        def fake_resolver(num_tasks, *a, **kw):
            seen["num_tasks"] = num_tasks
            raise _SentinelStop()

        with unittest.mock.patch.object(
            compare_optimizers, "resolve_taskset_config_path", side_effect=fake_resolver
        ), unittest.mock.patch.object(sys, "argv", ["compare_optimizers.py"] + cli_args):
            with self.assertRaises(_SentinelStop):
                compare_optimizers.main()
        return seen.get("num_tasks")

    def test_num_tasks_10_accepted_and_routed(self):
        """--num_tasks 10 is accepted (argparse no longer rejects it) and
        passed to resolve_taskset_config_path."""
        n = self._run_main_catching_resolver(
            ["--num_tasks", "10", "--n_tasksets", "1", "-v", "0"]
        )
        self.assertEqual(n, 10)

    def test_num_tasks_18_accepted_and_routed(self):
        """--num_tasks 18 is accepted and routed (well beyond the old 4/6/8)."""
        n = self._run_main_catching_resolver(
            ["--num_tasks", "18", "--n_tasksets", "1", "-v", "0"]
        )
        self.assertEqual(n, 18)

    def test_num_tasks_one_accepted_and_routed(self):
        """P17: --num_tasks 1 is accepted (the >= 2 floor was relaxed to >= 1)
        and routed to resolve_taskset_config_path (single-rate taskset)."""
        n = self._run_main_catching_resolver(
            ["--num_tasks", "1", "--n_tasksets", "1", "-v", "0"]
        )
        self.assertEqual(n, 1)

    def test_num_tasks_zero_rejected_by_argparse(self):
        """P17: --num_tasks 0 is rejected with a parser error (SystemExit)."""
        with unittest.mock.patch.object(
            sys, "argv", ["compare_optimizers.py", "--num_tasks", "0", "-v", "0"]
        ):
            with self.assertRaises(SystemExit):
                compare_optimizers.main()


class _SentinelStop(Exception):
    """Raised by the patched resolver to short-circuit main() after the
    routing assertion point (avoids needing the C++ binary)."""


if __name__ == "__main__":
    unittest.main()
