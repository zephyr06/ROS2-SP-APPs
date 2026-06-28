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

from simulation_experiments.compare_optimizers import (
    parse_interval_sp_metrics,
    plot_optimizer_bar_comparison,
    plot_optimizer_exec_time,
    plot_per_taskset_radar,
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

    def test_all_schedulers_list(self):
        """Ensure the default scheduler list includes the expected modes."""
        expected = {
            "INCR", "BF",
            "INCR_NO_TL", "INCR_WCET", "INCR_SCRATCH",
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
    @unittest.mock.patch("simulation_experiments.compare_optimizers.plt")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.np")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.matplotlib")
    def test_plot_optimizer_bar_comparison(self, mock_matplotlib, mock_np, mock_plt):
        """Bar plot generation runs without error given valid data."""
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
            out_path = os.path.join(temp_dir, "bar.png")
            plot_optimizer_bar_comparison(results, ["INCR", "BF"], out_path)
            mock_plt.subplots.assert_called_once()
            mock_plt.savefig.assert_called_once_with(out_path, dpi=300)
            mock_plt.close.assert_called_once()
        finally:
            shutil.rmtree(temp_dir)

    @unittest.mock.patch("simulation_experiments.compare_optimizers.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.compare_optimizers.plt")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.np")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.matplotlib")
    def test_plot_optimizer_exec_time(self, mock_matplotlib, mock_np, mock_plt):
        """Execution-time plot generation runs without error."""
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
            out_path = os.path.join(temp_dir, "exec.png")
            plot_optimizer_exec_time(results, ["INCR", "BF"], out_path)
            mock_plt.subplots.assert_called_once()
            mock_plt.savefig.assert_called_once_with(out_path, dpi=300)
            mock_plt.close.assert_called_once()
        finally:
            shutil.rmtree(temp_dir)

    @unittest.mock.patch("simulation_experiments.compare_optimizers.MATPLOTLIB_AVAILABLE", True)
    @unittest.mock.patch("simulation_experiments.compare_optimizers.plt")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.np")
    @unittest.mock.patch("simulation_experiments.compare_optimizers.matplotlib")
    def test_plot_per_taskset_radar(self, mock_matplotlib, mock_np, mock_plt):
        """Per-taskset plot generation runs without error for multiple tasksets."""
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
            out_path = os.path.join(temp_dir, "radar.png")
            plot_per_taskset_radar(results_by_taskset, ["INCR", "BF"], out_path)
            mock_plt.subplots.assert_called_once()
            mock_plt.savefig.assert_called_once_with(out_path, dpi=300)
            mock_plt.close.assert_called_once()
        finally:
            shutil.rmtree(temp_dir)


if __name__ == "__main__":
    unittest.main()
