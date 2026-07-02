"""Tests for simulation_experiments.plotting_config.

These tests verify that the plotting configuration utilities produce valid
output files (PNG + PDF) with synthetic data, without requiring real
simulation results.
"""
import os
import sys
import tempfile
import unittest

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from simulation_experiments.plotting_config import (
    setup_publication_style,
    get_scheduler_color_map,
    save_figure,
    DEFAULT_OUTPUT_FORMAT_LIST,
)


try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


@unittest.skipUnless(MATPLOTLIB_AVAILABLE, "matplotlib not available")
class TestSetupPublicationStyle(unittest.TestCase):

    def test_rcparams_set(self):
        setup_publication_style(
            font_base_size_points=16,
            font_axis_label_size_points=18,
            font_title_size_points=20,
        )
        self.assertEqual(plt.rcParams["font.size"], 16)
        self.assertEqual(plt.rcParams["axes.labelsize"], 18)
        self.assertEqual(plt.rcParams["axes.titlesize"], 20)
        self.assertTrue(plt.rcParams["axes.grid"])
        self.assertEqual(plt.rcParams["grid.linestyle"], "--")

    def test_default_values(self):
        setup_publication_style()
        self.assertEqual(plt.rcParams["font.size"], 14)
        self.assertEqual(plt.rcParams["axes.labelsize"], 16)
        self.assertEqual(plt.rcParams["axes.titlesize"], 18)


@unittest.skipUnless(MATPLOTLIB_AVAILABLE, "matplotlib not available")
class TestGetSchedulerColorMap(unittest.TestCase):

    def test_color_map_length(self):
        schedulers = ["INCR", "BF", "RM", "CFS"]
        cmap = get_scheduler_color_map(schedulers)
        self.assertEqual(len(cmap), 4)
        for s in schedulers:
            self.assertIn(s, cmap)
            # Verify it's a color tuple (rgba typically)
            self.assertTrue(isinstance(cmap[s], tuple))
            self.assertGreaterEqual(len(cmap[s]), 3)

    def test_position_based_coloring(self):
        # Colors are assigned by position in the scheduler list, not by name.
        # The first scheduler always gets the first palette color.
        schedulers1 = ["A", "B", "C"]
        cmap1 = get_scheduler_color_map(schedulers1)
        schedulers2 = ["C", "A", "B"]
        cmap2 = get_scheduler_color_map(schedulers2)
        # Position 0 color should match
        self.assertEqual(cmap1["A"], cmap2["C"])
        self.assertEqual(cmap1["B"], cmap2["A"])
        self.assertEqual(cmap1["C"], cmap2["B"])

    def test_different_palettes(self):
        schedulers = ["X", "Y"]
        cmap1 = get_scheduler_color_map(schedulers, palette_name="tab10")
        cmap2 = get_scheduler_color_map(schedulers, palette_name="tab10")
        self.assertEqual(cmap1["X"], cmap2["X"])


@unittest.skipUnless(MATPLOTLIB_AVAILABLE, "matplotlib not available")
class TestSaveFigure(unittest.TestCase):

    def test_save_png_and_pdf(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            output_stem = os.path.join(tmpdir, "test_figure")
            fig, ax = plt.subplots()
            ax.plot([0, 1, 2], [0, 1, 4])
            ax.set_title("Test")
            save_figure(fig, output_stem, format_list=["png", "pdf"])
            plt.close(fig)

            for fmt in ["png", "pdf"]:
                fpath = f"{output_stem}.{fmt}"
                self.assertTrue(os.path.exists(fpath), f"Missing {fmt}")
                self.assertGreater(os.path.getsize(fpath), 0, f"Empty {fmt}")

    def test_save_single_format(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            output_stem = os.path.join(tmpdir, "single")
            fig, ax = plt.subplots()
            ax.bar(["A", "B"], [1, 2])
            save_figure(fig, output_stem, format_list=["png"], dpi=150)
            plt.close(fig)

            fpath = f"{output_stem}.png"
            self.assertTrue(os.path.exists(fpath))
            self.assertGreater(os.path.getsize(fpath), 0)
            # PDF should NOT exist
            self.assertFalse(os.path.exists(f"{output_stem}.pdf"))

    def test_creates_directories(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            nested = os.path.join(tmpdir, "a", "b", "c")
            output_stem = os.path.join(nested, "fig")
            fig, ax = plt.subplots()
            ax.plot([0, 1], [1, 0])
            save_figure(fig, output_stem)
            plt.close(fig)
            self.assertTrue(os.path.exists(f"{output_stem}.png"))


@unittest.skipUnless(MATPLOTLIB_AVAILABLE, "matplotlib not available")
class TestPublicationQualityFigures(unittest.TestCase):
    """End-to-end tests: generate publication-style figures with synthetic data."""

    def setUp(self):
        setup_publication_style()

    def test_line_chart(self):
        """Simulate Fig 1A-style multi-line chart."""
        with tempfile.TemporaryDirectory() as tmpdir:
            schedulers = ["INCR", "BF", "RM", "CFS"]
            color_map = get_scheduler_color_map(schedulers)
            num_tasks_list = [4, 6, 8]

            fig, ax = plt.subplots(figsize=(12, 6))
            x = range(len(num_tasks_list))

            for sched in schedulers:
                means = [0.85 + 0.03 * (j + 1) + (0.05 if sched == "INCR" else 0) for j in range(len(num_tasks_list))]
                stds = [0.03] * len(num_tasks_list)
                ax.errorbar(list(x), means, yerr=stds, marker="o", markersize=8,
                            linewidth=2, capsize=4, label=sched,
                            color=color_map.get(sched, "gray"))

            ax.set_xticks(list(x))
            ax.set_xticklabels([str(n) for n in num_tasks_list])
            ax.set_xlabel("Number of Tasks")
            ax.set_ylabel("Mean SP-Metric")
            ax.set_title("Mean SP-Metric vs. Number of Tasks")
            ax.legend()

            output_stem = os.path.join(tmpdir, "fig1a_test")
            save_figure(fig, output_stem)
            plt.close(fig)

            for fmt in ["png", "pdf"]:
                fpath = f"{output_stem}.{fmt}"
                self.assertTrue(os.path.exists(fpath))
                self.assertGreater(os.path.getsize(fpath), 0)

    def test_box_plot(self):
        """Simulate Fig 1F-style box plot."""
        with tempfile.TemporaryDirectory() as tmpdir:
            import numpy as np
            schedulers = ["INCR", "BF", "RM", "CFS"]
            color_map = get_scheduler_color_map(schedulers)

            fig, ax = plt.subplots(figsize=(10, 6))
            data = [np.random.normal(0.9 - i * 0.02, 0.03, 100) for i in range(len(schedulers))]
            bp = ax.boxplot(data, labels=schedulers, patch_artist=True)
            for patch, label in zip(bp["boxes"], schedulers):
                patch.set_facecolor(color_map.get(label, "lightgray"))
            ax.set_ylabel("SP-Metric Value")
            ax.set_title("SP-Metric Distribution (6 Tasks)")

            output_stem = os.path.join(tmpdir, "fig1f_test")
            save_figure(fig, output_stem)
            plt.close(fig)

            for fmt in ["png", "pdf"]:
                fpath = f"{output_stem}.{fmt}"
                self.assertTrue(os.path.exists(fpath))
                self.assertGreater(os.path.getsize(fpath), 0)

    def test_line_chart_with_errorbars(self):
        """Simulate Fig 2-style line chart with error bars."""
        with tempfile.TemporaryDirectory() as tmpdir:
            intervals = [5, 10, 20, 30, 60]
            means = [0.92, 0.90, 0.87, 0.85, 0.82]
            stds = [0.02, 0.03, 0.02, 0.03, 0.04]
            color_map = get_scheduler_color_map(["INCR"])

            fig, ax = plt.subplots(figsize=(8, 6))
            ax.errorbar(intervals, means, yerr=stds, marker="o", markersize=8,
                        linewidth=2, color=color_map.get("INCR", "gray"),
                        capsize=4, label="INCR")

            # Add horizontal reference lines
            ax.axhline(0.85, linestyle="--", linewidth=1.5, label="BF (avg)", color="gray")
            ax.axhline(0.80, linestyle="--", linewidth=1.5, label="RM (avg)", color="lightgray")

            ax.set_xlabel("Trigger Interval (s)")
            ax.set_ylabel("Mean SP-Metric")
            ax.set_title("INCR SP-Metric vs. Optimizer Invocation Interval")
            ax.set_xticks(intervals)
            ax.legend()

            output_stem = os.path.join(tmpdir, "fig2_test")
            save_figure(fig, output_stem)
            plt.close(fig)

            for fmt in ["png", "pdf"]:
                fpath = f"{output_stem}.{fmt}"
                self.assertTrue(os.path.exists(fpath))
                self.assertGreater(os.path.getsize(fpath), 0)

    def test_single_line_chart(self):
        """Simulate Fig 3-style single line chart."""
        with tempfile.TemporaryDirectory() as tmpdir:
            schedulers = ["INCR", "BF", "RM", "CFS"]
            color_map = get_scheduler_color_map(schedulers)
            means = [0.05, 0.08, 0.10, 0.12]

            fig, ax = plt.subplots(figsize=(12, 6))
            x = list(range(len(schedulers)))
            ax.errorbar(x, means, marker="o", markersize=8, linewidth=2,
                        color=color_map.get(schedulers[0], "gray"))
            ax.set_xticks(x)
            ax.set_xticklabels(schedulers)
            ax.set_ylabel("Important-Task Miss Rate")
            ax.set_title("Important-Task Miss Rate (6 Tasks)")

            output_stem = os.path.join(tmpdir, "fig3_test")
            save_figure(fig, output_stem)
            plt.close(fig)

            for fmt in ["png", "pdf"]:
                fpath = f"{output_stem}.{fmt}"
                self.assertTrue(os.path.exists(fpath))
                self.assertGreater(os.path.getsize(fpath), 0)


if __name__ == "__main__":
    unittest.main()
