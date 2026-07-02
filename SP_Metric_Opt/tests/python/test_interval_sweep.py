"""Tests for simulation_experiments/interval_sweep.py -- Fig 2 + normalization."""
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


if __name__ == "__main__":
    unittest.main()
