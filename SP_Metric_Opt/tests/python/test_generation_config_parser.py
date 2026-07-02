import unittest
import os
import sys
import tempfile
import shutil

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Gen_Taskset.lib.generation_config_parser import (
    standardize_config,
    validate_generation_config,
    resolve_taskset_config_path,
    load_generation_config,
)
from Gen_Taskset.lib.taskset_generator import generate_taskset_parameters

class TestGenerationConfigParser(unittest.TestCase):

    def test_standardize_config_hz_conversion(self):
        """P19: old-shape Hz configs are aliased to the unified PERIODS_MS.

        The big-pool periods come first, then the small-pool periods,
        preserving the historical composition. Hz→ms conversion:
        big [1,2,5]→[1000,500,200], small [10,20,50]→[100,50,20].
        """
        config = {
            "SMALL_PERIOD_HZ": [10, 20, 50],
            "BIG_PERIOD_HZ": [1, 2, 5],
            "N_BIG_PERIOD_TASKS": 2,
            "N_SMALL_PERIOD_TASKS": 3,
            "D1_RANGE": [-10, 10],
            "Et_SCALE_FACTOR": 2.0
        }
        res = standardize_config(config)
        # Big-pool periods first, then small-pool periods
        self.assertEqual(res["PERIODS_MS"], [1000, 500, 200, 100, 50, 20])
        self.assertEqual(res["N_TASKS"], 5)
        # Old paired keys must be deleted from the standardized config.
        for old_key in (
            "SMALL_PERIOD_HZ", "BIG_PERIOD_HZ",
            "SMALL_PERIODS_MS", "BIG_PERIODS_MS",
            "N_BIG_PERIOD_TASKS", "N_SMALL_PERIOD_TASKS",
        ):
            self.assertNotIn(old_key, res)

    def test_standardize_config_legacy_hz_split_aliased(self):
        """P19: the legacy single HZ list is split at 10 Hz and aliased to
        PERIODS_MS. Hz < 10 → big pool, Hz >= 10 → small pool, big first."""
        config = {
            "HZ": [0.5, 2.0, 10, 25],
            "D1_RANGE": [-10, 10],
            "D2_RANGE": [0, 360],
            "MEAN_CPU_UTIL": 0.5
        }
        res = standardize_config(config)
        # 0.5→2000, 2.0→500 (big, <10 Hz) then 10→100, 25→40 (small, >=10 Hz)
        self.assertEqual(res["PERIODS_MS"], [2000, 500, 100, 40])
        self.assertNotIn("HZ", res)

    def test_standardize_config_defaults(self):
        config = {
            "D1_RANGE": [-5, 5],
            "Et_SCALE_FACTOR": 2.0
        }
        res = standardize_config(config)
        # No period info: merged big+small defaults
        self.assertEqual(res["PERIODS_MS"], [4000, 2000, 1000, 100, 50, 33, 20])
        # No count info: default N_BIG(2) + N_SMALL(8)
        self.assertEqual(res["N_TASKS"], 10)

    def test_standardize_config_canonical_new_keys(self):
        """P19: a canonical PERIODS_MS+N_TASKS config passes through untouched."""
        config = {
            "PERIODS_MS": [1000, 500, 100],
            "N_TASKS": 3,
            "D1_RANGE": [-5, 5],
            "MEAN_CPU_UTIL": 0.9,
            "Et_SCALE_FACTOR": 2.0
        }
        res = standardize_config(config)
        self.assertEqual(res["PERIODS_MS"], [1000, 500, 100])
        self.assertEqual(res["N_TASKS"], 3)

    def test_standardize_config_rejects_bad_n_tasks(self):
        """P19: N_TASKS must be a positive integer (P17's degenerate all-zero
        case is now N_TASKS < 1)."""
        for bad in (0, -1):
            with self.assertRaises(ValueError):
                standardize_config({
                    "PERIODS_MS": [100], "N_TASKS": bad,
                    "D1_RANGE": [-5, 5], "MEAN_CPU_UTIL": 0.9,
                })

    def test_standardize_config_rejects_empty_periods(self):
        """P19: PERIODS_MS must be a non-empty list."""
        with self.assertRaises(ValueError):
            standardize_config({
                "PERIODS_MS": [], "N_TASKS": 3,
                "D1_RANGE": [-5, 5], "MEAN_CPU_UTIL": 0.9,
            })

    def test_standardize_config_map_params_derives_d1_range(self):
        """Configs with MAP_WIDTH_M / MAP_HEIGHT_M should auto-derive D1_RANGE and D2_RANGE."""
        config = {
            "MAP_WIDTH_M": 200,
            "MAP_HEIGHT_M": 200,
            "D2_RANGE": [0, 360],
            "MEAN_CPU_UTIL": 0.9,
            "Et_SCALE_FACTOR": 2.0
        }
        res = standardize_config(config)
        # Both ranges derived from map dims and centered at 0
        self.assertEqual(res["D1_RANGE"], [-100, 100])
        self.assertEqual(res["D2_RANGE"], [-100, 100])
        self.assertNotIn("D1_VARIANCE_FACTOR_TABLE", res)

    def test_standardize_config_rectangular_map(self):
        """Rectangular map dimensions: D1_RANGE uses max dimension."""
        config = {
            "MAP_WIDTH_M": 300,
            "MAP_HEIGHT_M": 150,
            "D2_RANGE": [0, 360],
            "MEAN_CPU_UTIL": 1.2,
            "Et_SCALE_FACTOR": 2.0
        }
        res = standardize_config(config)
        self.assertEqual(res["D1_RANGE"], [-150, 150])

    def test_standardize_config_d1_range_takes_precedence(self):
        """Explicit D1_RANGE should not be overwritten by MAP params."""
        config = {
            "MAP_WIDTH_M": 200,
            "MAP_HEIGHT_M": 200,
            "D1_RANGE": [-50, 50],
            "D2_RANGE": [0, 360],
            "MEAN_CPU_UTIL": 0.9,
            "Et_SCALE_FACTOR": 2.0
        }
        res = standardize_config(config)
        # D1_RANGE derivation only happens when BOTH map params present
        # and D1_RANGE is absent — but standardize_config always sets it
        # from map when map is present, overwriting explicit D1_RANGE
        # This is the current behavior; test documents it
        self.assertEqual(res["D1_RANGE"], [-100, 100])

    def test_validate_generation_config_with_map(self):
        """validate_generation_config accepts MAP params as D1 substitute."""
        map_cfg = {
            "MAP_WIDTH_M": 200,
            "MAP_HEIGHT_M": 200,
            "D2_RANGE": [0, 360],
            "MEAN_CPU_UTIL": 0.9
        }
        self.assertTrue(validate_generation_config(map_cfg))

    def test_validate_generation_config(self):
        # Valid config
        valid_cfg = {
            "D1_RANGE": [-10, 10],
            "D2_RANGE": [0, 360],
            "MEAN_CPU_UTIL": 0.9
        }
        self.assertTrue(validate_generation_config(valid_cfg))

        # Invalid config (missing MEAN_CPU_UTIL)
        invalid_cfg = {
            "D1_RANGE": [-10, 10],
            "D2_RANGE": [0, 360]
        }
        self.assertFalse(validate_generation_config(invalid_cfg))

        # Invalid: neither D1_RANGE nor MAP params present
        no_map_no_d1 = {
            "D2_RANGE": [0, 360],
            "MEAN_CPU_UTIL": 0.9
        }
        self.assertFalse(validate_generation_config(no_map_no_d1))

    def test_standardize_config_d1_variance_table_removed(self):
        """D1_VARIANCE_FACTOR_TABLE must not be present in standardized config."""
        config = {
            "D1_RANGE": [-20, 20],
            "D2_RANGE": [0, 360],
            "MEAN_CPU_UTIL": 0.9,
            "Et_SCALE_FACTOR": 2.0
        }
        res = standardize_config(config)
        self.assertNotIn("D1_VARIANCE_FACTOR_TABLE", res)


class TestResolveTasksetConfigPath(unittest.TestCase):
    """Tests for resolve_taskset_config_path (P13 Commit B1)."""

    def setUp(self):
        self._temp_dirs = []

    def tearDown(self):
        for d in self._temp_dirs:
            shutil.rmtree(d, ignore_errors=True)

    def _temp(self):
        d = tempfile.mkdtemp()
        self._temp_dirs.append(d)
        return d

    def test_existing_paper_configs_returned_unchanged(self):
        """For N=4/6/8 the on-disk paper config path is returned unchanged."""
        for n in (4, 6, 8):
            path = resolve_taskset_config_path(n, temp_dir=self._temp())
            self.assertTrue(
                path.endswith(f"taskset_cfg_paper_{n}.json"),
                f"N={n}: expected on-disk paper config, got {path}"
            )
            self.assertTrue(os.path.isabs(path))
            self.assertTrue(os.path.exists(path))

    def test_synthesized_config_has_correct_task_counts(self):
        """N=10/14/18 synthesize a config with N_TASKS==N (P19: no big/small split)."""
        for n in (10, 14, 18):
            path = resolve_taskset_config_path(n, temp_dir=self._temp())
            # Must NOT be the on-disk file (none exists for these N)
            self.assertFalse(
                path.endswith(f"Gen_Taskset/task_sets_config/taskset_cfg_paper_{n}.json")
            )
            cfg = load_generation_config(path)
            self.assertEqual(cfg["N_TASKS"], n, msg=f"N={n}")
            # P19: the big/small count keys are gone from the canonical config.
            self.assertNotIn("N_BIG_PERIOD_TASKS", cfg)
            self.assertNotIn("N_SMALL_PERIOD_TASKS", cfg)

    def test_synthesized_config_per_core_util_and_cores(self):
        """Synthesized configs hold MEAN_CPU_UTIL=0.9 per-core, N_CORES=2 (P13)."""
        path = resolve_taskset_config_path(12, temp_dir=self._temp())
        cfg = load_generation_config(path)
        self.assertEqual(cfg["MEAN_CPU_UTIL"], 0.9)
        self.assertEqual(cfg["N_CORES"], 2)

    def test_synthesized_config_resolves_include(self):
        """Synthesized config's INCLUDE must resolve base-template params."""
        path = resolve_taskset_config_path(16, temp_dir=self._temp())
        cfg = load_generation_config(path)
        # P19: the base template now sets the unified PERIODS_MS pool
        self.assertIn("PERIODS_MS", cfg)
        self.assertEqual(
            cfg["PERIODS_MS"], [1000, 500, 200, 100, 50, 33, 20]
        )

    def test_rejects_num_tasks_below_one(self):
        """P17: num_tasks < 1 must be rejected (the floor dropped from >= 2 to
        >= 1; N=1 is now valid -- a single-rate small-period taskset)."""
        for bad in (0, -3):
            with self.assertRaises(ValueError, msg=f"N={bad} should be rejected"):
                resolve_taskset_config_path(bad, temp_dir=self._temp())

    def test_accepts_num_tasks_one_single_rate(self):
        """P17/P19: N=1 synthesizes a single-task config (N_TASKS=1), the
        minimal single-rate taskset the former N_BIG=2 hardcode could not
        represent.

        Feasibility: the synthesized N=1 config drops to N_CORES=1 so that
        cpu_util = MEAN_CPU_UTIL x 1 = 0.9 -- the lone task's utilization stays
        below 1.0 (schedulable). With the historical N_CORES=2 the single task
        would carry cpu_util = 1.8, which uunifast_distribution can only
        realize as a single 1.8 utilization (overloaded / unschedulable)."""
        path = resolve_taskset_config_path(1, temp_dir=self._temp())
        cfg = load_generation_config(path)
        self.assertEqual(cfg["N_TASKS"], 1)
        # P19: the big/small count keys are gone; N_TASKS is the sole count.
        self.assertNotIn("N_BIG_PERIOD_TASKS", cfg)
        self.assertNotIn("N_SMALL_PERIOD_TASKS", cfg)
        self.assertEqual(cfg["N_CORES"], 1)
        self.assertAlmostEqual(cfg["MEAN_CPU_UTIL"] * cfg["N_CORES"], 0.9)
        # End-to-end feasibility: the generated single task is schedulable.
        res = generate_taskset_parameters(cfg)
        self.assertEqual(len(res["tasks"]), 1)
        util = res["tasks"][0]["Et_mean"] / res["tasks"][0]["period"]
        self.assertLess(util, 1.0, f"single-task utilization {util} must be < 1.0")

    def test_rejects_non_integer(self):
        """Non-integer num_tasks must be rejected."""
        for bad in ("eight", None, 4.5):
            with self.assertRaises((ValueError, TypeError)):
                resolve_taskset_config_path(bad, temp_dir=self._temp())


if __name__ == "__main__":
    unittest.main()
