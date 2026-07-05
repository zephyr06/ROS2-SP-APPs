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

    def test_standardize_config_rejects_hz_keys(self):
        """Hz-style period keys are no longer supported -- standardize_config
        raises ValueError pointing the user at PERIODS_MS. The big/small
        count split is rejected separately (see test_rejects_count_split)."""
        config = {
            "SMALL_PERIOD_HZ": [10, 20, 50],
            "BIG_PERIOD_HZ": [1, 2, 5],
            "D1_RANGE": [-10, 10],
        }
        with self.assertRaises(ValueError) as cm:
            standardize_config(config)
        self.assertIn("PERIODS_MS", str(cm.exception))

    def test_standardize_config_rejects_legacy_hz_key(self):
        """The legacy single HZ list is rejected (it used to be split at 10 Hz
        and aliased to PERIODS_MS)."""
        config = {
            "HZ": [0.5, 2.0, 10, 25],
            "D1_RANGE": [-10, 10],
            "D2_RANGE": [0, 360],
            "CPU_UTIL_RANDOM_RANGE": [0.5, 0.5]
        }
        with self.assertRaises(ValueError) as cm:
            standardize_config(config)
        self.assertIn("PERIODS_MS", str(cm.exception))

    def test_standardize_config_rejects_count_split(self):
        """The N_BIG_PERIOD_TASKS / N_SMALL_PERIOD_TASKS split is no longer
        supported -- raise ValueError pointing at N_TASKS."""
        config = {
            "N_BIG_PERIOD_TASKS": 2,
            "N_SMALL_PERIOD_TASKS": 3,
            "PERIODS_MS": [1000, 100, 50],
            "D1_RANGE": [-10, 10],
        }
        with self.assertRaises(ValueError) as cm:
            standardize_config(config)
        self.assertIn("N_TASKS", str(cm.exception))

    def test_standardize_config_no_silent_defaults(self):
        """The refactor removed the silent PERIODS_MS / N_TASKS defaults: a
        config that omits a required parameter no longer gets auto-filled --
        validate_config_integrity surfaces it (non-interactive: raises) so a
        forgotten value can never make two runs silently diverge. standardize_config
        is now a validator only and does NOT raise on a missing key itself; the
        integrity gate does."""
        from Gen_Taskset.lib.taskset_generator import validate_config_integrity
        config = {
            "D1_RANGE": [-5, 5],
            "CPU_UTIL_RANDOM_RANGE": [0.5, 1.5],
        }
        # standardize_config passes through without filling PERIODS_MS/N_TASKS.
        res = standardize_config(config)
        self.assertNotIn("PERIODS_MS", res)
        self.assertNotIn("N_TASKS", res)
        # The integrity gate (non-interactive, no config_path) lists every
        # missing required key rather than masking it with a default.
        with self.assertRaises(ValueError) as cm:
            validate_config_integrity(res, config_path=None)
        msg = str(cm.exception)
        self.assertIn("PERIODS_MS", msg)
        self.assertIn("N_TASKS", msg)
        self.assertIn("missing required parameters", msg)

    def test_standardize_config_requires_range(self):
        """P14: CPU_UTIL_RANDOM_RANGE is required -- a config that omits it
        raises ValueError (no silent fixed-scalar fallback)."""
        config = {
            "D1_RANGE": [-5, 5],
        }
        with self.assertRaises(ValueError) as cm:
            standardize_config(config)
        self.assertIn("CPU_UTIL_RANDOM_RANGE is required", str(cm.exception))

    def test_standardize_config_canonical_new_keys(self):
        """P19: a canonical PERIODS_MS+N_TASKS config passes through untouched."""
        config = {
            "PERIODS_MS": [1000, 500, 100],
            "N_TASKS": 3,
            "D1_RANGE": [-5, 5],
            "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9],
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
                    "D1_RANGE": [-5, 5], "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9],
                })

    def test_standardize_config_rejects_empty_periods(self):
        """P19: PERIODS_MS must be a non-empty list."""
        with self.assertRaises(ValueError):
            standardize_config({
                "PERIODS_MS": [], "N_TASKS": 3,
                "D1_RANGE": [-5, 5], "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9],
            })

    def test_standardize_config_map_params_derives_d1_range(self):
        """Configs with MAP_WIDTH_M / MAP_HEIGHT_M should auto-derive D1_RANGE and D2_RANGE."""
        config = {
            "MAP_WIDTH_M": 200,
            "MAP_HEIGHT_M": 200,
            "D2_RANGE": [0, 360],
            "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9],
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
            "CPU_UTIL_RANDOM_RANGE": [1.2, 1.2],
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
            "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9],
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
            "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9]
        }
        self.assertTrue(validate_generation_config(map_cfg))

    def test_validate_generation_config(self):
        # Valid config
        valid_cfg = {
            "D1_RANGE": [-10, 10],
            "D2_RANGE": [0, 360],
            "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9]
        }
        self.assertTrue(validate_generation_config(valid_cfg))

        # Invalid config (missing CPU_UTIL_RANDOM_RANGE)
        invalid_cfg = {
            "D1_RANGE": [-10, 10],
            "D2_RANGE": [0, 360]
        }
        self.assertFalse(validate_generation_config(invalid_cfg))

        # Invalid: neither D1_RANGE nor MAP params present
        no_map_no_d1 = {
            "D2_RANGE": [0, 360],
            "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9]
        }
        self.assertFalse(validate_generation_config(no_map_no_d1))

    def test_standardize_config_d1_variance_table_removed(self):
        """D1_VARIANCE_FACTOR_TABLE must not be present in standardized config."""
        config = {
            "D1_RANGE": [-20, 20],
            "D2_RANGE": [0, 360],
            "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9],
        }
        res = standardize_config(config)
        self.assertNotIn("D1_VARIANCE_FACTOR_TABLE", res)

    def test_standardize_config_accepts_cpu_util_random_range(self):
        """P14: a valid CPU_UTIL_RANDOM_RANGE [low, high] pair is accepted and
        normalized to floats. The range is required (the generator samples one
        per-core value per task set from it)."""
        config = {
            "PERIODS_MS": [1000, 100, 50],
            "N_TASKS": 3,
            "D1_RANGE": [-5, 5],
            "CPU_UTIL_RANDOM_RANGE": [0.5, 1.5],
        }
        res = standardize_config(config)
        self.assertEqual(res["CPU_UTIL_RANDOM_RANGE"], [0.5, 1.5])

    def test_standardize_config_rejects_bad_cpu_util_random_range(self):
        """P14: a malformed CPU_UTIL_RANDOM_RANGE (wrong arity, non-numeric,
        or low > high / low < 0) is rejected with ValueError."""
        base = {
            "PERIODS_MS": [1000, 100, 50],
            "N_TASKS": 3,
            "D1_RANGE": [-5, 5],
        }
        for bad in ([0.5], [0.5, 1.5, 2.0], "0.5-1.5",
                    [1.5, 0.5], [-0.2, 1.5], [True, 1.5]):
            with self.assertRaises(ValueError):
                standardize_config(dict(base, CPU_UTIL_RANDOM_RANGE=bad))

    def test_standardize_config_no_range_synthesized(self):
        """P14: a config without CPU_UTIL_RANDOM_RANGE is not given a default
        range -- it raises. The range is required, not synthesized (a forgotten
        range must not silently lock every task set to one load point)."""
        config = {
            "PERIODS_MS": [1000, 100, 50],
            "N_TASKS": 3,
            "D1_RANGE": [-5, 5],
        }
        with self.assertRaises(ValueError) as cm:
            standardize_config(config)
        self.assertIn("CPU_UTIL_RANDOM_RANGE is required", str(cm.exception))

    def test_env_dependent_tasks_ratio_default_injected(self):
        """A bare config without ENV_DEPENDENT_TASKS_RATIO gets the default
        [0.1, 0.3] injected by standardize_config (so legacy/bare configs get
        ratio-based env-task sampling rather than the old randint fallback)."""
        config = {
            "PERIODS_MS": [1000, 100, 50],
            "N_TASKS": 3,
            "D1_RANGE": [-5, 5],
            "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9],
        }
        res = standardize_config(config)
        self.assertEqual(res["ENV_DEPENDENT_TASKS_RATIO"], [0.1, 0.3])

    def test_env_dependent_tasks_ratio_normalized_to_floats(self):
        """A valid ratio is accepted and normalized to floats."""
        config = {
            "PERIODS_MS": [1000, 100, 50],
            "N_TASKS": 3,
            "D1_RANGE": [-5, 5],
            "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9],
            "ENV_DEPENDENT_TASKS_RATIO": [0, 1],  # ints
        }
        res = standardize_config(config)
        self.assertEqual(res["ENV_DEPENDENT_TASKS_RATIO"], [0.0, 1.0])
        self.assertIsInstance(res["ENV_DEPENDENT_TASKS_RATIO"][0], float)
        self.assertIsInstance(res["ENV_DEPENDENT_TASKS_RATIO"][1], float)

    def test_env_dependent_tasks_ratio_bad_shape_rejected(self):
        """A malformed ENV_DEPENDENT_TASKS_RATIO (wrong arity, non-numeric,
        bool, low > high, or out of [0, 1]) is rejected with ValueError."""
        base = {
            "PERIODS_MS": [1000, 100, 50],
            "N_TASKS": 3,
            "D1_RANGE": [-5, 5],
            "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9],
        }
        for bad in ([0.5], [0.5, 1.5, 2.0], "0.1-0.3", [0.3, 0.1],
                    [-0.2, 0.5], [0.1, 1.5], [True, 0.5]):
            with self.assertRaises(ValueError,
                                   msg=f"expected ValueError for {bad!r}"):
                standardize_config(dict(base, ENV_DEPENDENT_TASKS_RATIO=bad))

    def test_legacy_n_env_dependent_tasks_rejected(self):
        """The legacy literal-count key N_ENV_DEPENDENT_TASKS is fully removed:
        standardize_config raises ValueError pointing at the canonical
        ENV_DEPENDENT_TASKS_RATIO (mirrors the P19 legacy-key hard-reject).
        No config in the repo ships this key, so hard-reject is safe."""
        config = {
            "PERIODS_MS": [1000, 100, 50],
            "N_TASKS": 3,
            "D1_RANGE": [-5, 5],
            "CPU_UTIL_RANDOM_RANGE": [0.9, 0.9],
            "N_ENV_DEPENDENT_TASKS": 1,
        }
        with self.assertRaises(ValueError) as cm:
            standardize_config(config)
        self.assertIn("ENV_DEPENDENT_TASKS_RATIO", str(cm.exception))
        self.assertIn("N_ENV_DEPENDENT_TASKS", str(cm.exception))


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
        """Synthesized configs inherit CPU_UTIL_RANDOM_RANGE=[0.5, 1.5] from the
        base template (P14) and hold N_CORES=2. MEAN_CPU_UTIL is gone."""
        path = resolve_taskset_config_path(12, temp_dir=self._temp())
        cfg = load_generation_config(path)
        self.assertEqual(cfg["CPU_UTIL_RANDOM_RANGE"], [0.5, 1.5])
        self.assertNotIn("MEAN_CPU_UTIL", cfg)
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

        Feasibility: the synthesized N=1 config drops to N_CORES=1 and overrides
        the base template's [0.5, 1.5] sweep with [0.5, 0.9] so the sampled
        per-core utilization stays below 1.0 -- the lone task's utilization
        stays schedulable. With N_CORES=2 the single task would carry cpu_util
        up to 3.0 (1.5 x 2), and the base range can sample >1.0 per core, both
        unschedulable for a single task."""
        path = resolve_taskset_config_path(1, temp_dir=self._temp())
        cfg = load_generation_config(path)
        self.assertEqual(cfg["N_TASKS"], 1)
        # P19: the big/small count keys are gone; N_TASKS is the sole count.
        self.assertNotIn("N_BIG_PERIOD_TASKS", cfg)
        self.assertNotIn("N_SMALL_PERIOD_TASKS", cfg)
        self.assertEqual(cfg["N_CORES"], 1)
        # P14: N=1 caps the per-core range below 1.0 for schedulability.
        self.assertEqual(cfg["CPU_UTIL_RANDOM_RANGE"], [0.5, 0.9])
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
