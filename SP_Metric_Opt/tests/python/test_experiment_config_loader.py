"""P2.8 — config layout + rename pin (red→green TDD).

Asserts the consolidated config directory layout:

    configs/
      paper_simulation_config.json   (renamed from experiment_config.json)
      gate_eval_config.json          (renamed from evaluation_suite_config.json)
      incr_et_profiling.json         (renamed from INCR_ET_Profiling.json)

and that the two removed configs are gone (`experiment_config.json` itself must
FAIL LOUDLY if referenced, not silently alias). Also pins that the load-bearing
default paths point at the new names.

This is a refactor (naming/consolidation), not a correctness change: the gate =
the loader + the eval suite still resolve their defaults to real files, and the
old names are not silently kept as aliases.
"""
import os
import sys
import unittest

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from simulation_experiments.experiment_config_loader import (
    DEFAULT_CONFIG_PATH, load_experiment_config,
)

CONFIGS_DIR = os.path.join(PROJECT_ROOT, "simulation_experiments", "configs")

EXPECTED_PRESENT = {
    "paper_simulation_config.json",
    "gate_eval_config.json",
    "incr_et_profiling.json",
}
# Old names that must NOT exist -- the renames are not silent aliases.
EXPECTED_ABSENT = {
    "experiment_config.json",
    "evaluation_suite_config.json",
    "INCR_ET_Profiling.json",
    "simulation_only_config.json",
}


class TestConfigLayout(unittest.TestCase):

    def test_expected_configs_present(self):
        actual = set(os.listdir(CONFIGS_DIR))
        missing = EXPECTED_PRESENT - actual
        self.assertFalse(missing, f"missing expected configs: {missing}")

    def test_old_config_names_absent(self):
        actual = set(os.listdir(CONFIGS_DIR))
        leftover = EXPECTED_ABSENT & actual
        self.assertFalse(
            leftover,
            f"old config names still present (rename must be total, not an alias): {leftover}")

    def test_no_extra_configs(self):
        actual = set(os.listdir(CONFIGS_DIR))
        extra = actual - EXPECTED_PRESENT
        self.assertFalse(extra, f"unexpected configs in configs/: {extra}")


class TestLoaderDefaultPath(unittest.TestCase):

    def test_default_points_at_paper_simulation_config(self):
        # The load-bearing default must follow the rename.
        self.assertEqual(os.path.basename(DEFAULT_CONFIG_PATH),
                         "paper_simulation_config.json")

    def test_default_path_exists(self):
        # The renamed default config must actually load (test mode = fast).
        cfg = load_experiment_config("test")
        self.assertIn("num_tasks_for_cross_task_comparison", cfg)


class TestStaleNameFailsLoudly(unittest.TestCase):

    def test_old_experiment_config_path_does_not_exist(self):
        stale = os.path.join(CONFIGS_DIR, "experiment_config.json")
        self.assertFalse(
            os.path.exists(stale),
            "experiment_config.json still exists -- rename was not applied")

    def test_loading_via_stale_name_raises(self):
        # Per the no-silent-alias convention (cf. P1.4 / P2.4), pointing the
        # loader at the old name must NOT silently fall back -- it raises.
        stale = os.path.join(CONFIGS_DIR, "experiment_config.json")
        with self.assertRaises(FileNotFoundError):
            load_experiment_config("test", config_path=stale)


class TestGateEvalConfigSurvives(unittest.TestCase):

    def test_gate_eval_config_carries_eval_keys(self):
        # D1=beta: the eval config is kept (renamed, not folded) BECAUSE it
        # carries the gate scheduler set + eval_* keys the suite needs.
        import json
        with open(os.path.join(CONFIGS_DIR, "gate_eval_config.json")) as f:
            raw = json.load(f)
        for mode_key in ("test_mode", "prod_mode"):
            block = raw[mode_key]
            self.assertIn("eval_quality_task_counts", block,
                          f"{mode_key} missing eval_quality_task_counts")
            self.assertIn("eval_overhead_task_count", block,
                          f"{mode_key} missing eval_overhead_task_count")
            self.assertIn("eval_period_arms", block,
                          f"{mode_key} missing eval_period_arms")

    def test_gate_eval_config_default_referenced_by_suite(self):
        # The eval suite's argparse default must follow the rename.
        import simulation_experiments.evaluation_suite as es
        src = open(es.__file__).read()
        self.assertIn("gate_eval_config.json", src)
        self.assertNotIn("evaluation_suite_config.json", src)


if __name__ == "__main__":
    unittest.main()
