"""P2.8 — config layout + rename pin (red->green TDD).

Asserts the consolidated config directory layout:

    configs/
      paper_simulation_config.json   (renamed from experiment_config.json;
                                      ALSO now the single config for the
                                      north-star gate eval -- eval_* keys
                                      folded in, D1 reversed beta->alpha)
      incr_et_profiling.json         (renamed from INCR_ET_Profiling.json)

and that the removed configs are gone (`experiment_config.json` and
`gate_eval_config.json` themselves must FAIL LOUDLY if referenced, not silently
aliased). Also pins that the load-bearing default paths point at the single
surviving config.

This is a refactor (naming/consolidation), not a correctness change: the gate =
the loader + the eval suite still resolve their defaults to a real file, and the
old names are not silently kept as aliases.

Note on the gate-eval consequences of the fold (D1=alpha):
  `paper_simulation_config.json` carries the PAPER scheduler set (5 main:
  INCR_Reopt_10, BF, RM_FAST, RM_SLOW, CFS) -- NOT the 10-scheduler gate set.
  The gate eval therefore runs against those schedulers (main+ablation union).
  Q1/Q2/Q3/E1 still evaluate (BF/INCR_Reopt_10/CFS/RM_FAST/RM_SLOW all
  simulated); E3 (period-
  monotonicity) needs >=2 INCR_Reopt_X arms but only INCR_Reopt_10 is simulated,
  so E3 reports MISSING at every N (non-fatal) -- it loses the signal it had
  under the dedicated gate config. Accepted by the user's "paper set; drop
  gate_eval" decision.
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

# After the D1=alpha fold: only two configs survive. gate_eval_config.json is
# GONE (its eval_* keys moved into paper_simulation_config.json).
EXPECTED_PRESENT = {
    "paper_simulation_config.json",
    "incr_et_profiling.json",
}
# Old names that must NOT exist -- the renames/fold are not silent aliases.
EXPECTED_ABSENT = {
    "experiment_config.json",
    "gate_eval_config.json",
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
            f"old config names still present (rename/fold must be total, not an alias): {leftover}")

    def test_no_folded_or_renamed_configs_linger(self):
        # P2.8 does NOT freeze the config directory against new experiment
        # configs (the repo actively authors per-purpose configs). It pins only
        # that the folded/renamed old names are GONE -- they must not linger as
        # silent aliases. (Positive presence of the two survivors is checked in
        # test_expected_configs_present; stale-name fail-loudly in
        # TestStaleNameFailsLoudly.)
        actual = set(os.listdir(CONFIGS_DIR))
        leftover = EXPECTED_ABSENT & actual
        self.assertFalse(
            leftover,
            f"folded/renamed old config names still present (must not be aliases): {leftover}")


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

    def test_folded_gate_eval_config_does_not_exist(self):
        # D1=alpha: gate_eval_config.json was DELETED (eval_* keys folded into
        # paper_simulation_config.json). It must not linger as an alias.
        stale = os.path.join(CONFIGS_DIR, "gate_eval_config.json")
        self.assertFalse(
            os.path.exists(stale),
            "gate_eval_config.json still exists -- fold was not applied")

    def test_loading_via_folded_gate_eval_name_raises(self):
        # Pointing the loader at the deleted gate-eval name must FAIL LOUDLY,
        # not silently alias to paper_simulation_config.json.
        stale = os.path.join(CONFIGS_DIR, "gate_eval_config.json")
        with self.assertRaises(FileNotFoundError):
            load_experiment_config("test", config_path=stale)


class TestPaperConfigCarriesEvalKeys(unittest.TestCase):
    """D1=alpha: the eval_* keys live in paper_simulation_config.json now (folded
    out of the deleted gate_eval_config.json), so the gate eval reads them from
    whichever config is active -- the pipeline ignores them."""

    def test_paper_config_carries_eval_keys(self):
        import json
        with open(os.path.join(CONFIGS_DIR, "paper_simulation_config.json")) as f:
            raw = json.load(f)
        for mode_key in ("test_mode", "prod_mode"):
            block = raw[mode_key]
            self.assertIn("eval_quality_task_counts", block,
                          f"{mode_key} missing eval_quality_task_counts")
            self.assertIn("eval_overhead_task_count", block,
                          f"{mode_key} missing eval_overhead_task_count")
            self.assertIn("eval_period_arms", block,
                          f"{mode_key} missing eval_period_arms")

    def test_eval_suite_default_points_at_paper_config(self):
        # The eval suite's argparse default must follow the fold: it now reads
        # paper_simulation_config.json (the single config), not a dedicated
        # gate-eval config.
        import simulation_experiments.evaluation_suite as es
        src = open(es.__file__).read()
        self.assertIn("paper_simulation_config.json", src)
        self.assertNotIn("gate_eval_config.json", src)

    def test_paper_config_scheduler_set_is_the_paper_set(self):
        # Pin the consequence of the fold: paper config's main_scheduler_list is
        # the PAPER set (5 schedulers), NOT the 10-scheduler gate set. E3 will
        # therefore be MISSING-only under the gate eval (documented, accepted).
        # RM was replaced by RM_FAST+RM_SLOW (the two RunOrchestrator time-limit
        # variants) in every config's main list, so the pin reflects both.
        import json
        with open(os.path.join(CONFIGS_DIR, "paper_simulation_config.json")) as f:
            raw = json.load(f)
        for mode_key in ("test_mode", "prod_mode"):
            main = raw[mode_key]["main_scheduler_list"]
            self.assertEqual(
                main, ["INCR_Reopt_10", "BF", "RM_FAST", "RM_SLOW", "CFS"],
                f"{mode_key} main_scheduler_list changed from the paper set: {main}")


class TestTimeLimitConfig(unittest.TestCase):
    """time_limit_seconds (default 1) is read by run_simulation_plot_eval_ns.sh
    and patched into sources/parameters.yaml's TIME_LIMIT before the C++ binary
    starts (the binary reads it at static init in Parameters.cpp). It bounds ONE
    optimizer call (per-activation budget), NOT the task time-limits.

    Pins that every shipped config carries the key in BOTH modes, so the .sh
    overwrite always resolves a real value instead of falling back to its 1s
    default. A config missing the key is a contract break, not a silent default.
    """

    ALL_CONFIGS = (
        "paper_simulation_config.json",
        "incr_et_profiling.json",
    )

    def test_all_configs_carry_time_limit_seconds(self):
        import json
        for name in self.ALL_CONFIGS:
            with open(os.path.join(CONFIGS_DIR, name)) as f:
                raw = json.load(f)
            for mode_key in ("test_mode", "prod_mode"):
                self.assertIn(
                    "time_limit_seconds", raw[mode_key],
                    f"{name} {mode_key} missing time_limit_seconds")
                self.assertEqual(
                    raw[mode_key]["time_limit_seconds"], 1,
                    f"{name} {mode_key} time_limit_seconds must be the 1s default, "
                    f"got {raw[mode_key]['time_limit_seconds']}")


if __name__ == "__main__":
    unittest.main()
