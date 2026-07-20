"""P1.15 Phase 1 (layer A) — harness loud-failure tests.

The experiment harness used to (a) swallow a crashing optimizer arm silently
(``concurrent.futures.wait`` with no ``.result()``, ``analyze_single_instance``
returning a silent ``0.0`` for a missing output file, and the aggregator
averaging over unequal per-scheduler taskset sets) and (b) write a
``0.000000 ± 0.000000`` row for the crashed arm that read as a "perfect"
result. This file pins the user's two explicit requirements:

1. **Stop the experiment when one optimizer crashes** — do not continue the
   remaining arms/tasksets as if nothing happened; exit non-zero naming the
   (taskset, arm).
2. **Show the crash loudly** — never treat a crashed arm's missing output as a
   silent ``0.000000`` row; capture the C++ stdout/stderr to a per-arm
   ``run.log`` (so the ``std::runtime_error`` text survives for the Phase 2
   investigation), write a ``crash_report.txt`` + a per-taskset × per-arm ✅/❌
   map, and surface NaN (not 0.0) for a missing arm.

These are written TDD-first: they are RED against the current (silent-failure)
code, then GREEN after the A1–A6 fixes land.
"""
import os
import sys
import csv
import json
import shutil
import tempfile
import unittest
import unittest.mock

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import simulation_experiments.compare_optimizers as compare_optimizers
from simulation_experiments.compare_optimizers import main as compare_main
from simulation_experiments.run_sim_experiments import (
    analyze_single_instance,
    run_single_simulation,
)
from simulation_experiments.utils import write_summary_and_plots


# ---------------------------------------------------------------------------
# Helpers shared by the integration + unit tests.
# ---------------------------------------------------------------------------

def _write_taskset_characteristics(taskset_dir, num_tasks=4):
    """Write a minimal taskset_characteristics_interval_0.yaml + a matching
    generator_config.json so ``_should_generate`` reuses the taskset (no real
    generation pipeline needed) and ``analyze_single_instance`` can read
    deadlines/sp_weights.

    ``generator_config.json`` is written to deep-equal the ``config_dict``
    that ``main()`` computes when ``load_generation_config`` is patched to
    return ``_FAKE_CONFIG`` — so the config-change check passes (reuse).
    """
    os.makedirs(taskset_dir, exist_ok=True)
    tasks = []
    for i in range(num_tasks):
        tasks.append({
            "id": i, "period": 100, "deadline": 1000,
            "sp_threshold": 0.5, "sp_weight": 1.25, "name": f"task_{i+1}",
        })
    char_path = os.path.join(taskset_dir, "taskset_characteristics_interval_0.yaml")
    import yaml
    with open(char_path, "w") as f:
        yaml.safe_dump({"tasks": tasks}, f)
    # Matching generator_config.json so _should_generate returns False (reuse).
    with open(os.path.join(taskset_dir, "generator_config.json"), "w") as f:
        json.dump(_FAKE_CONFIG, f)


# The config dict main() builds when load_generation_config is patched to
# return this exact object (with RANDOM_SEED + UPDATE_INTERVAL_S stamped per
# taskset). _should_generate compares this against the on-disk
# generator_config.json, so the saved file must deep-equal the stamped dict.
_FAKE_CONFIG = {
    "N_TASKS": 4,
    "N_CORES": 2,
    "RANDOM_SEED": 1000,       # stamped per-taskset in main()
    "UPDATE_INTERVAL_S": 10,   # stamped per-taskset in main()
}


def _fake_run_single_factory(crashing_arms, throw_text):
    """Build a fake ``run_single_simulation`` replacement.

    For each (taskset_idx, scheduler) in ``crashing_arms``: emulate a binary
    SIGABRT (exit 134) by writing the throw text to the per-arm ``run.log``
    and raising ``subprocess.CalledProcessError(returncode=134, ...)`` (what
    the real ``subprocess.run(..., check=True)`` raises on a crash). For every
    other arm: write a valid ``interval_sp_metrics.txt`` +
    ``miss_rate_summary.txt`` + ``scheduler_execution_time.txt`` into
    ``<sched_dir>/<scheduler>/`` (the layout analyze_single_instance reads).

    This fake stands in for the C++ RunOrchestrator binary entirely — no
    subprocess is spawned — so the test is hermetic.
    """
    crashing_set = set(crashing_arms)  # {(idx, scheduler)}

    def _fake(sim_bin_path, taskset_dir, sched_dir, interval_duration_ms,
              scheduler, inst, verbose=1, export_level=None):
        # Derive the taskset index from taskset_dir (ends with taskset_<idx>).
        ts_idx = int(os.path.basename(taskset_dir).split("_")[-1])
        nested = os.path.join(sched_dir, scheduler)
        os.makedirs(nested, exist_ok=True)

        if (ts_idx, scheduler) in crashing_set:
            # Emulate a crashed binary: capture the throw text to run.log,
            # then raise CalledProcessError(134) — exactly what the real
            # subprocess.run(check=True) does on a SIGABRT exit.
            run_log = os.path.join(sched_dir, "run.log")
            with open(run_log, "w") as f:
                f.write(throw_text)
            import subprocess
            raise subprocess.CalledProcessError(
                returncode=134, cmd=[sim_bin_path, scheduler],
                stderr=throw_text,
            )

        # Success path: write the outputs the real binary would write.
        with open(os.path.join(nested, "interval_sp_metrics.txt"), "w") as f:
            for i in range(3):
                f.write(f"{i},0.90\n")
        with open(os.path.join(nested, "miss_rate_summary.txt"), "w") as f:
            f.write("total_jobs,missed_jobs,miss_rate\n")
            f.write("30,1,0.033333\n")
        with open(os.path.join(nested, "scheduler_execution_time.txt"), "w") as f:
            f.write("0.05\n")

    return _fake


# The exact C++ throw text from RTA_Cache.cpp:357 (committed at HEAD). The
# harness must preserve this in run.log so the Phase 2 investigation has the
# kill site without needing to re-run under gdb.
_RTA_CACHE_THROW_TEXT = (
    "RTACache::ComputeTaskSetDifference: candidate differs from champion "
    "by more than one task — violates the P1.10 single-change invariant. "
    "Call IsSingleTaskChange to guard multi-change candidates.\n"
    "Aborted (core dumped)\n"
)


# ---------------------------------------------------------------------------
# Integration: a crashing arm must fail the run loudly (A0 RED, A1/A2/A6 GREEN)
# ---------------------------------------------------------------------------

class TestCrashingArmStopsRunLoudly(unittest.TestCase):
    """Drive ``compare_optimizers.main()`` with a fake ``run_single_simulation``
    that SIGABRTs (exit 134) for one arm, and assert the run stops, names the
    (taskset, arm), captures the throw text, writes a crash report + ✅/❌ map,
    and never emits a ``0.000000`` row for the crashed arm.
    """

    def _run_main_with_crashing_arm(self, tmp, crashing_arms,
                                     schedulers=("BF", "INCR_Reopt_5"),
                                     n_tasksets=2, expect_raise=True):
        """Run compare_optimizers.main() on a temp output dir with a fake
        ``run_single_simulation`` that crashes for ``crashing_arms``.

        Returns (rc_or_exc, run_dir). Patches:
        - ``resolve_taskset_config_path`` -> a temp config path (config content
          unused because generation is skipped via _should_generate).
        - ``load_generation_config`` -> _FAKE_CONFIG (so the stamped
          config_dict deep-equals the saved generator_config.json -> reuse).
        - ``_should_generate`` -> always False (skip the real generator).
        - ``run_single_simulation`` -> the crashing fake.
        - ``MATPLOTLIB_AVAILABLE`` -> False (no plotting in the test).

        ``main()`` resolves the run output dir to a subfolder
        ``tasks4_dur<n_sec>_interval10_seed1000`` under ``--output_dir``; the
        taskset dirs + crash report land there, so the per-taskset fixtures
        are pre-created in that resolved subfolder.
        """
        base_output = os.path.join(tmp, "out")
        os.makedirs(base_output, exist_ok=True)
        # Mirror resolve_run_output_dir's auto-name so fixtures land where
        # main() will look (num_tasks=4, n_sec=300 default, interval=10,
        # base_seed=1000).
        run_dir = os.path.join(base_output, "tasks4_dur300_interval10_seed1000")

        # Pre-create per-taskset dirs with characteristics + matching
        # generator_config.json so generation is skipped and analyze can read
        # deadlines/sp_weights.
        for idx in range(n_tasksets):
            ts_dir = os.path.join(run_dir, f"taskset_{idx}")
            _write_taskset_characteristics(ts_dir, num_tasks=4)

        fake_run = _fake_run_single_factory(crashing_arms, _RTA_CACHE_THROW_TEXT)

        argv = [
            "compare_optimizers.py",
            "--num_tasks", "4",
            "--n_tasksets", str(n_tasksets),
            "--schedulers", *schedulers,
            "-o", base_output,
            "--scheduler_trigger_interval", "10",
            "--base_seed", "1000",
            "-v", "0",
        ]

        captured = {"exc": None, "rc": None}

        def _patched_should_generate(*a, **kw):
            return False

        with unittest.mock.patch.object(
            compare_optimizers, "resolve_taskset_config_path",
            return_value=os.path.join(tmp, "fake_cfg.json"),
        ), unittest.mock.patch.object(
            compare_optimizers, "load_generation_config",
            return_value=dict(_FAKE_CONFIG),
        ), unittest.mock.patch.object(
            compare_optimizers, "_should_generate",
            side_effect=_patched_should_generate,
        ), unittest.mock.patch.object(
            compare_optimizers, "run_single_simulation",
            side_effect=fake_run,
        ), unittest.mock.patch.object(
            compare_optimizers, "MATPLOTLIB_AVAILABLE", False,
        ), unittest.mock.patch.object(sys, "argv", argv):
            try:
                compare_main()
            except SystemExit as e:
                captured["rc"] = e.code
            except Exception as e:  # the loud-failure path may raise
                captured["exc"] = e

        return captured, run_dir

    def test_crashing_arm_exits_nonzero_and_names_taskset_arm(self):
        """A SIGABRT in INCR_Reopt_5 on taskset_0 must stop the run and exit
        non-zero with a message naming (taskset_0, INCR_Reopt_5)."""
        tmp = tempfile.mkdtemp()
        try:
            captured, output_dir = self._run_main_with_crashing_arm(
                tmp, crashing_arms=[(0, "INCR_Reopt_5")],
            )
            # Either SystemExit(non-zero) or a raised exception naming the arm.
            if captured["exc"] is not None:
                self.assertIsNotNone(captured["exc"])
                msg = str(captured["exc"]).lower()
                self.assertIn("incr_reopt_5", msg)
                self.assertIn("taskset_0", msg)
            else:
                self.assertIsNotNone(captured["rc"])
                self.assertNotEqual(captured["rc"], 0)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_crash_report_and_map_written(self):
        """A ``crash_report.txt`` + a per-taskset × per-arm ✅/❌ map must be
        written to the output dir, naming the crashed (taskset, arm) + the
        captured run.log path."""
        tmp = tempfile.mkdtemp()
        try:
            captured, output_dir = self._run_main_with_crashing_arm(
                tmp, crashing_arms=[(0, "INCR_Reopt_5")],
                expect_raise=False,
            )
            crash_report = os.path.join(output_dir, "crash_report.txt")
            self.assertTrue(os.path.exists(crash_report),
                            "crash_report.txt must be written on a crash")
            with open(crash_report) as f:
                report = f.read()
            self.assertIn("INCR_Reopt_5", report)
            self.assertIn("taskset_0", report)
            # The ✅/❌ map file must exist and record the crash as ❌.
            map_path = os.path.join(output_dir, "taskset_arm_status.csv")
            self.assertTrue(os.path.exists(map_path),
                            "per-taskset × per-arm status map must be written")
            with open(map_path) as f:
                rows = list(csv.DictReader(f))
            crashed = [r for r in rows
                       if r.get("scheduler") == "INCR_Reopt_5"
                       and r.get("taskset") == "0"]
            self.assertTrue(crashed, "crashed arm must appear in the status map")
            self.assertNotEqual(
                crashed[0].get("status", "").lower(), "ok",
                "crashed arm must NOT be marked ok in the status map",
            )
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_crashed_arm_run_log_captures_throw_text(self):
        """The C++ throw text from RTA_Cache.cpp:357 must survive in a per-arm
        ``run.log`` so the Phase 2 investigation has the kill site."""
        tmp = tempfile.mkdtemp()
        try:
            captured, output_dir = self._run_main_with_crashing_arm(
                tmp, crashing_arms=[(0, "INCR_Reopt_5")],
                expect_raise=False,
            )
            run_log = os.path.join(
                output_dir, "taskset_0", "INCR_Reopt_5", "run.log",
            )
            self.assertTrue(os.path.exists(run_log),
                            "per-arm run.log must be written even on a crash")
            with open(run_log) as f:
                log_text = f.read()
            self.assertIn("ComputeTaskSetDifference", log_text)
            self.assertIn("single-change invariant", log_text)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_no_summary_written_on_crash(self):
        """User requirement (2026-07-19): "as long as there is one crash, then
        no results are useful anymore, no need to report any numbers, fail it
        loudly." On ANY crash, ``comparison_summary.csv`` and plots must NOT be
        written — a partial aggregate can be mistaken for a complete one. Only
        the diagnostic artifacts (crash_report.txt + taskset_arm_status.csv)
        are written, and the run exits non-zero."""
        tmp = tempfile.mkdtemp()
        try:
            captured, output_dir = self._run_main_with_crashing_arm(
                tmp, crashing_arms=[(0, "INCR_Reopt_5")],
                schedulers=("BF", "INCR_Reopt_5"),
                expect_raise=False,
            )
            # No numbers reported: the summary CSV and plots must be absent.
            csv_path = os.path.join(output_dir, "comparison_summary.csv")
            self.assertFalse(
                os.path.exists(csv_path),
                "comparison_summary.csv must NOT be written when an arm "
                "crashed — no partial aggregates (user requirement)",
            )
            plots_png = os.path.join(output_dir, "comparison_plots.png")
            self.assertFalse(
                os.path.exists(plots_png),
                "comparison_plots.png must NOT be written when an arm crashed",
            )
            # Diagnostics that locate the crash ARE written.
            self.assertTrue(
                os.path.exists(os.path.join(output_dir, "crash_report.txt"))
            )
            self.assertTrue(
                os.path.exists(os.path.join(output_dir, "taskset_arm_status.csv"))
            )
            # And the run still fails loudly.
            self.assertIsNotNone(captured["rc"])
            self.assertNotEqual(captured["rc"], 0)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# Unit: analyze_single_instance must raise on missing/empty output (A3)
# ---------------------------------------------------------------------------

class TestAnalyzeRaisesOnMissingOutput(unittest.TestCase):
    """``analyze_single_instance`` must raise (not return a silent
    ``miss_rate=0``) when ``interval_sp_metrics.txt`` is absent or
    header/empty. The silent zero is what let a crashed arm read as
    'perfect 0.0 miss rate'."""

    def test_raises_when_metrics_file_absent(self):
        tmp = tempfile.mkdtemp()
        try:
            # sched_dir exists but has NO interval_sp_metrics.txt (the exact
            # state a crashed binary leaves behind).
            sched_dir = os.path.join(tmp, "INCR", "INCR")
            os.makedirs(sched_dir, exist_ok=True)
            with self.assertRaises(Exception) as ctx:
                analyze_single_instance(tmp, "INCR", 0, {0: 1000.0}, 10)
            msg = str(ctx.exception).lower()
            self.assertIn("interval_sp_metrics", msg)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_raises_when_metrics_file_empty(self):
        """A 0-byte or header-only file (a half-written abort) is NOT a
        complete run — analyze must raise, not treat it as 0 SP values."""
        tmp = tempfile.mkdtemp()
        try:
            sched_dir = os.path.join(tmp, "INCR", "INCR")
            os.makedirs(sched_dir, exist_ok=True)
            with open(os.path.join(sched_dir, "interval_sp_metrics.txt"), "w") as f:
                pass  # 0-byte file
            with self.assertRaises(Exception):
                analyze_single_instance(tmp, "INCR", 0, {0: 1000.0}, 10)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_raises_when_metrics_header_only(self):
        """A file with only a header line (no data rows) is not a complete run."""
        tmp = tempfile.mkdtemp()
        try:
            sched_dir = os.path.join(tmp, "INCR", "INCR")
            os.makedirs(sched_dir, exist_ok=True)
            with open(os.path.join(sched_dir, "interval_sp_metrics.txt"), "w") as f:
                f.write("# interval,sp\n")  # header/comment only, no data
            with self.assertRaises(Exception):
                analyze_single_instance(tmp, "INCR", 0, {0: 1000.0}, 10)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# Unit: write_summary_and_plots must emit NaN (not 0.0) for an empty arm (A4)
# ---------------------------------------------------------------------------

class TestSummaryEmitsNaNForEmptyArm(unittest.TestCase):
    """An arm with no data (crashed) must surface as NaN in the summary CSV,
    never as a silent ``0.000000``. And schedulers with unequal N (one crashed,
    one did not) must fail loudly."""

    @unittest.mock.patch("simulation_experiments.utils.save_figure")
    def test_empty_arm_writes_nan_not_zero(self, mock_save):
        tmp = tempfile.mkdtemp()
        try:
            # BF has data; INCR_Reopt_5 has NONE (it crashed).
            results = {
                "BF": {
                    "sp_values": [0.9, 0.8], "miss_rates": [0.1, 0.2],
                    "intervals": {0: [0.9, 0.8]}, "sched_times": [0.01, 0.02],
                    "important_miss_rates": [0.05, 0.10],
                    "non_important_miss_rates": [0.15, 0.25],
                },
                "INCR_Reopt_5": {
                    "sp_values": [], "miss_rates": [], "intervals": {},
                    "sched_times": [], "important_miss_rates": [],
                    "non_important_miss_rates": [],
                },
            }
            write_summary_and_plots(
                results_by_scheduler=results,
                schedulers=["BF", "INCR_Reopt_5"],
                output_dir_abs=tmp,
                horizon_granularity=10,
            )
            csv_path = os.path.join(tmp, "comparison_summary.csv")
            with open(csv_path) as f:
                rows = list(csv.DictReader(f))
            reopt5 = [r for r in rows if r["Scheduler"] == "INCR_Reopt_5"][0]
            # NaN serializes as "nan" / "NaN" — never "0.000000".
            sp_val = reopt5["Mean_SP_Metric"].strip().lower()
            self.assertIn(sp_val, ("nan", "-nan", "1.#ind", "-1.#ind"),
                          f"empty arm SP must be NaN, got {sp_val!r}")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    @unittest.mock.patch("simulation_experiments.utils.save_figure")
    def test_unequal_n_fails_loudly(self, mock_save):
        """Schedulers scored over different taskset sets (unequal N) must fail
        loudly, not write a silently-incomparable summary.

        With per-taskset-mean-then-average semantics, a scheduler that has data
        for a strict subset of the tasksets another scheduler has data for is
        an apples-to-oranges comparison and must raise (or exit non-zero)."""
        tmp = tempfile.mkdtemp()
        try:
            # BF has 2 tasksets' worth of data; INCR_Reopt_5 has 1 (it crashed
            # on one). A per-taskset-mean-then-average that silently averages
            # over different taskset sets is the bug — this must raise.
            results = {
                "BF": {
                    "sp_values": [0.9, 0.8], "miss_rates": [0.1, 0.2],
                    "intervals": {0: [0.9, 0.8]}, "sched_times": [0.01, 0.02],
                    "important_miss_rates": [0.05, 0.10],
                    "non_important_miss_rates": [0.15, 0.25],
                },
                "INCR_Reopt_5": {
                    "sp_values": [0.6], "miss_rates": [0.3],
                    "intervals": {0: [0.6]}, "sched_times": [0.01],
                    "important_miss_rates": [0.20],
                    "non_important_miss_rates": [0.10],
                },
            }
            with self.assertRaises(Exception):
                write_summary_and_plots(
                    results_by_scheduler=results,
                    schedulers=["BF", "INCR_Reopt_5"],
                    output_dir_abs=tmp,
                    horizon_granularity=10,
                )
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
