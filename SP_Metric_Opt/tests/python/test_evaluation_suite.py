"""Tests for simulation_experiments.evaluation_suite.

The evaluation suite is a north-star integration test: it ingests the same
``comparison_summary.csv`` files the aggregate stage produces, normalizes SP
via the shared ``normalize_records_sp`` / ``compute_sp_upper_bound`` helpers,
and evaluates the 5 north-star gates (Q1-Q3, E1, E2) into a PASS/FAIL verdict.

These tests mock the filesystem (temporary directories + synthetic CSV files +
a fake ``taskset_characteristics_interval_0.yaml`` so the SP upper bound is a
known constant) to verify the gate logic and the end-to-end report writer
without needing a real ~20-minute simulation. They mirror the pattern in
``test_aggregate.py``.
"""
import csv
import json
import os
import sys
import tempfile
import shutil
import unittest

import yaml

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import simulation_experiments.evaluation_suite as ev
import simulation_experiments.aggregate_across_tasks as agg


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

SCHEDULERS_MAIN = ["INCR", "BF", "RM", "CFS"]
SCHEDULERS_ABLATION = ["BF", "INCR", "INCR_NO_TL", "INCR_WCET", "INCR_SCRATCH"]


def _write_summary_csv(dir_path, rows):
    """Write a comparison_summary.csv with the given rows.

    Each row is {scheduler, mean_sp, std_sp, mean_sched_time, ...}.
    Mirrors the helper in test_aggregate.py.
    """
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
                r.get("std_sp", 0.0),
                r.get("mean_miss", 0.0),
                r.get("std_miss", 0.0),
                r["mean_sched_time"],
                r.get("important_miss", 0.0),
                r.get("non_important_miss", 0.0),
            ])


def _write_upper_bound_yaml(experiment_dir, num_tasks, weight_per_task=0.25):
    """Write a taskset_characteristics_interval_0.yaml giving a known SP ceiling.

    ``compute_sp_upper_bound`` sums ``sp_weight * perf_coefficient`` over tasks
    (perf_coefficient is 1.0 for generated tasksets). With ``num_tasks`` tasks
    each carrying ``weight_per_task``, the ceiling is ``num_tasks * weight_per_task``.
    A taskset_0/ subdir is required because the function globs ``taskset_*``.
    """
    ts_dir = os.path.join(experiment_dir, "taskset_0")
    os.makedirs(ts_dir, exist_ok=True)
    char_path = os.path.join(ts_dir, "taskset_characteristics_interval_0.yaml")
    tasks = [{"id": i, "sp_weight": weight_per_task} for i in range(num_tasks)]
    with open(char_path, "w") as f:
        yaml.safe_dump({"tasks": tasks}, f)
    return num_tasks * weight_per_task


def _build_synthetic_run(temp_base, cfg, per_scheduler_sp, per_scheduler_et):
    """Build a synthetic run tree matching ``cfg``.

    ``per_scheduler_sp`` / ``per_scheduler_et`` are ``{scheduler: {N: value}}``.
    Returns the run_root. Only the schedulers present in those dicts are written;
    the union of main + ablation lists is the typical input.
    """
    run_root = agg.build_run_root(temp_base, cfg)
    sim_dir = os.path.join(run_root, "sim")
    os.makedirs(sim_dir, exist_ok=True)
    dur = cfg["simulation_duration_seconds"]
    interv = cfg["scheduler_trigger_interval_seconds"]
    seed = cfg["base_random_seed"]
    for n in cfg["num_tasks_for_cross_task_comparison"]:
        exp_dir = os.path.join(sim_dir, f"tasks{n}_dur{dur}_interval{interv}_seed{seed}")
        # Upper-bound YAML so normalize_records_sp has a ceiling. Use a ceiling
        # of 1.0 (weight 1/num_tasks per task) so norm == raw for easy reasoning.
        _write_upper_bound_yaml(exp_dir, n, weight_per_task=1.0 / n)
        rows = []
        for sched, vals in per_scheduler_sp.items():
            if sched not in per_scheduler_et:
                continue
            if n not in vals:
                continue
            rows.append({
                "scheduler": sched,
                "mean_sp": vals[n],
                "mean_sched_time": per_scheduler_et[sched][n],
            })
        _write_summary_csv(exp_dir, rows)
    return run_root


def _eval_cfg(temp_base, task_counts, dur=600, interv=10, seed=1000):
    """A minimal flat cfg dict accepted by aggregate_data_from_directories."""
    return {
        "num_tasks_for_cross_task_comparison": task_counts,
        "simulation_duration_seconds": dur,
        "scheduler_trigger_interval_seconds": interv,
        "base_random_seed": seed,
        "analysis": {"normalize_sp": True, "sp_normalization_method": "upper_bound"},
        "plotting": {"run_name_prefix": "evalsuite"},
        "_active_mode": "prod",
    }


# ---------------------------------------------------------------------------
# Lookup construction
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Gate->N mapping from config
# ---------------------------------------------------------------------------

class TestGateNsFromCfg(unittest.TestCase):
    """gate_ns_from_cfg resolves the gate->N mapping from the eval config."""

    def test_reads_eval_keys(self):
        cfg = {
            "num_tasks_for_cross_task_comparison": [4, 6, 8, 10],
            "eval_quality_task_counts": [4, 6, 8],
            "eval_overhead_task_count": 10,
        }
        qn, on = ev.gate_ns_from_cfg(cfg)
        self.assertEqual(qn, [4, 6, 8])
        self.assertEqual(on, 10)

    def test_falls_back_when_eval_keys_absent(self):
        """A standard experiment config (no eval_* keys) still resolves."""
        cfg = {"num_tasks_for_cross_task_comparison": [4, 6, 8]}
        qn, on = ev.gate_ns_from_cfg(cfg)
        self.assertEqual(qn, [4, 6, 8])
        # Overhead N defaults to the max quality N when not specified.
        self.assertEqual(on, 8)


class TestBuildLookup(unittest.TestCase):
    """build_metric_lookup must produce (N, scheduler) -> metrics with overhead."""

    def test_lookup_has_normalized_sp_and_overhead(self):
        temp_base = tempfile.mkdtemp()
        try:
            cfg = _eval_cfg(temp_base, [4, 8])
            sp = {"INCR": {4: 0.9, 8: 0.8}, "BF": {4: 0.95, 8: 0.6}}
            et = {"INCR": {4: 0.05, 8: 0.08}, "BF": {4: 0.2, 8: 2.0}}
            _build_synthetic_run(temp_base, cfg, sp, et)

            lookup = ev.build_metric_lookup(cfg, output_parent=temp_base)
            # Ceiling is 1.0, so normalized == raw.
            self.assertAlmostEqual(lookup[(4, "INCR")]["mean_sp_norm"], 0.9)
            self.assertAlmostEqual(lookup[(8, "BF")]["mean_sp_norm"], 0.6)
            # Overhead = mean_sched_time / interval (interval=10).
            self.assertAlmostEqual(lookup[(4, "INCR")]["overhead"], 0.005)
            self.assertAlmostEqual(lookup[(8, "BF")]["overhead"], 0.2)
            # Raw sched time is carried too.
            self.assertAlmostEqual(lookup[(8, "INCR")]["mean_sched_time"], 0.08)
        finally:
            shutil.rmtree(temp_base)

    def test_missing_run_returns_empty(self):
        temp_base = tempfile.mkdtemp()
        try:
            cfg = _eval_cfg(temp_base, [4])
            lookup = ev.build_metric_lookup(cfg, output_parent=temp_base)
            self.assertEqual(lookup, {})
        finally:
            shutil.rmtree(temp_base)

    def test_stale_dir_ignored(self):
        """A stale dir from a different seed/dur must not pollute the lookup."""
        temp_base = tempfile.mkdtemp()
        try:
            cfg = _eval_cfg(temp_base, [4], dur=600, seed=1000)
            sp = {"INCR": {4: 0.9}}
            et = {"INCR": {4: 0.05}}
            run_root = _build_synthetic_run(temp_base, cfg, sp, et)
            # Stale dir: same N, different dur, also under sim/.
            stale = os.path.join(run_root, "sim", "tasks4_dur300_interval10_seed1000")
            _write_summary_csv(stale, [
                {"scheduler": "INCR", "mean_sp": 0.10, "mean_sched_time": 5.0},
            ])
            _write_upper_bound_yaml(stale, 4, weight_per_task=0.25)

            lookup = ev.build_metric_lookup(cfg, output_parent=temp_base)
            # Only the matching run's INCR value (0.9), not the stale 0.10.
            self.assertAlmostEqual(lookup[(4, "INCR")]["mean_sp_norm"], 0.9)
        finally:
            shutil.rmtree(temp_base)


# ---------------------------------------------------------------------------
# Gate Q1: small-N BF gap <= 30%
# ---------------------------------------------------------------------------

class TestGateQ1(unittest.TestCase):
    """Q1: at N=4, BF >= INCR & SCRATCH, gap <= 30%."""

    def test_pass_small_gap(self):
        # BF=0.90, INCR=0.80 -> gap = 11.1% -> PASS
        lookup = {
            (4, "BF"): {"mean_sp_norm": 0.90},
            (4, "INCR"): {"mean_sp_norm": 0.80},
            (4, "INCR_SCRATCH"): {"mean_sp_norm": 0.78},
        }
        verdict = ev.evaluate_q1(lookup)
        self.assertEqual(verdict["status"], "PASS")
        self.assertIn("INCR", verdict["detail"])

    def test_fail_large_gap(self):
        # BF=0.90, INCR=0.50 -> gap = 44.4% -> FAIL
        lookup = {
            (4, "BF"): {"mean_sp_norm": 0.90},
            (4, "INCR"): {"mean_sp_norm": 0.50},
            (4, "INCR_SCRATCH"): {"mean_sp_norm": 0.78},
        }
        verdict = ev.evaluate_q1(lookup)
        self.assertEqual(verdict["status"], "FAIL")

    def test_fail_missing_bf(self):
        """No BF at N=4 -> cannot evaluate -> FAIL with a clear reason."""
        lookup = {(4, "INCR"): {"mean_sp_norm": 0.80}}
        verdict = ev.evaluate_q1(lookup)
        self.assertEqual(verdict["status"], "FAIL")
        self.assertIn("BF", verdict["detail"])


# ---------------------------------------------------------------------------
# Gate Q2: large-N INCR/SCRATCH >= BF
# ---------------------------------------------------------------------------

class TestGateQ2(unittest.TestCase):
    """Q2: at N=8 (largest SP-quality N), INCR & SCRATCH >= BF."""

    def test_pass(self):
        lookup = {
            (8, "BF"): {"mean_sp_norm": 0.55},
            (8, "INCR"): {"mean_sp_norm": 0.60},
            (8, "INCR_SCRATCH"): {"mean_sp_norm": 0.58},
        }
        verdict = ev.evaluate_q2(lookup, large_n=8)
        self.assertEqual(verdict["status"], "PASS")

    def test_fail_incr_below_bf(self):
        lookup = {
            (8, "BF"): {"mean_sp_norm": 0.60},
            (8, "INCR"): {"mean_sp_norm": 0.55},
            (8, "INCR_SCRATCH"): {"mean_sp_norm": 0.65},
        }
        verdict = ev.evaluate_q2(lookup, large_n=8)
        self.assertEqual(verdict["status"], "FAIL")
        self.assertIn("INCR", verdict["detail"])

    def test_fail_scratch_below_bf(self):
        lookup = {
            (8, "BF"): {"mean_sp_norm": 0.60},
            (8, "INCR"): {"mean_sp_norm": 0.65},
            (8, "INCR_SCRATCH"): {"mean_sp_norm": 0.50},
        }
        verdict = ev.evaluate_q2(lookup, large_n=8)
        self.assertEqual(verdict["status"], "FAIL")
        self.assertIn("SCRATCH", verdict["detail"])


# ---------------------------------------------------------------------------
# Gate Q3: INCR/SCRATCH >= all baselines at large N
# ---------------------------------------------------------------------------

class TestGateQ3(unittest.TestCase):
    """Q3: INCR & SCRATCH >= max(RM, CFS, INCR_NO_TL, INCR_WCET) at large N."""

    def test_pass(self):
        lookup = {
            (8, "INCR"): {"mean_sp_norm": 0.70},
            (8, "INCR_SCRATCH"): {"mean_sp_norm": 0.68},
            (8, "RM"): {"mean_sp_norm": 0.50},
            (8, "CFS"): {"mean_sp_norm": 0.45},
            (8, "INCR_NO_TL"): {"mean_sp_norm": 0.60},
            (8, "INCR_WCET"): {"mean_sp_norm": 0.55},
        }
        verdict = ev.evaluate_q3(lookup, large_n=8)
        self.assertEqual(verdict["status"], "PASS")

    def test_fail_baseline_beats_incr(self):
        lookup = {
            (8, "INCR"): {"mean_sp_norm": 0.55},
            (8, "INCR_SCRATCH"): {"mean_sp_norm": 0.68},
            (8, "RM"): {"mean_sp_norm": 0.50},
            (8, "CFS"): {"mean_sp_norm": 0.60},  # beats INCR
            (8, "INCR_NO_TL"): {"mean_sp_norm": 0.40},
            (8, "INCR_WCET"): {"mean_sp_norm": 0.45},
        }
        verdict = ev.evaluate_q3(lookup, large_n=8)
        self.assertEqual(verdict["status"], "FAIL")
        self.assertIn("INCR", verdict["detail"])

    def test_missing_baseline_is_noted(self):
        """A missing baseline is reported but does not auto-pass the gate."""
        lookup = {
            (8, "INCR"): {"mean_sp_norm": 0.70},
            (8, "INCR_SCRATCH"): {"mean_sp_norm": 0.68},
            (8, "RM"): {"mean_sp_norm": 0.50},
            # CFS, INCR_NO_TL, INCR_WCET absent
        }
        verdict = ev.evaluate_q3(lookup, large_n=8)
        # INCR/SCRATCH beat the only present baseline (RM), but the missing
        # ones must be surfaced in the detail so the run is auditable.
        self.assertIn("CFS", verdict["detail"])


# ---------------------------------------------------------------------------
# Gate E1: overhead <= 5% (ideal <= 1%) at N=10
# ---------------------------------------------------------------------------

class TestGateE1(unittest.TestCase):
    """E1: at N=10, INCR & SCRATCH overhead <= 5%."""

    def test_pass(self):
        lookup = {
            (10, "INCR"): {"overhead": 0.02},
            (10, "INCR_SCRATCH"): {"overhead": 0.03},
        }
        verdict = ev.evaluate_e1(lookup, overhead_n=10)
        self.assertEqual(verdict["status"], "PASS")
        # Ideal stretch (1%) is reported alongside.
        self.assertIn("ideal", verdict["detail"].lower())

    def test_fail_over_red_line(self):
        lookup = {
            (10, "INCR"): {"overhead": 0.06},  # 6% > 5%
            (10, "INCR_SCRATCH"): {"overhead": 0.03},
        }
        verdict = ev.evaluate_e1(lookup, overhead_n=10)
        self.assertEqual(verdict["status"], "FAIL")
        self.assertIn("INCR", verdict["detail"])

    def test_between_ideal_and_red_is_pass_with_note(self):
        """1% < overhead <= 5% passes the red line but misses the ideal."""
        lookup = {
            (10, "INCR"): {"overhead": 0.03},
            (10, "INCR_SCRATCH"): {"overhead": 0.04},
        }
        verdict = ev.evaluate_e1(lookup, overhead_n=10)
        self.assertEqual(verdict["status"], "PASS")

    def test_missing_n10(self):
        verdict = ev.evaluate_e1(lookup={}, overhead_n=10)
        self.assertEqual(verdict["status"], "FAIL")

    def test_missing_overhead_key_fails_not_crashes(self):
        """A present-but-overhead-less record must FAIL clearly, not KeyError."""
        lookup = {
            (10, "INCR"): {"mean_sp_norm": 0.55},            # no overhead
            (10, "INCR_SCRATCH"): {"overhead": 0.03},
        }
        verdict = ev.evaluate_e1(lookup, overhead_n=10)
        self.assertEqual(verdict["status"], "FAIL")
        self.assertIn("overhead missing", verdict["detail"])


# ---------------------------------------------------------------------------
# Gate E2: INCR_ET <= SCRATCH_ET at every N
# ---------------------------------------------------------------------------

class TestGateE2(unittest.TestCase):
    """E2: INCR.mean_sched_time <= INCR_SCRATCH.mean_sched_time at every N."""

    def test_pass_all_n(self):
        lookup = {
            (4, "INCR"): {"mean_sched_time": 0.05},
            (4, "INCR_SCRATCH"): {"mean_sched_time": 0.06},
            (8, "INCR"): {"mean_sched_time": 0.08},
            (8, "INCR_SCRATCH"): {"mean_sched_time": 0.10},
            (10, "INCR"): {"mean_sched_time": 0.12},
            (10, "INCR_SCRATCH"): {"mean_sched_time": 0.15},
        }
        verdict = ev.evaluate_e2(lookup, ns=[4, 8, 10])
        self.assertEqual(verdict["status"], "PASS")

    def test_fail_one_n(self):
        lookup = {
            (4, "INCR"): {"mean_sched_time": 0.05},
            (4, "INCR_SCRATCH"): {"mean_sched_time": 0.06},
            (8, "INCR"): {"mean_sched_time": 0.20},   # violates
            (8, "INCR_SCRATCH"): {"mean_sched_time": 0.10},
            (10, "INCR"): {"mean_sched_time": 0.12},
            (10, "INCR_SCRATCH"): {"mean_sched_time": 0.15},
        }
        verdict = ev.evaluate_e2(lookup, ns=[4, 8, 10])
        self.assertEqual(verdict["status"], "FAIL")
        self.assertIn("8", verdict["detail"])

    def test_missing_sched_time_fails_not_crashes(self):
        """A lookup entry lacking mean_sched_time must FAIL clearly, not KeyError.

        The structural claim "INCR <= SCRATCH" cannot be verified without an ET,
        so the gate reports a clear reason and FAILs rather than raising.
        """
        lookup = {
            (4, "INCR"): {"mean_sched_time": 0.05},
            (4, "INCR_SCRATCH"): {"mean_sched_time": 0.06},
            (8, "INCR"): {"mean_sp_norm": 0.60},          # no mean_sched_time
            (8, "INCR_SCRATCH"): {"mean_sched_time": 0.10},
        }
        verdict = ev.evaluate_e2(lookup, ns=[4, 8])
        self.assertEqual(verdict["status"], "FAIL")
        self.assertIn("missing", verdict["detail"].lower())


# ---------------------------------------------------------------------------
# Gate E3: INCR_P<n> period-monotonicity (ET non-increasing as period grows)
# ---------------------------------------------------------------------------

class TestGateE3(unittest.TestCase):
    """E3: ET(INCR_P1) >= ET(INCR_P10) >= ET(INCR_P30) >= ET(INCR_P60) per N."""

    ARMS = ["INCR_P1", "INCR_P10", "INCR_P30", "INCR_P60"]

    def test_pass_monotonic_all_n(self):
        lookup = {
            (4, "INCR_P1"): {"mean_sched_time": 0.10},
            (4, "INCR_P10"): {"mean_sched_time": 0.09},
            (4, "INCR_P30"): {"mean_sched_time": 0.08},
            (4, "INCR_P60"): {"mean_sched_time": 0.07},
            (8, "INCR_P1"): {"mean_sched_time": 0.20},
            (8, "INCR_P10"): {"mean_sched_time": 0.18},
            (8, "INCR_P30"): {"mean_sched_time": 0.16},
            (8, "INCR_P60"): {"mean_sched_time": 0.14},
        }
        verdict = ev.evaluate_e3(lookup, ns=[4, 8])
        self.assertEqual(verdict["status"], "PASS")

    def test_fail_inversion(self):
        """P10 rising above P1 (beyond tolerance) FAILs and names the pair."""
        lookup = {
            (4, "INCR_P1"): {"mean_sched_time": 0.04},
            (4, "INCR_P10"): {"mean_sched_time": 0.10},   # rises 150%
            (4, "INCR_P30"): {"mean_sched_time": 0.09},
            (4, "INCR_P60"): {"mean_sched_time": 0.08},
            (8, "INCR_P1"): {"mean_sched_time": 0.20},
            (8, "INCR_P10"): {"mean_sched_time": 0.18},
            (8, "INCR_P30"): {"mean_sched_time": 0.16},
            (8, "INCR_P60"): {"mean_sched_time": 0.14},
        }
        verdict = ev.evaluate_e3(lookup, ns=[4, 8])
        self.assertEqual(verdict["status"], "FAIL")
        self.assertIn("RISES", verdict["detail"])
        self.assertIn("INCR_P1->INCR_P10", verdict["detail"])

    def test_missing_arm_fails_not_crashes(self):
        """A missing period arm FAILs with a clear reason, not a KeyError."""
        lookup = {
            (4, "INCR_P1"): {"mean_sched_time": 0.10},
            (4, "INCR_P10"): {"mean_sched_time": 0.09},
            # INCR_P30 absent
            (4, "INCR_P60"): {"mean_sched_time": 0.07},
        }
        verdict = ev.evaluate_e3(lookup, ns=[4])
        self.assertEqual(verdict["status"], "FAIL")
        self.assertIn("missing", verdict["detail"].lower())
        self.assertIn("INCR_P30", verdict["detail"])

    def test_tolerance_allows_small_increase(self):
        """A 1% rise passes at the 2% tolerance (no flap on ET noise)."""
        lookup = {
            (4, "INCR_P1"): {"mean_sched_time": 0.100},
            (4, "INCR_P10"): {"mean_sched_time": 0.101},  # +1%, within tol
            (4, "INCR_P30"): {"mean_sched_time": 0.101},
            (4, "INCR_P60"): {"mean_sched_time": 0.100},
        }
        verdict = ev.evaluate_e3(lookup, ns=[4])
        self.assertEqual(verdict["status"], "PASS")


# ---------------------------------------------------------------------------
# End-to-end: full verdict + report writer
# ---------------------------------------------------------------------------

class TestEvaluateAllGates(unittest.TestCase):
    """evaluate_all_gates ties the 5 gates together into a verdict list."""

    def test_all_pass(self):
        lookup = {
            (4, "BF"): {"mean_sp_norm": 0.90, "mean_sched_time": 0.20},
            (4, "INCR"): {"mean_sp_norm": 0.80, "mean_sched_time": 0.05},
            (4, "INCR_SCRATCH"): {"mean_sp_norm": 0.78, "mean_sched_time": 0.06},
            (4, "INCR_P1"): {"mean_sched_time": 0.05},
            (4, "INCR_P10"): {"mean_sched_time": 0.045},
            (4, "INCR_P30"): {"mean_sched_time": 0.040},
            (4, "INCR_P60"): {"mean_sched_time": 0.035},
            (8, "BF"): {"mean_sp_norm": 0.55, "mean_sched_time": 2.0},
            (8, "INCR"): {"mean_sp_norm": 0.60, "mean_sched_time": 0.08},
            (8, "INCR_SCRATCH"): {"mean_sp_norm": 0.58, "mean_sched_time": 0.10},
            (8, "INCR_P1"): {"mean_sched_time": 0.08},
            (8, "INCR_P10"): {"mean_sched_time": 0.075},
            (8, "INCR_P30"): {"mean_sched_time": 0.070},
            (8, "INCR_P60"): {"mean_sched_time": 0.065},
            (8, "RM"): {"mean_sp_norm": 0.50},
            (8, "CFS"): {"mean_sp_norm": 0.45},
            (8, "INCR_NO_TL"): {"mean_sp_norm": 0.40},
            (8, "INCR_WCET"): {"mean_sp_norm": 0.45},
            (10, "INCR"): {"mean_sp_norm": 0.55, "overhead": 0.02,
                           "mean_sched_time": 0.20},
            (10, "INCR_SCRATCH"): {"mean_sp_norm": 0.54, "overhead": 0.03,
                                   "mean_sched_time": 0.30},
            (10, "INCR_P1"): {"mean_sched_time": 0.20},
            (10, "INCR_P10"): {"mean_sched_time": 0.19},
            (10, "INCR_P30"): {"mean_sched_time": 0.18},
            (10, "INCR_P60"): {"mean_sched_time": 0.17},
        }
        verdicts = ev.evaluate_all_gates(lookup, quality_ns=[4, 8],
                                         large_n=8, overhead_n=10)
        statuses = {v["gate"]: v["status"] for v in verdicts}
        self.assertEqual(statuses, {"Q1": "PASS", "Q2": "PASS",
                                    "Q3": "PASS", "E1": "PASS", "E2": "PASS",
                                    "E3": "PASS"})

    def test_any_fail_makes_overall_fail(self):
        lookup = {
            (4, "BF"): {"mean_sp_norm": 0.90, "mean_sched_time": 0.20,
                        "overhead": 0.02},
            (4, "INCR"): {"mean_sp_norm": 0.40, "mean_sched_time": 0.05,
                          "overhead": 0.005},  # Q1 fails (gap 55%)
            (4, "INCR_SCRATCH"): {"mean_sp_norm": 0.78, "mean_sched_time": 0.06,
                                  "overhead": 0.006},
        }
        # period_arms=[] short-circuits E3 to PASS so the overall FAIL is
        # attributable to Q1 alone (the gate under test), not a missing-arm E3.
        verdicts = ev.evaluate_all_gates(lookup, quality_ns=[4], large_n=4,
                                         overhead_n=4, period_arms=[])
        overall = ev.overall_status(verdicts)
        self.assertEqual(overall, "FAIL")


class TestWriteReport(unittest.TestCase):
    """write_report emits a machine-readable JSON next to the run."""

    def test_writes_json_with_verdicts(self):
        temp_base = tempfile.mkdtemp()
        try:
            cfg = _eval_cfg(temp_base, [4])
            run_root = agg.build_run_root(temp_base, cfg)
            os.makedirs(run_root, exist_ok=True)
            lookup = {
                (4, "BF"): {"mean_sp_norm": 0.90, "mean_sched_time": 0.2,
                            "overhead": 0.02},
                (4, "INCR"): {"mean_sp_norm": 0.80, "mean_sched_time": 0.05,
                              "overhead": 0.005},
                (4, "INCR_SCRATCH"): {"mean_sp_norm": 0.78,
                                      "mean_sched_time": 0.06,
                                      "overhead": 0.006},
                (4, "INCR_P1"): {"mean_sched_time": 0.05},
                (4, "INCR_P10"): {"mean_sched_time": 0.045},
                (4, "INCR_P30"): {"mean_sched_time": 0.040},
                (4, "INCR_P60"): {"mean_sched_time": 0.035},
            }
            verdicts = ev.evaluate_all_gates(lookup, quality_ns=[4],
                                             large_n=4, overhead_n=4)
            report_path = ev.write_report(run_root, cfg, lookup, verdicts)
            self.assertTrue(os.path.exists(report_path))
            with open(report_path) as f:
                report = json.load(f)
            self.assertIn("gates", report)
            self.assertEqual(len(report["gates"]), 6)
            self.assertIn("overall", report)
            self.assertIn("run_id", report)
            self.assertIn("metrics", report)
        finally:
            shutil.rmtree(temp_base)


class TestEndToEndMain(unittest.TestCase):
    """main() on a synthetic run tree writes the report and returns the right code."""

    def test_main_pass_exit_zero(self):
        temp_base = tempfile.mkdtemp()
        try:
            cfg = _eval_cfg(temp_base, [4, 8, 10])
            sp = {
                "INCR": {4: 0.80, 8: 0.60, 10: 0.55},
                "BF": {4: 0.90, 8: 0.55, 10: 0.50},
                "INCR_SCRATCH": {4: 0.78, 8: 0.58, 10: 0.54},
                "RM": {4: 0.50, 8: 0.45, 10: 0.40},
                "CFS": {4: 0.45, 8: 0.40, 10: 0.35},
                "INCR_NO_TL": {4: 0.40, 8: 0.40, 10: 0.35},
                "INCR_WCET": {4: 0.45, 8: 0.45, 10: 0.40},
                # E3 period arms -- SP is irrelevant to E3, but must be present
                # so _build_synthetic_run writes a CSV row for each arm.
                "INCR_P1": {4: 0.80, 8: 0.60, 10: 0.55},
                "INCR_P10": {4: 0.80, 8: 0.60, 10: 0.55},
                "INCR_P30": {4: 0.80, 8: 0.60, 10: 0.55},
                "INCR_P60": {4: 0.80, 8: 0.60, 10: 0.55},
            }
            et = {
                "INCR": {4: 0.05, 8: 0.08, 10: 0.20},
                "BF": {4: 0.20, 8: 2.0, 10: 5.0},
                "INCR_SCRATCH": {4: 0.06, 8: 0.10, 10: 0.30},
                "RM": {4: 0.01, 8: 0.01, 10: 0.01},
                "CFS": {4: 0.0, 8: 0.0, 10: 0.0},
                "INCR_NO_TL": {4: 0.04, 8: 0.07, 10: 0.18},
                "INCR_WCET": {4: 0.04, 8: 0.07, 10: 0.18},
                # E3: ET non-increasing as the reopt period grows (P1 >= P10
                # >= P30 >= P60) so the gate passes and main() exits 0.
                "INCR_P1": {4: 0.060, 8: 0.090, 10: 0.220},
                "INCR_P10": {4: 0.055, 8: 0.085, 10: 0.210},
                "INCR_P30": {4: 0.050, 8: 0.080, 10: 0.200},
                "INCR_P60": {4: 0.045, 8: 0.075, 10: 0.190},
            }
            _build_synthetic_run(temp_base, cfg, sp, et)

            # Write the cfg to disk so main() can load it via --config_json.
            # load_experiment_config reads plotting/analysis from the TOP LEVEL
            # (merged with the mode block), so write the file in that shape --
            # nesting them under prod_mode would drop run_name_prefix and make
            # main() compute a different run root than _build_synthetic_run did.
            cfg_path = os.path.join(temp_base, "eval_config.json")
            on_disk = {
                "prod_mode": {k: v for k, v in cfg.items()
                              if not k.startswith("_")},
                "plotting": cfg["plotting"],
                "analysis": cfg["analysis"],
            }
            with open(cfg_path, "w") as f:
                json.dump(on_disk, f)

            rc = ev.main([
                "--config_json", cfg_path,
                "--mode", "prod",
                "--output_parent", temp_base,
            ])
            self.assertEqual(rc, 0)
            report_path = os.path.join(
                agg.build_run_root(temp_base, cfg), "evaluation_report.json")
            self.assertTrue(os.path.exists(report_path))
        finally:
            shutil.rmtree(temp_base)


if __name__ == "__main__":
    unittest.main()
