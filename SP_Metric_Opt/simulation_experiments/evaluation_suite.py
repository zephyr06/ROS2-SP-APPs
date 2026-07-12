"""Project evaluation suite -- the north-star integration test.

This module is the *reporting layer* on top of the existing end-to-end
pipeline. It does **not** run simulations (``run_end_to_end.sh`` does); it
ingests the ``comparison_summary.csv`` files the simulate stage already wrote,
normalizes SP via the shared :mod:`simulation_experiments.aggregate_across_tasks`
helpers, and evaluates the five north-star gates from
``agents/project_evaluation_northstar.md`` into a PASS/FAIL verdict.

The point: a single, reproducible green/red dashboard that measures the
ultimate impact of any code or algorithm change. Every change re-runs the
suite; regressions show as red.

North-star gates
----------------
- **Q1** small N: BF >= INCR & SCRATCH, gap <= 30%           (N = smallest quality N)
- **Q2** large N: INCR & SCRATCH >= BF (BF time-out)          (N = largest quality N)
- **Q3** INCR & SCRATCH >= every baseline                      (N = largest quality N)
- **E1** overhead <= 5% (ideal <= 1%) at the overhead probe N  (N = overhead N, >= 10)
- **E2** INCR scheduler ET <= SCRATCH ET at every N
- **E3** INCR_Reopt_X period-monotonicity: per-activation ET   (every N)
  non-increasing as the reopt period grows, i.e.
  ET(INCR_Reopt_1) >= ET(INCR_Reopt_5) >= ET(INCR_Reopt_10) >=
  ET(INCR_Reopt_30) >= ET(INCR_Reopt_60).
  Investigation gate for the P1.1 residual; currently FAILs by design.

Normalized SP = ``raw_SP / ideal_SP`` (the P12 fix; ``ideal_SP`` from
:func:`aggregate_across_tasks.compute_sp_upper_bound`), so the ratio is in
[0, 1] with 1.0 = "all deadlines met perfectly" and is independent of any
scheduler's realized value. Overhead = ``Mean_Scheduler_Execution_Time_s /
scheduler_trigger_interval_seconds`` (the CSV column is already per-interval --
see ``run_sim_experiments.py``).

Usage
-----
    python3 -m simulation_experiments.evaluation_suite \\
        --config_json simulation_experiments/configs/evaluation_suite_config.json \\
        --mode prod

Or via the wrapper ``scripts/run_evaluation_suite.sh`` which runs the pipeline
first. Exit code is 0 only if every gate passes, so CI / the shell wrapper can
gate on it.
"""
import argparse
import json
import os
import sys
from datetime import datetime, timezone

# Ensure project root is importable when run as a script.
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from simulation_experiments.aggregate_across_tasks import (  # noqa: E402
    OPTIMIZER_COMPARISON_DIR,
    aggregate_data_from_directories,
    normalize_records_sp,
)
from simulation_experiments.experiment_config_loader import (  # noqa: E402
    build_run_id,
    build_run_root,
    load_experiment_config,
)

# Scheduler naming. The from-scratch optimizer is the ``INCR_SCRATCH`` mode
# (no bare ``SCRATCH`` mode exists in RunOrchestrator). ``INCR`` is the
# incremental optimizer the paper advances; ``BF`` is the brute-force ceiling.
INCR = "INCR"
SCRATCH = "INCR_SCRATCH"
BF = "BF"
# Baselines that INCR/SCRATCH must beat for gate Q3 (the ablation group minus
# INCR/SCRATCH themselves, plus the two non-optimizer schedulers).
Q3_BASELINES = ["RM", "CFS", "INCR_NO_TL", "INCR_WCET"]

# Default ordered period arms for gate E3 (the P1.1 A/B set). Bare ``INCR`` is
# NOT a member -- its default period isn't a sweep point. The config may
# override this via the eval-only key ``eval_period_arms``. P2.4 renamed the
# family from the retired INCR_P<n> form; X is the reopt period, ordered small
# (max reopt) to large (min reopt) so ET is expected non-increasing. X=5 is a
# NEW arm (the pre-P2.4 family was {1,10,30,60}).
DEFAULT_PERIOD_ARMS = ["INCR_Reopt_1", "INCR_Reopt_5", "INCR_Reopt_10",
                       "INCR_Reopt_30", "INCR_Reopt_60"]

# North-star thresholds (from agents/project_evaluation_northstar.md).
Q1_MAX_GAP = 0.30      # small-N BF-vs-INCR/SCRATCH gap red-flag line
E1_RED_LINE = 0.05     # overhead red-flag line
E1_IDEAL = 0.01        # overhead ideal
E3_TOLERANCE = 0.02    # E3: relative slack on each period step (ET next <= prev*(1+t))


# ---------------------------------------------------------------------------
# Lookup construction (reuses aggregate helpers -- no parallel metric math)
# ---------------------------------------------------------------------------

def build_metric_lookup(cfg, output_parent=None, run_root=None):
    """Build the ``(num_tasks, scheduler) -> metrics`` lookup for one run.

    Reuses :func:`aggregate_data_from_directories` (run-scoped ingestion of
    ``comparison_summary.csv``) and :func:`normalize_records_sp` (SP / ideal_SP),
    so the suite shares one source of truth with the figure pipeline.

    Parameters
    ----------
    cfg : dict
        Loaded experiment config (flat, as returned by
        :func:`load_experiment_config`).
    output_parent : str
        Base directory holding ``runs/<run_id>/`` (defaults to the shipped
        ``optimizer_comparison`` dir).
    run_root : str | None
        Override the run root (evaluate an already-completed run without
        re-deriving the path).

    Returns
    -------
    dict
        ``{(num_tasks, scheduler): {mean_sp_norm, mean_sp_raw, mean_sched_time,
        overhead, std_sp_norm}}``. Empty if no records were found.
    """
    if output_parent is None:
        output_parent = OPTIMIZER_COMPARISON_DIR
    records = aggregate_data_from_directories(cfg=cfg, output_parent=output_parent)
    if not records:
        return {}

    norm_records = normalize_records_sp(records, cfg)

    interval = cfg.get("scheduler_trigger_interval_seconds", 1) or 1
    lookup = {}
    for r in norm_records:
        key = (r["num_tasks"], r["scheduler"])
        mean_sched_time = r.get("mean_sched_time", 0.0)
        lookup[key] = {
            "mean_sp_norm": r["mean_sp"],
            "std_sp_norm": r.get("std_sp", 0.0),
            "mean_sched_time": mean_sched_time,
            "overhead": mean_sched_time / interval,
        }
    return lookup


# ---------------------------------------------------------------------------
# Gate evaluators -- one pure function per gate, each returns a verdict dict.
# ---------------------------------------------------------------------------

def _verdict(gate, status, measured, threshold, detail):
    """Build a verdict dict in the canonical shape."""
    return {
        "gate": gate,
        "status": status,
        "measured": measured,
        "threshold": threshold,
        "detail": detail,
    }


def _sp(lookup, n, sched):
    """Fetch normalized SP for (N, scheduler), or None if absent."""
    rec = lookup.get((n, sched))
    return rec["mean_sp_norm"] if rec else None


def evaluate_q1(lookup, small_n=4):
    """Q1: at small N, BF >= INCR & SCRATCH with gap <= 30%.

    Gap = ``(BF - X) / BF`` for X in {INCR, SCRATCH}. PASS only if BF is present
    and the gap to *both* is within the red-flag line. (BF below INCR is also a
    PASS -- the gate only flags BF *too far ahead*.)
    """
    bf = _sp(lookup, small_n, BF)
    if bf is None or bf <= 0:
        return _verdict("Q1", "FAIL", None, f"gap <= {Q1_MAX_GAP:.0%}",
                        f"BF missing at N={small_n}; cannot evaluate small-N gap")
    details = []
    overall_pass = True
    worst_gap = 0.0
    for sched, label in [(INCR, "INCR"), (SCRATCH, "SCRATCH")]:
        x = _sp(lookup, small_n, sched)
        if x is None:
            details.append(f"{label} missing at N={small_n}")
            overall_pass = False
            continue
        gap = (bf - x) / bf
        worst_gap = max(worst_gap, gap)
        verdict = "ok" if gap <= Q1_MAX_GAP else "EXCEEDS"
        if gap > Q1_MAX_GAP:
            overall_pass = False
        details.append(f"{label}: gap={gap:.1%} ({verdict})")
    status = "PASS" if overall_pass else "FAIL"
    return _verdict("Q1", status, f"worst gap={worst_gap:.1%}",
                    f"gap <= {Q1_MAX_GAP:.0%}", "; ".join(details))


def evaluate_q2(lookup, large_n=8):
    """Q2: at large N, INCR & SCRATCH >= BF (BF time-out)."""
    bf = _sp(lookup, large_n, BF)
    details = []
    overall_pass = True
    for sched, label in [(INCR, "INCR"), (SCRATCH, "SCRATCH")]:
        x = _sp(lookup, large_n, sched)
        if x is None or bf is None:
            details.append(f"{label}: missing at N={large_n}")
            overall_pass = False
            continue
        ok = x >= bf
        if not ok:
            overall_pass = False
        details.append(f"{label}={x:.4f} vs BF={bf:.4f} ({'ok' if ok else 'BELOW'})")
    status = "PASS" if overall_pass else "FAIL"
    measured = f"INCR,SCRATCH vs BF @ N={large_n}"
    return _verdict("Q2", status, measured, "INCR,SCRATCH >= BF", "; ".join(details))


def evaluate_q3(lookup, large_n=8):
    """Q3: INCR & SCRATCH >= every baseline at large N."""
    details = []
    overall_pass = True
    missing_baselines = []
    for sched, label in [(INCR, "INCR"), (SCRATCH, "SCRATCH")]:
        x = _sp(lookup, large_n, sched)
        if x is None:
            details.append(f"{label} missing at N={large_n}")
            overall_pass = False
            continue
        beats = []
        for b in Q3_BASELINES:
            bsp = _sp(lookup, large_n, b)
            if bsp is None:
                missing_baselines.append(b)
                continue
            ok = x >= bsp
            if not ok:
                overall_pass = False
            beats.append(f"{b}={bsp:.4f}({'ok' if ok else 'BEATS'})")
        details.append(f"{label}={x:.4f} vs [{', '.join(beats)}]")
    if missing_baselines:
        details.append("missing baselines: " + ", ".join(sorted(set(missing_baselines))))
    status = "PASS" if overall_pass else "FAIL"
    return _verdict("Q3", status, f"INCR,SCRATCH vs baselines @ N={large_n}",
                    "INCR,SCRATCH >= max(baselines)", "; ".join(details))


def evaluate_e1(lookup, overhead_n=10):
    """E1: scheduler overhead <= 5% (ideal <= 1%) at the overhead probe N."""
    details = []
    overall_pass = True
    ideal_met = True
    for sched, label in [(INCR, "INCR"), (SCRATCH, "SCRATCH")]:
        rec = lookup.get((overhead_n, sched))
        if rec is None:
            details.append(f"{label} missing at N={overhead_n}")
            overall_pass = False
            continue
        ov = rec.get("overhead")
        if ov is None:
            # build_metric_lookup always sets overhead, so a missing key means
            # a malformed/partial lookup -- the gate cannot verify the claim.
            details.append(f"{label}: overhead missing at N={overhead_n} -- cannot verify")
            overall_pass = False
            continue
        if ov > E1_RED_LINE:
            overall_pass = False
        if ov > E1_IDEAL:
            ideal_met = False
        details.append(f"{label}: overhead={ov:.2%} "
                       f"(red {E1_RED_LINE:.0%} {'ok' if ov <= E1_RED_LINE else 'EXCEEDS'}, "
                       f"ideal {E1_IDEAL:.0%} {'ok' if ov <= E1_IDEAL else 'missed'})")
    if not ideal_met:
        details.append("ideal (1%) not met by all -- red line (5%) is the gate")
    status = "PASS" if overall_pass else "FAIL"
    return _verdict("E1", status, f"overhead @ N={overhead_n}",
                    f"<= {E1_RED_LINE:.0%} (ideal {E1_IDEAL:.0%})", "; ".join(details))


def evaluate_e2(lookup, ns=None):
    """E2: INCR scheduler ET <= SCRATCH ET at every N.

    The north-star says "INCR cannot run slower than SCRATCH" -- checked at
    every N the suite ran, since the claim is structural, not N-specific.
    """
    if ns is None:
        ns = sorted({n for (n, s) in lookup.keys()})
    details = []
    overall_pass = True
    for n in ns:
        incr = lookup.get((n, INCR))
        scr = lookup.get((n, SCRATCH))
        if incr is None or scr is None:
            details.append(f"N={n}: INCR or SCRATCH missing")
            overall_pass = False
            continue
        incr_et = incr.get("mean_sched_time")
        scr_et = scr.get("mean_sched_time")
        if incr_et is None or scr_et is None:
            # The claim is structural ("INCR cannot run slower than SCRATCH");
            # a missing ET means it cannot be verified -> FAIL with a reason,
            # not a silent skip and not a KeyError.
            details.append(f"N={n}: mean_sched_time missing -- cannot verify")
            overall_pass = False
            continue
        ok = incr_et <= scr_et
        if not ok:
            overall_pass = False
        details.append(
            f"N={n}: INCR={incr_et:.4f}s vs "
            f"SCRATCH={scr_et:.4f}s ({'ok' if ok else 'SLOWER'})"
        )
    status = "PASS" if overall_pass else "FAIL"
    return _verdict("E2", status, "INCR_ET vs SCRATCH_ET per N",
                    "INCR_ET <= SCRATCH_ET", "; ".join(details))


def evaluate_e3(lookup, ns=None, period_arms=None):
    """E3: INCR_Reopt_X period-monotonicity -- per-activation ET non-increasing.

    For each N, the ordered period arms' ``mean_sched_time`` must be
    non-increasing (each step within the relative tolerance): the cheapest arm
    is the largest period. This is the P1.1 investigation gate; it currently
    FAILs (P1 is still the cheapest) and tracks that residual rather than
    being a clean regression gate. Bare ``INCR`` is not a member.
    """
    if period_arms is None:
        period_arms = DEFAULT_PERIOD_ARMS
    if ns is None:
        ns = sorted({n for (n, s) in lookup.keys()})
    if len(period_arms) < 2:
        return _verdict("E3", "PASS", "no pair to compare",
                        f"non-increasing over {period_arms}",
                        "fewer than 2 period arms configured")
    details = []
    overall_pass = True
    for n in ns:
        ets = []
        missing = []
        for arm in period_arms:
            rec = lookup.get((n, arm))
            et = rec.get("mean_sched_time") if rec else None
            if et is None:
                missing.append(arm)
            ets.append((arm, et))
        if missing:
            details.append(f"N={n}: missing {missing} -- cannot verify")
            overall_pass = False
            continue
        pairs = []
        n_ok = True
        for (arm_prev, et_prev), (arm_next, et_next) in zip(ets, ets[1:]):
            # non-increasing within tolerance: next <= prev * (1 + tol)
            limit = et_prev * (1.0 + E3_TOLERANCE)
            ok = et_next <= limit
            if not ok:
                n_ok = False
                overall_pass = False
            pairs.append(f"{arm_prev}->{arm_next}: "
                         f"{et_prev:.4f}->{et_next:.4f} "
                         f"({'ok' if ok else 'RISES'})")
        details.append(f"N={n}: " + "; ".join(pairs) + (" ok" if n_ok else ""))
    status = "PASS" if overall_pass else "FAIL"
    arms_str = " >= ".join(period_arms)
    return _verdict("E3", status, f"period-monotonicity per N",
                    f"{arms_str} (tol +/-{E3_TOLERANCE:.0%})", "; ".join(details))


# ---------------------------------------------------------------------------
# Top-level: run all gates, summarize, write report.
# ---------------------------------------------------------------------------

def evaluate_all_gates(lookup, quality_ns=None, large_n=None, overhead_n=None,
                       period_arms=None):
    """Run all 6 north-star gates against the lookup.

    Parameters
    ----------
    lookup : dict
        Output of :func:`build_metric_lookup`.
    quality_ns : list[int] | None
        N values used for SP-quality gates. ``small_n = min(quality_ns)``,
        ``large_n`` defaults to ``max(quality_ns)``.
    large_n : int | None
        Override the N for Q2/Q3 (defaults to max of quality_ns).
    overhead_n : int | None
        N for E1 (defaults to the max N in the lookup, the overhead probe).
    period_arms : list[str] | None
        Ordered INCR_Reopt_X arms for E3 (defaults to :data:`DEFAULT_PERIOD_ARMS`).
    """
    if quality_ns:
        ns_present = quality_ns
    else:
        ns_present = sorted({n for (n, _) in lookup.keys()})
    small_n = min(ns_present) if ns_present else 4
    if large_n is None:
        large_n = max(ns_present) if ns_present else 8
    if overhead_n is None:
        all_ns = sorted({n for (n, _) in lookup.keys()})
        overhead_n = max(all_ns) if all_ns else 10
    e2_ns = sorted({n for (n, _) in lookup.keys()})

    return [
        evaluate_q1(lookup, small_n=small_n),
        evaluate_q2(lookup, large_n=large_n),
        evaluate_q3(lookup, large_n=large_n),
        evaluate_e1(lookup, overhead_n=overhead_n),
        evaluate_e2(lookup, ns=e2_ns),
        evaluate_e3(lookup, ns=e2_ns, period_arms=period_arms),
    ]


def gate_ns_from_cfg(cfg):
    """Read the gate->N mapping from the eval config.

    Returns ``(quality_ns, overhead_n)`` from the eval-specific keys
    (``eval_quality_task_counts`` / ``eval_overhead_task_count``); falls back to
    ``(num_tasks_for_cross_task_comparison, max of that)`` when the eval keys
    are absent, so any standard experiment config still works.
    """
    quality_ns = list(cfg.get("eval_quality_task_counts") or
                      cfg.get("num_tasks_for_cross_task_comparison") or [])
    overhead_n = cfg.get("eval_overhead_task_count")
    if overhead_n is None:
        overhead_n = max(quality_ns) if quality_ns else 10
    return quality_ns, overhead_n


def overall_status(verdicts):
    """PASS only if every gate passed."""
    return "PASS" if all(v["status"] == "PASS" for v in verdicts) else "FAIL"


def _format_table(verdicts, overall):
    """Render the human-readable PASS/FAIL table as a string."""
    lines = []
    lines.append("=" * 72)
    lines.append("PROJECT EVALUATION SUITE -- NORTH-STAR VERDICT")
    lines.append("=" * 72)
    header = f"{'Gate':<5} {'Status':<6} {'Threshold':<28} {'Measured':<22}"
    lines.append(header)
    lines.append("-" * 72)
    for v in verdicts:
        lines.append(
            f"{v['gate']:<5} {v['status']:<6} {str(v['threshold']):<28} "
            f"{str(v['measured']):<22}"
        )
        lines.append(f"      -> {v['detail']}")
    lines.append("-" * 72)
    lines.append(f"OVERALL: {overall}")
    lines.append("=" * 72)
    return "\n".join(lines)


def write_report(run_root, cfg, lookup, verdicts):
    """Write the machine-readable ``evaluation_report.json`` into the run root.

    Contains: per-gate verdict, overall status, the full metric lookup, the
    run_id, and a timestamp -- enough to diff two runs for trend tracking.
    """
    overall = overall_status(verdicts)
    report = {
        "run_id": build_run_id(cfg),
        "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "overall": overall,
        "gates": verdicts,
        "metrics": {
            f"N{n}_{s}": m for (n, s), m in sorted(lookup.items())
        },
        "thresholds": {
            "Q1_max_gap": Q1_MAX_GAP,
            "E1_red_line": E1_RED_LINE,
            "E1_ideal": E1_IDEAL,
            "E3_tolerance": E3_TOLERANCE,
        },
    }
    report_path = os.path.join(run_root, "evaluation_report.json")
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    return report_path


def main(argv=None):
    """CLI entry. Returns the process exit code (0 = all gates PASS)."""
    parser = argparse.ArgumentParser(
        description=(
            "Project evaluation suite: evaluate a completed run against the "
            "north-star gates (Q1-Q3, E1, E2). Does not run simulations -- "
            "run scripts/run_evaluation_suite.sh (or run_end_to_end.sh) first."
        )
    )
    parser.add_argument(
        "--config_json",
        default=os.path.join(os.path.dirname(__file__), "configs",
                             "evaluation_suite_config.json"),
        help="Path to the evaluation suite config JSON.",
    )
    parser.add_argument(
        "--mode", choices=["test", "prod"], default="prod",
        help="Experiment mode (selects the config block; default prod).",
    )
    parser.add_argument(
        "--output_parent", default=None,
        help="Base dir holding runs/<run_id>/. Defaults to the shipped "
             "optimizer_comparison dir.",
    )
    parser.add_argument(
        "--run_root", default=None,
        help="Override the run root (evaluate a specific completed run).",
    )
    parser.add_argument(
        "--overhead_n", type=int, default=None,
        help="N for the E1 overhead gate (default: max N in the run).",
    )
    args = parser.parse_args(argv)

    cfg = load_experiment_config(mode=args.mode, config_path=args.config_json)

    output_parent = args.output_parent
    if output_parent is None:
        output_parent = OPTIMIZER_COMPARISON_DIR
    elif not os.path.isabs(output_parent):
        output_parent = os.path.join(_PROJECT_ROOT, output_parent)

    if args.run_root:
        run_root = args.run_root
    else:
        run_root = build_run_root(output_parent, cfg)

    quality_ns, overhead_n = gate_ns_from_cfg(cfg)

    print(f"Run root: {run_root}")
    print("Loading metrics ...")
    lookup = build_metric_lookup(cfg, output_parent=output_parent, run_root=run_root)
    if not lookup:
        sim_dir = os.path.join(run_root, "sim")
        print(f"No records found under {sim_dir}.")
        print("Run the pipeline first: ./scripts/run_evaluation_suite.sh")
        return 1
    print(f"Loaded {len(lookup)} (N, scheduler) records.")

    # CLI override wins over the config's overhead N.
    if args.overhead_n is not None:
        overhead_n = args.overhead_n

    # Period arms for E3 come from the eval-only config key; fall back to the
    # module default so a standard (non-eval) config still evaluates E3.
    period_arms = cfg.get("eval_period_arms") or DEFAULT_PERIOD_ARMS

    verdicts = evaluate_all_gates(lookup, quality_ns=quality_ns,
                                  overhead_n=overhead_n,
                                  period_arms=period_arms)
    overall = overall_status(verdicts)

    print()
    print(_format_table(verdicts, overall))

    report_path = write_report(run_root, cfg, lookup, verdicts)
    print(f"\nReport: {report_path}")
    return 0 if overall == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
