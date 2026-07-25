"""Project evaluation suite -- the north-star integration test.

This module is the *reporting layer* on top of the existing end-to-end
pipeline. It does **not** run simulations (``run_simulation_and_plot_figures.sh`` does); it
ingests the ``comparison_summary.csv`` files the simulate stage already wrote,
normalizes SP via the shared :mod:`simulation_experiments.aggregate_across_tasks`
helpers, and evaluates the five north-star gates from
``agents/project_evaluation_northstar.md`` into a PASS/FAIL verdict.

The point: a single, reproducible green/red dashboard that measures the
ultimate impact of any code or algorithm change. Every change re-runs the
suite; regressions show as red.

North-star gates (evaluated PER-N at every conducted N)
-------------------------------------------------------
Each gate carries a ``per_n`` list of per-N verdicts (see :func:`_verdict`).
A gate's overall status is **FAIL only if some CONDUCTED N FAILs** -- a
missing N (in the regime but not simulated) is reported as ``MISSING`` and is
NOT fatal, so a fast run that only conducted N=4,6 is not red just because
N=8/10 were not run. The one loud-fail: if NO N was conducted at all (empty
regime / all missing) the gate is FAIL (stale/degenerate config must not
silently PASS).

- **Q1** small N (smaller half of quality_ns): BF >= INCR, gap <= 30%.
  Q1 is the BF-feasible regime. ``_split_regimes`` routes the smaller half of
  ``quality_ns`` here.
- **Q2** large N (larger half of quality_ns): INCR >= BF (BF time-out).
  Q1/Q2 are complementary regimes -- at small N BF>INCR is expected, so Q2 is
  not evaluated there (it would fail by design).
- **Q3** large N: INCR >= every baseline.
- **E1** overhead <= 5% (ideal <= 1%) at every conducted N (the configured
  overhead N is the headline probe).

The incremental optimizer the paper advances is exposed as the
``INCR_Reopt_10`` arm (reopt period 10 -- the same scheduler the bare ``INCR``
arm used to name; see ``tests/RunOrchestrator.cpp``). The bare ``INCR`` arm was
removed from every config's scheduler list as a redundant duplicate of
``INCR_Reopt_10`` (same dispatch path via ``IsINCRPeriodVariant``, same
``ReoptimizationPeriod``). All gates that previously read ``INCR`` as their
subject now read ``INCR_Reopt_10``: Q1 (BF-vs-INCR gap), Q2 (INCR>=BF), Q3
(INCR>=baselines), and E1 (INCR overhead).
- **E3** INCR_Reopt_X period-monotonicity: mean SP non-increasing as the reopt
  period grows, i.e.
  SP(INCR_Reopt_1) >= SP(INCR_Reopt_5) >= SP(INCR_Reopt_10) >=
  SP(INCR_Reopt_30) >= SP(INCR_Reopt_60), at every conducted N.
  More frequent reopt (small period) keeps TL configs fresher, so the
  smallest period carries the highest SP. (Earlier this gate read
  per-activation ET, but the recorded ``mean_sched_time`` is a
  scheduler-only (``DeterminePrioritiesAndBudgets``) wall-clock summed over
  intervals and averaged over tasksets -- dominated by OS/IO contention noise,
  not the reopt-period signal -- so it read SP instead, the metric the gate
  actually cares about. See ``p25-incr-et-grows-with-period``.)

P2.5 removed the ``INCR_SCRATCH`` ablation arm and the E2 gate (its only
subject pair was INCR-vs-SCRATCH); Q1/Q2/Q3/E1 now check INCR alone, and the
north-star dropped 6 gates -> 5.

Normalized SP = ``raw_SP / ideal_SP`` (the P12 fix; ``ideal_SP`` from
:func:`aggregate_across_tasks.compute_sp_upper_bound`), so the ratio is in
[0, 1] with 1.0 = "all deadlines met perfectly" and is independent of any
scheduler's realized value. Overhead = ``Mean_Scheduler_Execution_Time_s /
scheduler_trigger_interval_seconds`` (scheduler-only: the CSV column is the
per-interval ``DeterminePrioritiesAndBudgets`` time -- excludes RTDA rollout /
SP-metric / I/O; see ``run_sim_experiments.py``).

Usage
-----
    python3 -m simulation_experiments.evaluation_suite \\
        --config_json simulation_experiments/configs/gate_eval_config.json \\
        --mode prod

Or via the wrapper ``scripts/run_simulation_plot_eval_ns.sh`` which runs the pipeline
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

# Scheduler naming. ``INCR`` names the incremental optimizer the paper advances
# and is the gate subject for Q1/Q2/Q3/E1. The canonical arm exposing it is
# ``INCR_Reopt_10`` (reopt period 10) -- the bare ``INCR`` arm was a redundant
# duplicate of ``INCR_Reopt_10`` (same dispatch path, same
# ``ReoptimizationPeriod``) and was dropped from every config's scheduler list,
# so the gates read the one arm that is actually run. ``BF`` is the brute-force
# ceiling. (The ``INCR_SCRATCH`` ablation arm was removed in P2.5 -- see
# agents/active_tasks/P2_5_incr_scratch_removal/.)
INCR = "INCR_Reopt_10"
BF = "BF"
# Baselines that INCR must beat for gate Q3 (the ablation group minus INCR
# itself, plus the two non-optimizer schedulers).
Q3_BASELINES = ["RM", "CFS", "INCR_NO_TL", "INCR_WCET"]

# Default ordered period arms for gate E3 (the P1.1 A/B set). ``INCR_Reopt_10``
# (the canonical incremental arm the Q1/Q2/Q3/E1 gates read) IS the period=10
# member of this sweep. The config may override this via the eval-only key
# ``eval_period_arms``. P2.4 renamed the family from the retired INCR_P<n> form;
# X is the reopt period, ordered small (max reopt) to large (min reopt) so SP is
# expected non-increasing along the list (smaller period = fresher TL = higher
# SP). X=5 is a NEW arm (the pre-P2.4 family was {1,10,30,60}).
DEFAULT_PERIOD_ARMS = ["INCR_Reopt_1", "INCR_Reopt_5", "INCR_Reopt_10",
                       "INCR_Reopt_30", "INCR_Reopt_60"]

# North-star thresholds (from agents/project_evaluation_northstar.md).
Q1_MAX_GAP = 0.30      # small-N BF-vs-INCR gap red-flag line
E1_RED_LINE = 0.05     # overhead red-flag line
E1_IDEAL = 0.01        # overhead ideal
E3_TOLERANCE = 0.02    # E3: relative slack on each period step (SP_next <= SP_prev*(1+t))


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
#
# Per-N structure: each gate carries a ``per_n`` list of per-N entries
# {n, status, measured, detail} (E3 also adds a ``pairs`` sub-list). A gate's
# overall status is FAIL only if some CONDUCTED N FAILs; a MISSING N (in the
# regime but no data) is reported but not fatal. See the module docstring.
# ---------------------------------------------------------------------------

# Per-N status closed set. Only FAIL flips a gate red.
PER_N_STATUSES = ("PASS", "FAIL", "MISSING", "SKIP")


def _verdict(gate, status, measured, threshold, detail, per_n=None):
    """Build a verdict dict in the canonical shape.

    ``per_n`` (optional) is a list of per-N entry dicts (see :func:`_assemble`).
    Kept as a list, not a dict, so JSON serialization does not coerce int N to a
    string key and so ascending-N order is explicit.
    """
    v = {
        "gate": gate,
        "status": status,
        "measured": measured,
        "threshold": threshold,
        "detail": detail,
    }
    if per_n is not None:
        v["per_n"] = per_n
    return v


def _sp(lookup, n, sched):
    """Fetch normalized SP for (N, scheduler), or None if absent."""
    rec = lookup.get((n, sched))
    return rec["mean_sp_norm"] if rec else None


def _split_regimes(quality_ns):
    """Split quality_ns into (small_ns, large_ns) by floor-midpoint.

    The smaller half feeds Q1 (BF-feasible regime); the larger half feeds
    Q2/Q3 (BF-timeout regime). For odd lengths the EXTRA element goes to LARGE,
    so [4,6,8] -> small=[4], large=[6,8]. Rationale: Q2/Q3 are the "INCR wins"
    gates and benefit from more data points; Q1 is the "BF still feasible" gate
    and one point suffices.

    Returns ([], []) for empty input. Dedupes + sorts ascending.
    """
    ns = sorted(set(quality_ns))
    if not ns:
        return [], []
    mid = len(ns) // 2          # floor division; odd -> extra to large
    return ns[:mid], ns[mid:]


def _assemble(gate, per_n, threshold, summary):
    """Collapse per-N entries into one verdict.

    Overall status: FAIL if any CONDUCTED N FAILs; FAIL (not PASS) if no N was
    conducted at all (regime empty / all missing) -- the loud-fail for a stale
    or degenerate config. The top-level ``measured`` names the worst N; the
    ``detail`` joins the per-N details.
    """
    conducted = [e for e in per_n if e["status"] not in ("MISSING", "SKIP")]
    if not per_n or not conducted:
        # Loud-fail: no N was conducted (regime empty / all missing). Surface
        # the per-N details (not a generic message) so the specific missing
        # thing -- "BF missing", "INCR_Reopt_30", "overhead missing" -- is
        # named at the top level for a fast run that hit only MISSING entries.
        detail = "; ".join(f"N={e['n']}: {e['detail']}" for e in per_n) or \
            f"no conducted N for {gate} (regime empty or all missing)"
        return _verdict(gate, "FAIL", summary, threshold, detail, per_n=per_n)
    failed = [e for e in conducted if e["status"] == "FAIL"]
    overall = "FAIL" if failed else "PASS"
    worst = failed[0] if failed else conducted[-1]
    detail = "; ".join(f"N={e['n']}: {e['detail']}" for e in per_n)
    measured = f"worst: N={worst['n']} {worst['status']}"
    return _verdict(gate, overall, measured, threshold, detail, per_n=per_n)


# --- internal per-N helpers (return one per_n entry, not a verdict) ---

def _q1_at_n(lookup, n):
    """One per_n entry for Q1 at N: BF >= INCR, gap <= 30%."""
    bf = _sp(lookup, n, BF)
    if bf is None or bf <= 0:
        return {"n": n, "status": "MISSING", "measured": None,
                "detail": f"BF missing at N={n}; cannot evaluate small-N gap"}
    x = _sp(lookup, n, INCR)
    if x is None:
        return {"n": n, "status": "MISSING", "measured": None,
                "detail": f"INCR missing at N={n}; cannot evaluate small-N gap"}
    gap = (bf - x) / bf
    ok = gap <= Q1_MAX_GAP
    return {"n": n, "status": "PASS" if ok else "FAIL",
            "measured": f"gap={gap:.1%}",
            "detail": f"INCR={x:.4f} vs BF={bf:.4f} (gap={gap:.1%}, "
                      f"{'ok' if ok else f'EXCEEDS {Q1_MAX_GAP:.0%}'})"}


def _q2_at_n(lookup, n):
    """One per_n entry for Q2 at N: INCR >= BF."""
    bf = _sp(lookup, n, BF)
    x = _sp(lookup, n, INCR)
    if x is None or bf is None:
        return {"n": n, "status": "MISSING", "measured": None,
                "detail": f"INCR or BF missing at N={n}"}
    ok = x >= bf
    return {"n": n, "status": "PASS" if ok else "FAIL",
            "measured": f"INCR={x:.4f} vs BF={bf:.4f}",
            "detail": f"INCR={x:.4f} vs BF={bf:.4f} ({'ok' if ok else 'BELOW'})"}


def _q3_at_n(lookup, n):
    """One per_n entry for Q3 at N: INCR >= every PRESENT baseline.

    A missing baseline is noted in the detail but is NOT fatal for this N --
    only a baseline BEATING INCR fails the N (the missing ones are auditable
    via the detail string but do not auto-pass or auto-fail).
    """
    x = _sp(lookup, n, INCR)
    if x is None:
        return {"n": n, "status": "MISSING", "measured": None,
                "detail": f"INCR missing at N={n}"}
    beats = []
    missing_baselines = []
    n_fail = False
    for b in Q3_BASELINES:
        bsp = _sp(lookup, n, b)
        if bsp is None:
            missing_baselines.append(b)
            continue
        ok = x >= bsp
        if not ok:
            n_fail = True
        beats.append(f"{b}={bsp:.4f}({'ok' if ok else 'BEATS'})")
    parts = [f"INCR={x:.4f} vs [{', '.join(beats)}]"]
    if missing_baselines:
        parts.append("missing baselines: " + ", ".join(sorted(set(missing_baselines))))
    return {"n": n, "status": "FAIL" if n_fail else "PASS",
            "measured": f"INCR={x:.4f}",
            "detail": "; ".join(parts)}


def _e1_at_n(lookup, n):
    """One per_n entry for E1 at N: overhead <= 5% (ideal <= 1%)."""
    rec = lookup.get((n, INCR))
    if rec is None:
        return {"n": n, "status": "MISSING", "measured": None,
                "detail": f"INCR missing at N={n}"}
    ov = rec.get("overhead")
    if ov is None:
        # build_metric_lookup always sets overhead, so a missing key means a
        # malformed/partial lookup -- the gate cannot verify the claim.
        return {"n": n, "status": "MISSING", "measured": None,
                "detail": f"INCR: overhead missing at N={n} -- cannot verify"}
    ok = ov <= E1_RED_LINE
    ideal_met = ov <= E1_IDEAL
    detail = (f"INCR: overhead={ov:.2%} "
              f"(red {E1_RED_LINE:.0%} {'ok' if ok else 'EXCEEDS'}, "
              f"ideal {E1_IDEAL:.0%} {'ok' if ideal_met else 'missed'})")
    if not ideal_met:
        detail += "; ideal (1%) not met -- red line (5%) is the gate"
    return {"n": n, "status": "PASS" if ok else "FAIL",
            "measured": f"overhead={ov:.2%}", "detail": detail}


def _e3_at_n(lookup, n, period_arms):
    """One per_n entry for E3 at N: period arms' mean SP non-increasing.

    Reads ``mean_sp_norm`` (not ``mean_sched_time``) -- the SP-quality metric
    this gate cares about, not the noisy wall-clock ET (see module docstring).
    Each entry carries a structured ``pairs`` sub-list
    ``{prev, next, sp_prev, sp_next, ok}``. A missing arm at this N yields
    MISSING (reported, not fatal) -- so a missing arm at one N coexisting with
    a clean PASS at another N leaves the gate PASS.
    """
    if len(period_arms) < 2:
        return {"n": n, "status": "SKIP", "measured": "no pair",
                "detail": "fewer than 2 period arms configured", "pairs": []}
    sps = []
    missing = []
    for arm in period_arms:
        rec = lookup.get((n, arm))
        sp = rec.get("mean_sp_norm") if rec else None
        if sp is None:
            missing.append(arm)
        sps.append((arm, sp))
    if missing:
        return {"n": n, "status": "MISSING", "measured": None,
                "detail": f"missing arms {missing} at N={n}; cannot verify",
                "pairs": []}
    pairs = []
    n_ok = True
    for (arm_prev, sp_prev), (arm_next, sp_next) in zip(sps, sps[1:]):
        # non-increasing within tolerance: next <= prev * (1 + tol). SP is
        # highest at the smallest period (freshest TL), so a rise of next over
        # prev beyond tol is the violation.
        limit = sp_prev * (1.0 + E3_TOLERANCE)
        ok = sp_next <= limit
        if not ok:
            n_ok = False
        pairs.append({"prev": arm_prev, "next": arm_next,
                      "sp_prev": sp_prev, "sp_next": sp_next, "ok": ok})
    pair_str = "; ".join(
        f"{p['prev']}->{p['next']}: {p['sp_prev']:.4f}->{p['sp_next']:.4f} "
        f"({'ok' if p['ok'] else 'RISES'})" for p in pairs)
    return {"n": n, "status": "PASS" if n_ok else "FAIL",
            "measured": "monotonic" if n_ok else "RISES",
            "detail": pair_str, "pairs": pairs}


# --- public evaluators (legacy scalar params map to single-element N-lists) ---

def evaluate_q1(lookup, small_n=None, small_ns=None):
    """Q1: per-N over small_ns. BF >= INCR with gap <= 30% at each small N.

    ``small_ns`` wins; the legacy scalar ``small_n`` maps to ``[small_n]``; if
    neither is given, defaults to ``[4]``.
    """
    if small_ns is None:
        small_ns = [small_n] if small_n is not None else [4]
    per_n = [_q1_at_n(lookup, n) for n in sorted(set(small_ns))]
    return _assemble("Q1", per_n, f"gap <= {Q1_MAX_GAP:.0%}",
                     "small-N BF>=INCR, gap<=30%")


def evaluate_q2(lookup, large_n=None, large_ns=None):
    """Q2: per-N over large_ns. INCR >= BF (BF time-out) at each large N."""
    if large_ns is None:
        large_ns = [large_n] if large_n is not None else [8]
    per_n = [_q2_at_n(lookup, n) for n in sorted(set(large_ns))]
    return _assemble("Q2", per_n, "INCR >= BF",
                     "large-N INCR>=BF (BF time-out)")


def evaluate_q3(lookup, large_n=None, large_ns=None):
    """Q3: per-N over large_ns. INCR >= every baseline at each large N."""
    if large_ns is None:
        large_ns = [large_n] if large_n is not None else [8]
    per_n = [_q3_at_n(lookup, n) for n in sorted(set(large_ns))]
    return _assemble("Q3", per_n, "INCR >= max(baselines)",
                     "large-N INCR>=every baseline")


def evaluate_e1(lookup, overhead_n=None, overhead_ns=None):
    """E1: per-N over overhead_ns. overhead <= 5% (ideal <= 1%) at each N.

    ``overhead_ns`` wins; the legacy scalar ``overhead_n`` maps to
    ``[overhead_n]``; if neither is given, defaults to ``[10]``. In
    :func:`evaluate_all_gates` the suite passes EVERY conducted N so E1 reports
    overhead at all of them (the configured overhead N is the headline probe).
    """
    if overhead_ns is None:
        overhead_ns = [overhead_n] if overhead_n is not None else [10]
    per_n = [_e1_at_n(lookup, n) for n in sorted(set(overhead_ns))]
    return _assemble("E1", per_n, f"<= {E1_RED_LINE:.0%} (ideal {E1_IDEAL:.0%})",
                     "overhead<=5% (ideal 1%)")


def evaluate_e3(lookup, ns=None, period_arms=None):
    """E3: INCR_Reopt_X period-monotonicity -- mean SP non-increasing.

    For each N, the ordered period arms' ``mean_sp_norm`` must be
    non-increasing (each step within the relative tolerance): the highest-SP
    arm is the smallest period (freshest TL configs). The canonical incremental
    arm ``INCR_Reopt_10`` (the Q1/Q2/Q3/E1 subject) is the period=10 member of
    this sweep. (Previously read ``mean_sched_time``; that metric was a noisy
    whole-run wall-clock average, not the per-activation ET the gate intended
    -- see module docstring.)
    """
    if period_arms is None:
        period_arms = DEFAULT_PERIOD_ARMS
    if ns is None:
        ns = sorted({n for (n, s) in lookup.keys()})
    per_n = [_e3_at_n(lookup, n, period_arms) for n in sorted(set(ns))]
    arms_str = " >= ".join(period_arms)
    return _assemble("E3", per_n, f"period-monotonicity per N",
                     f"{arms_str} (tol +/-{E3_TOLERANCE:.0%})",
                     )


# ---------------------------------------------------------------------------
# Top-level: run all gates, summarize, write report.
# ---------------------------------------------------------------------------

def evaluate_all_gates(lookup, quality_ns=None, large_n=None, overhead_n=None,
                       period_arms=None):
    """Run all 5 north-star gates against the lookup.

    Routing (each gate is evaluated PER-N at every conducted N):

    - **Q1** over the smaller half of ``quality_ns`` (``_split_regimes``).
    - **Q2/Q3** over the larger half.
    - **E1** over EVERY N present in the lookup (the configured overhead N is
      the headline probe, not the only one).
    - **E3** over every N present in the lookup.

    So a fast run that conducted only N=4,6 evaluates Q1@N=4, Q2/Q3@N=6, and
    E1/E3 at both -- the points actually run, not a missing N=8/10 headline.

    Parameters
    ----------
    lookup : dict
        Output of :func:`build_metric_lookup`.
    quality_ns : list[int] | None
        N values used for SP-quality gates; split into small (Q1) / large
        (Q2/Q3) regimes. Falls back to the lookup's N set when ``None``.
    large_n : int | None
        Override Q2/Q3 to a single N (otherwise the larger half of quality_ns).
    overhead_n : int | None
        Headline overhead N for E1 (folded into the every-N set; CLI override).
    period_arms : list[str] | None
        Ordered INCR_Reopt_X arms for E3 (defaults to :data:`DEFAULT_PERIOD_ARMS`).
    """
    if quality_ns:
        ns_present = quality_ns
    else:
        ns_present = sorted({n for (n, _) in lookup.keys()})
    # Regime split: smaller half -> Q1 (BF-feasible), larger half -> Q2/Q3
    # (BF-timeout). [4,6]->small=[4],large=[6] so a fast run evaluates Q1 at
    # N=4 and Q2/Q3 at N=6 -- the points actually conducted.
    small_ns, large_ns = _split_regimes(ns_present)
    if large_n is not None:
        large_ns = [large_n]   # explicit override wins
    # E1 reports overhead at EVERY conducted N (the configured overhead N is
    # the headline probe, not the only one); CLI/config override is folded in.
    all_conducted_ns = sorted({n for (n, _) in lookup.keys()})
    if overhead_n is not None and overhead_n not in all_conducted_ns:
        all_conducted_ns = sorted(set(all_conducted_ns) | {overhead_n})

    return [
        evaluate_q1(lookup, small_ns=small_ns),
        evaluate_q2(lookup, large_ns=large_ns),
        evaluate_q3(lookup, large_ns=large_ns),
        evaluate_e1(lookup, overhead_ns=all_conducted_ns),
        evaluate_e3(lookup, ns=all_conducted_ns, period_arms=period_arms),
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
        # Per-N breakdown: one line per conducted (and MISSING/SKIP) N so the
        # status at each N actually run is visible at a glance. A missing N
        # shows as MISSING (not a red headline) -- the "not fatal" guarantee.
        per_n = v.get("per_n") or []
        if per_n:
            for e in per_n:
                lines.append(f"      -> N={e['n']}: {e['status']:<6} {e['detail']}")
        else:
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
            "north-star gates (Q1-Q3, E1, E3). Does not run simulations -- "
            "run scripts/run_simulation_plot_eval_ns.sh (or run_simulation_and_plot_figures.sh) first."
        )
    )
    parser.add_argument(
        "--config_json",
        default=os.path.join(os.path.dirname(__file__), "configs",
                             "gate_eval_config.json"),
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
        print("Run the pipeline first: ./scripts/run_simulation_plot_eval_ns.sh")
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
