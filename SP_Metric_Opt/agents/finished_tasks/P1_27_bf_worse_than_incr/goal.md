# P1.27 — BF Worse Than INCR_Reopt_10 in Comparison Run

**Priority:** P1 (correctness investigation; violates the P0.2 `INCR ≤ BF` invariant)
**Status:** FIX LANDED (git add-only — NOT committed). Root cause = hypothesis #2
(P0.10 fallback firing) + #3-shape (gate post-search, not in-search). See
`dev_log.md` 2026-08-07.
**Reference:** overall task `P0.2` (focused BF correctness audit, `INCR ≤ BF`);
memory `p114-bf-time-limit-violation.md`, `p010-bf-important-task-fallback.md`,
`p218-p07-gate-arms-rta-cache-mid-beam.md`.

## The anomaly (user, 2026-08-07)

In the comparison run
`simulation_experiments/optimizer_comparison/runs/compare_against_bf_run_test_dur600_interval10_seed1000_tasks4/sim/tasks4_dur600_interval10_seed1000/`,
`comparison_summary.csv` reports:

| Scheduler        | Mean_SP_Metric | Mean_Miss_Rate | Mean_Scheduler_Execution_Time_s |
|-----------------|----------------|----------------|---------------------------------|
| `INCR_Reopt_10` | **0.922016**   | 0.001989       | 0.001779                        |
| `BF`            | **0.908552**   | 0.005779       | 0.097156                        |
| `INCR_NO_FALLBACK` | 0.929849    | 0.000000       | 0.001265                        |
| `INCR_WCET`     | 0.916634       | 0.000000       | 0.000248                        |

BF is **~1.5% WORSE** than `INCR_Reopt_10` (0.9086 < 0.9220). This is impossible
if BF enumerates the global SP optimum per interval: the incremental arm is a
heuristic over the same search space, so `INCR ≤ BF` must hold invariantly
(the P0.2 guarantee). BF also shows a **non-zero miss rate (0.005779)** while
`INCR_NO_FALLBACK` / `INCR_WCET` are 0.000, and BF's exec time (0.097 s) is
~55× the INCR arm's.

Run config: N=4, 10 tasksets, duration 600 s, interval 10 s, seed 1000.

## Hypotheses (to rule in/out)

1. **BF time-limit truncation (P1.14 regression):** the `BFDLSharedBudget` timer
   guard cancels the search mid-enumeration → BF adopts a sub-optimal incumbent
   instead of the global optimum. (BF exec time 0.097 s ≪ TIME_LIMIT=10 s makes
   pure timeout unlikely, but a *per-call* budget shared across intervals could
   still truncate a single interval's search.)
2. **BF important-task fallback (P0.10) firing:** the gate
   `AdoptRmFastFallbackIfUnschedulable` substitutes an RM-Fast group-locked plan
   for BF on some intervals/tasksets when the global-optimum plan is judged
   unschedulable → SP drops and miss rate rises. The non-zero BF miss rate is the
   fingerprint. Need to check whether BF fell back and on how many intervals.
3. **BF and INCR optimize different objectives / metric surfaces:** BF picks the
   global-optimum PA+TL for the *analytical* SP, but the reported
   `Mean_SP_Metric` is aggregated differently (e.g. simulation RT vs analytical),
   so a lower-analytical-SP plan can score higher on the reported metric.
4. **BF seed / search-space restriction:** BF does not enumerate the full
   PA×TL space (e.g. TL grid granularity, or PA constrained by DM/important-first
   lock from P0.9) while INCR escapes that restriction → INCR reaches a point BF
   cannot.
5. **Stale/corrupt run artifacts:** a crashed or partial BF run.log produced
   degenerate metrics (cf. P1.15 silent-sim-failure inflation). `taskset_arm_status`
   shows all BF inst `OK`, but a 0-byte / silent-failure path must be ruled out.

## Approach

1. **Localize WHERE BF loses:** per-taskset `interval_sp_metrics.txt` → compute
   per-taskset `Mean_SP_Metric` for BF vs INCR_Reopt_10; identify the taskset(s)
   and intervals driving the gap.
2. **Inspect the BF run.log** for the losing taskset: did the search complete,
   hit the budget guard, or adopt the RM-Fast fallback? Look for
   `AdoptRmFastFallbackIfUnschedulable` / `BFDLSharedBudget` / cancellation
   markers.
3. **Compare the adopted PA+TL** of BF vs INCR on a losing interval to see
   whether BF's plan is strictly dominated or merely different.
4. **Cross-check the invariant at small scale:** re-run the single losing
   taskset through `OptimizeSP_TL_BF` directly (or `check.SP_OPT` BF test) and
   confirm `INCR ≤ BF` on it — does the anomaly reproduce in isolation?
5. **Root-cause + fix** (or, if BF genuinely cannot reach a point INCR reaches,
   re-examine whether the `INCR ≤ BF` invariant still holds given P0.9's
   important-first lock / P0.10's fallback).

## Files

- `comparison_summary.csv` + `taskset_arm_status.csv` (run root, quoted above).
- `taskset_{0..9}/{BF,INCR_Reopt_10}/run.log` + `interval_sp_metrics.txt`.
- `sources/Optimization/OptimizeSP_TL_BF.cpp` — `OptimizePA_with_TimeLimitsStatus::Optimize`,
  `AdoptRmFastFallbackIfUnschedulable` gate (P0.10).
- `sources/Optimization/OptimizeSP_Base.cpp` — `BFDLSharedBudget` timer guard (P1.14).
- `simulation_experiments/.../compare_optimizers.py` — arm definitions / metric
  aggregation.

## Done when

- Per-taskset localization of the BF loss (which tasksets/intervals).
- Identified mechanism: time-limit truncation / fallback firing / objective
  mismatch / search-space restriction / artifact corruption.
- Either a fix (BF regains `INCR ≤ BF`) OR a documented explanation of why the
  invariant no longer holds under current BF semantics (P0.9/P0.10).
- Milestone to top-level `agents/dev_log.md`; cross-link to P0.2.

## Out of scope

- `git commit` — user's standing constraint (`git add` only).
- Re-running the full comparison sweep before the root cause is found.
