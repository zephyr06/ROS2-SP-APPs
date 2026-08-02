# P2.20 — Iterate `ComputeSafeFallback` to convergence

## Origin
Filed from P2.19 (2026-08-01). P2.19's user-supplied 3-point design split into:
(1) worst-case DAG perf → min TL option + (3) rename → **P2.19** (crash fix);
(2) iterate-to-convergence → **this task** (P2.20).

## Why separate
Point (1) alone stops the `INCR_Reopt_10` SIGABRT (the crash). Point (2) is a
behavior-change ENHANCEMENT aimed at improving the fallback's SP — reducing the
SP lost when the online optimizer falls back to `safe_fallback_`. It is NOT
required for correctness, so it gets its own TDD + regression and must NOT be
coupled to the crash fix.

## Goal
When `ComputeSafeFallback` (`OptimizeSP_TL_Incre.cpp:955-1064`) builds the
offline safe fallback on the cross-interval worst-case DAG, keep running the
incremental optimizer until convergence: a full pass over all tasks cannot
improve the best SP found. Today `OptimizeIncre_w_TL` (`:839-858`) runs
**single-pass** (`PerformSerializedTaskQueueOptimization` once).

## Design points (to nail down at P2.20 start)
- `OptimizeIncre_w_TL` is single-pass → needs a convergence WRAPPER (loop the
  serialized pass), not a rewrite of the walk.
- Convergence predicate: reuse `ApproxEqualSP(best, prev_best)` (existing SP
  tolerance) — confirm the helper exists / signature.
- **Guaranteed termination:** max-iteration cap alongside the SP predicate (an
  SP that oscillates by epsilon must not loop forever).
- The `BFDLSharedBudget` installed in `ComputeSafeFallback` (`:974`) must still
  bound the looped runtime — confirm the budget survives multiple passes.
- The in-walk gate (P0.7; `enable_fallback_use_ = true` here at `:986`) must
  keep rejecting gate-infeasible candidates EACH pass (loud-fail re-gate at
  `:1052-1060` stays as the final backstop).
- Seed: still `SeedTimeLimitsAtOrBelowEtMean` + DM PA (`:993-999`); after P2.19
  lands the seed is the min-TL option (least interference) — a sound starting
  point for convergence.

## Prerequisite
P2.19 (points 1 + 3) LANDED + green regression (so the crash is gone and the
worst-case DAG uses min-TL for perf tasks before adding the loop).

## Out of scope
- Any change to the loud-fail throw itself (P1.15 safety net, P2.19 keeps it).
- Online (during-run) fallback trigger logic (P0.7's three triggers — unchanged).

## Files (preliminary)
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:955-1064` — `ComputeSafeFallback`.
- `sources/Optimization/OptimizeSP_TL_Incre.cpp:839-858` — `OptimizeIncre_w_TL`.
- `sources/Optimization/OptimizeSP_TL_Incre.h` — decls if a new wrapper is added.
- `tests/testIncreOpt_w_TL.cpp` — new convergence TDD test(s).
