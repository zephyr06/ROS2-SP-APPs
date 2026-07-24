# P1.23 — A/B: Does the RTA cache speed up the optimization?

## The Goal
Empirically determine whether the `RTACache` (introduced P1.9/P1.12, engaged in
the optimization path from `3e9b6518`) actually **speeds up** the per-interval
scheduler optimization, or whether it is perf-neutral / a net cost. Settled by a
controlled A/B run on the fixed scheduler-only timer (`cc9aa0ce`): a no-cache
baseline binary vs the HEAD cache binary, identical tasksets, identical
scheduler mode, apples-to-apples `Mean_Scheduler_Execution_Time_s`.

This is the **cache speedup** A/B — distinct from:
- **P1.21/P1.22** (the *Transaction* CoW overhead vs the eager full-cache copy —
  both arms *have* the cache; that is a cache-internal question).
- The prior `_perf_old_ecbed896` A/B (old `ecbed896` cache-vs-transaction; also
  cache-internal, and run under the *old mis-scoped* timer so its numbers are
  incomparable now).

## A/B boundary (committed)

| arm | commit | cache in opt path? | scheduler-only timer? |
|-----|--------|--------------------|-----------------------|
| **OLD (no cache)** | `d67aaa65` ("update ObtainSP_DAG_From_Dists") | NO — zero `RTACache` usage | needs `cc9aa0ce` cherry-picked ON |
| **NEW (cache)** | `7c47b93b` (HEAD, "fix log axis issue") | YES — `RTACache rta_cache_` engaged | YES (already on HEAD) |

- `3e9b6518` ("add cache to optimimizeSP") is the **first** commit to engage the
  cache in `OptimizeSP_Incre`; its direct parent `d67aaa65` has **zero**
  `RTACache`/`rta_cache` references in the optimization path. So a single commit
  (`3e9b6518`) is the cache-engagement boundary — the cleanest possible A/B.
- There is **no runtime/compile flag** to disable the cache at HEAD (it is an
  unconditional `RTACache rta_cache_` member gated by `rta_cache_active_`, set
  internally by the optimizer, never from config). So the only way to get a true
  no-cache arm is to traceback to `d67aaa65`.
- `d67aaa65` already has the full `INCR_Reopt_X` dispatch plumbing
  (`MaybeOverrideReoptPeriod`, `IsINCRPeriodVariant`,
  `Optimize_w_TL_ScratchOrIncre`) — the eval config's `INCR_Reopt_10` arm runs
  unchanged on it.
- `cc9aa0ce` (scheduler-only timer) anchors all exist verbatim on `d67aaa65`
  (`DeterminePrioritiesAndBudgets` body, `BaseSimulationOrchestrator`,
  `RunOrchestrator.cpp` timing blocks), so it cherry-picks cleanly → both arms
  measure scheduler-only `DeterminePrioritiesAndBudgets` time (no RTDA+I/O
  dilution).

## What "speedup" means here
- **Primary signal:** `Mean_Scheduler_Execution_Time_s` (per-interval scheduler
  time) in `comparison_summary.csv`, OLD vs NEW at matched N. Cache wins iff
  NEW < OLD by a non-noise margin.
- **Correctness gate:** SP must be **bit-identical** OLD vs NEW at matched N
  (the cache is a memoization, not an algorithm change — same SP). If SP
  diverges, that is a correctness bug to surface, not a perf result.
- **N points:** N=10 first (matches the established perf-probe sizing; ~enough
  to see a real scheduler delta now that the timer is scheduler-only). Extend
  to N=6/16 only if N=10 is ambiguous (the cache signal should *grow* with N,
  since the cache amortizes more RTA work at larger task counts).

## Why the timer fix is a precondition (not optional)
Under the **old** mis-scoped timer, `scheduler_execution_time.txt` bracketed all
of `RunSimulation()` (taskset I/O + optimizer + ~10k-tick×60-interval RTDA
rollout + SP-metric + export), of which the optimizer was a small fraction. A
cache delta in the optimizer got washed out by RTDA+I/O noise — exactly the trap
that made the prior transaction A/B read +5.02%@N=10 / +0.32%@N=16 (an
inversion = noise). With `cc9aa0ce` on **both** arms, the numerator is the
scheduler only, so a real cache effect is visible. **Both arms MUST carry
`cc9aa0ce`.**

## Open follow-up (flag, the user's call)
The Transaction A/B (P1.21/P1.22) is a *different* question (Transaction vs
eager full-copy, both with cache). It was run under the old timer and is
incomparable now. Whether to re-run *that* A/B on the fixed timer is separate
from this task and is the user's decision — NOT assumed here.
