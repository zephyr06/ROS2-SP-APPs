# P1.14 — BF Scheduler Execution Time Violates Its 10s `TIME_LIMIT` Cap

> Filed 2026-07-19. **Phase 2 BF fix + Phase 2b INCR mirror IMPLEMENTED +
> TDD-green + COMMITTED at HEAD `bfbec7e5` (branch `clean_simulation`)
> 2026-07-19.** Phase 3 (re-run the A/B on the release binary to confirm BF
> mean drops to ≤ ~10 s) is **BLOCKED on P1.15**: the same A/B's `INCR_Reopt_X>1`
> arms abort with SIGABRT (exit 134) on the hard tasksets — the harness
> currently swallows the crash and aggregates over unequal taskset sets, so
> the A/B cannot be trusted until P1.15 lands (harness loud-failure fix +
> optimizer crash fix). D1=(a) RESOLVED: `TIME_LIMIT` bounds the whole
> `EnumeratePA_with_TimeLimits` call per interval via a single shared budget.

---

## The symptom (grounded in the run output)

In the P25 period A/B run
`simulation_experiments/optimizer_comparison/runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6/sim/tasks6_dur600_interval10_seed1000/`:

- `comparison_summary.csv` reports BF `Mean_Scheduler_Execution_Time_s = 24.384498`
  while every `INCR_Reopt_X` arm is ~0.19–0.21 s (a ~120× gap).
- BF's raw per-taskset `taskset_<i>/BF/BF/scheduler_execution_time.txt` totals
  (whole-process wall clock, divided by 60 intervals to get the per-call mean):

  | taskset | BF total (s) | per-interval (s) |
  |---------|-------------:|-----------------:|
  | 0       | 10925        | ~182             |
  | 1       | 34.07        | ~0.57            |
  | 2       | 606.72       | ~10.11           |
  | 3       | 605.01       | ~10.08           |
  | 4       | 61.92        | ~1.03            |
  | 5       | 248.46       | ~4.14            |
  | 6       | 603.77       | ~10.06           |
  | 7       | 607.27       | ~10.12           |
  | 8       | 606.38       | ~10.11           |
  | 9       | 332.09       | ~5.53            |

- The `INCR_Reopt_X` arms on the same tasksets sit at ~8–17 s *total* (i.e.
  ~0.13–0.28 s per interval) — the expected order of magnitude.

## Why this is a bug

`sources/parameters.yaml` sets `TIME_LIMIT: 10` ("the time limit to run
optimization for one time", in seconds). `ifTimeout` enforces it
(`sources/Optimization/OptimizeSP_Base.cpp:7-16`) by comparing the elapsed
wall-clock against `GlobalVariables::TIME_LIMIT`. The BF scheduler is supposed
to be bounded by this cap — yet 6 of 10 tasksets land at ~10.1 s/interval
(saturating the cap and then spilling one whole leaf past it) and taskset_0
blows up to ~182 s/interval (≈18× the cap). The user's framing — *"BF's time
limit should be 10s"* — maps directly to `TIME_LIMIT=10`; the measured numbers
prove the cap is **not actually bounding** BF's wall-clock.

## Root-cause hypothesis (to confirm, not yet patched)

Two compounding defects in how the `TIME_LIMIT` is checked during the BF
search:

1. **The inner brute force resets the timer per leaf.**
   `EnumeratePA_with_TimeLimits` (`sources/Optimization/OptimizeSP_TL_BF.cpp:65-70`)
   constructs `OptimizePA_with_TimeLimitsStatus` whose `start_time_` is captured
   at construction (`OptimizeSP_TL_BF.h:22`). Its TL-combination recursion
   `Optimize(...)` (`OptimizeSP_TL_BF.cpp:36-58`) checks
   `ifTimeout(start_time_)` only at **recursion entry** — and at each leaf it
   calls `OptimizePA_BruteForce` (`OptimizeSP_TL_BF.cpp:44`), which constructs a
   **fresh** `OptimizePA_BF` whose `start_time_` is set to *now* at construction
   (`OptimizeSP_BF.h:11` → `OptimizeSP_Base.h:60`). That fresh timer is what the
   inner N! enumeration's `ifTimeout` checks (`OptimizeSP_BF.cpp:8`). So the
   outer TL-walk's 10 s budget is measured against the *outer* timer, but a
   single leaf's full N! priority enumeration is measured against a *per-leaf*
   timer that **resets every leaf** — a single leaf that does not itself exceed
   10 s never trips the inner timeout, while the *aggregate* across leaves is
   only loosely bounded by the outer checks spaced one full N! enumeration
   apart.

2. **The timeout is checked between coarse-grained units, not per SP-eval.**
   `ifTimeout` is consulted only at the boundary of an outer TL combination or
   an inner permutation recursion — **never** inside the leaf's SP computation
   itself (`EvaluateSPWithPriorityVec` → `ObtainSP_DAG`,
   `OptimizeSP_Base.cpp:148-164`). For N=6, one leaf = up to 720 permutations,
   each one full RTA; the gap between two `ifTimeout` checks is therefore one
   entire N! enumeration's worth of work. The cap can be overshot by up to one
   leaf's runtime.

The ~10.1 s cluster (tasksets 2/3/6/7/8) is the signature of defect 2 — the
search overshoots the 10 s cap by exactly one leaf's worth of work before the
next check fires. taskset_0's 182 s is the signature of defect 1 — the per-leaf
timer reset lets the aggregate run away because no single leaf individually
exceeds 10 s, so the inner timeout never trips while the outer timer's checks
are spaced too coarsely to catch it (or the outer check is bypassed by the
recursion structure). **Exact attribution per taskset is the first thing to
confirm.**

## Scope (what P1.14 owns)

1. **Investigate** (no code yet): instrument `ifTimeout` (a one-line
   `std::cout` of elapsed time at each check) + the
   `OptimizePA_with_TimeLimitsStatus`/`OptimizePA_BF` constructors (log
   `start_time_` capture), run BF on the 10 tasksets at N=6, and confirm which
   defect drives each taskset's overshoot. Decide whether the cap is meant to
   bound the **whole `EnumeratePA_with_TimeLimits` search** (the user's "BF's
   time limit should be 10s" reading) or only **one `OptimizePA_BruteForce`
   leaf**.
2. **Fix** (after the design call): make a single `TIME_LIMIT` budget bound the
   intended scope, checked at a granularity fine enough that the overshoot is
   bounded (per-leaf or per-permutation, not per-N!-enumeration). The fix must
   not change BF's *result* on runs that finish within the budget — only cap
   the runaway ones.
3. **Verify**: re-run the P25 A/B; BF `Mean_Scheduler_Execution_Time_s` must
   drop to ≤ ~10 s (the cap, plus at most one SP-eval of overshoot) and the SP
   results for the runs that already finished within budget must be
   byte-identical.

## Out of scope / non-goals

- No change to the `INCR_Reopt_X` arms (they are already ~0.2 s, well under the
  cap; the gap is BF-specific).
- No change to `TIME_LIMIT`'s value (10 s) — the bug is that the cap is not
  *honored*, not that 10 s is the wrong number. If the user wants a different
  cap for BF specifically, that is a separate config decision, not this fix.
- No change to the brute-force *algorithm* (still exhaustive enumeration); only
  to how/where the timeout is checked.

## Design call — RESOLVED 2026-07-19 (user: (a))

**D1 — RESOLVED 2026-07-19: (a).** `TIME_LIMIT` bounds ONE invocation of
`EnumeratePA_with_TimeLimits` — i.e. the **whole BF scheduler call per
interval** gets a single shared 10 s budget (the user's "BF's time limit should
be 10s" reading). This is the option (a) that was recommended.

**Fix direction locked by D1=(a):**
- Thread the **outer** `OptimizePA_with_TimeLimitsStatus::start_time_`
  (captured once at `EnumeratePA_with_TimeLimits` entry, `OptimizeSP_TL_BF.h:22`)
  into the inner `OptimizePA_BF` so its `ifTimeout` checks against the **same**
  budget instead of the per-leaf timer that currently resets at every
  `OptimizePA_BruteForce` construction (`OptimizeSP_BF.h:22` → `OptimizeSP_Base.h:60`).
- Check the shared budget at fine enough granularity that the overshoot is
  bounded — at minimum per-permutation (the inner `IterateAllPAs` node check at
  `OptimizeSP_BF.cpp:8`, now against the shared timer), and **if Phase 0 shows a
  single `EvaluateSPWithPriorityVec` call itself exceeds the cap** (the
  taskset_0 = 182 s signature), additionally Cooperatively cancel inside
  `EvaluateSPWithPriorityVec` / `ObtainSP_DAG` (`OptimizeSP_Base.cpp:148-164`).
  Which granularity is actually needed is a Phase 0 output, not assumed.
- The fix must NOT change BF's result on runs that finish within budget — only
  cap the runaway ones.

## Grounded code locations (verified in source)

- `sources/parameters.yaml:3` — `TIME_LIMIT: 10`.
- `sources/Utils/Parameters.cpp:12` — `int TIME_LIMIT = loaded_doc["TIME_LIMIT"].as<int>();`.
- `sources/Optimization/OptimizeSP_Base.h:8` + `OptimizeSP_Base.cpp:7-16` —
  `ifTimeout(TimerType)`, the single enforcement point.
- `sources/Optimization/OptimizeSP_TL_BF.h:22` — outer `start_time_` captured
  at `OptimizePA_with_TimeLimitsStatus` construction.
- `sources/Optimization/OptimizeSP_TL_BF.cpp:36-58` — outer TL-combination
  recursion; `ifTimeout` at `:38` (recursion entry only); leaf calls
  `OptimizePA_BruteForce` at `:44`.
- `sources/Optimization/OptimizeSP_BF.h:11` → `OptimizeSP_Base.h:60` — inner
  `OptimizePA_BF` `start_time_` captured fresh at construction (the reset).
- `sources/Optimization/OptimizeSP_BF.cpp:8` — inner `ifTimeout` (against the
  per-leaf timer); `:19` the per-permutation `EvaluateSPWithPriorityVec` call.
- `sources/Optimization/OptimizeSP_Base.cpp:148-164` — `EvaluateSPWithPriorityVec`
  → `ObtainSP_DAG`, the per-leaf SP computation (no timeout check inside).
- `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp:322-323` — BF
  dispatch (`res = EnumeratePA_with_TimeLimits(dag_tasks, sp_parameters);`).
- `simulation_experiments/run_sim_experiments.py:114-141` —
  `scheduler_execution_time.txt` (C++ total) ÷ interval count = per-call mean.
