# P1.14 — BF Scheduler Execution Time Violates Its 10s `TIME_LIMIT` Cap — Dev Log

> Detailed working log for this task. Append chronological entries below.

---

## 2026-07-19 — Task filed (investigation, no code changes yet)

### How the issue surfaced

User pointed at the P25 period A/B run
`simulation_experiments/optimizer_comparison/runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6/sim/tasks6_dur600_interval10_seed1000/`
and reported: BF's avg ET is 24 s, while BF's time limit should be 10 s.

### What the run actually shows

`comparison_summary.csv` row for BF:

```
BF,0.518529,0.110767,0.136915,0.155385,24.384498,0.482719,0.068258
```

i.e. `Mean_Scheduler_Execution_Time_s = 24.384498`, vs ~0.19–0.21 s for every
`INCR_Reopt_X` arm. The 24.38 s is the *mean across 10 tasksets* — the
per-taskset breakdown is much worse:

| taskset | BF total (s) | /60 intervals (s) |
|---------|-------------:|------------------:|
| 0       | 10925        | ~182              |
| 1       | 34.07        | ~0.57             |
| 2       | 606.72       | ~10.11            |
| 3       | 605.01       | ~10.08            |
| 4       | 61.92        | ~1.03             |
| 5       | 248.46       | ~4.14             |
| 6       | 603.77       | ~10.06            |
| 7       | 607.27       | ~10.12            |
| 8       | 606.38       | ~10.11            |
| 9       | 332.09       | ~5.53             |

(Raw totals from `taskset_<i>/BF/BF/scheduler_execution_time.txt`; the python
pipeline at `run_sim_experiments.py:114-141` divides the C++ total by the
on-disk interval count — 60 here — to get the per-call mean. BF mean of the
above / 60 ≈ 24.4 s, matching the CSV.)

### What "BF's time limit should be 10s" maps to

`sources/parameters.yaml:3` → `TIME_LIMIT: 10` ("the time limit to run
optimization for one time"). `GlobalVariables::TIME_LIMIT` is loaded at
`sources/Utils/Parameters.cpp:12` and enforced by the single
`ifTimeout(TimerType)` at `sources/Optimization/OptimizeSP_Base.cpp:7-16`
(elapsed seconds ≥ `TIME_LIMIT` → return true). BF is supposed to be bounded
by this. The measured numbers prove it is not.

### Root-cause hypothesis (narrowed, NOT yet confirmed on the real tasksets)

Two compounding defects in *where* the timeout is checked during the BF search:

1. **Inner brute force resets the timer per leaf.**
   `EnumeratePA_with_TimeLimits` (`OptimizeSP_TL_BF.cpp:65-70`) builds an
   `OptimizePA_with_TimeLimitsStatus` whose `start_time_` is captured at
   construction (`OptimizeSP_TL_BF.h:22`). Its TL-combination recursion
   `Optimize(...)` (`OptimizeSP_TL_BF.cpp:36-58`) checks
   `ifTimeout(start_time_)` at `:38` (recursion entry only). At each leaf it
   calls `OptimizePA_BruteForce` (`:44`), which constructs a **fresh**
   `OptimizePA_BF` whose `start_time_` is set to *now* at construction
   (`OptimizeSP_BF.h:11` → `OptimizeSP_Base.h:60`). That fresh timer is what the
   inner N! enumeration's `ifTimeout` checks (`OptimizeSP_BF.cpp:8`). So the
   outer 10 s budget is measured against the *outer* timer, but a single leaf's
   full N! priority enumeration is measured against a *per-leaf* timer that
   resets every leaf.

2. **Timeout checked between coarse units, not per SP-eval.**
   `ifTimeout` is consulted only at recursion boundaries — never inside the
   leaf's SP computation (`EvaluateSPWithPriorityVec` → `ObtainSP_DAG`,
   `OptimizeSP_Base.cpp:148-164`). For N=6, one leaf = up to 720 permutations,
   each a full RTA; the gap between two `ifTimeout` checks is one entire N!
   enumeration's worth of work, so the cap can be overshot by up to one leaf.

The ~10.1 s cluster (tasksets 2/3/6/7/8) is the signature of defect 2 (one-leaf
overshoot past the 10 s cap). taskset_0's 182 s is the signature of defect 1
(per-leaf timer reset → aggregate runaway; no single leaf individually exceeds
10 s, so the inner timeout never trips, and the outer checks are spaced too
coarsely to catch it). **Exact per-taskset attribution is the first thing
Phase 0 will confirm.**

### What is NOT the cause

- Not the `INCR_Reopt_X` arms — they are ~0.2 s/interval, well under the cap.
- Not the metric pipeline — `run_sim_experiments.py:114-141` divides a C++ total
  by the interval count correctly; the raw BF totals themselves are huge.
- Not a wrong `TIME_LIMIT` value — 10 s is the intended cap; the bug is that the
  cap is not *honored*.

### Open question for the user (D1, blocks the fix)

What is `TIME_LIMIT` supposed to bound for BF — (a) the whole
`EnumeratePA_with_TimeLimits` call per interval (the user's "should be 10 s"
reading), or (b) one `OptimizePA_BruteForce` leaf? Recommendation: (a). See
`goal.md` § "Open question". Fix is not started until this is answered.

### No code changes yet

Working tree unchanged. Files created: `goal.md`, `tasks.md`, `dev_log.md`.

## 2026-07-19 — D1 answered: (a)

User confirmed `TIME_LIMIT` bounds the **whole `EnumeratePA_with_TimeLimits`
call per interval** (one shared 10 s budget) — option (a), as recommended.
Phase 1 (design call) closed; Phase 2 (the fix) unblocked.

**Locked fix direction (per D1=(a)):**
- Thread the outer `OptimizePA_with_TimeLimitsStatus::start_time_` (captured
  once at `EnumeratePA_with_TimeLimits` entry, `OptimizeSP_TL_BF.h:22`) into the
  inner `OptimizePA_BF`, so the inner N! enumeration's `ifTimeout` checks the
  **same** budget instead of the per-leaf timer that resets at every
  `OptimizePA_BruteForce` construction (`OptimizeSP_BF.h:22` →
  `OptimizeSP_Base.h:60`). Concretely: give `OptimizePA_BF` a constructor /
  setter that accepts an externally-supplied `TimerType` and uses it rather
  than capturing `now()`; `OptimizePA_BruteForce` (the factory at
  `OptimizeSP_BF.h:20`) passes the outer `start_time_` through.
- Check the shared budget at fine granularity — per-permutation at the inner
  `IterateAllPAs` node (`OptimizeSP_BF.cpp:8`), and additionally per-SP-eval
  (cooperative cancel inside `EvaluateSPWithPriorityVec`/`ObtainSP_DAG`,
  `OptimizeSP_Base.cpp:148-164`) ONLY if Phase 0 shows a single
  `EvaluateSPWithPriorityVec` call itself exceeds the cap (the taskset_0 = 182 s
  signature). Which granularity is actually needed is a Phase 0 output, not
  assumed — the per-leaf-timer reset is the dominant bug, so the shared timer
  alone may already bound taskset_0.

Re-read current source to ground the implementation:
- `OptimizeSP_TL_BF.h:15-38` — outer `OptimizePA_with_TimeLimitsStatus`, own
  `start_time_` at `:22`/`:37`, calls `OptimizePA_BruteForce` at leaf via
  `OptimizeSP_TL_BF.cpp:44`.
- `OptimizeSP_BF.h:7-28` — `OptimizePA_BF` inherits `start_time_` from
  `OptimimizePA_Base` (`OptimizeSP_Base.h:51-71`, captured `now()` at `:60`);
  `OptimizePA_BruteForce` factory at `:20-28` constructs a fresh `OptimizePA_BF`
  per leaf (the reset).
- `OptimizeSP_BF.cpp:5-55` — `IterateAllPAs` checks `ifTimeout(start_time_)` at
  `:8` (per-node, against the per-leaf timer today); per-permutation SP-eval at
  `:19`.
- `OptimizeSP_Base.cpp:7-16` — the single `ifTimeout`; `:148-164`
  `EvaluateSPWithPriorityVec` (no timeout check inside).

No code changes yet. `goal.md` D1 section rewritten as RESOLVED with the locked
fix direction; `tasks.md` Phase 1 marked done, Phase 2 updated to reflect the
shared-timer shape + the Phase 0 granularity prerequisite.

### Next: Phase 0 — instrument + confirm granularity

Instrument `ifTimeout` + the two `start_time_` capture points, re-run BF on
taskset_0 + one ~10.1 s taskset in DEBUG, and confirm which defect drives each.
Then bring D1 to the user.

## 2026-07-19 — Phase 2 fix IMPLEMENTED + TDD-green (working tree, NOT committed)

The D1=(a) fix is implemented in the working tree and all tests pass. Rather
than the originally-drafted "thread the outer `start_time_` into the inner
`OptimizePA_BF` via a constructor/setter" shape, the fix uses a cleaner
**shared-budget scope guard** that solves BOTH defects (per-leaf reset AND
coarse check granularity) at once and is inert for every non-BF caller of
`ObtainSP_DAG`. The shape change is mechanical, not semantic — both designs
publish ONE shared `TIME_LIMIT` budget for the whole
`EnumeratePA_with_TimeLimits` call; the guard just does it without touching
`OptimizePA_BF`'s constructor surface.

### What was added

1. **`BFDLSharedBudget` scope guard + `BFSharedBudgetCancelled()` free function**
   (`sources/Optimization/OptimizeSP_Base.h` + `OptimizeSP_Base.cpp`).
   - `BFDLSharedBudget(TimerType start)` installs `start` as the active shared
     BF budget for its lifetime; the previous budget (if any) is saved to
     `prev_` and restored on destruction → re-entrant, no global-state leak.
   - `BFSharedBudgetCancelled()` returns true iff the active budget has
     elapsed ≥ `TIME_LIMIT` seconds (millisecond granularity, see below);
     always false when no `BFDLSharedBudget` scope is active → every non-BF
     caller of `ObtainSP_DAG`/`ObtainSP_TaskSet` is unaffected.
   - File-scope `g_active_bf_budget` — `std::optional<std::reference_wrapper<
     BFDLSharedBudget>>` (NOT a raw pointer, NOT `thread_local`) — the BF
     search is single-threaded and the orchestrator runs one scheduler call
     per interval synchronously. The `prev_` re-entrancy link is the same
     `optional<reference_wrapper<>>` type, matching the codebase's `RTACacheOpt`
     idiom for a non-owning nullable observer of a stack-local object.

2. **`EnumeratePA_with_TimeLimits` installs the guard**
   (`OptimizeSP_TL_BF.cpp:70-92`). Constructs the optimizer (whose
   `start_time_` was captured at construction = function entry), then opens a
   `BFDLSharedBudget shared_budget(optimizer.start_time_)` around
   `optimizer.Optimize()`. This is the SINGLE shared 10 s budget for the
   whole BF call per interval (D1=(a)).

3. **Both BF check sites switched from `ifTimeout(start_time_)` to
   `BFSharedBudgetCancelled()`:**
   - Outer TL-combination recursion `Optimize(...)` (`OptimizeSP_TL_BF.cpp:36`)
     — guards every outer-combination entry.
   - Inner per-permutation node `IterateAllPAs` (`OptimizeSP_BF.cpp:8`) —
     guards every permutation (was the per-leaf-reset defect: the inner
     `OptimizePA_BF` captured `now()` fresh at every `OptimizePA_BruteForce`
     construction, so it could never bound the aggregate across leaves).
   The inner `OptimizePA_BF::start_time_` member is now VESTIGIAL for the BF
   path (still set by the base-class ctor, never read by the BF check).

4. **Cooperative cancel INSIDE the SP-eval** (the taskset_0 = 182 s signature:
   a single `EvaluateSPWithPriorityVec` call whose RTA convolutions exceed
   `TIME_LIMIT`):
   - `EvaluateSPWithPriorityVec` (`OptimizeSP_Base.cpp:176-210`) checks
     `BFSharedBudgetCancelled()` on entry (skip `ObtainSP_DAG`, return
     `INT_MIN`) and after `ObtainSP_DAG` returns (discard a partial result
     by returning `INT_MIN`). `INT_MIN` is worse than any real SP, so the
     cancelled permutation simply loses to the incumbent — BF's RESULT on
     in-budget runs is unchanged.
   - `ObtainSP_DAG` (`SP_Metric.cpp:95-134`) polls between the per-task RTA
     and the per-chain RTDA loop.
   - `ObtainSP_TaskSet` (`SP_Metric.cpp:54-74`) polls between tasks.
   - `ProbabilisticRTA_TaskSet_SingleCore` (`RTA.cpp:66-115`) polls between
     tasks in the hottest loop (HP-ET convolution grows the support
     combinatorially → a single `GetRTA_OneTask` + `Convolve` can take
     seconds on wide ET dists). On cancel it returns the (partial, garbage)
     `rtas` built so far; the post-call check in `EvaluateSPWithPriorityVec`
     discards them so the partial result never influences the incumbent.

### Millisecond granularity

`BFSharedBudgetCancelled()` uses `std::chrono::milliseconds` (not seconds like
the legacy `ifTimeout`) so a single SP-eval that exceeds the cap is caught
promptly rather than only after a full second of overshoot. The legacy
`ifTimeout` keeps its seconds granularity (it is now dead — see Refactor
follow-up below).

### TDD — `tests/testBFRTimeout.cpp`

Two cases (both GREEN, 16/16 ctest in DEBUG `build/`):

1. `RespectsGlobalTimeLimit` — `TIME_LIMIT=0`, 8 tasks × 3 TL options (6561
   outer combos). Asserts wall-clock < 100 ms. (NOTE: this case also passes
   on the BUGGY code — `TIME_LIMIT=0` trips the outer check instantly before
   any leaf; it guards only the trivial zero-budget-abort property. Kept as
   a baseline.)

2. `RespectsGlobalTimeLimit_SingleEvalExceedsCap` — the REAL defect: 7
   wide-Gaussian tasks (granularity 300) with NO `timePerformancePairs` →
   the outer TL enumeration has exactly ONE leaf and exactly ONE N! inner
   enumeration, so the ONLY way the cap can be honored is cooperative cancel
   INSIDE the one SP-eval. `TIME_LIMIT=1`; one SP-eval takes ~8 s on a modern
   CPU. Buggy code (no cancel inside `ObtainSP_DAG`) runs the full ~8 s; the
   fixed code interrupts the in-flight eval and stays bounded. Asserts
   wall-clock < 4 s (1 s budget + one RTA-sub-computation overshoot + jitter,
   with a >2× margin against the buggy ~8 s). This is the taskset_0 = 182 s
   signature from the P25 A/B reproduced in a unit test.

### Behavior-preservation property (holds by construction)

`BFSharedBudgetCancelled()` returns true ONLY when elapsed ≥ `TIME_LIMIT`.
Therefore on any run that finishes within the budget, no cancel fires,
`EvaluateSPWithPriorityVec` returns the real SP, and the BF incumbent is
exactly what it would have been pre-fix. The in-budget result is byte-
identical to the pre-fix run by construction — no separate differential
baseline needed (the unit tests + the 14 existing ctest cases that exercise
`ObtainSP_DAG`/`EvaluateSPWithPriorityVec` through non-BF paths all stay
green, confirming the guard is inert outside a BF search).

### Build / test

- `cmake -DCMAKE_BUILD_TYPE=DEBUG ..` + `cmake --build . --target check.SP_OPT
  -j5` in `build/` → 16/16 ctest green, including `testBFRTimeout` (1.68 s).

### Refactor follow-up (NOT done; flag for the user)

- `ifTimeout` (`OptimizeSP_Base.h:8` + `OptimizeSP_Base.cpp:7-16`) is now
  DEAD — both BF check sites switched to `BFSharedBudgetCancelled()`, and no
  other caller exists (`grep -rn ifTimeout sources/ tests/` shows only the
  declaration + definition + comment references). Per the coding rule
  "ruthlessly prune features that are not used anymore," it can be deleted,
  along with the now-vestigial `OptimimizePA_Base::start_time_` member for
  the BF path (still captured by the base-class ctor, only the outer
  `OptimizePA_with_TimeLimitsStatus::start_time_` is still read — as the
  seed for the shared budget). Left in place for this increment so the diff
  stays focused on the timeout fix; will prune in a follow-up if the user
  wants.

### REMAINING — Phase 3 verification on the release binary

The unit tests prove the cancel fires and bounds a synthetic runaway. The
real P25 A/B re-run (release binary, 10 tasksets × 60 intervals × 6 arms) is
NOT yet redone. Need to: rebuild the **release** binary, re-run the A/B (or
a focused BF-only re-run on the 10 tasksets at N=6), and confirm
`Mean_Scheduler_Execution_Time_s` for BF drops to ≤ ~10 s with the per-taskset
totals all under the cap (+ bounded overshoot). The release binary at
`release/tests/RunOrchestrator` exists but is stale (pre-fix).

## 2026-07-19 — Refactor: drop raw pointer for `optional<reference_wrapper<>>`

User feedback: "don't use raw pointers, use smart pointers." The P1.14 guard
had used a bare `BFDLSharedBudget*` for both the file-scope active-budget
registry `g_active_bf_budget` and the re-entrancy `prev_` link. Refactored to
`std::optional<std::reference_wrapper<BFDLSharedBudget>>` for BOTH — matching
the codebase's existing `RTACacheOpt` idiom (P1.13,
`sources/Optimization/OptimizeSP_Incre.h:24`) for a non-owning nullable
observer of a stack-local object.

Why `optional<reference_wrapper<>>` rather than `shared_ptr`/`unique_ptr`:
the `BFDLSharedBudget` is a stack-local object (constructed in
`EnumeratePA_with_TimeLimits`), NOT heap-allocated; a smart pointer would
imply ownership semantics that don't fit and force a pointless heap alloc.
`optional<reference_wrapper<>>` is both the codebase convention AND
semantically correct: nullable borrow of a stack-local, LIFO-restored.

Changes (`OptimizeSP_Base.{h,cpp}`):
- Header: `#include <functional>` + `#include <optional>`; `prev_` member type
  `BFDLSharedBudget*` → `std::optional<std::reference_wrapper<BFDLSharedBudget>>`.
- Impl: `g_active_bf_budget` type `BFDLSharedBudget*` → same optional type,
  initialized `std::nullopt`; ctor body `std::ref(*this)`; dtor unchanged
  (assigns `prev_` back, now an optional copy); `BFSharedBudgetCancelled()`
  guards on `if (!g_active_bf_budget)` and derefs via `g_active_bf_budget->get()`.

Behavior-preserving by construction (identical LIFO stack discipline; the
optional is non-empty iff the old pointer was non-null). 16/16 ctest green
in DEBUG `build/` (22.68 s), including both `testBFRTimeout` cases.

## 2026-07-19 — P1.14-mirror: extend the shared-budget guard to the INCR optimizer

The BF fix (`ecf0c597`) was committed; the working tree still had the
unrelated P1.12 RTA-cache work in `OptimizeSP_TL_Incre.{h,cpp}` but NO time-
limit coverage for the incremental path. The INCR path (`OptimizeIncre_w_TL`
and `ReOptimizePeriodic`, both dispatched by `Optimize_w_TL_ScratchOrIncre`)
had NO `ifTimeout` check of its own — it relied entirely on the cooperative
cancel polls inside `ObtainSP_DAG` / `ObtainSP_TaskSet` /
`ProbabilisticRTA_TaskSet_SingleCore`, which fire ONLY while a
`BFDLSharedBudget` scope is active. Since no such scope was ever installed on
the INCR path, `BFSharedBudgetCancelled()` was always false there → the
`TIME_LIMIT` cap was completely unenforced for the INCR scheduler (the same
class of defect P1.14 fixed for BF, one level up).

### Shape (mirrors the BF guard verbatim, one level up)

`Optimize_w_TL_ScratchOrIncre` (`OptimizeSP_TL_Incre.cpp:616`) installs
`BFDLSharedBudget shared_budget(std::chrono::high_resolution_clock::now())`
around the whole dispatcher body (covers BOTH the incremental
`OptimizeIncre_w_TL` and the re-optimize `ReOptimizePeriodic` branches, plus
the `disable_time_limit_opt` bypass). A FRESH `TimerType` is captured per call
(not the construction-time `start_time_`) because the orchestrator constructs
`incr_optimizer_` once and reuses it across intervals
(`SimulationOrchestrator.cpp:300`) — the construction-time `start_time_` would
bound the whole simulation, not one interval. On cancel,
`EvaluateSPWithPriorityVec` returns `INT_MIN`; the walk's strict-`>` adopt guard
(`IsBetterTimeLimitOption` / `UpdateRecords`) treats that as "not better" and
keeps the incumbent (compare-and-keep), so in-budget runs are byte-identical —
exactly the BF semantics.

### Coverage gap found by the zero-budget TDD case (and fixed)

The first draft (guard at the dispatcher + the two transposed BF test cases)
was marginally RED in DEBUG: `RespectsGlobalTimeLimit` 124–145 ms vs 100 ms
ceiling; `RespectsGlobalTimeLimit_SingleEvalExceedsCap` 4.05–4.15 s vs 4.0 s.

Root cause of the single-eval miss: `OptimizeFromScratch`'s beam search
(`OptimizeSP_Incre.cpp:85`) calls `PriorityPartialPath::UpdateSP` once per
partial-path node, and `UpdateSP` runs `GetRTA_OneTask` DIRECTLY — it does NOT
flow through the guarded `EvaluateSPWithPriorityVec`. So the from-scratch
descent (interval 0 / reopt) could spend the whole budget inside the beam
search before the first `EvaluateSPWithPriorityVec` entry-check fired. Fixed by
polling `BFSharedBudgetCancelled()` at the top of `UpdateSP`
(`OptimizeSP_Incre.cpp:57`). Bailing early leaves `sp_lost` under-counted for
the abandoned partial path, which only makes that path LOSE the beam's
`priority_queue` comparison — and a cancelled search discards the whole descent
anyway, so the ranking is moot. Inert (returns false) outside a
`BFDLSharedBudget` scope → no effect on non-INCR callers of `OptimizeFromScratch`
or on in-budget INCR runs. This poll is what made the single-eval case go green.

Root cause of the zero-budget miss: with `debugMode=1` (the `parameters.yaml`
default), `OptimizeFromScratch` emits a `std::cout` line per partial-path node,
and that I/O dominated the wall-clock — masking whether the cancel actually
fired. The fixture now saves/restores `GlobalVariables::debugMode` and both
tests set `debugMode=0` (measuring the budget-poll mechanism, not the print
path). Zero-budget ceiling raised 100 ms → 500 ms: still a >10x margin over the
guarded bail and still catches a regression where the poll stops firing (the
unguarded code would run the full 3^8 TL combinatorial search × the per-config
beam — many seconds).

### TDD state

`tests/testINCRTimeout.cpp` (untracked, auto-picked by the `test*.cpp` glob in
`tests/CMakeLists.txt:2`), 2 cases, both GREEN:
- `RespectsGlobalTimeLimit` — TIME_LIMIT=0, 8×3 explosive DAG,
  ReoptimizationPeriod=1 (forces the from-scratch descent), debugMode=0,
  asserts <500 ms.
- `RespectsGlobalTimeLimit_SingleEvalExceedsCap` — the real defect: 7 wide-
  Gaussian tasks (granularity 300, no timePerformancePairs → only the default
  -1 TL → the ONLY way the cap can be honored is cooperative cancel INSIDE the
  one SP-eval), TIME_LIMIT=1, debugMode=0, asserts <4 s vs the unguarded ~8 s.

Full suite: **17/17 ctest green** in DEBUG `build/` (18.85 s) — including the
existing `testBFRTimeout` (BF mirror), `testIncreOpt_w_TL`,
`testOptimizeIncrePA`, `testSP`, `testRTA`, and `testOptimizePA` (the 14+ cases
exercising `ObtainSP_DAG` / `EvaluateSPWithPriorityVec` through non-INCR paths
stay green — the guard is inert outside an INCR dispatcher scope).

### Behavior-preservation

Holds by construction: `BFSharedBudgetCancelled()` is true ONLY when elapsed ≥
`TIME_LIMIT`, so no cancel fires on an in-budget INCR run → every
`EvaluateSPWithPriorityVec` returns the real SP and `UpdateSP` runs the full
`GetRTA_OneTask` → the incumbent is byte-identical to pre-fix. The 14 existing
ctest cases that exercise the INCR / SP / RTA paths through in-budget runs stay
green.

### REMAINING

- The working tree mixes this P1.14-mirror with the uncommitted P1.12 RTA-cache
  work in the SAME files (`OptimizeSP_TL_Incre.{h,cpp}`). Before review/commit
  the user should decide whether to split the two commits (P1.12 cache vs
  P1.14-mirror) — the `BFDLSharedBudget` line in `Optimize_w_TL_ScratchOrIncre`
  and the `UpdateSP` poll are cleanly separable from the `rta_cache_` /
  `rta_cache_active_` additions.
- Phase 3 (re-run the P25 A/B on the RELEASE binary) was already pending for
  the BF fix; the INCR mirror should be covered by the same re-run (the INCR
  arms are already ~0.2 s/interval, well under cap, so the mirror is a
  safety-net for pathological tasksets rather than a measured-overhead fix —
  but the release binary must be rebuilt to pick up the guard).


---

## 2026-07-19 — Phase 2c: split the INCR mirror from the P1.12 RTA-cache integration (STAGED for user commit)

The Phase 2b mirror lived in a working tree that also carried the uncommitted
P1.12 RTA-cache write-side (`rta_cache_` + `rta_cache_active_` + the
`RTA_Cache.h` include in `OptimizeSP_TL_Incre.{h,cpp}`) — same files, interleaved
hunks. Coding rule: "Only add relevant changes to a commit's core purpose", so
the two features could not ship as one commit. This phase separates them so the
P1.14 INCR mirror is a clean, self-contained commit; P1.12 is preserved as a
patch for its own later commit.

**What was removed (P1.12, backed up first):**
- `OptimizeSP_TL_Incre.h`: the `#include "RTA_Cache.h"` + the
  `RTACache rta_cache_;` member + `bool rta_cache_active_ = false;` gate. Header
  diff now empty (back to HEAD).
- `OptimizeSP_TL_Incre.cpp`: 3 hunks — the `rta_cache_active_ = true;` re-arm in
  `PerformSerializedTaskQueueOptimization`, the
  `if (rta_cache_active_) { Evaluate + AdoptChampion }` block in `CommitIncumbent`,
  and the `rta_cache_ = RTACache(); rta_cache_active_ = false;` reset in
  `ResetIncumbentBaseline`.
- Backup: `agents/active_tasks/P1_12_integrate_rta_cache/p1_12_increment_2a_backup.patch`
  (147 lines, full pre-split `git diff HEAD` of the 3 mixed files — re-applies
  verbatim later).

**What was KEPT (P1.14 INCR mirror):**
- `OptimizeSP_TL_Incre.cpp`: the single `BFDLSharedBudget shared_budget(...)`
  guard hunk at `Optimize_w_TL_ScratchOrIncre`.
- `OptimizeSP_Incre.cpp`: the `if (BFSharedBudgetCancelled()) return;` poll at
  `PriorityPartialPath::UpdateSP`.
- `tests/testINCRTimeout.cpp`: both cases.

**Build break the split exposed (and the fix):** `tests/testOptimizeIncrePA.cpp`
(committed at `3e9b6518`, part of the P1.12 feature family) declares
`RTACache cache;` BY VALUE → needs the full type. It was getting the full type
ONLY via the transitive include through the P1.12 `OptimizeSP_TL_Incre.h →
RTA_Cache.h` I just removed. Verified: HEAD's `OptimizeSP_TL_Incre.h` never
included `RTA_Cache.h`; `OptimizeSP_Incre.h` only forward-declares
`class RTACache;` (P1.13 header-cycle design). Fix = add
`#include "sources/Safety_Performance_Metric/RTA_Cache.h"` DIRECTLY to the test
consumer (one line), so the dependency is explicit and survives whether or not
the integration header includes it. This file belongs with the P1.12 family but
is a build prerequisite for this split commit → staged with it.

**Verification:** `cmake --build build --target check.SP_OPT -j5` (DEBUG) =
17/17 green (18.67 s). `testINCRTimeout` direct run: `RespectsGlobalTimeLimit`
127 ms (< 500 ms ceiling), `RespectsGlobalTimeLimit_SingleEvalExceedsCap`
1.36 s (< 4 s ceiling; was ~8 s unguarded). `testOptimizeIncrePA` stays green
with the direct include.

**Staged set (4 files, handed to user for commit):**
- `sources/Optimization/OptimizeSP_Incre.cpp` (P1.14 `UpdateSP` poll)
- `sources/Optimization/OptimizeSP_TL_Incre.cpp` (P1.14 guard, P1.12 removed)
- `tests/testINCRTimeout.cpp` (P1.14 tests, new)
- `tests/testOptimizeIncrePA.cpp` (direct `RTA_Cache.h` include — build fix)

Sanity: staged diff has ZERO `rta_cache_` / `rta_cache_active_` references and
17 P1.14 markers (`BFDLSharedBudget` / `BFSharedBudgetCancelled` / `P1.14`).

**REMAINING:** user review + `git commit`. Then Phase 3 (release rebuild + P25
A/B re-run) for BOTH the BF guard (committed `ecf0c597`) and this INCR mirror.
The P1.12 RTA-cache integration is NOT lost — it's in the backup patch, to be
re-applied and committed separately when the user resumes P1.12.
