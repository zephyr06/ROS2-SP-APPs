# P1.14 — Tasks (working checklist)

> See `goal.md` for scope, the grounded numbers, and the open design question.
> Investigation-first: confirm the root cause on the real tasksets, get the D1
> design call from the user, then fix. One small sub-task at a time, review +
> commit after each.

---

## Phase 0 — Reproduce + pin the per-taskset attribution

- [x] **Baseline confirmed from the existing A/B run** (no re-run needed for
      attribution). `comparison_summary.csv` BF row =
      `Mean_Scheduler_Execution_Time_s = 24.384498`; raw per-taskset totals
      (`taskset_<i>/BF/BF/scheduler_execution_time.txt` ÷ 60): taskset_0 =
      10925 s → ~182 s/interval (defect-1 signature: aggregate runaway), 2/3/6/7/8
      ≈ 606 s → ~10.1 s/interval (defect-2 signature: one-leaf overshoot). Both
      signatures are reproduced synthetically in the Phase-2 TDD cases below, so
      no separate instrumented re-run was needed.
- [~] **Instrumentation** — skipped. The D1=(a) fix publishes ONE shared budget
      polled at per-permutation + per-RTA-sub-computation granularity, which
      bounds BOTH signatures regardless of which defect dominated each taskset;
      per-taskset attribution became moot once D1=(a) locked the fix shape.
- [~] **Per-taskset attribution** — moot (see above); both defects addressed by
      the shared-budget + cooperative-cancel fix.

## Phase 1 — Design call (RESOLVED)

- [x] **D1 — RESOLVED 2026-07-19: (a)** — `TIME_LIMIT` bounds the **whole
      `EnumeratePA_with_TimeLimits` call per interval** (one shared 10 s budget).
      User confirmed. Fix direction: thread the outer
      `OptimizePA_with_TimeLimitsStatus::start_time_` into the inner
      `OptimizePA_BF`; check the shared budget at fine granularity (per-
      permutation at minimum; per-SP-eval only if Phase 0 shows one
      `EvaluateSPWithPriorityVec` call itself exceeds the cap).

## Phase 2 — Fix (D1=(a) confirmed)

- [x] **Granularity needed** — both. Phase 0's two signatures map to two
      granularities, both implemented: per-permutation (outer + inner BF nodes,
      defect-1 / aggregate runaway) AND per-SP-eval (cooperative cancel inside
      `ObtainSP_DAG` / `ObtainSP_TaskSet` / `ProbabilisticRTA_TaskSet_SingleCore`,
      defect-2 / single-runaway-eval). The synthetic single-leaf test proves the
      per-SP-eval cancel is necessary (a one-leaf search cannot be bounded
      otherwise).
- [x] **TDD: red→green tests** — `tests/testBFRTimeout.cpp`, 2 cases:
      `RespectsGlobalTimeLimit` (zero-budget abort) +
      `RespectsGlobalTimeLimit_SingleEvalExceedsCap` (the real defect: one
      ~8 s SP-eval with `TIME_LIMIT=1`, asserts wall-clock < 4 s; would run ~8 s
      on the buggy code). Both GREEN; 16/16 ctest in DEBUG `build/`.
- [x] **Implement the fix per D1=(a)** — `BFDLSharedBudget` scope guard +
      `BFSharedBudgetCancelled()` in `OptimizeSP_Base.{h,cpp}`;
      `EnumeratePA_with_TimeLimits` installs the guard around `optimizer.Optimize()`;
      both BF check sites (`OptimizeSP_TL_BF.cpp:36` outer, `OptimizeSP_BF.cpp:8`
      inner) switched to `BFSharedBudgetCancelled()`; cooperative cancel wired
      into `EvaluateSPWithPriorityVec` (returns `INT_MIN` on cancel),
      `ObtainSP_DAG`, `ObtainSP_TaskSet`, `ProbabilisticRTA_TaskSet_SingleCore`.
      Shape note: used a shared-budget GUARD rather than the originally drafted
      constructor/setter on `OptimizePA_BF` — same single-shared-budget semantics,
      cleaner (no `OptimizePA_BF` ctor surface change), and inert for non-BF
      callers of `ObtainSP_DAG`.
- [x] **Behavior-preservation** — holds by construction:
      `BFSharedBudgetCancelled()` is true ONLY when elapsed ≥ `TIME_LIMIT`, so
      no cancel fires on an in-budget run → `EvaluateSPWithPriorityVec` returns
      the real SP → incumbent is byte-identical to pre-fix. The 14 existing
      ctest cases that exercise `ObtainSP_DAG`/`EvaluateSPWithPriorityVec`
      through non-BF paths stay green (guard inert outside a BF search).

## Phase 2b — Mirror the shared-budget guard to the INCR optimizer

> The BF fix (Phase 2, commit `ecf0c597`) bound the BF scheduler. The INCR
> scheduler (`Optimize_w_TL_ScratchOrIncre` → `OptimizeIncre_w_TL` /
> `ReOptimizePeriodic`) had NO `ifTimeout` of its own and relied entirely on the
> cooperative cancel polls inside `ObtainSP_DAG` / `ObtainSP_TaskSet` /
> `ProbabilisticRTA_TaskSet_SingleCore` — which fire ONLY while a
> `BFDLSharedBudget` scope is active. No such scope was installed on the INCR
> path → `BFSharedBudgetCancelled()` was always false there → `TIME_LIMIT` was
> completely unenforced for INCR. Same defect class as BF, one level up.

- [x] **TDD: red→green tests** — `tests/testINCRTimeout.cpp`, 2 cases
      transposed from `testBFRTimeout.cpp`: `RespectsGlobalTimeLimit`
      (TIME_LIMIT=0, 8×3 explosive DAG, forces the from-scratch descent,
      debugMode=0, asserts <500 ms) +
      `RespectsGlobalTimeLimit_SingleEvalExceedsCap` (7 wide-Gaussian tasks,
      granularity 300, no timePerformancePairs → only the default -1 TL → the
      ONLY way the cap can be honored is cooperative cancel INSIDE the one
      SP-eval; TIME_LIMIT=1, debugMode=0, asserts <4 s vs unguarded ~8 s). Both
      GREEN.
- [x] **Install the guard at the INCR dispatcher** —
      `Optimize_w_TL_ScratchOrIncre` (`OptimizeSP_TL_Incre.cpp:616`) opens
      `BFDLSharedBudget shared_budget(std::chrono::high_resolution_clock::now())`
      around the whole dispatcher body (covers both INCR + reopt branches +
      the `disable_time_limit_opt` bypass). FRESH `TimerType` per call (not the
      construction-time `start_time_`, which would bound the whole simulation —
      the orchestrator reuses `incr_optimizer_` across intervals). On cancel,
      `EvaluateSPWithPriorityVec` returns `INT_MIN` → the walk's strict-`>`
      adopt guard keeps the incumbent (compare-and-keep) → in-budget runs
      byte-identical.
- [x] **Poll the beam-search RTA path** — `PriorityPartialPath::UpdateSP`
      (`OptimizeSP_Incre.cpp:57`) polls `BFSharedBudgetCancelled()` at entry.
      The beam search runs `GetRTA_OneTask` directly (NOT through the guarded
      `EvaluateSPWithPriorityVec`), so without this poll a from-scratch descent
      could spend the whole budget inside the beam before the first entry-check
      fired. Bailing leaves `sp_lost` under-counted → the abandoned partial path
      loses the `priority_queue` comparison (moot — a cancelled search discards
      the whole descent). Inert outside a `BFDLSharedBudget` scope. This poll is
      what made the single-eval TDD case go green.
- [x] **Silence debugMode in the timeout tests** — fixture saves/restores
      `GlobalVariables::debugMode`; both tests set `debugMode=0`. With the yaml
      default `debugMode=1`, `OptimizeFromScratch` emits a `std::cout` line per
      partial-path node and that I/O dominates the wall-clock, masking whether
      the cancel fired. The mechanism under test is the budget poll, not the
      print path.
- [x] **Behavior-preservation** — holds by construction:
      `BFSharedBudgetCancelled()` is true ONLY when elapsed ≥ `TIME_LIMIT`, so
      no cancel fires in-budget → `EvaluateSPWithPriorityVec` returns the real
      SP and `UpdateSP` runs the full `GetRTA_OneTask` → incumbent byte-identical
      to pre-fix. 17/17 ctest green in DEBUG `build/` (18.85 s), including the
      14+ existing cases exercising `ObtainSP_DAG` / `EvaluateSPWithPriorityVec`
      through non-INCR paths.

## Phase 2c — Split the INCR mirror from the P1.12 RTA-cache integration

> The Phase 2b mirror was implemented in a working tree that ALSO carried the
> uncommitted P1.12 RTA-cache integration (write-side: `rta_cache_` +
> `rta_cache_active_` + `RTA_Cache.h` include in `OptimizeSP_TL_Incre.{h,cpp}`).
> Same files, interleaved hunks → could not be committed as one clean module
> (coding rule: "Only add relevant changes to a commit's core purpose"). This
> phase surgically separates them so the P1.14 INCR mirror lands as a clean,
> self-contained commit; the P1.12 integration is preserved as a patch for its
> own later commit.

- [x] **Back up the P1.12 integration** —
      `agents/active_tasks/P1_12_integrate_rta_cache/p1_12_increment_2a_backup.patch`
      (147 lines; the full `git diff HEAD` of the 3 mixed files pre-split, so
      P1.12 can be re-applied verbatim later). Non-destructive.
- [x] **Remove the P1.12 RTA-cache hunks from `OptimizeSP_TL_Incre.h`** —
      reverted the single hunk: the `#include "RTA_Cache.h"` + the
      `RTACache rta_cache_;` member + the `bool rta_cache_active_ = false;` gate
      + their comments. Header diff now empty (back to HEAD).
- [x] **Remove the 3 P1.12 RTA-cache hunks from `OptimizeSP_TL_Incre.cpp`,
      KEEPING the 1 P1.14 guard hunk** — reverted: the
      `rta_cache_active_ = true;` re-arm in `PerformSerializedTaskQueueOptimization`,
      the `if (rta_cache_active_) { Evaluate + AdoptChampion }` block in
      `CommitIncumbent`, and the `rta_cache_ = RTACache(); rta_cache_active_ = false;`
      reset in `ResetIncumbentBaseline`. KEPT: the
      `BFDLSharedBudget shared_budget(...)` guard at `Optimize_w_TL_ScratchOrIncre`.
      Verified `grep rta_cache/RTACache` in both TL_Incre files = none.
- [x] **Fix the build break the split exposed** — `tests/testOptimizeIncrePA.cpp`
      (committed at `3e9b6518`, part of the P1.12 feature family) declares
      `RTACache cache;` BY VALUE, which needs the full type. It was getting the
      full type ONLY via the transitive include through the P1.12
      `OptimizeSP_TL_Incre.h → RTA_Cache.h` I just removed (HEAD's
      `OptimizeSP_TL_Incre.h` never included `RTA_Cache.h`; `OptimizeSP_Incre.h`
      only forward-declares `class RTACache;` per the P1.13 header-cycle design).
      Fix = add `#include "sources/Safety_Performance_Metric/RTA_Cache.h"` DIRECTLY
      to the test (the consumer), so the dependency is explicit and survives
      whether or not the integration header includes it. Minimal, one line,
      belongs with the P1.12 family (the test already does) but is a build
      prerequisite for this split commit → staged with it.
- [x] **Build + test clean** — `cmake --build build --target check.SP_OPT -j5`
      (DEBUG) = 17/17 green (18.67 s). `testINCRTimeout` both cases pass directly:
      `RespectsGlobalTimeLimit` 127 ms (< 500 ms ceiling), `RespectsGlobalTimeLimit_SingleEvalExceedsCap`
      1.36 s (< 4 s ceiling; was ~8 s unguarded). `testOptimizeIncrePA` (the
      P1.12 test) stays green with the direct include.
- [x] **Stage the clean P1.14 set** — 4 files staged (`OptimizeSP_Incre.cpp`,
      `OptimizeSP_TL_Incre.cpp`, `tests/testINCRTimeout.cpp`, +
      `tests/testOptimizeIncrePA.cpp` include fix). Verified staged diff has
      ZERO `rta_cache_`/`rta_cache_active_` references and 17 P1.14 markers.
      Handed to user for review + commit (agents only `git add`).

## Phase 3 — Verify on the A/B (BLOCKED on P1.15)

> **Status 2026-07-19: blocked.** The Phase 2 + 2b fixes are committed at HEAD
> `bfbec7e5`, but the P25 A/B used to verify them is the *same* run where
> P1.15 found the `INCR_Reopt_X>1` SIGABRT: the Reopt_5/10/30/60 arms abort
> (exit 134) on the hard tasksets, the harness swallows the crash, and the
> aggregate then compares schedulers over unequal taskset sets. Re-running
> the A/B before P1.15 (harness loud-failure + crash fix) lands would just
> reproduce the broken aggregate. Unblocks when P1.15 Phase 1+2 are done.

- [ ] **Rebuild the release binary** (`release/tests/RunOrchestrator` is stale,
      pre-fix). The DEBUG `build/` is current (17/17 ctest green) but the A/B
      runs against the release binary. Must pick up BOTH the BF guard
      (Phase 2) AND the INCR mirror (Phase 2b) — both committed at HEAD, so a
      clean rebuild picks them up. (Note: P1.15 Phase 2 may add a further
      optimizer change to the INCR path; rebuild after that too.)
- [ ] **Re-run the P25 period A/B** (or a focused BF-only re-run on the 10
      tasksets at N=6) on the **fixed harness** so any remaining crash is loud,
      not silent. Confirm BF `Mean_Scheduler_Execution_Time_s` drops to
      ≤ ~10 s and the per-taskset totals in the `goal.md` table all come under
      the cap (plus bounded overshoot). The INCR arms are already ~0.2 s/interval
      (well under cap), so the mirror is a safety-net for pathological tasksets
      rather than a measured-overhead fix — but confirm INCR stays bounded too.
- [ ] **Confirm SP unchanged** for the tasksets that already finished within
      budget (tasksets 1, 4 at minimum; the ~10.1 s cluster may see a tiny SP
      change if a leaf is now cut off — document it).
- [ ] **Update `dev_log.md`** with the before/after numbers + the final
      attribution.

## Refactor follow-up (deferred; flag for user)

- [ ] **Prune dead `ifTimeout`** (`OptimizeSP_Base.h:8` + `OptimizeSP_Base.cpp:7-16`)
      — no callers left after both BF check sites switched to
      `BFSharedBudgetCancelled()` (verified `grep -rn ifTimeout sources/ tests/`:
      only decl + defn + comments). Coding rule: "ruthlessly prune features that
      are not used anymore." Optionally also drop the now-vestigial
      `OptimimizePA_Base::start_time_` capture for the BF path (the outer
      `OptimizePA_with_TimeLimitsStatus::start_time_` is still read as the
      shared-budget seed; the inner `OptimizePA_BF::start_time_` is not).
