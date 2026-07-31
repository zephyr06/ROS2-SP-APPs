# P0.6 — Tasks (working checklist)

> See `goal.md` for the algorithm (REDESIGNED 2026-07-30; hard gate added same day). Seed at
> the P0.8-certified operating point (DM-grouped PA + TL = largest grid option ≤ `et_mean`) →
> run a **TL-only walk with a hard per-candidate feasibility gate** → **keep** the best-SP-
> feasible result as `static_solution_`.
> **No WCET-mode flip, no global-max WCET precompute** (old step 2 / old D2 DROPPED). The
> filter IS revived in a NEW form (old step 4 / old D5/D6/D7): a hard gate on the **probabilistic**
> `ddl_miss_chance ≤ threshold`, in normal ET mode, offline-scoped — NOT the old WCET point-mass
> RTA. The user's objective: "find the best possible performance while guaranteeing all important
> tasks' DDL miss chance ≤ SP thresholds" → a constrained optimization (max SP s.t. the gate).
> Depends on P0.8 (certifies the seed sub-region) + P0.9 (group-locked DM seed, LANDED).
> Blocks P0.7 (fall-back invocation).

## 0. Design decisions
- [x] **D1 (LANDED via P0.9):** top-50% by `sp_weight`, persisted as `bool Task::is_important`.
- [x] **D3 (2026-07-27):** per-taskset (== once per worker invocation).
- [x] **D4 (2026-07-27):** in-memory `static_solution_` member.
- [x] **2026-07-30 redesign:** old D2 (global-max WCET) DROPPED (leans on P0.8's seed-sub-region
      certification + the gate). Old D5/D6/D7 REVIVED-RESCOPED: hard gate on probabilistic
      `ddl_miss_chance ≤ threshold` (normal-ET, offline-scoped, skip-and-continue) — NOT the old
      WCET point-mass RTA.
- [x] **D8 (RESOLVED 2026-07-30, via the gate):** the from-scratch path reorders PA, but the hard
      gate makes PA descent safe-to-add-later (rejects any threshold-violating PA move). v1 =
      TL-only walk (group lock preserved by construction); PA descent = deferred (gated) enhancement.
- [ ] Record decisions in `dev_log.md` + memory; sync with P0.7/P0.8 owners.

## 0.5. `bool is_important` on `Task` + generator labeling — DONE (via P0.9)
- [x] C++ `bool is_important = false` at `RegularTasks.h:111`; parse `RegularTasks.cpp:87`;
      emit `:126`. Generator `yaml_exporter.py:73`; labels top-`IMPORTANT_TASK_RATIO` (0.5) by
      `sp_weight` desc (`taskset_generator.py:577-581`; config `:38`). P0.8 RTA reads the same
      bool → no drift. **LANDED — no P0.6 work here.**

## 1. DM-grouped PA seed — DONE (via P0.9)
- [x] `DeadlineMonotonicPriorityVec()` (`OptimizeSP_TL_Incre.cpp:705-721`) already group-locks
      (reads `is_important` at `:713-714`); `SeedIncumbentFromDMFast()` (`:772-780`) commits
      the DM-grouped + min-TL seed. **Old `AssignDMRespectingGroupOrder` extraction =
      SUPERSEDED — LANDED, no P0.6 work here.**

## 2. ~~Global-max-across-intervals WCET precompute~~ — DROPPED (2026-07-30 redesign)
- [x] **SUPERSEDED + DROPPED.** P0.6 no longer computes a WCET; it relies on P0.8's grid-wide
      certification (perf WCET = `et_mean`). The whole TL grid is safe (runtime ET ≤ et_mean =
      WCET for any TL ≤ et_mean). See `goal.md` "Why no per-candidate filter."

## 3. Static-solution seed (PA + TL) — DONE (PA via P0.9; TL LANDED 2026-07-30)
- [x] PA seed (DM-grouped): DONE via P0.9 (`DeadlineMonotonicPriorityVec`).
- [x] **TL seed (LANDED 2026-07-30):** for each perf task, the **largest TL grid option ≤
      `et_mean`** (`execution_time_dist.GetAvgValue()`); non-perf TL = -1. Implemented as the
      pure directional helper `FindLargestTimeLimitAtOrBelow(pairs, et_mean)`
      (`OptimizeSP_TL_Incre.cpp`, after `Find_Close_ExecutionTime`) + the per-task vector method
      `SeedTimeLimitsAtOrBelowEtMean()` (mirrors `InitializeTimeLimitsFromETConfig`'s structure,
      uses the at-or-below helper instead of the bidirectional `Find_Close_ExecutionTime`).
      Falls back to the smallest grid option + `CoutWarning` when no option ≤ et_mean exists.
- [x] TDD red→green (6 helper tests + 1 vector test, all green): (a) et_mean above largest →
      largest; (b) et_mean below smallest → smallest + warning; (c) et_mean between options →
      largest ≤ et_mean (the directional guarantee vs `Find_Close_ExecutionTime`'s tie); (d)
      et_mean exactly equals an option → that option; (e) et_mean equals smallest → smallest;
      (f) empty grid → 0; (vector) T_perf mean 500 grid [400,600,800,1000] → 400, T_noise → -1.
      `cmake --build build_test --target check.SP_OPT -j5` = 17/17 green, no regressions.

## 4. ~~Skip-and-continue schedulability filter~~ — REVIVED as a hard per-candidate gate (2026-07-30 constraint mode)
- [x] **REVIVED-RESCOPED 2026-07-30** (the user's objective revived the filter in a new form):
      a HARD per-candidate gate on the probabilistic `ddl_miss_chance ≤ sp_threshold`, in
      normal ET mode, offline-scoped — NOT the old WCET point-mass RTA. (Old "DROPPED" wording
      below was the *first* 2026-07-30 redesign's "no filter" limb, since reversed by the user's
      objective — see `goal.md` "Why a hard gate.")
- [x] **D9 grounded (2026-07-30):** the gate cannot hook at the walk's `IsBetterTimeLimitOption`
      (`:581`) — adoption is a side-effect of `eval`→`OptimizeIncreSingleTask`→`UpdateRecords`→
      `CommitIncumbent` (`:259`,`:154`), which runs BEFORE the `:581` best-tracking. The gate must
      sit at the commit chokepoint. See memory `p06-gate-hook-point-discrepancy`.
- [x] **D9 RESOLVED (user: "i don't care how you do this") → step 4a (pure predicate) LANDED:**
      `ImportantTasksMeetThresholds(dag, sp_params, pa, tl, node_rtas) -> bool` (SP_Metric.{h,cpp}).
      TDD red→green (4 tests); 17/17 ctest green. Mirrors the cache-path SP eval's
      bake+prioritize+miss-chance path. **Zero-extra-eval (user-directed 2026-07-30):** the
      predicate takes the ALREADY-COMPUTED `node_rtas` the caller materialized when scoring SP
      (`rta_cache_.Evaluate` → `ObtainSP_Full_From_NodeRTAs`), NOT a re-derived RTA — the prior
      shape A (`ProbabilisticRTA_TaskSet` per call) would have duplicated the cache's work on every
      ADOPTED candidate. The bake+prioritize still happens inside (so `node_rtas[i]` pairs with the
      prioritized task at i — same CONTRACT as `ObtainSP_Full_From_NodeRTAs`). This is the design
      doc's shape (B) achieved by passing RTAs through (not by threading per-task miss-chances out
      of `ObtainSP_DAG_From_Dists`).
- [x] **Step 4b — wire the gate into the commit chokepoint (LANDED 2026-07-30, RELOCATED same
      day per user review, TDD).** Three pieces in `OptimizeSP_TL_Incre.{h,cpp}` +
      `tests/testIncreOpt_w_TL.cpp`: (1) `WouldBeatIncumbent(challenger_sp, time_limits) -> bool` —
      the exact commit predicate `UpdateRecords` uses; `UpdateRecords` delegates to it. (2) `bool
      enforce_important_task_gate_ = false` — the offline-only gate flag (SINGLE duty: enable the
      gate check). (3) The gate INSIDE `UpdateRecords` (the commit chokepoint), firing ONLY on
      would-beat (the user's rule): a candidate that would beat the incumbent is committed only if
      `ImportantTasksMeetThresholds` passes; on a gate-REJECT of an SP-better candidate → return
      `false` (no `CommitIncumbent`), and the eval (`OptimizeIncreSingleTask`) reports the INCUMBENT
      SP (ghost-SP fix) so the walk's `IsBetterTimeLimitOption` sees "no progress". **KEY DESIGN
      (user-directed 2026-07-30): the gate is a PURE extra accept/reject criterion at the
      comparison step — the optimization process, INCLUDING PA descent, runs IDENTICALLY whether
      the flag is on or off.** The prior v1 (gate in `OptimizeIncreSingleTask` + PA descent
      SKIPPED when the gate was on) over-coupled two concerns into the flag and was DROPPED: PA
      descent now runs unconditionally (`!BFSharedBudgetCancelled()` only). RTA source: the gate
      re-runs the SAME `rta_cache_.Evaluate(dag_tasks_, opt_pa_, tl)` `CommitIncumbent` uses
      (`:824`) — the candidate's FINAL {pa, tl} (PA descent applied), NOT the pre-descent
      `baseline_rtas` pointer (which PA descent invalidates by overwriting the cache's scratch
      buffer). `|diff|<=1`-cheap on the incremental path (the only path the gate is on): after PA
      descent the champion is either adopted to the candidate (`|diff|==0`, FullReuse) or the prior
      incumbent (`|diff|==1`); `Evaluate` never mutates the champion, so the caller's `cache_backup`
      snapshot-revert stays sound. Zero extra RTA eval flag-off (gate skipped entirely). Flag written
      true ONLY in tests (no prod site — `ComputeStaticSolution` not built yet) → prod byte-identical.
      Seed-feasibility invariant: TL ≤ et_mean → `ddl_miss_chance = 0` → seed gate-feasible by
      construction → the gate can only REJECT. TDD: 5 tests (precondition + flag-off-commits /
      flag-on-rejects / flag-on-keeps / flag-on-non-beat-doesn't-gate) all green WITH PA descent
      re-enabled (on the reject fixture T_perf is already top-priority, so PA descent finds no
      strict-improving move and the gate sees the candidate's actual {pa,tl} and rejects).
      `cmake --build build_test --target check.SP_OPT -j5` = 17/17 green. NOT committed (git add-only).
- [ ] **Step 5 — `ComputeStaticSolution` wiring (NEXT).** Construct the optimizer; seed DM-grouped
      PA (P0.9 `DeadlineMonotonicPriorityVec`) + et_mean-bounded TL (step 3
      `SeedTimeLimitsAtOrBelowEtMean`); set `enforce_important_task_gate_ = true`; run the walk
      (`OptimizeIncre_w_TL` → `RunIntervalDescent`; PA descent runs normally under the gate);
      keep the committed `ResourceOptResult` as `static_solution_`; reset the flag. Call after
      `incr_optimizer_` construction (`SimulationOrchestrator.cpp:300-302`), before the interval
      loop (304-308), OUTSIDE `DeterminePrioritiesAndBudgets`'s ET bracket (316-322). Verify the
      safety assumption sim perf ET = `min(et_mean,TL)` downward cap. (The from-scratch group-lock
      concern is now MOOT — the gate is the constraint, not the group lock; PA descent that finds a
      higher-SP PA still passing the gate is strictly better for the constrained objective.)

## 4b-followup — UNIFY the three acceptance predicates (DEFERRED 2026-07-30 per user)
- [x] **FINDING (2026-07-30).** The ghost-SP override (step 4b v2) is a SYMPTOM of three
      divergent acceptance predicates that can disagree: (1) PA-move loop `OptimizeIncre_SingleTask`
      `sp_eval > opt_sp_` (strict, vs LOCAL member, in-place adopt); (2) TL-walk adoption
      `IsBetterTimeLimitOption` (strict-`>` OR approx-tie+`step<0`, vs LOCAL walk `best_sp`, does
      NOT commit); (3) champion commit `WouldBeatIncumbent`/`UpdateRecords` (strict-`>` OR
      approx-tie+smaller-TL, vs GLOBAL `res_opt_`, commits via `CommitIncumbent`). The override
      papers over #2-vs-#3 disagreeing on a gate-reject (phantom SP). See `dev_log.md` 2026-07-30
      "ghost-SP fix: ROOT CAUSE" entry.
- [x] **DEFERRED 2026-07-30 (user):** "if code change is a lot, i'll give up and let you adopt the
      original proposal by adding extra check, even though the code doesn't read elegantly." The
      unification was scoped to "TL-walk only" (align `IsBetterTimeLimitOption` with the commit
      predicate vs global `res_opt_`; leave the PA-loop's local in-place adoption alone).
      Grounding found it is a LARGE change, not the simple cleanup it appeared:
      (1) **Breaks the walk-core test framework** — `StubTLWalkOptimizer::CallOptimizerGivenTimeLimits`
      (`tests/testIncreOpt_w_TL.cpp`) returns SP from a `tl_to_sp` map WITHOUT committing →
      `res_opt_` never advances; routing adoption through the committed incumbent would make the
      walk never adopt → the 7 `TrialAndErrorTLWalkSynthetic` tests break (requires reworking the
      stub to commit + assertion updates).
      (2) **NOT byte-identical flag-off** — `res_opt_.sp_opt == best_sp` throughout a flag-off walk
      (each adoption commits), so routing adoption through `res_opt_` makes every non-commit look
      like an approx-SP tie; on the downward pass (`step<0`) `IsBetterTimeLimitOption` adopts on a
      tie → the walk adopts smaller TLs on every non-commit, never spending patience → exhaustive
      to the grid boundary even flag-off. The delta reaches BEYOND the gate-on path the
      "TL-walk only" scope targeted.
      (3) **The override is already correct** — this is a cleanliness refactor, not a bug fix.
      → User accepted the elegance tradeoff. **Step 4b v2 stands as-is (final).** Re-file as P2.x
      if revisited; do NOT start coding it.
- [x] **The gate + ghost-SP override (step 4b v2) is the chosen design.** No further work here;
      step 5 is the active next item.

## 4-old. ~~Skip-and-continue schedulability filter~~ — (first 2026-07-30 "no filter" limb, SUPERSEDED above)
- [x] **SUPERSEDED + REVIVED.** The first 2026-07-30 redesign dropped the filter (structural
      safety argument: P0.8 certifies the whole TL grid). The user's objective ("best performance
      s.t. all important tasks' DDL miss chance ≤ SP thresholds") REVIVED it in a new form (the
      hard gate above) because `ddl_miss_chance` is PROBABILISTIC and binds for TL > et_mean.
      See `goal.md` "Why a hard gate (not soft)" + the constraint-mode dev-log entry.

## 5. Wire `ComputeStaticSolution()` offline + ET-excluded
- [ ] Resolve **D8** (from-scratch path & the group lock) before wiring.
- [ ] Implement `ComputeStaticSolution()`: seed DM-grouped PA (P0.9) + et_mean-bounded TL
      (step 3), run the normal TL walk (`OptimizeIncre_w_TL` → `RunIntervalDescent`,
      TL-only → group lock preserved), return the committed `ResourceOptResult`. **NO WCET-mode
      flip, NO filter.**
- [ ] Call `ComputeStaticSolution()` after `incr_optimizer_` construction
      (`SimulationOrchestrator.cpp:300-302`), before the interval loop (304-308).
- [ ] Store result in an in-memory `static_solution_` member (per D4).
- [ ] Verify the compute is OUTSIDE `DeterminePrioritiesAndBudgets`'s ET bracket
      (316-322): reported `scheduler_execution_time.txt` unchanged vs. baseline at the same N.
- [ ] Emit a separate `static_solution_compute_time` profile (does NOT enter the online ET
      metric).
- [ ] **Verify the two safety assumptions:** (1) sim perf ET = `min(et_mean, TL)` (downward
      cap, NOT an overrunable budget) at the `SimulateInterval` ET-draw site; (2) the
      from-scratch path preserves the group lock (or is constrained).

## 6. Verification + records
- [ ] `cmake --build build_test --target check.SP_OPT -j5` green (17/17 ctest).
- [ ] Spot-check: at a smoke-test N, `static_solution_` is populated; its PA is DM-grouped;
      its TL vector seeds at-or-below et_mean and is the optimizer's result from that seed.
- [ ] Confirm the seed point is at-or-below P0.8's certified WCET (TL ≤ et_mean → runtime ET ≤
      et_mean = P0.8 perf WCET).
- [ ] `dev_log.md` (this folder + top-level) + memory updated.
- [ ] `git add` staged; user reviews (no commit).
