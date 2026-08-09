# P1.29 — Add the important-tasks schedulability gate to the re-opt from-scratch beam (in-search)

**Priority:** P1 (correctness / latent defect — the INCR reopt beam runs UNGATED)
**Status:** CLOSED 2026-08-08 (git add-only, NOT committed; 17/17 ctest green;
speed PASS). Code work COMPLETE — moved to "Resolved / shipped" in MEMORY.md.
DEFERRED: full comparison sweep re-run. Does NOT fix P1.28 (separate; fix pending
user decision A vs C).
Prior flag-based impl (sticky `has_unschedulable_important` + top-K prefer-unflagged/
fall-back-to-flagged) REJECTED by user as wrong-by-design ("there should not be a
class-level flag"). REFRAMED as an INCR optimizer API refactor:
`OptimizeFromScratch`/`OptimizeIncre`/`OptimizeIncre_SingleTask` return a new
`PriorityOptResult` struct (priority + `bool schedulable`, mirroring
`ResourceOptResult`). Phased: (1) task records [DONE]; (2) behavior-preserving
API refactor (return the struct; compute `schedulable`; do NOT yet change control
flow — bit-identical) [DONE]; (3) hard-prune at push time + the empty-vec contract
[DONE]. Phase-3 design fork SETTLED by user (2026-08-08): "this priority optimization
doesn't need to consider fallback, if unschedulable, return unschedulable with an
empty priority assignment vector." So the priority optimizer's CONTRACT is:
schedulable → `{priority_vec=<plan>, sp_opt, true}`; unschedulable →
`{priority_vec={}, sp_opt=INT_MIN, false}`. The explicit "wire `schedulable`
through `CallOptimizerGivenTimeLimits` → `SeedBaselineAndArmCache` →
`ReOptimizePeriodic`" step was DECLINED — the priority optimizer does NOT itself
fall back; the reopt layer's existing backstop `AdoptFallbackIfUnschedulable`
(`OptimizeSP_TL_Incre.cpp:784`) handles the unschedulable result (which it already
does: an emptied beam leaves `res_opt_` = pre-reopt incumbent; `UpdateRecords`
won't commit `INT_MIN`; the backstop then adopts the safe fallback if the final is
still unsched). `OptimizeSP_TL_Incre.cpp` is UNTOUCHED. The `schedulable` bool IS
the failure-propagation signal (no exception, no sentinel).
**Cross-links:** P1.27 (`cbdde63b` — the BF-only in-search gate this mirrors/completes
the INCR side of), P1.28 (the 21/22 collapses — NOTE: those are INCREMENTAL-interval
collapses and are NOT fixed by this gate; see "Relationship to P1.28" below), P0.7
(trigger b-i during-walk gate / b-ii backstop), P0.10 (BF fallback gate).

## The latent defect

The re-opt from-scratch beam commits its SP-max leaf **without any important-task
schedulability check**. The during-walk gate (P0.7 trigger b-i, `UpdateRecords`
@ `OptimizeSP_TL_Incre.cpp:238-258`) runs **only when `rta_cache_active_` is true**.
The reopt beam runs **DISARMED**:

- `ReOptimizePeriodic` → `RunIntervalDescent(Reopt)` → `SeedBaselineAndArmCache`
  calls `ResetIncumbentBaseline(/*from_scratch=*/true)` (`:1204-1205`), which
  default-constructs the cache and sets `rta_cache_active_ = false` BEFORE
  `CallOptimizerGivenTimeLimits(..., from_scratch=true)` runs the beam.
- In `CallOptimizerGivenTimeLimits` the `from_scratch` branch (`:275-280`) calls
  `OptimizePA_Incre::OptimizeFromScratch` then `UpdateRecords` — with
  `rta_cache_active_=false`, so `UpdateRecords` falls into the `else` branch
  (`:252-257`) which explicitly does **nothing** ("no per-step check here").
- `CommitIncumbent` (`:263`) then commits the SP-max leaf unchecked.
- The cache is re-armed only AFTER the beam (`SeedBaselineAndArmCache` re-arms,
  then `AdoptChampion`).

So the FIRST schedulability gate the reopt result ever sees is the **post-hoc
backstop** `AdoptFallbackIfUnschedulable` (`:1106`, called from
`Optimize_w_TL_ScratchOrIncre:784`), which on failure **binary-swaps the whole plan
to RM-Fast** instead of re-searching for a schedulable high-SP plan.

This is the **INCR analogue of P1.27**. P1.27 fixed it BF-only: `cbdde63b` moved the
gate **in-search** at each BF `Optimize` leaf in `OptimizeSP_TL_BF.cpp`
(`if (res_cur.sp_opt > res_opt.sp_opt && ImportantTasksMeetThresholds(...))`).
**That fix never touched the INCR reopt path.**

## Revision (2026-08-08) — API refactor to `PriorityOptResult` [CURRENT DIRECTION]

User rejected the prior flag-based design (sticky `has_unschedulable_important` struct
field + top-K prefer-unflagged/fall-back-to-flagged split) as wrong-by-design — the
gate must act at the **partial-path level**, not via persistent class/struct state.
Revised mechanism + carrier:

- **Hard prune at push time (no flag):** in `OptimizeFromScratch`, a candidate partial
  path that makes a just-decided **important** task unschedulable (`GetDDL_MissProbability(rta,ddl) > thresholds_node[id]`) is simply NOT pushed into the `pq` —
  reusing the `UpdateSP` RTA (already final; HP set = unassigned same-proc tasks), no
  recompute. Top-K reverts to plain greedy extraction. Vacuous (never prunes) when no
  task is `is_important` → legacy bit-identical.
- **API refactor (carrier):** `OptimizePA_Incre::OptimizeFromScratch` /
  `OptimizeIncre` / `OptimizeIncre_SingleTask` change return type `PriorityVec` → new
  `PriorityOptResult` struct (mirrors `ResourceOptResult` in `OptimizeSP_Base.h:80`):
  at minimum `{PriorityVec priority_vec; double sp_opt; bool schedulable;}` (extras as
  needed). `schedulable` is the failure-propagation signal — replaces the prior
  sentinel/exception ideas.
- **Empty-beam failure (user-directed):** if a beam level yields zero schedulable
  candidates → `OptimizeFromScratch` returns `PriorityOptResult{.schedulable=false}`
  (no result). Re-optimization then FAILS and the **fall-back triggers at the end of
  re-optimization** (the `schedulable=false` propagates up via the struct; the reopt
  flow adopts the safe fallback). Exact wiring decided in phase 3.

**Phased plan (user-directed 2026-08-08):**
1. ✅ Update task records (this file + `dev_log.md` + memory).
2. ✅ Behavior-preserving API refactor: change the three functions' return type to
   `PriorityOptResult`; have callers unpack `{priority_vec, sp_opt}` where they used
   the bare `PriorityVec`; compute `schedulable` (via `ImportantTasksMeetThresholds`)
   but do NOT yet branch on it. Bit-identical. Hard-prune at push time DEFERRED to
   phase 3 (keeps phase 2 truly bit-identical to HEAD). 17/17 ctest green.
3. ✅ Hard-prune at push time + the empty-vec contract. `UpdateSP`/`AssignAndUpdateSP`
   return `bool` (false iff the just-decided IMPORTANT task is unschedulable, reusing
   the already-final `UpdateSP` RTA — no recompute); `OptimizeFromScratch` skips the
   `pq.push` on false; an emptied beam returns
   `{PriorityVec{}, INT_MIN, /*schedulable=*/false}` (the contract). 17/17 ctest green.
   **The explicit fall-back wiring was DECLINED by the user (2026-08-08):** "this
   priority optimization doesn't need to consider fallback, if unschedulable, return
   unschedulable with an empty priority assignment vector." So `schedulable` is NOT
   branched on by production callers — the reopt layer's existing backstop
   `AdoptFallbackIfUnschedulable`@`:784` already handles the unschedulable result (an
   emptied beam leaves `res_opt_` = pre-reopt incumbent; `UpdateRecords` won't commit
   `INT_MIN`; the backstop then adopts the safe fallback if the final is still unsched).
   `OptimizeSP_TL_Incre.cpp` is UNTOUCHED. Verified the empty-beam flow is safe end-to-end
   (champion adoption reads the member `opt_pa_`/`res_opt_`, NOT the local emptied vec;
   `INT_MIN` as `current_config_sp` is a benign "first eval improves" walk baseline).

The sections below ("The fix direction", "Approach") describe the **REJECTED** prior
flag-based design and are kept only for the defect history / cross-references.

## The fix direction (user-directed, 2026-08-08) — REJECTED (see Revision above)

Add the important-tasks gate **in-search** during the reopt from-scratch beam search.
User's described mechanism:

> during beam search, for the tasks that we have already searched, check whether
> those tasks are important tasks and schedulable; discard one (beam) option if the
> tasks whose priority are decided are unschedulable, when the task is also an
> important task.

I.e. a **partial-assignment** schedulability prune: as the beam decides priorities
for tasks one at a time, check the already-decided **important** tasks for
schedulability under the partial plan, and **discard a beam option early** if it
makes a decided important task unschedulable. This is finer-grained than P1.27's
leaf-level (full-assignment) gate — it prunes bad branches mid-beam rather than only
rejecting the final leaf.

Hook point: inside `OptimizePA_Incre::OptimizeFromScratch` (the beam, called from
`CallOptimizerGivenTimeLimits:278`) — at each partial priority assignment, run the
4-arg fresh-RTA `ImportantTasksMeetThresholds(dag, sp, pa, tl)` overload
(`SP_Metric.cpp:245`, cache-free — the beam is disarmed) on the decided important
tasks; prune the option on failure. Keep the post-hoc backstop
`AdoptFallbackIfUnschedulable` as the safety net (the "no schedulable candidate"
case), exactly as P1.27 kept the BF post-hoc gate.

## Relationship to P1.28 (important — do NOT conflate)

P1.28's `taskset_2` collapses to 0.527888 at intervals **16/21/38/41/44/52**. These
are **INCREMENTAL** intervals, NOT reopt intervals (reopt fires at 0/10/20/30/40/50,
all of which are `kept_walk` with high SP). The reopt-committed plan is **schedulable
at the reopt interval** (e.g. interval 20 = `kept_walk`, SP 0.954072); the collapse
happens at interval 21, one step later, under a drifted ET. Therefore an in-search
gate on the reopt beam (this task) would **pass the exact same schedulable reopt
plan** and would **NOT prevent the 21/22 collapses**. P1.28's collapse mechanism is
ET-drift-at-incremental-intervals + the incremental walk's SP-strict compare-and-keep
+ stale incumbent benchmark + backstop binary-swap — a different defect, tracked
separately under P1.28.

P1.29 is still a legitimate latent-defect fix: in a scenario where the reopt beam's
SP-max leaf is ITSELF unschedulable (no ET drift needed), the current code commits it
unchecked and relies on the backstop to swap DOWN to RM-Fast. The in-search gate would
instead select a schedulable high-SP leaf at reopt time. Whether such a scenario
actually occurs in the current tasksets is TBD during implementation.

## Approach — SUPERSEDED by the Revision (API refactor) above; kept for history

1. ✅ Confirm the beam's partial-assignment structure in `OptimizeFromScratch`
   (where per-task priority decisions are made) and identify the prune hook point.
   → `PriorityPartialPath::UpdateSP` already computes each decided task's FINAL
   RTA (HP set = unassigned same-proc tasks); gate signal is free.
2. ✅ TDD RED→GREEN: `TaskSetForTest_p129_unschedulable_important` in
   `tests/testOptimizeIncrePA.cpp` — point-mass 2-task fixture whose SP-max leaf
   is unschedulable for an important task; asserts the schedulable (lower-SP)
   leaf is adopted (opt_sp_≈0.2, not 0.8; `ImportantTasksMeetThresholds`=true).
3. ✅ GREEN: in-search partial-assignment gate. REUSES the `UpdateSP` RTA (NOT the
   4-arg fresh-RTA recompute — user-directed 2026-08-08: avoid double RTA cost +
   order-dependence). Sticky `has_unschedulable_important` flag + top-K prune.
4. ✅ Verify: 17/17 ctest PASS; bit-identical w/o `is_important`; backstop retained.

## Files

- `sources/Optimization/OptimizeSP_TL_Incre.cpp` — `UpdateRecords:220` (b-i gate,
  `rta_cache_active_`-gated; `:252-257` the no-op else branch),
  `CallOptimizerGivenTimeLimits:268` (`from_scratch` branch `:275-280`),
  `ResetIncumbentBaseline:1198` (disarms cache `:1204-1205`),
  `AdoptFallbackIfUnschedulable:1106` (b-ii backstop, keep as safety net),
  `Optimize_w_TL_ScratchOrIncre:722` (backstop call `:784`),
  `ReOptimizePeriodic:1225`.
- `sources/Optimization/OptimizePA_Incre.{h,cpp}` (or wherever `OptimizeFromScratch`
  lives) — the beam; the in-search prune hook point.
- `sources/Optimization/OptimizeSP_TL_BF.cpp` — P1.27 reference in-search gate shape.
- `sources/Safety_Performance_Metric/SP_Metric.cpp:245` — 4-arg fresh-RTA
  `ImportantTasksMeetThresholds` overload.

## Done when — ALL MET (2026-08-08)

- ✅ In-search partial-assignment important-task gate lands on the reopt
  from-scratch beam (hard-prune at push time in `OptimizeFromScratch`); a reopt
  beam whose SP-max leaf is unschedulable now adopts a schedulable high-SP leaf
  (when one exists under the beam's TLs) instead of falling through unchecked.
- ✅ When NO schedulable leaf exists, the beam empties and returns the contract:
  `{priority_vec={}, sp_opt=INT_MIN, schedulable=false}` (the priority optimizer
  does NOT itself fall back — the existing backstop@:784 handles it).
- ✅ 17/17 ctest; legacy bit-identical (no `is_important` → prune vacuous).
- ✅ Post-hoc `AdoptFallbackIfUnschedulable` retained as safety net (untouched).
- ✅ Milestone to `agents/dev_log.md`; memory entry; cross-link P1.27 (completes
  the INCR side that `cbdde63b` left BF-only).

## Out of scope

- `git commit` — user's standing constraint (`git add` only).
- Fixing the P1.28 incremental-interval collapses (separate task; this gate does not
  address them — see "Relationship to P1.28").
- Re-running the full comparison sweep.

## BF optimization follow-up — DONE 2026-08-08 (git add-only, separate commit)

The prior note here (apply the INCR follow-up's 3 items to BF) was **wrong about
BF's structure**. Investigation found items (1) and (3) are N/A for BF and (2) is
marginal; the REAL BF analogue is an in-search per-candidate gate in the PA
enumeration. User reframed the follow-up accordingly (2026-08-08): "during BF
searching, it essentially tries each possible solution, you can add a
schedulability check when a candidate solution outperforms the current best
found solution; for BF, don't have to worry about implementation efficiency."

- **What was done:** in `OptimizePA_BF::IterateAllPAs` (`OptimizeSP_BF.cpp`), the
  adoption `if (sp_eval > opt_sp_)` became `if (sp_eval > opt_sp_ &&
  ImportantTasksMeetThresholds(dag_tasks_, sp_parameters_, priority_assignment))`.
  So within each TL combination BF now keeps the best SCHEDULABLE PA rather than
  the SP-max PA (which may be unschedulable) and relying on the TL-level
  P1.27 gate / post-hoc backstop to reject/swap it. New 3-arg
  `ImportantTasksMeetThresholds(dag, sp, pa)` overload (`SP_Metric.{h,cpp}`) for a
  dag whose TLs are ALREADY baked (mirrors `EvaluateSPWithPriorityVec`).
  **Refactored 2026-08-08** (see dev_log): originally delegated to the 4-arg via a
  `tl=-1` no-op trick; now prioritizes the already-baked `dag_tasks.tasks` + fresh
  RTA + a shared anonymous-namespace core `ImportantTasksBelowThresholds` (no `tl`
  sentinel). Bit-identical when no task is `is_important` (overload vacuously true
  → `sp_eval > opt_sp_` unchanged).
  `OptimizeSP_TL_BF.cpp` UNTOUCHED — the P1.27 TL-level leaf gate is kept (still
  needed: when a TL combo has NO schedulable PA, `IterateAllPAs` returns the
  unschedulable seed and the TL-level gate rejects it → backstop).
- **TDD:** `TaskSetForTest_p129_bf_unschedulable_important` /
  `InSearchGate_AdoptsSchedulablePAOverUnschedulableSpMax` in
  `tests/testOptimizePA.cpp` (same point-mass fixture as the INCR P1.29 test).
  RED before gate: BF returns `{1,0}` sp 0.8 (important task 0 unsched). GREEN
  after: returns `{0,1}` sp 0.2 + `ImportantTasksMeetThresholds`=true. 17/17 ctest.
- **Why the 3 INCR follow-up items do NOT mirror to BF:**
  (1) **budget-timeout prune — N/A.** BF has no INCR-style partial-path defect:
  `EvaluateSPWithPriorityVec` returns `INT_MIN` on ANY budget interruption (both
  on entry and post-`ObtainSP_DAG`, `OptimizeSP_Base.cpp:181-215`), so an
  interrupted/half-evaluated permutation loses to the incumbent and is never
  adopted. BF only ever commits COMPLETE PAs, and the P1.27 gate runs on every
  `res_cur`. The `return;` at recursion entries already prunes subtrees.
  (3) **`reserve(K*N)` — N/A.** BF has no beam (no `K`); `IterateAllPAs` recurses
  over permutations. The only `reserve` is `time_limit_option_for_each_task.reserve(N)`
  (`OptimizeSP_TL_BF.cpp:22`), already correct.
  (2) **minimize P1.27 comments — marginal/skipped.** The P1.27 comment is already
  ≤2 lines; the longer blocks in `OptimizeSP_TL_BF.cpp` are P1.14/P0.10-tagged
  (different tasks), not P1.27.
