# P1.29 — Re-opt in-search important-task gate — Dev Log

> Detailed working log. Append chronological entries below. On completion, append a
> one-line milestone to the top-level `agents/dev_log.md`.

## 2026-08-08

- Task created (user-directed). Goal: add the important-tasks schedulability gate
  IN-SEARCH to the re-opt from-scratch beam (currently runs disarmed/ungated).
  User's mechanism: during the beam, for tasks already searched (priority decided),
  check whether those tasks are important AND schedulable; discard a beam option if
  a decided important task is unschedulable under it. Partial-assignment prune, finer
  than P1.27's leaf-level gate.
- No investigation/implementation done (per user). Code locations referenced in
  `goal.md` are carried over from P1.28's verified walk.
- IMPORTANT framing note recorded in `goal.md`: this gate does NOT fix P1.28's
  21/22 collapses — those are INCREMENTAL-interval collapses (reopt intervals are all
  `kept_walk`/schedulable; the collapse is one step later under drifted ET). P1.29
  fixes the latent defect where a reopt beam's SP-max leaf is itself unschedulable.
  P1.28's collapse mechanism is tracked separately.

## 2026-08-08 (implementation)

- Step 1 (hook point) CONFIRMED: `PriorityPartialPath::UpdateSP`
  (`OptimizeSP_Incre.cpp`) already computes each decided task's FINAL RTA —
  its HP set = the still-unassigned same-processor tasks = exactly its true
  future HP interferers (Audsley: lower priority decided first; unassigned →
  higher priority). So per-decided-task `GetDDL_MissProbability > threshold` is
  the natural gate signal, available with ZERO extra RTA.
- Design decision RESOLVED with user (2026-08-08): reuse the RTA already
  computed in `UpdateSP` for SP-loss — do NOT recompute via the 4-arg
  `ImportantTasksMeetThresholds` (which would double RTA cost AND be
  semantically muddy: it checks unassigned important tasks under an arbitrary
  appended order, and `GetRTA_OneTask`'s lossy `CompressDistributionWithOnlySize`
  makes the recompute order-dependent). The per-decided-task reuse is exact.
- Step 3 (GREEN) LANDED:
  - `OptimizeSP_Incre.h`: added `bool has_unschedulable_important = false` to
    `PriorityPartialPath` (sticky; copied by value on `new_path = path`, carries
    across beam levels; monotone false→true since a decided task's RTA is final).
  - `OptimizeSP_Incre.cpp` `UpdateSP`: after `rta_curr`, if the decided task is
    `is_important` and `GetDDL_MissProbability(rta_curr, deadline) >
    thresholds_node[task_id]`, set the flag. Mirrors `ImportantTasksMeetThresholds`'
    predicate exactly. Vacuous (never set) when no task is important → bit-identical.
  - `OptimizeSP_Incre.cpp` `OptimizeFromScratch`: top-K extraction now prefers
    unflagged paths; falls back to flagged (best-SP first) only if too few
    unflagged remain, so the beam never empties — an unschedulable final leaf
    then falls through to the post-hoc backstop `AdoptFallbackIfUnschedulable`
    (kept as safety net, matching P1.27). With no important tasks the flag is
    never set → identical to the old greedy top-K → bit-identical.
- Step 2 (TDD RED/GREEN) LANDED: new test
  `TaskSetForTest_p129_unschedulable_important` /
  `InSearchGate_AdoptsSchedulableLeafOverUnschedulableSpMax` in
  `tests/testOptimizeIncrePA.cpp`. Two point-mass (deterministic) tasks on one
  core: task 0 IMPORTANT (ET=5, ddl=6, w=0.2), task 1 not (ET=10, ddl=12, w=0.8).
  Point-mass → RTA is a point mass → ddl_miss is exactly 0/1 → SP exactly
  computable (SP_Func(0,thr)=1, SP_Func(1,thr)=0). Two leaves: "task 0 low pri"
  → task 0 unsched, SP=0.8 (SP-max); "task 0 high pri" → task 0 sched, SP=0.2.
  The SP-max leaf makes the IMPORTANT task unschedulable. Test asserts the gate
  adopts the schedulable leaf (opt_sp_≈0.2, `ImportantTasksMeetThresholds`=true).
  Non-vacuous: the `opt_sp_≈0.2` assertion only passes because the gate pruned
  the 0.8 SP-max leaf — without the gate the beam returns 0.8 (RED). Directly
  tests `OptimizePA_Incre::OptimizeFromScratch` (where the gate lives), not the
  full reopt pipeline — cleaner isolation of P1.29's defect.
- RED empirically CONFIRMED: with the gate's flag-set temporarily disabled
  (`if (false && is_important)`), the test FAILS exactly as predicted —
  `ImportantTasksMeetThresholds`=false AND `opt_sp_`=0.8 (the unschedulable
  SP-max leaf). Gate restored → test PASSES (opt_sp_=0.2). Full TDD cycle clean.
- Step 4 (verify): 17/17 ctest PASS (exit 0), incl. the new test and the P1.27
  important-task fixture (its reopt leaf is schedulable → gate doesn't fire → no
  regression). Bit-identical for no-`is_important` tasksets (flag never set).
  Post-hoc backstop retained (untouched).
- NOT committed (standing `git add`-only constraint). Awaits user review/commit.
  Does NOT address P1.28's 21/22 incremental collapses (by design).
- Refactor pass: trimmed the P1.29 comment blocks (header field, `UpdateSP`
  gate, `OptimizeFromScratch` top-K, test fixture) to ≤3-line comments per the
  coding rules (no line-to-line logic explanation). Behavior unchanged; 17/17
  ctest re-verified. Added P1.28 + P1.29 rows to `agents/overall_tasks.md`.

## 2026-08-08 (revision — API refactor to PriorityOptResult)

- User DESIGN REVIEW of the flag-based impl: REJECTED. The sticky
  `has_unschedulable_important` struct field is "wrong by design" — the gate must
  act at the partial-path level, NOT via persistent class/struct state. Verified
  the user is correct: `PriorityPartialPath::UpdateSP` already computes each
  decided task's FINAL RTA (HP set = still-unassigned same-proc tasks = all future
  higher-pri interferers, because Audsley assigns lowest-pri first and
  `AssignAndUpdateSP` erases `task_id` from `tasks_to_assign` BEFORE `UpdateSP`).
  So a decided important task's schedulability is final at decision time → a single
  per-candidate check on the just-decided task suffices → NO cross-level flag.
- User edge-case directive: if the beam empties (no schedulable partial path),
  re-optimization FAILS and returns no result; the fall-back triggers at the END
  of re-optimization (NOT a least-bad-keep mid-beam).
- User NEW directive (the reframing): change the API of
  `OptimizePA_Incre::OptimizeFromScratch` / `OptimizeIncre` /
  `OptimizeIncre_SingleTask` to return a STRUCT, not a bare `PriorityVec`. New
  struct `PriorityOptResult` (mirrors `ResourceOptResult` @ `OptimizeSP_Base.h:80`):
  at minimum `{PriorityVec priority_vec; double sp_opt; bool schedulable;}`. The
  `schedulable` bool IS the failure-propagation signal (no exception, no -1.0
  sentinel) — this cleanly resolves the earlier "how does empty-beam failure reach
  the fall-back" question.
- Phased plan (user-directed): (1) update task records [DONE this entry]; (2)
  behavior-preserving API refactor — change return types, unpack at callers,
  compute `schedulable` but do NOT yet branch on it (bit-identical; 17/17 green);
  (3) add the reopt fallback mechanism — route `schedulable=false` to the
  end-of-reopt fall-back (`AdoptFallbackIfUnschedulable` @
  `OptimizeSP_TL_Incre.cpp:784`, or `AdoptSafeFallbackAsIncumbent` like the ET-jump
  short-circuit @ `:750`), wiring the bool through `CallOptimizerGivenTimeLimits`
  (from_scratch `:275-280`) → `SeedBaselineAndArmCache` (`:544-545`) →
  `ReOptimizePeriodic`.
- Hard-prune at push time (skip `pq.push` for an unschedulable important-task
  candidate) lands in phase 2 or 3; top-K reverts to plain greedy. Bit-identical
  w/o `is_important`. The staged flag-based code (`OptimizeSP_Incre.{h,cpp}` +
  test) will be REPLACED by this refactor; the existing P1.29 test fixture
  (`InSearchGate_AdoptsSchedulableLeafOverUnschedulableSpMax`, point-mass 2-task)
  stays valid: leaf A (SP-max, unsched important) pruned at push → only leaf B →
  opt_sp_≈0.2; the assertion holds under the hard-prune just as under the flag.
- `goal.md` updated (Status + Revision section; old Fix direction/Approach marked
  REJECTED/SUPERSEDED but kept for defect history). Memory file + MEMORY.md index
  updated next.

## 2026-08-08 (phase 2 — behavior-preserving API refactor)

- Phase 2 LANDED (behavior-preserving; 17/17 ctest green; bit-identical to HEAD
  for no-`is_important` tasksets). No control-flow change on `schedulable` yet.
- `OptimizeSP_Incre.h`: new `PriorityOptResult` struct
  (`{PriorityVec priority_vec; double sp_opt; bool schedulable;}`, mirrors
  `ResourceOptResult` @ `OptimizeSP_Base.h:80`). Removed the REJECTED
  `PriorityPartialPath::has_unschedulable_important` field. Changed return types
  of `OptimizeFromScratch`/`OptimizeIncre`/`OptimizeIncre_SingleTask` and the two
  no-caller wrappers `PerformOptimizePA_Incre` /
  `PerformOptimizePA_Incre_w_TimeLimits` (`OptimizeSP_TL_Incre.h:486`) from
  `PriorityVec` → `PriorityOptResult`.
- `OptimizeSP_Incre.cpp`: reverted `UpdateSP` to HEAD (flag-set block removed);
  reverted `OptimizeFromScratch` top-K to HEAD's plain greedy
  (`while (size < K && !empty)`) — the flagged/unflagged split is gone. Added an
  anon-namespace `ComputeSchedulable(dag, sp, pa)` that early-returns `true` (no
  RTA, vacuous) when no task is `is_important`, else calls the 4-arg
  `ImportantTasksMeetThresholds(dag, sp, pa, all-(-1) tl)`. Each of the three
  functions returns `{opt_pa_, opt_sp_, ComputeSchedulable(...)}`. `schedulable`
  is COMPUTED but NOT branched on — phase 3 wires the fallback.
- Call sites: production callers in `OptimizeSP_TL_Incre.cpp` (`:278/:285/:362`)
  DISCARD the return (use `opt_sp_` member) → unchanged; phase 3 will consume
  `.schedulable` there. Test call sites in `testOptimizeIncrePA.cpp` (~20) +
  `testOptimizePA.cpp` (1) unpack `.priority_vec` where they held the bare
  `PriorityVec`; the differential bit-identity tests (oracle vs cache arms) stay
  green. `OptimizeSP_TL_Incre.cpp` itself is UNTOUCHED this phase.
- P1.29 test rewritten to PHASE-2 semantics: the beam still adopts the SP-max
  0.8 leaf (no prune yet) → asserts `res.sp_opt≈0.8` AND `res.schedulable==false`
  (the signal correctly reports the unschedulable important task on the SP-max
  leaf). This proves the signal is computed + propagated — the foundation phase 3
  builds on. Phase 3 (hard-prune + reopt fall-back) will re-assert ADOPTION of
  the 0.2 schedulable leaf.
- Decision: hard-prune at push time DEFERRED to phase 3 (keeps phase 2 truly
  bit-identical to HEAD — the prune changes beam shape when `is_important` tasks
  exist). `ComputeSchedulable` here is the RESULT-level signal (fresh RTA via
  `ImportantTasksMeetThresholds`); phase 3 replaces the per-candidate path with
  the reused-`UpdateSP`-RTA prune (no extra RTA) and routes `schedulable=false`
  to the end-of-reopt fall-back.
- NOT committed (`git add` only). Awaits user review.

## 2026-08-08 (phase 3 — hard-prune + empty-vec contract; fall-back wiring DECLINED)

- Phase 3 LANDED (17/17 ctest green). Hard-prune at push time + the
  `schedulable=false`-with-empty-`priority_vec` return contract. The explicit
  "wire `schedulable` through `CallOptimizerGivenTimeLimits` →
  `SeedBaselineAndArmCache` → `ReOptimizePeriodic`" step was DECLINED by the
  user (see below) — the priority optimizer does NOT itself consider fall-back.
- User directive (2026-08-08, settling the phase-3 design fork): "this priority
  optimization doesn't need to consider fallback, if unschedulable, return
  unschedulable with an empty priority assignment vector." I.e. the priority
  optimizer's CONTRACT is: schedulable → `{priority_vec=<plan>, sp_opt, true}`;
  unschedulable → `{priority_vec={}, sp_opt=INT_MIN, false}`. Fall-back is the
  reopt layer's existing backstop's job (`AdoptFallbackIfUnschedulable`
  @ `OptimizeSP_TL_Incre.cpp:784`), NOT the priority optimizer's. No new
  fall-back/short-circuit logic anywhere.
- `OptimizeSP_Incre.cpp`:
  - Removed the phase-2 `ComputeSchedulable` anon-namespace helper (result-level
    fresh-RTA signal) — superseded by the per-candidate reused-RTA prune.
  - `PriorityPartialPath::UpdateSP`: returns `bool`. After computing the decided
    task's FINAL `rta_curr` (HP set = still-unassigned same-proc tasks = all
    future higher-pri interferers; Audsley assigns lowest-pri first and
    `AssignAndUpdateSP` erases `task_id` from `tasks_to_assign` BEFORE
    `UpdateSP`), if `is_important` &&
    `GetDDL_MissProbability(rta_curr, deadline) > thresholds_node[task_id]` →
    `return false` (the hard-prune signal). NO extra RTA — reuses the RTA
    already computed for SP-loss. The `BFSharedBudgetCancelled()` early-return
    returns `true` ("descent discarded — don't prune"). Mirrors
    `ImportantTasksMeetThresholds`' predicate exactly. Vacuous (never false)
    when no task is `is_important` → legacy bit-identical.
  - `AssignAndUpdateSP`: returns the `UpdateSP` bool.
  - `OptimizeFromScratch` push site: `if (new_path.AssignAndUpdateSP(task_id))
    { pq.push(new_path); ... }` — skips the push for an unschedulable
    important-task candidate. Top-K stays HEAD's plain greedy
    (`while (size < K && !empty)`); `if (partial_paths.empty()) break;` after
    extraction. After the loop: emptied beam → `opt_pa_.clear(); opt_sp_ =
    INT_MIN; return {PriorityVec{}, opt_sp_, /*schedulable=*/false};`.
    Non-empty leaf → `return {res, opt_sp_, /*schedulable=*/true};`.
    `OptimizeIncre`/`OptimizeIncre_SingleTask` → `return {opt_pa_, opt_sp_,
    /*schedulable=*/true};` (their schedulability is enforced by the ARMED
    during-walk gate + backstop, so the committed result is always schedulable).
- Empty-beam flow VERIFIED SAFE (no fall-back wiring needed): when the beam
  empties, `CallOptimizerGivenTimeLimits`'s `UpdateRecords` sees
  `opt_sp_=INT_MIN` → `WouldBeatIncumbent=false` → NO commit → `res_opt_`
  KEEPS the pre-reopt incumbent (re-scored by `ResetIncumbentBaseline(true)`).
  `SeedBaselineAndArmCache`'s champion adoption (`:553-557`) reads the MEMBER
  `opt_pa_` + `ReconstructTimeLimitVecFromResOpt()` (= pre-reopt incumbent,
  VALID) — NOT the local emptied `optimizer.opt_pa_` — so the cache arms off a
  valid incumbent. The `INT_MIN` returned as `current_config_sp` flows into
  `WalkSerializedTaskQueue` only as the `best_sp` baseline in
  `WalkOneTaskWithTimeLimitOptions` (`:664`), where it just means "first real
  eval always improves" (benign — re-evals the incumbent, a no-op commit; the
  real committed state is `res_opt_`). The walk may rescue schedulability via TL
  relaxation; the existing backstop `AdoptFallbackIfUnschedulable`@`:784` guards
  the final. `OptimizeSP_TL_Incre.cpp` is UNTOUCHED this phase.
- Tests (`testOptimizeIncrePA.cpp`): the phase-2 test rewritten back to the
  HARD-PRUNE semantics. `InSearchGate_AdoptsSchedulableLeafOverUnschedulableSpMax`:
  the prune drops the 0.8 SP-max leaf (task 0 low-pri → important task 0
  unsched) and adopts the 0.2 schedulable leaf (task 0 high-pri) → asserts
  `EXPECT_NEAR(res.sp_opt, 0.2, 1e-6)` + `EXPECT_TRUE(res.schedulable)`.
  NEW `InSearchGate_EmptyBeamReportsUnschedulable`: both tasks important
  (point-mass RTA 15 > both ddls) → every order pruned → beam empties → asserts
  `EXPECT_FALSE(res.schedulable)` + `EXPECT_TRUE(res.priority_vec.empty())`
  (the empty-vec half of the contract). Non-vacuous: the 0.2 assertion only
  passes because the prune dropped the 0.8 leaf.
- P1.29 COMPLETE (per the user's final contract). Does NOT address P1.28's
  21/22 incremental collapses (by design). NOT committed (`git add` only).

## 2026-08-08 (follow-up refinements)

- `UpdateSP` budget-timeout return: the `if (BFSharedBudgetCancelled())` branch
  returned `true` ("don't prune"). Changed to `return false`. Rationale: on
  timeout the RTA + important-task gate below it never run, so a half-evaluated
  candidate would be pushed with under-counted `sp_lost` AND its important-task
  schedulability never checked (the P1.29 defect resurfacing on timeout).
  Returning false skips the push; a beam that empties (all candidates timed out)
  returns the unschedulable contract (`{PriorityVec{}, INT_MIN, false}`) and the
  reopt backstop handles it. Confirmed the from-scratch reopt beam DOES run
  inside a `BFDLSharedBudget` scope (established at
  `Optimize_w_TL_ScratchOrIncre:766`, wrapping `ReOptimizePeriodic` ->
  `RunIntervalDescent(Reopt)` -> `SeedBaselineAndArmCache` ->
  `CallOptimizerGivenTimeLimits(from_scratch=true)` -> `OptimizeFromScratch`),
  so this branch is live, not inert, on the reopt path.
- TDD: added `InSearchGate_BudgetTimeoutEmptiesBeam` — wraps the call in a
  `BFDLSharedBudget` scope with `TIME_LIMIT=0` (saved/restored) so
  `BFSharedBudgetCancelled()` is true at the first node; asserts
  `schedulable==false` + `priority_vec.empty()`. Non-vacuous: under the old
  `return true` the beam would NOT empty and both asserts fail.
- `OptimizeFromScratch`: `partial_paths.reserve(K)` -> `partial_paths.reserve(K *
  N)` (N = number of tasks in the task set; the member `N`).
- Comment minimization: trimmed the P1.29-tagged comment blocks in
  `OptimizeSP_Incre.{h,cpp}` + `testOptimizeIncrePA.cpp` (struct doc, `UpdateSP`
  gate, push site, empty-break, emptied-beam block, test fixtures) to <=3-line
  comments, dropped the "P1.29"/"Phase 3" tags. Behavior unchanged.
- 17/17 ctest PASS (incl. the new timeout test). NOT committed (`git add` only).

## 2026-08-08 — BF optimization follow-up (separate commit; git add-only; 17/17 ctest)

The earlier "apply the same three updates to BF" follow-up note was **wrong
about BF's structure**. Investigation before editing:

- **Item (1) budget-timeout prune — N/A.** BF has no INCR-style partial-path
  defect. `EvaluateSPWithPriorityVec` (`OptimizeSP_Base.cpp:181-215`) returns
  `INT_MIN` on ANY budget interruption — both on entry (skip the expensive
  `ObtainSP_DAG`) and post-call (discard `ObtainSP_DAG`'s partial result). So an
  interrupted/half-evaluated permutation loses to the incumbent
  (`INT_MIN > opt_sp_` is false) and is never adopted. BF only ever commits
  COMPLETE PAs (one per leaf of `IterateAllPAs`), and the P1.27 in-search gate
  runs `ImportantTasksMeetThresholds` on every adopted `res_cur`. The `return;`
  at the recursion entries (`IterateAllPAs` + `Optimize()`) already prunes
  subtrees on cancel. No ungated half-evaluated leaf can be committed.
- **Item (3) `reserve(K*N)` — N/A.** BF has no beam (no `K`); `IterateAllPAs`
  recurses over the N! permutations. The only `reserve` is
  `time_limit_option_for_each_task.reserve(N)` (`OptimizeSP_TL_BF.cpp:22`),
  already correct.
- **Item (2) minimize P1.27 comments — marginal, skipped.** The P1.27 comment
  is already <=2 lines; the longer comment blocks in `OptimizeSP_TL_BF.cpp` are
  P1.14/P0.10-tagged (different tasks), not P1.27.

**The REAL BF analogue** (user reframed the follow-up 2026-08-08: "during BF
searching it tries each possible solution; add a schedulability check when a
candidate outperforms the current best; for BF don't worry about implementation
efficiency") = an **in-search per-candidate gate in the PA enumeration**:
`OptimizePA_BF::IterateAllPAs` (`OptimizeSP_BF.cpp`) currently adopts a PA on
`if (sp_eval > opt_sp_)` alone — SP-max, no sched check. The P1.27 TL-level leaf
gate (`OptimizeSP_TL_BF.cpp:52-59`) only gates the ONE SP-max PA per TL combo
(`res_cur`), so a TL combo whose SP-max PA is unschedulable contributes NOTHING
even if it contains a lower-SP schedulable PA. Pushing the gate DOWN into
`IterateAllPAs` makes each TL combo surface its best SCHEDULABLE PA.

- `OptimizeSP_BF.cpp` `IterateAllPAs`: `if (sp_eval > opt_sp_)` ->
  `if (sp_eval > opt_sp_ && ImportantTasksMeetThresholds(dag_tasks_,
  sp_parameters_, priority_assignment))`. `dag_tasks_` already has TLs baked
  (`UpdateExtDistBasedOnTimeLimit` at the TL-recursion leaf), so the no-tl
  overload is correct. Bit-identical when no task `is_important` (overload
  vacuously true -> `sp_eval > opt_sp_` unchanged). Seed floor preserved
  (`opt_sp_=initial_sp`, `opt_pa_=GetPriorityAssignments`): when a TL combo has
  NO schedulable PA, `IterateAllPAs` returns the (possibly unschedulable) seed;
  the P1.27 TL-level gate then rejects it -> backstop. So `OptimizeSP_TL_BF.cpp`
  is UNTOUCHED (P1.27 TL-level gate KEPT — still needed for that fall-through).
- New 3-arg `ImportantTasksMeetThresholds(dag, sp, pa)` overload
  (`SP_Metric.{h,cpp}`) for a dag whose TLs are ALREADY baked into the ET dists
  (mirrors `EvaluateSPWithPriorityVec`, which also takes a baked dag + PA, no
  `tl`). Delegates to the 4-arg overload via `tl=-1`:
  `ApplyTimeLimitsToTasksExecutionTime` is a no-op when every `tl[i]==-1`
  (it only re-bakes where `tl!=-1`), so the baked dists are preserved. The -1
  trick is localized + documented in the overload. No `OptimizePA_BF` ctor /
  `OptimizePA_BruteForce` signature changes (it has standalone no-TL test
  callers), no test-caller breakage.
- TDD: `TaskSetForTest_p129_bf_unschedulable_important` /
  `InSearchGate_AdoptsSchedulablePAOverUnschedulableSpMax` in
  `tests/testOptimizePA.cpp` — same point-mass fixture as the INCR P1.29 test
  (task 0 IMPORTANT ET=5 ddl=6 w=0.2; task 1 ET=10 ddl=12 w=0.8; both proc 0).
  PA `{1,0}` = SP 0.8 but important task 0 RTA 15 > ddl 6 -> unsched; PA `{0,1}`
  = SP 0.2 sched. RED before gate: BF returns `{1,0}` sp 0.8 (Initial SP 0.2 ->
  Optimal SP 0.8, `ImportantTasksMeetThresholds`=false). GREEN after: returns
  `{0,1}` sp 0.2 + `ImportantTasksMeetThresholds`=true. 17/17 ctest PASS.
- NOT committed (`git add` only). Distinct commit from the INCR P1.29 work.

## 2026-08-08 (refactor — remove the `tl=-1` trick from the gate overloads)

- User review flagged the 3-arg `ImportantTasksMeetThresholds(dag, sp, pa)`
  overload as wrong-by-design: it delegated to the 4-arg overload via a synthetic
  `tl_noop = [-1,...]` vector so the 4-arg's `ApplyTimeLimitsToTasksExecutionTime`
  became a no-op. Unintuitive — the `tl=-1` sentinel is an implementation trick
  leaking into the call graph, not a real TL.
- Refactor (`SP_Metric.{h,cpp}`): extracted the gate's per-task check into a
  single anonymous-namespace core `ImportantTasksBelowThresholds(tasks_prioritized,
  sp, node_rtas)` — takes ALREADY-prioritized tasks + index-aligned RTAs (no `tl`,
  no baking). All three public overloads now reduce to it:
  - 5-arg contract: bake + prioritize + core (caller's precomputed RTAs).
  - 4-arg: bake + prioritize + fresh RTA + core.
  - 3-arg (already-baked dag): prioritize + fresh RTA + core — **no `tl=-1`**;
    directly prioritizes `dag_tasks.tasks` (TLs already baked).
- Behavior-identical: the 3-arg previously baked with `tl=-1` (a no-op →
  `tasks_baked == dag_tasks.tasks`) then prioritized + fresh RTA + check; the new
  3-arg does prioritize + fresh RTA + check on `dag_tasks.tasks` — same ops, same
  result. 17/17 ctest PASS (incl. the BF P1.29 test that exercises the 3-arg
  overload). Header comment on the 3-arg updated (dropped the `tl=-1` rationale).
- NOT committed (`git add` only). Same commit as the BF follow-up (same file).
