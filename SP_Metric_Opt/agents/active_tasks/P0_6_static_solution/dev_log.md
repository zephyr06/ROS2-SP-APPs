# P0.6 Safe Fallback — Dev Log

> Detailed working log. Append chronological entries below. On completion, append a one-line
> milestone to the top-level `agents/dev_log.md`. Algorithm + soundness in `goal.md`;
> checklist in `tasks.md`. **COMPLETE + COMMITTED `630cda4d` (2026-07-31): sections 0.5–10
> LANDED; 17/17 ctest green.**

## 2026-07-31 — worst-case-DAG redesign (cross-interval safety for P0.7 trigger (a))

**Trigger.** P0.7 review found trigger (a) (ET-jump swap-in of `safe_fallback_`) UNSAFE under the
2026-07-30 design: it certified `safe_fallback_` against the *single taskset* `ComputeSafeFallback`
ran on, but trigger (a) swaps it in on a *different* (jumped) interval. The old "safe at WCET
across all intervals" argument had been dropped with old-D2 (global-max WCET).

**Two code facts (verified):**
1. `ComputeSafeFallback` forces `use_wcet_execution_time=false` (`OptimizeSP_TL_Incre.cpp:893`)
   → env tasks (TL=−1) keep their **base Gaussian** (NOT a WCET point mass) during the compute.
2. The generator's per-interval `Et_sigma=np.std(subset)` (`orchestrator.py:375`; `=Et_mean*
   uniform(0.5,0.6)` at `taskset_generator.py:257-258`) is **mean-independent** → "longest by avg
   ET" does NOT bound `ddl_miss_chance`.

**User direction (verbatim):** caller iterates all interval YAMLs, records each task's "longest"
ET, constructs a DAG where each task uses its longest, runs `ComputeSafeFallback` on it → safe
across all intervals; if no safe fallback can be found → "fail loudly, ask user to re-generate."

**Refinement (soundness).** The user's "longest by avg ET" is NOT sound (sigma mean-independent
→ not stochastic dominance), but the *intuition* ("WCET of all possible combination") IS. Resolved
as: per task, take the **point mass at `max(execution_time_max)` across all interval YAMLs**
(env/non-perf; perf tasks already point-masses at TL, bounded by `min(et_mean,TL)`). Any interval's
ET draw ≤ its `max_time` ≤ worst-case max → **stochastic dominance** → the gate's `ddl_miss_chance`
on the worst-case DAG upper-bounds every interval → trigger (a) sound. **D9 RESOLVED.**

**Loud-fail:** post-walk gate re-check on the FINAL stored result (not just the seed); on failure
→ raise loud, do NOT store (`HasSafeFallback()` stays false). Raising TL worsens interference, so
a seed-level miss is not walk-fixable; the check also (cheaply) catches "drift to infeasible"
(can't happen — the gate only rejects — but it's cheap insurance).

**Scope:** worst-case DAG fed to OFFLINE `ComputeSafeFallback` only. Online sim + optimizer keep
the actual per-interval DAGs → online byte-identical. NOT the old `use_wcet_execution_time` global
flag (which collapsed the online path too).

**Status:** design + docs LANDED. **NOT yet implemented (steps 8a–8d open).** Next: implement the
worst-case-DAG builder (orchestrator) + wire into the pre-call + the loud-fail check, TDD.

## Load-bearing findings (LANDED — compressed; full detail in prior sessions)

- **Timeout-regression fix (step 5a).** `ComputeSafeFallback` installs its OWN `BFDLSharedBudget`
  around the WHOLE compute (seed eval + walk); without it the lazy dispatcher ran the compute
  before the dispatcher's budget was installed → runaway seed SP-eval (`testINCRTimeout` 21.8s →
  fixed 17/17). On cancel, `EvaluateSPWithPriorityVec` returns `INT_MIN` → walk keeps the
  incumbent (compare-and-keep) → stored result stays gate-feasible.
- **Gate hook-point (D9 grounding).** The gate cannot hook at the walk's `IsBetterTimeLimitOption`
  best-tracking — adoption is a side-effect of `eval`→`OptimizeIncreSingleTask`→`UpdateRecords`→
  `CommitIncumbent`, which runs BEFORE the best-tracking. The gate sits at `UpdateRecords` (the
  commit chokepoint), fires ONLY on would-beat. Type-E queue is EMPTY offline (one static DAG).
- **Step 4b v2 (FINAL).** Gate INSIDE `UpdateRecords` via `WouldBeatIncumbent` +
  `enforce_important_task_gate_` (true only in tests/`ComputeSafeFallback` → prod byte-identical).
  Gate-reject → return false (no `CommitIncumbent`); eval reports incumbent SP (ghost-SP override).
  PA descent RE-ENABLED unconditionally. RTA source = re-runs `CommitIncumbent`'s
  `rta_cache_.Evaluate(dag_tasks_,opt_pa_,tl)` (candidate's FINAL {pa,tl}); `|diff|<=1`-cheap,
  never mutates champion → `cache_backup` revert sound. Zero extra eval flag-off. Seed TL ≤ et_mean
  → `ddl_miss_chance=0` → gate can only REJECT. TDD (5 tests); 17/17.
- **Unification DEFERRED (user 2026-07-30).** The ghost-SP override is a symptom of 3 divergent
  acceptance predicates (PA-loop local; TL-walk local; commit-global). Unifying them is LARGE
  (breaks `StubTLWalkOptimizer` walk-test framework; NOT byte-identical flag-off; override already
  correct = cleanliness not bugfix). User accepted the elegance tradeoff. 4b v2 is FINAL. Re-file P2.x.
- **Throwaway-sibling isolation (step 5a).** `ComputeSafeFallback` runs the walk on a throwaway
  `fallback_solver` (same class) so `this`'s live `res_opt_`/cache/flag are UNTOUCHED → online
  byte-identical (else interval-0 reopt's `IfInitialized()` would warm-start from the artifact =
  P0.7 leaking into P0.6). Forces `use_wcet_execution_time=false` (gate is probabilistic).
- **ET-exclusion (step 5b).** STRUCTURAL (disjoint call sites: `safe_fallback_compute_time_s_`
  writes ONLY at `RunSimulation:316-317`; `scheduler_exec_time_s_` writes ONLY at
  `DeterminePrioritiesAndBudgets:429-430`) + ASSERTED (`PreComputesSafeFallback_ExcludesSchedulerET`
  — DM-mode `==0.0` exact, non-flaky). 17/17.
- **Two safety assumptions VERIFIED.** (1) Sim perf ET = `min(et_mean,TL)` downward cap
  (`SimulationOrchestrator.cpp:514-516`) → metric `ddl_miss_chance` ≥ sim → gate sound. (2) Gate IS
  the constraint → group-lock concern moot (PA descent under the gate; higher-SP PA still passing
  is strictly better). 5 `GateWiring_*` tests pass w/ PA descent on.
- **Cross-interval safety leg (step 8d, 2026-07-31).** EXTENDS the two assumptions to the
  cross-interval case P0.7 trigger (a) needs: (3) the worst-case point mass stochastically
  dominates every interval's per-task dist — for every interval j, every task i,
  `worst.tasks[i].max_time >= interval[j].tasks[i].max_time` (any interval's ET draw ≤ its
  `max_time` ≤ the worst-case max) → the gate's `ddl_miss_chance` on the worst-case DAG
  upper-bounds every interval → trigger (a)'s swap-in is safe by construction. Grounded by
  `BuildWorstCaseDag.StochasticallyDominatesEveryInterval` (§8d.1) +
  `ComputeSafeFallback_ReGatesCleanAgainstEveryInterval` (§8d.2, re-gates clean against EACH
  interval's DAG individually). 17/17 green.

## Chronology (one line per date; full narrative in prior-session commits/records)

- **2026-07-26** — task scaffolded from the fall-back design direction; D1 (important-task rule)
  flagged as the shared blocker (P0.6/P0.7/P0.8). Code grounding: RM sort, INCR_WCET path,
  offline compute site between optimizer construction and the interval loop; ET bracket at
  `DeterminePrioritiesAndBudgets` (static solution stays OUTSIDE).
- **2026-07-27** — **D1 RESOLVED:** top-50% by `sp_weight`, persisted `bool is_important`
  (generator-set, YAML-emitted, read by all consumers → no drift); tie-break by task id. Budget
  asymmetry (user): offline skip-and-continue (find best-SP-safe), online HALT = P0.7 trigger (b).
  D2 (global-max WCET across intervals), D3 (per-taskset), D4 (in-memory), D5/D6 (virtual
  `ShouldAdoptCandidate` hook, RTA form), D7 (offline skip-and-continue) locked.
- **2026-07-28** — P0.9 LANDED the seed PA system-wide (RM→DM, group-locked
  `DeadlineMonotonicPriorityVec` + `SeedIncumbentFromDMFast`); plan docs relabeled RM→DM.
- **2026-07-30** — **REDESIGN:** seed at the P0.8-certified operating point (largest grid option
  ≤ et_mean) + normal optimization, KEEP the result; WCET-mode flip + global-max D2 DROPPED
  (D8 open: does from-scratch reorder PA?). Then the user's objective ("best perf s.t. all
  important tasks' DDL miss chance ≤ SP thresholds") REVIVED the filter as a HARD probabilistic
  gate (D8 RESOLVED via the gate). LANDED: step 3 (`FindLargestTimeLimitAtOrBelow` TL seed, 7
  tests), step 4a (`ImportantTasksMeetThresholds`, zero-extra-eval RTA pass-through, 4 tests),
  step 4b v2 (gate at `UpdateRecords` + ghost-SP override + PA descent on, 5 tests) — `500665d5`.
  4b-followup unification DEFERRED (user). All 17/17.
- **2026-07-31** — LANDED step 5a (`ComputeSafeFallback` + `safe_fallback_` + dispatcher lazy-hook
  + throwaway-sibling isolation + timeout-regression fix, 4 tests) + step 5b (orchestrator pre-call
  + separate compute-time profile + ET-exclusion guard, 4 assertions) + section 7 post-review
  refinement (rename `static_solution`→`SafeFallback` + `sub`→`fallback_solver` + comment trim;
  item 3 no-op; item 5 reopt-then-incre SKIPPED per user — gate-vs-reopt-cache incompatible).
  Then the worst-case-DAG redesign (above) — design+docs LANDED, steps 8a–8d open. All 17/17,
  git add-only — NOT committed.
- **2026-07-31 (cont.) — §8 COMPLETE.** Steps 8a (worst-case-DAG builder, 3 tests) + 8b (wire into
  `ComputeSafeFallback`, signature takes the worst-case DAG; 1 test) + 8c (loud-fail post-walk
  re-gate, raises + unstored on unschedulable; 2 tests) LANDED earlier this date. Step 8d — the
  cross-interval soundness leg — VERIFIED this session: §8d.1
  `BuildWorstCaseDag.StochasticallyDominatesEveryInterval` (worst-case point mass dominates every
  interval's per-task dist) + §8d.2 `ComputeSafeFallback_ReGatesCleanAgainstEveryInterval`
  (fallback re-gates clean against EACH interval's DAG). Both pass; 17/17 ctest green. The two
  safety assumptions in `goal.md` "Done when" + this file extended with the cross-interval leg
  (assumption 3). Sections 0.5–8 LANDED, git add-only — NOT committed; awaits user commit.
  P0.6 PRODUCES the artifact end-to-end; P0.7 wires the fall-back USE.
- **2026-07-31 (cont. 2) — §9: dispatcher throw + gate overload.** Two user-requested
  tightenings of the §8 contract. **(9a)** The dispatcher's lazy backstop
  (`Optimize_w_TL_ScratchOrIncre` → `ComputeSafeFallback(dag_tasks_)` on first dispatch
  if not pre-called) was UNSOUND: `dag_tasks_` is one interval, NOT the worst-case DAG,
  so it would silently re-introduce the cross-interval unsoundness §8 removed — and the
  dispatcher has no `dag_tasks_vecs_`, so it CANNOT build the worst case soundly.
  Replaced with a `throw std::runtime_error` (fail loud; the orchestrator pre-call is
  the ONLY sound caller). Header comment updated (dropped the "1-interval degenerate
  worst-case backstop" text). Test (3) rewritten to assert the throw
  (`Dispatcher_ThrowsWhenNoSafeFallbackPreComputed`); 6 `CounterDispatcherSynthetic`
  routing tests + the `opt_reopt` line + `testINCRTimeout`'s 2 budget-guard tests got a
  `ComputeSafeFallback(dag)` pre-call (single-interval fixtures where `dag` IS the
  degenerate worst case; gate vacuous — no important tasks → stores the seed fast; the
  pre-call runs OUTSIDE the timed window so the budget-guard measurement is unaffected).
  **(9b)** New self-contained overload `ImportantTasksMeetThresholds(dag, sp_params,
  pa, tl)` (no `node_rtas`) in `SP_Metric.{h,cpp}`: bakes TL → applies PA →
  `ProbabilisticRTA_TaskSet` → delegates to the contract overload. Simplifies the §8c
  loud-fail re-gate (dropped the manual bake+prioritize+RTA boilerplate at the call
  site); the contract overload stays the zero-extra-eval path for the in-walk gate.
  `--clean-first` build green (16/17; the 1 failure = pre-existing
  `testPublisher` `PeriodicReleaser.v1` wall-clock flake, passes in isolation,
  unrelated). 6 targeted dispatcher + §8 tests pass by name. Sections 0.5–9 LANDED,
  git add-only — NOT committed; awaits user commit.
- **2026-07-31 (cont. 3) — §8e: readability refactor of the worst-case-DAG
  builder.** `BuildWorstCaseDagAcrossIntervals` had its structural-equality check
  inlined as a convoluted `bool structure_matches` flag (initial-scalar-check +
  a separate `timePerformancePairs` count check + an inner pair loop that could
  flip the flag false, with an `else` fallback) nested two loops deep.
  Extracted it into its own header function `TaskStructureMatches(const Task&,
  const Task&)` (declared in `DAG_Model.h` next to
  `BuildWorstCaseDagAcrossIntervals`, defined in `WorstCaseDAG.cpp`; precedent =
  `FindLargestTimeLimitAtOrBelow` — header-declared so it's directly unit-testable).
  Behavior-preserving: compares the SAME fields (id/period/deadline/processorId/
  name + the full `timePerformancePairs` grid — size + each `time_limit`/
  `performance`); the ET dist is DELIBERATELY excluded (the builder fuses the max
  across intervals — differing dists are the whole point). The builder's
  per-task loop collapses to a single `if (!TaskStructureMatches(...)) throw`.
  TDD-first: 5 new `TaskStructureMatches.*` tests (identical match; differing ET
  dist still matches; each scalar-field mismatch; TL-pair count mismatch;
  TL-pair value mismatch) written BEFORE the helper existed (red), then green.
  Doc fix: the `BuildWorstCaseDagAcrossIntervals` header comment listed
  `is_important` among the compared fields, but the predicate never compared it
  (`is_important` is generator-set once on the base taskset → stable across
  intervals, so the gap is latent, not a live bug) — corrected the comment to
  match the actual field set. `--clean-first` build green **17/17** (the
  `testPublisher` flake passed this run). NOT committed; awaits user commit.
- **2026-07-31 (cont. 4) — COMMIT.** User committed all of §0.5–10 (code + records) as
  `630cda4d` ("add code to build dag with WCET from multiple interval files, and use it in
  optimization"). Verified post-commit: `git status` clean for P0.6 paths; `630cda4d`
  contains `BuildWorstCaseDagAcrossIntervals`/`WorstCaseDAG.cpp` (§8a), the dispatcher
  `throw` + `ImportantTasksMeetThresholds` self-contained overload (§9), and
  `TaskStructureMatches` (§10). `cmake --build build_test --target check.SP_OPT -j5` =
  **17/17 green** (re-verified on the committed tree, incl. the `testPublisher` flake
  passing). Records + memory refreshed to drop the stale "NOT committed / awaits user
  commit" framing. **P0.6 COMPLETE.** P0.6 PRODUCES the artifact end-to-end; P0.7 wires
  the fall-back USE.
