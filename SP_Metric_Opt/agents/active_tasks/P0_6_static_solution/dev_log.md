# P0.6 Static Solution — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-30 (step 4b LANDED: gate wired into the TL walk's adoption, TDD)

- **User direction (verbatim, this session):** "one issue:
  ImportantTasksMeetThresholds, maybe also pass rta inside, as i believe the
  caller should already somehow obtained rta, so no need to re-calculate it
  again, it's expensive." Verified the user is ALREADY served: commit
  `ad356bc3 pass rta to ImportantTasksMeetThresholds` (today) changed the
  predicate's signature to `...(dag, sp_params, pa, tl, node_rtas)` and dropped
  the `ProbabilisticRTA_TaskSet` re-derivation — the body reads
  `GetDDL_MissProbability(node_rtas[i], deadline)` off the caller's already-
  materialized RTAs (zero-extra-eval, the step-4a revision). This message is the
  same direction, restated; no code change needed for the predicate itself.
- **The in-flight work this session = step 4b** (wiring the gate into the TL
  walk's adoption — `enforce_important_task_gate_` + `WouldBeatIncumbent`).
  Verified the working tree already contained a complete, green implementation
  (built fresh): `cmake --build build_test --target check.SP_OPT -j5` = **17/17
  green**, including the 5 new `GateWiring_*` tests (filter run confirms all 5
  ran + passed).
- **What step 4b adds (3 pieces, all in `OptimizeSP_TL_Incre.{h,cpp}` + tests):**
  1. **`WouldBeatIncumbent(challenger_sp, time_limits) -> bool`** (member, `const`):
     the exact "would this candidate beat the incumbent?" predicate `UpdateRecords`
     commits on — strictly-greater SP, OR an approx-equal SP tie with a strictly-
     smaller total TL (tie-break prefers the tighter budget). Extracted from the
     prior inline logic in `UpdateRecords` (`:129-157`), which now delegates to it.
     Behavior-identical refactor (the `UpdateRecords` path is byte-identical).
  2. **`bool enforce_important_task_gate_ = false`** (member): the offline-only
     gate flag. When true, `OptimizeIncreSingleTask` (the eval the Type-L walk
     calls per trial TL) gates ADOPTION: a would-beat candidate is committed only
     if `ImportantTasksMeetThresholds` passes. Default false → prod byte-identical.
  3. **The gate block** (`OptimizeIncreSingleTask`, after the baseline SP eval):
     fires ONLY on would-beat (the user's rule: "checked whenever we make
     progress from the champion; if challenger doesn't beat champion, we don't do
     the check"). On a gate-REJECT of an SP-better candidate: revert the cache
     (`rta_cache_ = cache_backup`) + return the INCUMBENT SP (NOT the rejected
     candidate's better SP) → the walk's `IsBetterTimeLimitOption` sees "no
     progress" and never tracks the rejected TL into the working vector (`:591`
     ghost-SP fix). The incumbent itself is never mutated (no `CommitIncumbent`)
     → `res_opt_` TL/SP/PA stay at the pre-candidate values.
- **Two soundness guards the gate block relies on (both verified):**
  - **PA descent is SKIPPED when the gate is on** (`OptimizeIncreSingleTask`'s
    `if (!BFSharedBudgetCancelled() && !enforce_important_task_gate_)` around
    `challenger.OptimizeIncre_SingleTask`). Two reasons it MUST be skipped under
    the gate (the D8 v1 resolution, made concrete): (1) the gate runs on
    `baseline_rtas` — the RTA just scored at `:250` under the candidate TL +
    carried PA; PA descent reorders `challenger.opt_pa_`, after which
    `baseline_rtas` is no longer the RTA for `{opt_pa_, time_limits}`. Skipping
    descent keeps `challenger.opt_pa_` == carried PA → `baseline_rtas` stays the
    valid gate RTA → zero extra RTA eval (matches step-4a's directive). (2) PA
    descent could break the P0.8 group lock the seed certifies; a TL-only walk
    preserves it by construction. The gate checks per-task `ddl_miss_chance`, NOT
    the group lock, so it cannot substitute for "no cross-group moves".
  - **The flag is set true ONLY in tests** (`grep`-verified: no prod site writes
    `enforce_important_task_gate_` — `ComputeStaticSolution` does not exist yet).
    So the online arms' PA descent + commit path is untouched → prod byte-identical.
- **Seed-feasibility invariant (why the gate can only REJECT, never make the
  solution infeasible):** the walk seeds at TL ≤ et_mean (`SeedTimeLimitsAtOrBelow
  EtMean`, step 3), where the perf ET point-mass = TL ≤ et_mean = P0.8's perf
  WCET → the whole RTA dist sits ≤ deadline → `ddl_miss_chance = 0` for important
  tasks → the seed is gate-feasible by construction (P0.8's gate also certifies
  it). The gate only fires on would-beat candidates (TL RAISED toward the SP max,
  into the `ddl_miss_chance > 0` region); a rejected candidate leaves the (still-
  feasible) incumbent untouched. So the returned solution is feasible-by-
  construction AND best-SP-among-feasible (the walk kept searching past skipped
  candidates). This is exactly the user's constrained-optimization objective:
  max SP s.t. ∀ important task `ddl_miss_chance ≤ sp_threshold`.
- **TDD:** 5 tests (1 precondition + 4 behaviors), all green:
  (precondition) TL=1000 strictly SP-better than TL=400 on the fixture's perf term
  (guards against a silent fixture drift making the reject test vacuously pass);
  (i) flag OFF (default) → SP-better candidate commits, no gate (byte-identical to
  prod); (ii) flag ON → gate REJECTS an SP-better threshold-violating candidate
  (incumbent TL/SP/PA unchanged; eval returns incumbent SP — the ghost-SP fix);
  (iii) flag ON → gate KEEPS a feasible SP-better candidate (commits — the gate is
  permissive when the constraint holds); (iv) flag ON, non-beating candidate →
  gate does NOT run (the user's would-beat-only rule) + no commit. Fixture: T_perf
  (id 0, NOT important, weight 1.0) drives SP up with TL; T_noise (id 1, important,
  weight 0.01) at lower priority, deadline wedged between the two TLs' response
  times so the gate passes at TL=400 and fails at TL=1000 — the candidate is
  SP-better yet gate-rejected (the constrained-optimization tension the gate exists
  for: raw SP-maximization would accept the cheap miss; the gate enforces the
  constraint regardless of weight).
- **NOT committed** (git add-only per the coding rules; awaits user review). Files
  touched this sub-step: `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` +
  `tests/testIncreOpt_w_TL.cpp`. **This is a self-contained module** (gate wiring,
  no `ComputeStaticSolution` yet) → staged alone for review per "work by module,
  commit by module".
- **Next:** step 5 — `ComputeStaticSolution` wiring: construct the optimizer,
  seed DM-grouped PA (P0.9 `DeadlineMonotonicPriorityVec`) + et_mean-bounded TL
  (step 3 `SeedTimeLimitsAtOrBelowEtMean`), set `enforce_important_task_gate_ =
  true`, run the TL walk (`OptimizeIncre_w_TL` → `RunIntervalDescent`, TL-only →
  group lock preserved), keep the committed `ResourceOptResult` as
  `static_solution_`, then reset the flag. Plus verify the 2 safety assumptions
  (sim perf ET = `min(et_mean,TL)` downward cap; from-scratch path preserves the
  group lock — moot for v1 since the gate pins TL-only). Call site after
  `incr_optimizer_` construction (`SimulationOrchestrator.cpp:300-302`), before
  the interval loop (304-308); OUTSIDE `DeterminePrioritiesAndBudgets`'s ET bracket
  (316-322) so it doesn't inflate the online ET metric.

## 2026-07-30 (step 4a revision — gate predicate takes caller's RTAs; zero-extra-eval)

- **User direction:** "ImportantTasksMeetThresholds, maybe also pass rta inside,
  as i believe the caller should already somehow obtained rta, so no need to
  re-calculate it again, it's expensive." Verified the user is correct.
- **Grounding — the caller ALREADY has the node RTAs at the adoption site.** The
  TL walk's eval (`OptimizeSP_TL_Incre.cpp:238-243`) is:
  ```
  const std::vector<FiniteDist>& baseline_rtas =
      rta_cache_.Evaluate(dag_tasks_cur, challenger.opt_pa_, time_limits);
  challenger.opt_sp_ = ObtainSP_Full_From_NodeRTAs(..., baseline_rtas);
  ```
  So the very place the gate will make its adoption decision ALREADY materializes
  `baseline_rtas`. The prior shape-A predicate re-derived RTAs via
  `ProbabilisticRTA_TaskSet` — duplicating exactly the work the cache just did, on
  every ADOPTED candidate. Wasteful, as the user flagged.
- **Indexing consistency check (the thing that makes passing RTAs SAFE):** the
  cache's `Evaluate` returns `champion_.rta = ProbabilisticRTA_TaskSet(champ_
  prioritized)` (`RTA_Cache.cpp:203`), and `ProbabilisticRTA_TaskSet` writes its
  output `rtas[task_id2index[...]]` where `task_id2index` is positional to the
  INPUT `tasks` (`RTA.cpp:130-146`) = the prioritized set → **cache RTAs are
  positional to the prioritized vector**, i.e. `rtas[i]` ↔ `prioritized[i]`. That
  is EXACTLY the contract `ObtainSP_Full_From_NodeRTAs` already relies on
  (`SP_Metric.h:109-118`: "`node_rtas[i]` MUST be the RTA of the task at position i
  in `UpdateTaskSetPriorities(ApplyTimeLimitsToTasksExecutionTime(...), pa)`").
  (The RTA_Cache.h:68 "indexed by task id" wording is loose; `:203` confirms
  positional-to-prioritized. The gate does the SAME bake+prioritize internally, so
  its indices align with the caller's RTAs as long as the same {pa, tl} is used —
  which it is, the gate evaluates the SAME candidate the caller just scored.)
- **Change (mirrors `ObtainSP_Full_From_NodeRTAs`):** signature
  `ImportantTasksMeetThresholds(dag, sp_params, pa, tl)` →
  `...(dag, sp_params, pa, tl, node_rtas)`. Body drops the
  `ProbabilisticRTA_TaskSet(tasks_prioritized)` call and reads
  `GetDDL_MissProbability(node_rtas[i], deadline)` instead. The bake+prioritize
  stays (needed to know which position is which important task) — identical to
  `ObtainSP_Full_From_NodeRTAs`, which also bakes+prioritizes internally despite
  receiving RTAs. Header comment rewritten to state the CONTRACT (caller passes
  the RTAs it already has; `node_rtas[i]` pairs with `prioritized[i]`).
- **This is the design doc's shape (B)** ("thread per-task miss-chances out of
  `ObtainSP_DAG_From_Dists`, zero extra RTA eval — cheapest"), achieved by
  passing the RTAs through the gate predicate rather than threading miss-chances
  out of the SP eval. Cleaner API, same zero-extra-eval property.
- **TDD:** updated the 4 gate tests to build `node_rtas` via the SAME path the
  cache-path caller uses (apply TLs → apply pa → `ProbabilisticRTA_TaskSet` — the
  oracle the cache is byte-identical to, per the RTACache oracle tests at
  `testIncreOpt_w_TL.cpp:382-386`). Extracted a shared `NodeRTAsForCandidate(dag,
  pa, tl)` helper (was inlined 4×) to reduce repetition. Tests still assert the
  same 4 behaviors (admit below threshold; reject when important task misses;
  ignore non-important misses; vacuously admit when none important).
- **Build:** `cmake --build build_test --target check.SP_OPT -j5` = **17/17 green**
  (4 gate tests + 13 prior, no regressions). Filtered run confirms the 4 gate
  tests ran + passed. No stale `.o` issues (signature change, not header layout).
- **NOT committed** (git add-only per the coding rules; awaits user review). Files
  touched: `sources/Safety_Performance_Metric/SP_Metric.{h,cpp}` + `tests/
  testIncreOpt_w_TL.cpp`. `tasks.md` step 4a + `goal.md` "Done when" updated to
  note the zero-extra-eval RTA-pass-through form.
- **Next:** step 4b — wire the gate into the TL walk's adoption decision (at the
  commit chokepoint, per the D9 grounding — NOT the walk's `:581` best-tracking).
  The wiring can now feed `baseline_rtas` (already in hand at `:238`) straight
  into the gate — no extra RTA eval at the call site either. Then step 5 —
  `ComputeStaticSolution` wiring.

## 2026-07-30 (step 4 grounding — gate hook-point DISCREPANCY found; D9 open)

- **Started step 4** (per-task `ddl_miss_chance` API + the hard-gate hook into the
  TL walk's adoption decision). Read the actual C++ walk/commit architecture
  against the design doc BEFORE writing any code.
- **DISCREPANCY (the doc's stated hook point is WRONG as an adoption control).**
  The plan doc (`goal.md`/`tasks.md`, incl. the "Where it hooks in" reference
  list) says the gate hooks at `WalkOneTaskWithTimeLimitOptions`'s adoption
  decision — `if (IsBetterTimeLimitOption(...))` at `OptimizeSP_TL_Incre.cpp:581`
  (was `:536` when the doc was written). Verified this does NOT control adoption:
  - `WalkOneTaskWithTimeLimitOptions` (`:545-593`, the 1D walk core) calls
    `eval(time_limits)` per trial TL, tracks `best_sp`/`best_option_val`, and at
    the END sets `time_limits[task_idx] = best_option_val`. It does NOT call
    `CommitIncumbent`.
  - `eval` = `OptimizeOneTaskWithTimeLimit`'s closure →
    `OptimizeIncreSingleTask(tl, task_idx, et_up)` (`:192-264`), which calls
    `UpdateRecords(challenger, time_limits)` at `:259` as a SIDE-EFFECT.
    `UpdateRecords` (`:129-157`) calls `CommitIncumbent(...)` (`:154`) when the
    challenger is SP-better → **the candidate commits INSIDE `eval`, BEFORE the
    `:581` best-tracking runs.** A gate at `:581` would observe SP-after-commit
    but cannot prevent the commit — the violating candidate already landed in
    `res_opt_`. To actually gate adoption, the gate must sit at the commit
    chokepoint (`UpdateRecords`/`CommitIncumbent`), OR the eval must be split
    into "score only" (no commit) + a separate gated adopt call.
- **Second finding (scope of "the walk"):** the static solution's TL walk runs
  through `WalkSerializedTaskQueue` (`:368-399`), which dispatches BOTH Type-L
  (TL walk → `OptimizeOneTaskWithTimeLimit` → `WalkOneTaskWithTimeLimitOptions`)
  AND Type-E (env re-search → `OptimizeIncreSingleTask` DIRECTLY at `:387-389`,
  no TL walk). Both commit via `UpdateRecords`. BUT offline (P0.6, pre-loop,
  single static DAG) there is NO interval-to-interval env movement → the Type-E
  queue is EMPTY for the static solution (Type-E needs
  `FindEnvTaskWithDifferentEt(dag_tasks_prev_pre_tl, dag_tasks_)` with TWO
  different DAGs; the static compute uses one). So Type-E is structurally absent
  for P0.6 — the gate only needs to cover Type-L's commit path. (Type-E matters
  for P0.7's ONLINE guard, P0.7's separate trigger (b).)
- **API-shape grounding (doc's (A) vs (B)):** shape (B) "thread per-task miss-
  chances out of `ObtainSP_DAG_From_Dists`" is the zero-extra-RTA-eval path. The
  cache-path SP eval is `ObtainSP_Full_From_NodeRTAs` →
  `ObtainSP_DAG_From_Dists(dag_tasks_eval, ...)` where `dag_tasks_eval.tasks` is
  the PRIORITIZED set (`SP_Metric.cpp:186-198`): `node_rtas[i]` pairs with
  `prioritized[i]`, `dag_tasks_eval.tasks[i].id` = the prioritized task's id.
  Internally `ObtainSP_DAG_From_Dists` already computes per-task miss-chance via
  `ObtainSP` → `GetDDL_MissProbability(node_rts_dists[i], deadline)` (`:14`,`:160`).
  So surfacing it = reading `GetDDL_MissProbability(node_rtas[i],
  dag_tasks_eval.tasks[i].deadline)` for each important task — same data, no
  extra RTA. Thresholds: `sp_parameters_.thresholds_node.at(task_id)` (`:69`,
  `ParametersSP.h:50`). Confirmed APIs: `GetDDL_MissProbability(FiniteDist,ddl)`
  (`RTA.h:31`), `GetUnitExecutionTimeDist(tl)` (`Probability.h:177`),
  `FiniteDist(GaussianDist(mu,sigma),gran)` ctor used in tests, `bool
  Task::is_important` (`RegularTasks.h:111`).
- **NEW open decision D9 (for the user):** WHERE does the gate sit?
  - (i) Inside `UpdateRecords`/`CommitIncumbent` (the real chokepoint) — guards
    every commit (Type-L + Type-E + reopt beam) uniformly, but couples P0.6's
    OFFLINE gate into the SHARED commit path used by ALL online arms → needs an
    offline-only flag `enforce_important_task_gate_` set during
    `ComputeStaticSolution` only (and reset after), or a virtual hook.
  - (ii) P0.6's `ComputeStaticSolution` runs its OWN TL walk with a gated eval
    (NOT the shared `WalkSerializedTaskQueue`/`OptimizeIncreSingleTask`), so the
    gate is local to P0.6 and never touches prod. Cleaner separation; duplicates
    walk scaffolding (or reuses the walk core with a score-only eval variant).
  - Trade-off: (i) is DRY (one gate, one place) but risks touching prod behavior
    if the flag is missed; (ii) is isolated but duplicates the walk.
- **NOT started on code.** Surfacing D9 to the user first (per the coding rules:
  don't make design decisions yourself when the user can decide). Memory
  `p06-gate-hook-point-discrepancy` written; `MEMORY.md` index updated.
- **Next (after D9):** whichever gate site is chosen → TDD the gate predicate
  (rejects an SP-better threshold-violating candidate; keeps a feasible one) →
  then step 5 `ComputeStaticSolution` wiring.

## 2026-07-30 (step 4a LANDED: pure gate predicate ImportantTasksMeetThresholds, TDD)

- **D9 DECIDED by user ("i don't care how you do this").** My choice: a PURE
  free-function gate predicate `ImportantTasksMeetThresholds(dag, sp_params,
  pa, tl) -> bool` in SP_Metric (shape A — re-derives per-task RTAs via
  `ProbabilisticRTA_TaskSet`, one eval per ADOPTED candidate; the zero-extra-
  eval shape B is deferred until the wiring proves the gate's value). This is
  the TDD-able core, touching ONLY SP_Metric.{h,cpp} + tests — NO optimizer or
  prod walk touched. The wiring (a virtual `ShouldCommitCandidate` gate on the
  commit path, or a P0.6-owned gated walk) is step 4b, separate.
- **Predicate mirrors the cache-path SP eval exactly** (bake TLs → apply PA →
  `ProbabilisticRTA_TaskSet` → for each IMPORTANT task `GetDDL_MissProbability
  (rta, deadline) ≤ thresholds_node[id]`). Returns false on the first important
  task whose miss-chance exceeds its threshold; vacuously true when no task is
  important (the constraint is free — the seed-region degenerate case).
  Pessimistic-bound soundness preserved (metric keeps non-perf Gaussian
  variance the sim drops; perf ET = TL = the sim's cap → metric miss-chance ≥
  sim's).
- **TDD red→green:** wrote 4 tests FIRST (RED — `ImportantTasksMeetThresholds
  was not declared`), then implemented → GREEN. Cases: (1) admit when the
  important task's miss-chance ~0 < threshold 0.5; (2) REJECT when the
  important task's deadline is impossibly tight (RT >> deadline → miss-chance
  ~1.0 > 0.5); (3) IGNORE a NON-important task's miss (gate admits despite a
  non-important task missing — the guarantee is scoped to important tasks);
  (4) vacuously admit when NO task is important.
- **Tests:** `cmake --build build_test --target check.SP_OPT -j5` = **17/17
  green** (4 new + 13 prior, no regressions). Filtered run confirms the 4 new
  tests ran + passed.
- **NOT committed** (git add-only per the coding rules; awaits user review).
  Files touched this sub-step: `sources/Safety_Performance_Metric/SP_Metric.{h,
  cpp}` + `tests/testIncreOpt_w_TL.cpp`.
- **Next:** step 4b — wire the gate into the TL walk's adoption decision. Per
  the D9 grounding (the `p06-gate-hook-point-discrepancy` memo): the commit is
  a side-effect of `eval`→`OptimizeIncreSingleTask`→`UpdateRecords`→
  `CommitIncumbent` (`:259`,`:154`), so the gate must sit at the commit
  chokepoint (NOT the walk's `:581` best-tracking, which the doc wrongly named).
  Decision pending: virtual `ShouldCommitCandidate` hook on `UpdateRecords` (a
  guarded reject returns false + leaves `res_opt_` untouched, but the walk's
  `current_config_sp` would carry a ghost SP → needs the eval to report the
  committed SP on reject), OR a P0.6-owned gated walk. Then step 5 —
  `ComputeStaticSolution` wiring.

## 2026-07-30 (step 3 LANDED: et_mean-bounded TL seed helper + vector, TDD)

- **Started on code (first code change of the task).** Verified the design's code
  grounding against current HEAD before implementing — all referenced line numbers
  + API shapes match: `Find_Close_ExecutionTime` (`:42-56`, bidirectional closest),
  `InitializeTimeLimitsFromETConfig` (`:483-498`), `WalkOneTaskWithTimeLimitOptions`
  adoption site (`:536` `IsBetterTimeLimitOption`), `SeedIncumbentFromDMFast` +
  `DeadlineMonotonicPriorityVec` (`:705-780`), `ObtainSP_DAG_From_Dists` /
  `ApplyTimeLimitsToTasksExecutionTime` / `GetDDL_MissProbability` (the metric's
  per-task miss-chance the gate will enforce).
- **Implemented step 3 (the TL seed — the genuinely-new piece):** the directional
  ≤ et_mean variant of the bidirectional `Find_Close_ExecutionTime`.
  - Pure helper `FindLargestTimeLimitAtOrBelow(time_perf_pairs, et_mean)`
    (`OptimizeSP_TL_Incre.cpp`, declared in `OptimizeSP_TL_Incre.h` near
    `FindTimeLimitOptionIndex`): linear scan of the ascending grid; returns the
    index of the LARGEST grid option ≤ et_mean. Falls back to index 0 (smallest)
  + `CoutWarning` when no option ≤ et_mean exists (still feasible: sim caps perf
    runtime ET at `min(et_mean, TL)` ≤ et_mean). Returns 0 on empty grid (caller
    treats empty pairs as non-perf → TL = -1, index never read).
  - Per-task vector method `SeedTimeLimitsAtOrBelowEtMean()` (member, `const`):
    mirrors `InitializeTimeLimitsFromETConfig`'s structure but routes each perf
    task through the at-or-below helper (so no task seeds above the P0.8-certified
    WCET = et_mean); non-perf → -1.
- **TDD red→green (per the coding rules):** wrote 6 helper tests +
  1 vector test FIRST (build RED — `'FindLargestTimeLimitAtOrBelow' was not
  declared`), then implemented → GREEN. Cases: (a) et_mean above largest → largest;
  (b) et_mean below smallest → smallest + warning fires; (c) et_mean between two
  options → largest ≤ et_mean (the directional guarantee — diverges from
  `Find_Close_ExecutionTime`, which at et_mean=700 would tie 600 vs 800); (d) et_mean
  exactly equals an option → that option; (e) et_mean equals smallest → smallest;
  (f) empty grid → 0; (vector) `CompareAndKeepSynthetic` fixture T_perf (Gaussian
  mean 500, grid [400,600,800,1000]) → 400, T_noise (no pairs) → -1.
- **Tests:** `cmake --build build_test --target check.SP_OPT -j5` = **17/17 green**,
  no regressions. (lib = `libSP_OPTDebug.so`, DEBUG build per the standing config.)
- **NOT committed** (git add-only per the coding rules; awaits user review). Files
  touched: `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` + `tests/testIncreOpt_w_TL.cpp`.
- **Next:** step 4 — the per-task `ddl_miss_chance` API (surface what
  `ObtainSP_DAG_From_Dists` already computes) + the hard-gate hook into
  `WalkOneTaskWithTimeLimitOptions`'s adoption decision. Prefer API shape (B) (thread
  per-task values out of `ObtainSP_DAG_From_Dists`, zero extra RTA eval); (A)
  (a separate `ImportantTasksFeasible(pa, tl) -> bool` re-deriving node RTAs) as
  fallback. Then step 5 — `ComputeStaticSolution` wiring.

## 2026-07-30 (OBJECTIVE + constraint mode settled: hard per-candidate gate on probabilistic ddl_miss_chance)

- **User objective (verbatim):** "i need you to find the best possibile performance while
  guaranteeing all the improtant tasks' DDL miss chance is below the SP thresholds."
- **This is a CONSTRAINED optimization:** maximize SP (performance) subject to
  **∀ important task i: `ddl_miss_chance_i ≤ sp_threshold_i`** (a HARD guarantee, not a soft
  preference). It partially REVERSES the "drop the filter" limb of the 2026-07-30 redesign: the
  filter comes back, but in a NEW form (see below) — NOT the old WCET point-mass RTA (old D6).
- **Constraint mode chosen (AskUserQuestion):** **Hard per-candidate gate.** The TL walk adopts
  an SP-better candidate TL ONLY if it keeps every important task's `ddl_miss_chance ≤
  sp_threshold`; otherwise it skips the candidate and continues. Lets important-task TLs roam up
  to the threshold for max perf, then stops. (Alternatives rejected: (b) hard TL-bound on
  important tasks ≤ et_mean (caps important perf TLs — but sim caps their runtime ET at et_mean
  anyway, so exceeding et_mean may give nil sim-level perf gain); (c) soft / trust `SP_Func`'s
  exponential penalty — NOT a strict guarantee, mild violations on low-weight important tasks can
  be SP-optimal.)
- **KEY grounding — `ddl_miss_chance` is PROBABILISTIC, not a point mass** (this changes the
  redesign's "whole grid safe → constraint vacuous" claim):
  - `SP = Σ_i SP_Func(ddl_miss_chance_i, threshold_i) · weight_i · perf_coefficient_i`
    (`SP_Metric.cpp:65-71`), where `ddl_miss_chance_i = GetDDL_MissProbability(rta_dist_i,
    deadline_i)` (`RTA.cpp:154`) = the probability mass of task i's RTA distribution ABOVE its
    deadline.
  - The RTA dist is built by CONVOLVING ET distributions (`GetRTA_OneTask`, `RTA.cpp:32-44`):
    `rta_cur = task.execution_time_dist; for each hp_task: rta_cur.Convolve(hp.execution_time_dist)`.
  - **The metric's ET model (the pessimistic bound the gate must respect):** `ApplyTimeLimitsToTasksExecutionTime`
    (`SP_Metric.cpp:76-86`) sets a perf task's ET dist to a POINT MASS at `time_limits[i]`
    (`GetUnitExecutionTimeDist`); -1 leaves the base (Gaussian) dist untouched. So in the metric,
    perf ET = TL exactly (a delta), and non-perf ET = the full Gaussian (with variance). The sim
    itself runs non-perf ET = `GetAvgValue()` (point mass at mean, `SimulationOrchestrator.cpp:
    426/489/700/752`) and perf runtime ET = `min(et_mean, TL)` (downward cap, P0.8 `7c8748c0`).
    → metric `ddl_miss_chance` ≥ sim's actual miss chance (metric keeps non-perf variance the sim
    drops; metric perf ET = TL = the sim's cap). So a gate on the METRIC is a sound (pessimistic)
    guarantee of the user's stated runtime condition. GOOD.
  - `SP_Func` (`SP_Metric.h:31-41`) is monotone-DECREASING in ddl_miss_chance with an EXPONENTIAL
    penalty past threshold (`PenaltyFunc = -0.01·exp(10·|th−v|)`). So maximizing SP already SOFTLY
    discourages violations — but NOT strictly: a mild violation on a low-weight important task can
    be SP-optimal if the perf gain elsewhere outweighs its penalty → the soft path is NOT a hard
    guarantee → the hard gate is genuinely needed for the user's objective.
- **TWO regions (why "best performance" makes the constraint BINDING):**
  - **TL ≤ et_mean (the seed region):** perf ET point-mass = TL ≤ et_mean = P0.8's perf WCET;
    non-perf ET = the same Gaussian P0.8 certified (max = `execution_time_max` = P0.8's non-perf
    WCET). P0.8's WCET-RTA certification (R_i(WCET) ≤ deadline_i) + interference monotonicity →
    the whole RTA dist sits ≤ deadline → **ddl_miss_chance = 0 for important tasks, AUTOMATICALLY.**
    Here the constraint is FREE (the 2026-07-30 redesign's "whole grid safe" claim holds in this
    sub-region only).
  - **TL > et_mean (raising TL for performance):** the metric's perf point-mass ET exceeds P0.8's
    certified WCET → the RTA dist extends past the deadline → **ddl_miss_chance becomes a real
    nonzero quantity and the constraint BINDS.** "Best possible performance" means the optimizer
    SHOULD push TLs up against the thresholds — into this region — so the gate is load-bearing.
- **Gate is NEAR-FREE (the per-task miss-chance is already computed inside the SP eval):** the
  walk's `eval` = `OptimizeIncreSingleTask` (`OptimizeSP_TL_Incre.cpp:167-239`), which routes
  through `rta_cache_.Evaluate` → `ObtainSP_Full_From_NodeRTAs` → `ObtainSP_DAG_From_Dists`, which
  internally calls `GetDDL_MissProbability(rtas[i], deadline)` PER TASK (`SP_Metric.cpp:160`) but
  only aggregates into SP. The node RTAs are ALREADY materialized for the SP eval → surfacing
  per-task miss-chances (or a single important-task feasibility bool) is a SMALL API addition over
  already-computed data. Two candidate API shapes (decide at impl): (A) a new
  `ImportantTasksFeasible(pa, tl) -> bool` that re-derives node RTAs + checks thresholds (cleanest
  separation; one extra RTA eval per adopted candidate); (B) thread per-task miss-chances out of
  the existing `ObtainSP_DAG_From_Dists` (zero extra RTA eval — cheapest). Prefer (B) if the API
  fits cleanly; (A) as the fallback.
- **D8 RESOLVED via the gate:** "does the from-scratch optimizer reorder PA (risking the group
  lock)?" — the from-scratch path IS a PA beam search (`OptimizePA_Incre::OptimizeFromScratch`,
  `OptimizeSP_Incre.cpp:100`) that DOES reorder PA. BUT with the hard per-candidate gate as the
  safety mechanism, PA descent becomes SAFE-TO-ADD-LATER (the gate rejects any PA move that
  violates a threshold, including a group-lock-breaking reorder). For v1: keep the walk **TL-only**
  (group lock preserved BY CONSTRUCTION as belt-and-suspenders; the incremental walk
  `OptimizeIncreSingleTask` is TL-only — `WalkOneTaskWithTimeLimitOptions` mutates only
  `time_limits[task_idx]`, PA inherited + never changed). PA descent = deferred enhancement
  (gated, so safe when added).
- **ALGORITHM (authoritative, supersedes the 2026-07-30 "no filter" version):**
  1. **Seed** at the P0.8-certified operating point: DM-grouped PA (P0.9 `DeadlineMonotonicPriorityVec`)
     + TL = largest grid option ≤ `et_mean` (`FindLargestTimeLimitAtOrBelow`, new directional
     variant of bidirectional `Find_Close_ExecutionTime`); non-perf TL = -1. (At the seed, TL ≤
     et_mean → ddl_miss_chance = 0 → seed is feasible by construction; P0.8's gate also certifies it.)
  2. **Optimize (TL-only walk) from the seed** for best SP, with the **hard per-candidate gate**:
     adopt an SP-better candidate TL only if `∀ important i: ddl_miss_chance_i(candidate) ≤
     sp_threshold_i`; else skip + continue the walk. Ample offline budget → run to convergence.
  3. **Keep** the committed incumbent as `static_solution_` (a `ResourceOptResult`). It is
     feasible-by-construction (the gate never adopted a violating candidate) AND best-SP-among-
     feasible (the walk kept searching past skipped candidates).
- **DROPPED / SUPERSEDED (unchanged from 2026-07-30):** the WCET-mode flip; the global-max-
  across-intervals WCET precompute (old D2). The OLD skip-and-continue filter (old D5/D6/D7) was
  dropped on 2026-07-30 then PARTIALLY REVIVED here in a new form (hard gate on probabilistic
  ddl_miss_chance, normal ET mode, offline-scoped — NOT the old WCET point-mass RTA).
- **Docs updated this entry:** `goal.md` (algorithm + "Why a hard gate (not soft)" + D8 resolved),
  `tasks.md` (step 3 TL seed; step 4 REVIVED as the hard gate; step 5 wiring incl. the gate hook),
  memory `p06-…` + `MEMORY.md` index. NOT started on code.
- **Next:** implement (a) `FindLargestTimeLimitAtOrBelow` (step 3, pure, TDD); (b) the per-task
  miss-chance API + the hard-gate hook into `WalkOneTaskWithTimeLimitOptions`'s adoption decision
  (step 4); (c) `ComputeStaticSolution` wiring (step 5). `cmake --build build_test --target
  check.SP_OPT -j5` green; spot-check feasibility of the returned solution; `git add` (no commit).

## 2026-07-30 (REDESIGN — seed at P0.8-certified point + normal optimization; filter DROPPED)

- **User direction (verbatim):** "for p0.6 finding static solution, start with an initial
  solution that's the same as tested in schedulability test in p0_8 (use time limit option
  that is close but lower than ET_mu), then perform incremental / from-scratch optimization to
  find a good solution, and keep it. update related task doc."
- **This SIMPLIFIES the algorithm.** Prior (2026-07-27) design: seed DM-grouped PA + min-TL →
  WCET-mode TL walk → **skip-and-continue filter** → best-SP-safe incumbent. New design:
  1. **Seed** = the SAME operating point P0.8 certifies: DM-grouped PA (LANDED by P0.9) + TL =
     the **largest grid option ≤ `et_mean`** (`execution_time_dist.GetAvgValue()`,
     `RegularTasks.h:87`); non-perf TL = -1.
  2. **Optimize** normally (incremental / from-scratch TL walk) from that seed — ample offline
     budget → run to convergence for good SP.
  3. **Keep** the committed incumbent as `static_solution_`.
- **DROPPED (superseded):** the WCET-mode flip (`use_wcet_execution_time = true`); the
  global-max-across-intervals WCET precompute (old D2); the skip-and-continue filter (old
  D5/D6/D7). Old steps 2 + 4 of `tasks.md` are gone.
- **Why dropping the filter is sound (structural safety argument):** P0.8 certifies, at gen
  time, that under the DM-grouped PA with every task at WCET (perf = `et_mean`, non-perf =
  global-max `execution_time_max`), important tasks meet deadlines — `R_i(WCET) ≤ deadline_i`.
  That certification is TL-independent. The sim runs perf ET = `min(et_mean, TL)` ≤ et_mean =
  WCET for ANY TL (TL is a downward cap), non-perf runtime ET ≤ `max_time` ≤ certified WCET →
  interference monotonic in ET → **runtime `R_i` ≤ `R_i(WCET)` ≤ `deadline_i` for EVERY TL grid
  point, as long as the PA stays group-locked.** The whole grid is certified → the optimizer
  can freely adopt any SP-better TL; no per-candidate RTA needed.
- **Dissolves the prior D2-vs-P0.8 tension** I'd flagged: P0.6 no longer defines its own WCET
  (global-max) that P0.8's gate (et_mean) would have to agree with. No divergence possible.
- **Key code grounding found (makes the new seed trivial-ish):** `InitializeTimeLimitsFromETConfig()`
  (`OptimizeSP_TL_Incre.cpp:483-498`) ALREADY seeds each perf task's TL to
  `Find_Close_ExecutionTime(timePerformancePairs, GetAvgValue())`. BUT
  `Find_Close_ExecutionTime` (`:42-56`) is **bidirectional** (closest by abs distance, may pick
  ABOVE et_mean). P0.6 needs the directional ≤ et_mean variant → add
  `FindLargestTimeLimitAtOrBelow(pairs, et_mean)` (largest grid option ≤ et_mean; fall back to
  smallest if none ≤ et_mean). Generator TL grid = `period * FINAL_Et_OVER_PERIOD_RANGE`
  `[0.05, 0.9]` (`taskset_generator.py:32`).
- **TWO safety assumptions to verify during implementation (the only things the redesign's
  safety rests on):**
  1. Sim perf ET = `min(et_mean, TL)` (downward cap, NOT an overrunable budget) — confirm at
     the `SimulateInterval` ET-draw site. P0.8's config-tuning memo (`7c8748c0`) asserts this.
     (P0.7's online trigger (a) ET-jump logic shares this assumption.)
  2. The from-scratch optimization preserves the group lock (or is constrained to TL-only /
     within-group reordering). The incremental walk is TL-only → safe by construction; a
     from-scratch PA descent would NOT be.
- **NEW open decision D8:** does the from-scratch optimizer reorder PA (risking the group
  lock) or is it TL-only? Must be settled before wiring step 5 (constrain from-scratch to
  TL-only / within-group, OR disable PA descent for the static-solution compute).
- **Docs updated this entry:** `goal.md` fully rewritten around the redesign (algorithm,
  "Why no per-candidate filter" section, scope IS/IS-NOT, P0.8 relationship, Done-when,
  Open decisions incl. D8, reference docs); `tasks.md` rewritten (steps 0.5/1/3 = DONE via
  P0.9; steps 2/4 = DROPPED; remaining = et_mean-bounded TL seed + `ComputeStaticSolution` +
  verify 2 assumptions). Memory `p06-static-solution-fallback-seed` to be updated next.
- **NOT started on code.** Next: update memory; settle D8 with the user; then implement the TL
  seed helper (step 3) + `ComputeStaticSolution` (step 5).

## 2026-07-30 (resuming — P0.9 superseded steps 0.5/1/3; plan docs STALE)

- **P0.9 LANDED most of P0.6's prerequisites; the `tasks.md` checklist is STALE
  (written 2026-07-27/28, pre-P0.9).** Verified in code (not the plan docs):
  - **Step 0.5 (`bool is_important` on `Task`): DONE.** `bool is_important = false`
    at `RegularTasks.h:111`; parsed `tasksNode[i]["important"].as<bool>()` at
    `RegularTasks.cpp:87`; emitted `task_node["important"] = task.is_important` at
    `:126`. Generator side also done: `yaml_exporter.py:73` emits `important:`;
    `taskset_generator.py:577-581` labels the top `IMPORTANT_TASK_RATIO` (0.5) by
    sp_weight desc; `IMPORTANT_TASK_RATIO: 0.5` in CONFIG_SPECS (`:38`). Cross-lang
    seed consistency with P0.8 confirmed (P0.8 RTA ranks by deadline ↔ C++ DM seed).
  - **Step 1 (`AssignDMRespectingGroupOrder` helper): SUPERSEDED, NOT NEEDED.** P0.9
    baked the group lock directly into `DeadlineMonotonicPriorityVec()`
    (`OptimizeSP_TL_Incre.cpp:705-721`): sorts important-first (`ta.is_important !=
    tb.is_important → ta.is_important` at `:713-714`), then deadline asc within group,
    ties by avg ET asc. There is no separate helper to extract — the group-locked DM PA
    is one function. (The orchestrator `DM`/`DM_FAST`/`DM_SLOW` branches call a plain
    deadline sort, NOT this — to be confirmed by the in-flight C++ mapping.)
  - **Step 3 (DM-grouped PA + min-TL seed): DONE.** `SeedIncumbentFromDMFast()`
    (`OptimizeSP_TL_Incre.cpp:772-780`) = `SmallestTimeLimitVec()` +
    `DeadlineMonotonicPriorityVec()`, scored via `EvaluateSPWithPriorityVec`, committed
    via `SeedStateFromIncumbent`. This is exactly the P0.6 seed (under WCET mode the
    static solution reuses it).
- **Genuinely remaining (pending the C++ grounding agent's report):**
  - Step 2 — global-max-across-intervals WCET precompute (D2). OPEN feasibility
    question: is `dag_tasks_vecs_` populated BEFORE the interval loop (so an offline
    global-max per task is computable)? If it's filled inside the loop, D2 as written
    is infeasible offline and needs rework.
  - **D2 vs P0.8 tension (to flag to user):** P0.6's plan says WCET = global-max ET
    across intervals; P0.8's gate (config-tuning `7c8748c0`) uses perf WCET =
    `execution_time_mu` (= et_mean, a runtime DOWNWARD cap, NOT the global max). The
    two must agree on the seed WCET (P0.8 certifies the point P0.6 seeds from). If
    P0.6 uses global-max and P0.8 uses et_mean, P0.6's seed is more conservative than
    what P0.8 certified → P0.6's seed is safe-but-pessimistic; the filter still
    preserves schedulability during the walk. Likely acceptable (conservative = safe)
    but needs an explicit decision, not a silent divergence.
  - Step 4 — skip-and-continue schedulability filter (the NOVEL piece): virtual hook
    `ShouldAdoptCandidate` in the TL walk's per-candidate adoption decision; inline
    important-task RTA (R_i ≤ deadline_i, D6); skips SP-better-but-unsafe candidates,
    continues the walk.
  - Step 5 — wire `ComputeStaticSolution()` offline (after `incr_optimizer_`
    construction, before the interval loop), WCET-mode flip, ET-excluded (outside
    `DeterminePrioritiesAndBudgets`'s bracket), store in-memory `static_solution_`.
- **Action:** a read-only Explore agent is mapping the 3 remaining pieces' exact C++
  grounding (TL-walk adoption site + virtuals, `ApplyWCETAblationIfRequired` +
  `dag_tasks_vecs_` population timing, offline compute site + `scheduler_exec_time_s_`
  bracket, `ResourceOptResult` shape). Awaiting its report before finalizing the plan.
  NOT started on code.

## 2026-07-28 (P0.9 supersedence — plan docs relabeled RM→DM)

- **P0.9 (DM + important-first group lock) LANDED** for the seed PA system-wide.
  The seed's plain-RM sort is now `DeadlineMonotonicPriorityVec` (group-locked DM),
  and the seed bootstrap is `SeedIncumbentFromDMFast` / `BootstrapIncumbentFromDMFast`.
- To keep this task's forward-looking plan (`goal.md` + `tasks.md`) consistent with
  the landed seed, those two files were relabeled RM→DM on 2026-07-28: the planned
  helper is now `AssignDMRespectingGroupOrder` (extracted from the now-deadline-based
  orchestrator sort; the bare `DM`/`DM_FAST`/`DM_SLOW` branches call it with an empty
  group = behavior-identical, the static solution calls it with `important_ids`), the
  seed point is "DM-grouped + min-TL + WCET", and within-group ordering is "DM-ordered".
- **Historical entries below retain their original RM wording** (point-in-time records,
  not rewritten — same "don't falsify history" treatment as the P2.11 leave). They
  describe what was true when written; the relabeled plan docs above are authoritative.

## 2026-07-26

- Task scaffolded from the user's fall-back design direction. Scope + 4 open design
  decisions (D1–D4) recorded in `goal.md`; `tasks.md` checklist written.
- **BLOCKER:** D1 (important-task selection rule) must be settled with the user before
  any code — it is shared with P0.7 (fall-back) and P0.8 (schedulability). Candidates
  (a) top-X% by `sp_weight` [existing convention], (b) by `sp_weight * perf_coefficient`,
  (c) by strict `sp_threshold`, (d) union/intersection. Recommendation to bring: (a)
  for v1, (c) as a follow-up filter post-P2.13.
- Code grounding confirmed: RM period-sort at `SimulationOrchestrator.cpp:356-367`
  (triplicated at 368/388 → candidate for the `AssignRMRespectingGroupOrder` helper);
  `INCR_WCET` WCET-semantics path at 348-355 (`use_wcet_execution_time = true`);
  offline compute site between optimizer construction (300-302) and the interval loop
  (304-308); ET bracket at `DeterminePrioritiesAndBudgets` 316-322 (static solution
  stays OUTSIDE so it doesn't inflate the online ET metric).
- Not started; awaiting D1 resolution + P0.8's schedulability guarantee.

## 2026-07-27

- **ALGORITHM REFINEMENT (user direction).** The static solution is no longer "RM-grouped
  + flat scalar WCET, no optimization." New algorithm: (1) seed RM-grouped PA + min-TL
  (`SmallestTimeLimitVec`); (2) under `use_wcet_execution_time = true`, run the incremental
  TL walk from that seed; (3) early-stop the walk on the first candidate TL that improves
  global SP but breaks important-task schedulability (inline RTA, R_i ≤ deadline_i); (4)
  return the committed incumbent. `goal.md` + `tasks.md` rewritten around this.
- **Key code grounding confirmed during the refinement:**
  - The existing TL walk is **TL-only** — `WalkOneTaskWithTimeLimitOptions`
    (`OptimizeSP_TL_Incre.cpp:500+`) mutates only `time_limits[task_idx]`; `opt_pa_` is
    inherited from the seed and never changed. So "fix priority, walk TL" is the walk's
    natural behavior — **no new fix-priority flag needed** (the existing
    `disable_time_limit_opt` is the inverse: fix TL, walk PA).
  - `SeedIncumbentFromRMFast` (`OptimizeSP_TL_Incre.cpp:758-766`) already seeds RM PA +
    min-TL — but plain RM (`RateMonotonicPriorityVec`), not grouped. P0.6 needs the grouped
    variant (`AssignRMRespectingGroupOrder`).
  - Under WCET mode, non-perf tasks collapse to a point mass (TL=-1, no freedom); only perf
    tasks retain TL freedom → the walk adjusts perf-task TLs only, from min (safest) upward.
  - The early-stop guard is genuinely NEW — `IsBetterTimeLimitOption` adopts any SP-better
    TL; there is no schedulability guard in the walk today.
  - Under WCET mode the RTA dist is a point mass, so DDL-miss (0/1) ≡ R_i ≤ deadline_i →
    the guard's RTA form and the DDL-miss form coincide (D6).
- **New open decisions added:** D5 (guard shape: virtual hook / wrapper / post-walk
  filter), D6 (guard quantity: RTA vs DDL-miss — equivalent under WCET), D7 (HALT vs
  SKIP-AND-CONTINUE on first unsafe candidate — user said "we'll stop" → default HALT).
- **P0.8 coordination:** P0.8's RTA must certify the SAME seed point P0.6 seeds from
  (RM-grouped + min-TL + WCET). If even the seed is unschedulable for important tasks, the
  walk has no safe start → P0.8's re-generate handles it. P0.6's guard preserves
  schedulability DURING the walk. Recorded in `goal.md` "Relationship to P0.8."
- Still not started; awaiting D1 (+ D5/D6/D7) resolution + P0.8.

## 2026-07-27 (later)

- **D1 RESOLVED.** The cross-cutting "how to decide important tasks" question is settled:
  **50% of the tasks in a taskset are important, indicated by SP weights; the rest are
  non-important.** The label is **persisted** as `bool is_important` on the C++ `Task`
  class — set at generation, read by all consumers. This is candidate (a) (top-X% by
  `sp_weight`) with **X = 50** (overriding the old 10%) and the label persisted rather
  than recomputed per consumer.
- **Why persisted (grounding):** `sp_weight` is NOT on `Task` today — it lives in
  `ParametersSP::weights_node` (`ParametersSP.h:51`), loaded from YAML
  (`ParametersSP.cpp:25-29`). The generator assigns it at `taskset_generator.py:551`
  (P2.15 continuous uniform) and `yaml_exporter.py:68` emits it. So the natural place to
  SET the bool is the generator, right after `sp_weight` assignment: sort by weight desc,
  mark the top-`ceil(N/2)` = `(N+1)//2` `is_important = true`, emit `important:` to YAML.
  C++ `Task` reads it at `ReadTaskSet` construction. This promotes "important" from a
  post-hoc *analysis* label (`compute_important_task_miss_rate`, `utils.py:182-220`, sort
  + `ceil(N*0.10)`) to a **generation-time property** — unifying P0.6 (priority lock +
  early-stop guard), P0.7 (safety check), P0.8 (Python RTA), and the analysis path on ONE
  definition (all read the bool → no drift). It also gives P0.8 the important set directly
  from the generated taskset — no duplicate top-X% logic in Python.
- **Tie-breaking:** `sp_weight` continuous uniform [0.1, 1.0] → ties measure-zero → clean
  50% boundary. If a tie lands exactly on the boundary, break by task id (deterministic).
- **Config migration:** the old `important_task_top_percentage = 0.10`
  (`paper_simulation_config.json:66`) + `minimum_important_tasks_count = 1` (line 67) are
  ANALYSIS-config knobs. Under the new rule the fraction is a GENERATION parameter —
  default `IMPORTANT_TASK_RATIO: 0.5` in the generator CONFIG_SPECS; the analysis path
  reads the bool and drops its own pct (ruthless-prune the dead knobs). Exact knob
  placement is an implementation detail, NOT a blocker.
- **Recorded as a SHARED enabling change** in `tasks.md` step 0.5 (new): add `bool
  is_important` to `Task` + generator labeling + `ReadTaskSet` parse + analysis migration,
  landed BEFORE the P0.6 walk (and pairing with P0.8's Python pass). TDD: a taskset of N
  tasks has exactly `ceil(N/2)` important; the important set = the top-weight half; ties
  broken by id. D1 checkbox in `goal.md` "Done when" + `tasks.md` section 0 marked done.
- **Remaining open:** D2 (perf-task WCET + min-TL seed fields, shared with P0.8), D3
  (per-taskset), D4 (in-memory vs file), D5/D6/D7 (guard shape/quantity/halt). D5/D6/D7
  have defaults from the 2026-07-27 refinement; D2 is the next one to confirm with the
  user. Still not started on code.

## 2026-07-27 (final design lock)

- **BUDGET ASYMMETRY (user direction).** "overall, we have high budget for offline
  analysis, so we can try more 'walk' during offline analysis, just find a safe solution
  with good performance. during online, budget is very tight, use this to adjust your
  design." This splits the prior single "early-stop HALT guard" into two:
  - **Offline (THIS task, ample budget):** the walk **skip-and-continues** on unsafe
    candidates — keep exploring to find the **best-SP safe** point (not just the first
    safe point). Returns a stronger safe floor.
  - **Online (P0.7, tight budget):** the guard **halts** on the first unsafe candidate,
    adopts the incumbent-so-far, compares its SP vs `static_solution_`'s SP, picks the
    higher-SP winner. The HALT semantics live in P0.7, NOT here.
  - `goal.md`/`tasks.md` rewritten: D7 = skip-and-continue offline; the HALT guard is
    P0.7's trigger (b). The filter hook renamed `ShouldAdoptCandidate` (was
    `ShouldStopWalk` — the old name implied halt).
- **D2 RESOLVED (WCET value).** WCET per task = the **max ET that task exhibits across all
  generated interval tasksets** (global max, NOT the task's own `execution_time_dist.max_time`).
  User confirmed interval ETs are pre-generated → global max computable offline. This is
  what makes P0.7's ET-jump trigger (a) safe by construction: any interval's jumped ET ≤
  the global max the static solution was computed at. The existing
  `ApplyWCETAblationIfRequired` (`OptimizeSP_TL_Incre.cpp:830-840`) collapses to per-task
  `execution_time_dist.max_time`; P0.6 adds a `ComputeGlobalMaxWCETPerTask(dag_tasks_vecs_)`
  precompute that takes the max across intervals per task id, then collapses to that point
  mass. Exact acquisition mechanism code-verified separately (task #7).
- **D3/D4 RESOLVED.** D3 = per-taskset (orchestrator runs one taskset per worker invocation
  → per-taskset == once per run; no cross-taskset reuse). D4 = in-memory `static_solution_`
  member (the fall-back reads it inside `SimulateInterval`); file dump is inspectability-only,
  default off.
- **D5/D6 RESOLVED.** D5 = virtual hook `ShouldAdoptCandidate(candidate_sp,
  candidate_schedulable)` on the walk (the walk's existing virtuals
  `CallOptimizerGivenTimeLimits`/`OptimizeIncreSingleTask` suggest the pattern). D6 = RTA
  form (R_i ≤ deadline_i) offline — under WCET the RTA dist is a point mass, so DDL-miss
  is 0/1 ≡ R_i ≤ vs > deadline_i (the two coincide offline). The online guard (P0.7), which
  runs OUTSIDE WCET mode, uses the real DDL-miss-chance from the SP metric.
- **Two online triggers clarified (P0.7 scope, recorded here for cross-reference):**
  (a) ET-jump, BEFORE optimization — optimizer holds the old dag; new dag comes in; if any
  task's Gaussian `et_mean` ≥ 1.5× saved old dag's → use `static_solution_` directly for
  that interval (skip the online walk); (b) during-walk early-stop guard — first candidate
  with important-task DDL-miss-chance > its SP threshold → HALT, adopt incumbent-so-far,
  compare its SP vs `static_solution_`'s SP, pick higher-SP winner.
- **A/B purpose clarified (user).** {INCR-family + full fall-back chain (static solution +
  2 triggers + compare)} vs {today's INCR-family, no fall-back}, user re-runs prod, metric
  = SP the fall-back costs. Quantifies the performance penalty of adding the fall-back.
- All D1–D7 now RESOLVED. `goal.md` "Open decisions" + "Done when" updated; `tasks.md`
  step 0 (design) all checked. Next: step 0.5 (`bool is_important`) — gated on the
  parallel P0.8 owner syncing the same seed point. Still no code; awaiting step-0.5 start.

---

## 2026-07-30 — step 4b RELOCATED: gate moved to `UpdateRecords`, PA descent re-enabled

**User correction (the key design fix).** My step-4b v1 coupled TWO concerns into
`enforce_important_task_gate_`: (1) the gate check, and (2) SKIPPING PA descent
(TL-only walk). The user's mental model, stated plainly: "the overall optimization
process is very similar no matter whether enforce_important_task_gate_ is true or
false, the difference is that we'll run important tasks' schedulability check and
only accept a challenger that beats champion if the challenger guarantees important
tasks' schedulability... i think the simplest solution is just to add the check in
comparing sp values' function." The TL-only coupling was my over-cautious design
choice, NOT a user requirement — and it was wrong: the gate is a PURE extra
accept/reject criterion at the comparison step, not a process change.

**What changed (the refactor).**
1. **Gate moved** from `OptimizeIncreSingleTask` (the eval lambda) INTO
   `UpdateRecords` (the commit chokepoint the user pointed at). Fires ONLY on
   `WouldBeatIncumbent` (the user's would-beat rule); on reject → return `false`,
   no `CommitIncumbent`.
2. **PA descent re-enabled** unconditionally: `!BFSharedBudgetCancelled()` only
   (the `&& !enforce_important_task_gate_` guard DROPPED). PA descent runs whether
   or not the gate is on.
3. **Ghost-SP fix relocated**: the eval still reports the INCUMBENT SP on a
   gate-reject (so the walk sees "no progress"), now driven by `!updated &&
   enforce_important_task_gate_ && WouldBeatIncumbent(...)` in the eval tail.
4. **RTA source fixed (the real wrinkle).** v1 read the pre-descent `baseline_rtas`
   pointer — INVALIDATED by PA descent (overwrites the cache's `candidate_rta_`
   scratch buffer per the RTACache header's ref-validity contract). The relocated
   gate re-runs the SAME `rta_cache_.Evaluate(dag_tasks_, opt_pa_, tl)` that
   `CommitIncumbent` uses at `:824` — the candidate's FINAL {pa, tl}, post-descent.
   `|diff|<=1`-cheap on the incremental path (the only path the gate is on): after
   PA descent the champion is either adopted to the candidate (`|diff|==0`,
   FullReuse) or the prior incumbent (`|diff|==1`); `Evaluate` never mutates the
   champion, so the caller's `cache_backup` snapshot-revert stays sound. Zero extra
   RTA eval flag-off (gate skipped entirely). The gate is literally running the
   commit path's own RTA fetch, then checking it — zero divergence risk.

**D8 status.** The "from-scratch path & the group lock" concern that motivated v1's
TL-only pin is now MOOT. The gate IS the constraint (max SP s.t. all important tasks
meet threshold); the P0.8/P0.9 group lock is a means, not the end. A PA descent that
finds a higher-SP PA still passing the gate is strictly BETTER for the constrained
objective — the lock is not sacred once the gate holds.

**Tests.** All 5 `GateWiring_*` tests pass WITH PA descent re-enabled (no scenario
rework needed): on the reject fixture T_perf is already top-priority, so PA descent's
`FindPriorityVec1D_Variations` finds no strict-improving move and the gate sees the
candidate's actual {pa,tl} and rejects exactly as before. Section-header + reject-test
comments updated to describe the new location; the precondition/keep/non-beat tests
unchanged. `cmake --build build_test --target check.SP_OPT -j5` = 17/17 green. Git
add-only; NOT committed (awaits user review).

---

## 2026-07-30 — ghost-SP fix: ROOT CAUSE = three divergent acceptance predicates (NEW FINDING)

**User question.** "is this necessary: [the ghost-SP override block]? since `updated`
already decided whether to update champion, why do differently between
`enforce_important_task_gate_` true/false". The instinct (correct): `UpdateRecords`
already decided the commit, so the eval's return shouldn't need flag-dependent surgery.

**Answer: the override IS necessary *as-is*, but it is a SYMPTOM, not the fix.** The
TL walk (`WalkOneTaskWithTimeLimitOptions`) tracks its best TL by the eval's RETURNED
SP via `IsBetterTimeLimitOption(sp_val, best_sp, step)` — it does NOT see the
`updated` bool. A gate-rejected candidate carries a "phantom" SP that is *higher* than
the committed incumbent's. Returning it raw would (a) inflate the walk's `best_sp` →
later trial TLs look like "no progress" → patience exhausted → walk stops early,
missing feasible improvements; and (b) propagate as the next task's walk baseline via
`OptimizeOneTaskWithTimeLimit`'s return. The override masks the phantom by reporting
`res_opt_.sp_opt`. It must be flag-guarded + `WouldBeatIncumbent`-guarded (not
unconditional) because: flag-off non-commits are ALWAYS not-better (no phantom —
candidate SP ≤ incumbent, returning it is correct + byte-identical); flag-on
non-commits include better-but-rejected (phantom). The `WouldBeatIncumbent` check
distinguishes phantom (mask) from genuine non-beat (leave candidate SP).

**The deeper issue the question surfaced (the REAL problem).** There are THREE
acceptance predicates in the optimizer, and they are NOT unified:

| # | Site | Predicate | Compares against | Commits? |
|---|------|-----------|------------------|----------|
| 1 | PA-move loop `OptimizeIncre_SingleTask:354` | `sp_eval > opt_sp_` (strict) | LOCAL `opt_sp_` (member, in-place adopted) | yes, in-place (`opt_pa_ = ...; AdoptChampion`) |
| 2 | TL-walk adoption `IsBetterTimeLimitOption:116` | strict `>` OR approx-tie+`step<0` | LOCAL `best_sp` (walk-local, NOT committed) | NO — only tracks `best_option_val`; commit is downstream |
| 3 | Champion commit `WouldBeatIncumbent:129` (→ `UpdateRecords:156` → `CommitIncumbent`) | strict `>` OR approx-tie+smaller-TL | GLOBAL incumbent `res_opt_` | yes, via `CommitIncumbent` |

The ghost-SP override exists PRECISELY because **predicate #2 (walk-local best_sp) can
disagree with predicate #3 (global incumbent)** on a gate-reject: #2 sees the
phantom better SP and would track the rejected TL; #3 correctly refused to commit.
They compare against DIFFERENT references (`best_sp` vs `res_opt_.sp_opt`) and #2 has
no notion of the gate. When the gate is off they can't disagree (a non-beat for #3 is
also non-better for #2), so the override is inert — which is why it felt like surgery
only the flag-on path needs.

**Unified fix (proposed, NOT yet decided by user — awaiting direction).** Make the
walk's best-tracking and the champion commit use the SAME predicate against the SAME
reference, so they can never disagree and the phantom cannot arise. Concretely: route
every acceptance (PA move, TL trial, challenger) through ONE `IsAcceptable(...)`
predicate that combines `WouldBeatIncumbent`'s strict-better / tie-smaller-TL rule WITH
the gate, all against the global incumbent `res_opt_`. Then the walk tracks the
committed incumbent's SP as its baseline (the rejected candidate simply isn't
"acceptable," so its phantom SP never enters `best_sp`), and the ghost-SP override
becomes dead code → deletable. Open sub-decisions for the user: (a) does the TL walk's
downward-tie preference (`step<0`) survive unification, or does the global tie-break
(smaller total TL) subsume it? (b) is the PA loop's in-place adoption (`opt_sp_ =
sp_eval; opt_pa_ = ...`) reconciled with the global commit, or kept local? These touch
core optimization behavior (P2.10/P2.11 territory) and may be a behavior change, so
NOT bit-identical — needs explicit user go-ahead before any code.

**Status: step 4b v2 LANDED code-wise (override + gate in `UpdateRecords`, PA descent
on, 17/17 green, git add-only, NOT committed) STANDS as-is for now.** The unification
is a SEPARATE, larger refactor — recorded here so it isn't lost. No code changed in
this finding session.

---

## 2026-07-30 — unification DEFERRED (user decision); step 4b v2 is FINAL

**User decision (verbatim):** "if code change is a lot, i'll give up and let you adopt
the original proposal by adding extra check, even though the code doesn't read elegantly."

The unification was scoped to "TL-walk only" — align the TL-walk adoption predicate
(`IsBetterTimeLimitOption`) with the commit predicate (`WouldBeatIncumbent` + gate),
both vs the global incumbent `res_opt_`, leaving the PA-loop's local in-place adoption
(predicate #1) alone. Grounding under that scope found the change is LARGE, not the
simple cleanup it appeared:

1. **Breaks the walk-core test framework.** `StubTLWalkOptimizer::CallOptimizerGivenTimeLimits`
   (`tests/testIncreOpt_w_TL.cpp`) returns SP from a `tl_to_sp` map WITHOUT committing →
   `res_opt_` never advances. Any unification that routes adoption through the committed
   incumbent (`res_opt_.sp_opt`) instead of the returned SP would make the stubs see a
   static `res_opt_` → the walk never adopts → the 7 `TrialAndErrorTLWalkSynthetic` tests
   break (would require reworking the stub to commit + updating assertions). Framework
   surgery, not a localized predicate swap.

2. **NOT byte-identical flag-off.** `res_opt_.sp_opt == best_sp` throughout a flag-off
   walk (each adoption commits), so routing adoption through `res_opt_` makes every
   non-commit look like an approx-SP *tie*, not a "worse candidate." On the downward
   pass (`step < 0`), `IsBetterTimeLimitOption` adopts on a tie → the walk adopts smaller
   TLs on every non-commit, never spending patience → exhaustive to the grid boundary
   even flag-off. The behavioral delta reaches BEYOND the gate-on path the "TL-walk only"
   scope targeted, so it is not a contained change.

3. **The override is already correct.** This is a cleanliness refactor, not a bug fix —
   the gate-reject path is masked correctly, the flag-off path is byte-identical, and the
   `WouldBeatIncumbent` guard prevents the override from firing on a genuine non-beat.

**Outcome: keep step 4b v2 as-is (FINAL).** The gate in `UpdateRecords` + the ghost-SP
override in the `OptimizeIncreSingleTask` eval tail stand. The user accepts the
elegance tradeoff ("the code doesn't read elegantly") in exchange for NOT taking on the
framework + behavior-change cost of unification. The 4b-followup unification is
**DEFERRED** — re-file as P2.x if revisited; do NOT start coding it. Next active item:
step 5 (`ComputeStaticSolution` wiring), once step 4b v2 is reviewed/committed.

---

## 2026-07-31 — step 5 START (ComputeStaticSolution — lazy-populated artifact)

**Step 4b v2 COMMITTED** (`500665d5`) at the start of the session. Tree clean → step 5.

**Plan approved** (after two rounds with the user):
- **Q1 (call site) — user-directed redesign:** "let's add one member function into
  `OptimizePA_Incre_with_TimeLimits` to calculate the static solution, the caller who
  calls the class to perform incremental optimization will first call the static solution
  method to generate static solution. at the start of `Optimize_w_TL_ScratchOrIncre`, it
  will first check if static solution is stored. if not, it will call the method to
  generate static solution to store one. in this case, i suppose caller doesn't have to
  call the method to generate static solution." → `ComputeStaticSolution()` is a MEMBER of
  the persistent optimizer; `Optimize_w_TL_ScratchOrIncre` lazy-checks at its start
  (safety net); the orchestrator pre-calls (to keep the compute ET-excluded).
- **Re-opt requirement — user:** "static solution fall-back should also be used in re-opt
  mode" → the lazy check sits in `Optimize_w_TL_ScratchOrIncre` (the dispatcher BOTH
  `OptimizeIncre_w_TL` AND `ReOptimizePeriodic` route through), so `static_solution_` is
  available to re-opt arms too. P0.6 only PRODUCES it; P0.7 wires the actual fall-back
  USE in both paths.
- Q2 (modes) = full INCR family; Q3 (ablation) = none (normal-ET mode).

**KEY byte-identical isolation:** `ComputeStaticSolution()` runs the gate-governed walk on
a THROWAWAY sibling sub-optimizer, NOT on `this`. Reason: the walk commits via
`CommitIncumbent` → writes `res_opt_`; if it ran on `this`, interval-0 reopt's
`IfInitialized()` check (`:911`,`:947`) would flip true → `ResetIncumbentBaseline(true)`
would re-eval the static solution's {pa,tl} as the baseline instead of
`SeedIncumbentFromDMFast()` → interval 0 warm-starts from the static solution = P0.7's
injection leaking into P0.6 (NOT byte-identical). The sibling isolates the compute; `this`'s
live `res_opt_`/cache/flag are untouched. Same class → `this` may touch the sibling's
private members (C++ access control).

**Ablation save/restore:** the gate is probabilistic — needs the real ET dist, not a WCET
point-mass (`use_wcet_execution_time` collapses the dist to its max → zero variance →
gate can't bind) and not a TL-frozen config (`disable_time_limit_opt` pins TLs to smallest
→ skips the walk). So `ComputeStaticSolution` saves/restores
`disable_time_limit_opt` + `use_wcet_execution_time` around the walk, forcing normal-ET +
TL-opt-on regardless of the running arm (mirrors `SimulationOrchestrator.cpp:341-355`).

**Sub-task 5a (IN PROGRESS):** `ComputeStaticSolution()` method + `static_solution_`
member + `HasStaticSolution()`/`GetStaticSolution()` accessors + dispatcher lazy-check.
TDD: 4 tests added to `testIncreOpt_w_TL.cpp` (red first): (1) populates gate-held
artifact (PA DM-grouped, gate holds on the result); (2) byte-identical — live incumbent
untouched after compute; (3) dispatcher lazy-populates when not pre-called; (4) dispatcher
short-circuits when pre-called (no double compute). Build running (clean, after header
layout change).

---

## 2026-07-31 — step 5a LANDED + TIMEOUT REGRESSION fixed (git add-only, NOT committed)

**Step 5a code complete:** `ComputeStaticSolution()` + `static_solution_` member +
`HasStaticSolution()`/`GetStaticSolution()` accessors + the dispatcher lazy-check at the
top of `Optimize_w_TL_ScratchOrIncre` (`:699-701`). The 4 TDD tests pass.

**REGRESSION found + fixed (the load-bearing discovery this session).** The clean build
went 16/17: `testINCRTimeout.RespectsGlobalTimeLimit_SingleEvalExceedsCap` FAILED (21.8 s
vs the 4 s cap). Root cause = my lazy `ComputeStaticSolution()` at the dispatcher entry
runs **before** the dispatcher installs its `BFDLSharedBudget` (`:711`), and
`ComputeStaticSolution` installed NO budget itself → its seed `EvaluateSPWithPriorityVec`
(~8 s on the wide-Gaussian 7-task fixture, granularity 300) + walk ran unbounded past
`TIME_LIMIT=1`. NOT pre-existing — introduced by step 5a's lazy dispatcher hook.

**Fix:** `ComputeStaticSolution` now installs its OWN `BFDLSharedBudget` around the WHOLE
compute (seed eval + walk), mirroring the dispatcher's idiom. Two iterations to land it:
  - v1 (budget around the walk only) → 9.5 s: the walk was bounded but the SEED eval
    (`:939`, also an `EvaluateSPWithPriorityVec` on the same taskset) still ran unguarded.
  - v2 (budget around the whole compute, including the seed eval) → test PASSES (the
    cooperative cancel fires inside the runaway seed eval via `ObtainSP_DAG`'s
    `BFSharedBudgetCancelled()` poll, `EvaluateSPWithPriorityVec` returns `INT_MIN`).

**Cancel-safety (why a cancelled static-solution compute is still sound):** on cancel,
`EvaluateSPWithPriorityVec` returns `INT_MIN` → the walk's strict-`>` adopt guard treats it
as "not better" → keeps the incumbent (compare-and-keep) → the stored result stays
GATE-FEASIBLE (the gate only REJECTS, never makes the seed infeasible; the seed itself is
feasible-by-construction at TL ≤ et_mean), just possibly short of full convergence. The
seed eval being cancelled mid-way does commit an incumbent with a sentinel `sp_opt =
INT_MIN` in the degenerate `TIME_LIMIT << seed-eval-cost` case — acceptable: the artifact
is still a valid `{pa, tl}` (DM-grouped PA + et_mean-bounded TL) with a pessimistic SP;
P0.7's fall-back compares SP and would just not pick it. The 4 `ComputeStaticSolution_*`
tests use small granularity (5) → fast SP-evals → never hit the budget → assert the
non-degenerate path (gate held, byte-identical, lazy + short-circuit). The timeout test
asserts only the TIME BOUND, not result quality.

**Build:** `cmake --build build_test --target check.SP_OPT -j5` = **17/17 green** (4 new
step-5a tests + 5 `GateWiring_*` step-4b tests + 8 prior, no regressions). Filtered runs
confirm all 9 P0.6 tests ran + passed.

**NOT committed** (git add-only per the coding rules; awaits user review). Files touched
this sub-step: `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` +
`tests/testIncreOpt_w_TL.cpp`. Self-contained module (the static-solution artifact +
dispatcher hook; no orchestrator wiring yet) → staged alone for review per "work by module,
commit by module".

**Next:** step 5b — orchestrator wiring. Pre-call `ComputeStaticSolution()` after
`incr_optimizer_` construction (`SimulationOrchestrator.cpp:300-302`), before the interval
loop (304-308), OUTSIDE `DeterminePrioritiesAndBudgets`'s ET bracket (316-322) so the
compute does NOT inflate `scheduler_execution_time.txt`. Emit a separate
`static_solution_compute_time` profile. Then verify the 2 safety assumptions (sim perf ET =
`min(et_mean,TL)` downward cap; the gate is the constraint so the group-lock concern is
moot). Then P0.7 wires the actual fall-back USE.

---

## 2026-07-31 — step 5b LANDED (orchestrator wiring + ET-exclusion guard; git add-only, NOT committed)

**Resuming the session found the working tree already contained a COMPLETE step 5b**
(orchestrator pre-call + separate compute-time member + `RunOrchestrator` output + the
`PreComputesStaticSolution_ExcludesSchedulerET` integration test) that the prior session's
dev-log entry had not recorded — an interrupted handoff. This entry records + audits it.

**What step 5b adds (4 pieces, `SimulationOrchestrator.{h,cpp}` + tests):**
1. **The orchestrator pre-call** (`SimulationOrchestrator.cpp:313-317`): inside the INCR-
   family construction branch, AFTER `incr_optimizer_` is built (`:300-301`) and BEFORE the
   interval loop (`:320-324`), `incr_optimizer_.ComputeStaticSolution()` runs once; its
   wall-time is captured into `static_solution_compute_time_s_`. Only the INCR family
   (`INCR`/`INCR_NO_TL`/`INCR_WCET`/`INCR_NO_REOPT`/`INCR_*Period`) pre-computes — the gate
   + walk are an INCR artifact.
2. **The separate compute-time member** (`SimulationOrchestrator.h:111` +
   `GetStaticSolutionComputeTime()` `:94-96`): `double static_solution_compute_time_s_`,
   DISTINCT from `scheduler_exec_time_s_` (the online scheduler-ET metric). The static
   solution is an offline fall-back artifact, NOT an online scheduler decision, so it must
   not inflate `scheduler_execution_time.txt`.
3. **`RunOrchestrator.cpp` output** (`:214-224`): prints `StaticSolutionComputeTime_s:` and
   writes `static_solution_compute_time.txt` next to `scheduler_execution_time.txt`.
4. **Test accessor + integration test** (`testScheduleSimulate.cpp`): `TestOrchestrator::
   HasStaticSolution()` (delegates to `GetIncrOptimizer().HasStaticSolution()`) +
   `PreComputesStaticSolution_ExcludesSchedulerET`. `GetIncrOptimizer()` (`SimulationOrchestrator.h:102-104`,
   protected) exposes the persistent optimizer to tests (the artifact lives on it).

**ET-exclusion — STRUCTURAL (disjoint call sites), now ACTUALLY asserted.** The static-
solution compute counter is written ONLY at `RunSimulation:316-317` (outside the interval
loop); `scheduler_exec_time_s_` is written ONLY at `DeterminePrioritiesAndBudgets:429-430`
(called from `SimulateInterval:545` inside the loop). Disjoint → no leakage by construction.
The test was NAMED `..._ExcludesSchedulerET` but only asserted (1) artifact populated, (2)
compute timed separately >0, (3) sim runs — it did not verify the exclusion itself. **This
session strengthened it with a 4th assertion:** a non-INCR (`DM`) run reports
`GetStaticSolutionComputeTime() == 0.0` exactly. DM ALSO runs `DeterminePrioritiesAndBudgets`
every interval, so if the static compute leaked into the scheduler-ET path, DM would report
>0. Exact-zero is non-flaky (no timing sensitivity). Now green.

**The 2 safety assumptions — BOTH VERIFIED:**
1. **Sim perf ET = `min(et_mean, TL)` downward cap** (the redesign's structural-safety
   premise): at `SimulationOrchestrator.cpp:499-516`, a perf task's sim runtime ET is
   `drawn_et = traces[i][idx]` (or `GetAvgValue()` = et_mean when no traces), then
   `if (budget > 0 && execution_time > budget) execution_time = budget` where `budget` =
   the task's time limit. So sim perf ET ≤ TL. The metric's perf ET model is a POINT MASS
   at TL (`ApplyTimeLimitsToTasksExecutionTime`, `SP_Metric.cpp:76-86` — perf ET = TL
   exactly). → metric perf ET (= TL) ≥ sim perf ET (≤ TL) → metric interference ≥ sim →
   **metric `ddl_miss_chance` ≥ sim's actual miss chance** → a gate on the METRIC is a
   sound (pessimistic) guarantee of the user's stated runtime condition. ✓
2. **The gate IS the constraint (group-lock concern moot):** structural via the COMMITTED
   step 4b v2 (`500665d5`): `enforce_important_task_gate_` gates the commit chokepoint
   (`UpdateRecords`); PA descent runs UNCONDITIONALLY (`!BFSharedBudgetCancelled()` only,
   NOT `&& !enforce_important_task_gate_`). The gate rejects ANY threshold-violating
   candidate (TL move OR PA move). A PA descent that finds a higher-SP PA STILL PASSING the
   gate is strictly BETTER for the constrained objective (max SP s.t. the gate) — the
   P0.8/P0.9 group lock is a means, not the end. The 5 `GateWiring_*` tests pass WITH PA
   descent on (on the reject fixture T_perf is already top-priority → PA descent finds no
   strict-improving move → the gate sees the candidate's actual {pa,tl} and rejects). ✓

**Build:** `cmake --build build_test --target check.SP_OPT -j5` = **17/17 green**
(4 step-5a `ComputeStaticSolution_*` + 5 step-4b `GateWiring_*` + 1 strengthened step-5b
`PreComputesStaticSolution_ExcludesSchedulerET` (now 4 assertions) + 7 prior). No regressions.

**NOT committed** (git add-only per the coding rules; awaits user review). Files touched
THIS session: `tests/testScheduleSimulate.cpp` (the ET-exclusion guard). The full P0.6
working-tree footprint (all steps 3+4a+4b+5a+5b): `sources/Optimization/
OptimizeSP_TL_Incre.{h,cpp}` + `sources/Safety_Performance_Metric/SP_Metric.{h,cpp}` +
`sources/RTDA/ImplicitCommunication/SimulationOrchestrator.{h,cpp}` + `tests/RunOrchestrator.cpp`
+ `tests/testIncreOpt_w_TL.cpp` + `tests/testScheduleSimulate.cpp`. Per "work by module,
commit by module," step 5b is its own reviewable module (orchestrator wiring; the optimizer
internals are step 5a's already-reviewed module).

**Step 5 COMPLETE (all of section 5 + section 6 verification boxes).** P0.6 now PRODUCES
the fall-back artifact end-to-end (seed → gate-governed walk → store → orchestrator-pre-call
→ separate profile). **P0.7 wires the actual fall-back USE** (the two online triggers: (a)
ET-jump before opt → use `static_solution_` directly; (b) in-walk HALT on first unsafe
candidate → compare incumbent vs static SP → pick higher). The artifact is ready for P0.7 to
consume via `incr_optimizer_.GetStaticSolution()`.
