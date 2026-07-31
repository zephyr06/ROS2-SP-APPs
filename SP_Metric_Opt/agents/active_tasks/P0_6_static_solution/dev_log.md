# P0.6 Static Solution — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

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
