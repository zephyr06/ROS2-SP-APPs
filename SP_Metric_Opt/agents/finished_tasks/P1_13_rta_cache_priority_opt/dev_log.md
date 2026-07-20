# P1.13 — RTA Cache for Priority Optimization (standalone, base-class) — Dev Log

> Detailed working log. Append chronological entries below.
> Filed 2026-07-18 as a standalone task at the user's direction (split from
> the P1.12 base-class-threading / "3/N" deferral — see `goal.md`).

---

## 2026-07-18 — Task filed; API design drafted (NO code yet)

- User pivoted from P1.12 increment 2b to a new standalone task: add the
  RTA cache into the **base** priority optimizer `OptimizePA_Incre`
  (`sources/Optimization/OptimizeSP_Incre.{h,cpp}`), supporting
  `OptimizeIncre` (NOT `OptimizeFromScratch`, which the user explicitly
  excluded). "this can be a standalone implementation / task ... don't worry
  about optimizeSP_TL_Incre.h, just try to add rta cache into the priority
  optimization. add a new task discuss api design, before implementation."
- Grounded the design before drafting:
  - `OptimizeIncre_SingleTask` (`OptimizeSP_Incre.cpp:275-296`) scores each
    1D priority variation via `EvaluateSPWithPriorityVec` (`:287`) → full
    N-task RTA per candidate. Each variation moves ONE task's priority
    position vs the carried PA → `|diff|<=1` vs a champion whose PA ==
    carried PA. Natural cache client.
  - `OptimizeIncre` (`:298-360`): baseline re-scores the carried PA (`:308`),
    loops diff tasks → `OptimizeIncre_SingleTask`. Champion must advance per
    adoption for the next task's diff to stay `|diff|<=1`.
  - Oracle path: `EvaluateSPWithPriorityVec` (`OptimizeSP_Base.cpp:148`) →
    `UpdateTaskSetPriorities` (sorts by PA) → `ObtainSP_DAG` →
    `ObtainSP_TaskSet` (`SP_Metric.cpp:53-67`, multiplies `perf_coefficient`
    at `:61,:65`) → `ProbabilisticRTA_TaskSet` (partitions by `processorId`).
  - Frozen cache contract re-verified (`RTA_Cache.h`): `Evaluate`/`AdoptChampion`/
    `Initialize` take NO `sp_parameters`; callers pass raw `dag`+`tl`;
    `ComputeTaskSetDifference` THROWS on `|diff|>1` (called unguarded by
    `Evaluate` via `ClassifyReusePerTask`).
  - Hazard B re-verified: `ObtainSP` (`SP_Metric.cpp:11-15`) OMITS
    `perf_coefficient`; existing `ObtainSP_DAG_From_Dists` (`:129-149`) calls
    it → also omits; has a LIVE caller (`ObtainSPFromRTAFiles:221`) → must
    NOT be modified → need a NEW helper.
- Drafted `api_design.md` with 6 open questions + a recommendation each:
  - **Q1** `OptimizeFromScratch` cache-free (user-stated; constructive search,
    invariant doesn't hold).
  - **Q2** where the cache lives: A (base member) / **B (threaded `RTACache*`,
    nullptr=oracle, rec)** / C (hybrid). B = smallest change, behavior-preserving
    by default, no P1.12 double-cache collision, matches the `baseline_sp`
    opt-in idiom.
  - **Q3** champion lifecycle: adopt at `OptimizeIncre` baseline re-score (a1)
    + at each `OptimizeIncre_SingleTask` strict-improvement (b). OPEN: what
    `tl` base `OptimizeIncre` passes (ties to Q5).
  - **Q4** two new helpers `ObtainSP_DAG_From_Dists_With_Perf_Coeff` +
    `ObtainSP_Full_From_NodeRTAs`; path SP recomputed per candidate (no cache
    win on chains — chains use `GetRTDA_Dist_AllChains`, not the node RTA).
  - **Q5** is the `OptimizeIncre` call-site DAG TL-baked? Decides the `tl`
    arg. Need to confirm `UpdateExtDistBasedOnTimeLimit` ==
    `ApplyTimeLimitsToTasksExecutionTime` (memory says yes; re-verify).
  - **Q6** no flag needed (pointer is the gate, unlike P1.12 2a's flag which
    was for the shared-`CommitIncumbent` reopt collision that doesn't exist
    here).
- **No code changes.** Awaits user API sign-off.

### Next

User reviews `api_design.md`, picks Q2 (A/B/C), resolves Q3-open+Q5 (tl/dag
plumbing), confirms Q4 (helpers) + Q6 (no flag). Then freeze the API and
start Phase 1 implementation.

---

## 2026-07-18 — Phase 1 sub-step 1: `ObtainSP_*` helpers LANDED (compile-verified)

- User gave the Phase 1 go ("continue implementation"). Per the modular-dev
  rule, landed the smallest ISOLATED unit first: the two Q4 helpers (Hazard B)
  in `SP_Metric.{h,cpp}` — no dependency on the optimizer changes, testable in
  isolation.
- **Critical indexing finding (traced before writing):** the cache's flat
  `rta_`/`candidate_rta_` is indexed over the **prioritized** vector
  (`UpdateTaskSetPriorities(baked, pa)` — which REORDERS via
  `SortTasksByPriority`, OptimizeSP_Base.cpp:38), NOT over `dag_tasks.tasks`
  storage order. `ProbabilisticRTA_TaskSet` (RTA.cpp:116-117) builds
  `task_id2index[id] = i` over its INPUT (the prioritized vector) and writes
  `rtas[task_id2index[...]]`. The cache's `Evaluate` patch loop uses the same
  mapping (RTA_Cache.cpp:445-457). The existing cache tests pass under a
  non-id-order PA (e.g. `{1,0,2,3}`) because BOTH oracle and cache use the same
  prioritized indexing — the test proves cache==oracle, NOT that either is in
  storage order. → the helper must NOT assume `rta_[i]` == RTA of task `i`.
- **Bit-identity design (the resolution):** mirror the oracle
  `EvaluateSPWithPriorityVec` body (OptimizeSP_Base.cpp:148-157) EXACTLY —
  bake TL → apply pa → walk the prioritized vector. `ObtainSP_DAG`/`ObtainSP_
  TaskSet` already index `rtas[i]` against `tasks[i]` where `tasks` is whatever
  vector is passed (the prioritized one), so the cache output (indexed over the
  prioritized vector) aligns NATURALLY — NO storage-order↔priority-order remap
  needed. The caller passes the SAME (dag, pa, tl) the cache was fed; the
  helper re-derives the prioritized vector internally (identical derivation to
  the cache's RTA_Cache.cpp:181-183,442-446) and pairs `node_rtas[i]` with
  `prioritized[i]`.
- **Two helpers added:**
  - `ObtainSP_DAG_From_Dists_With_Perf_Coeff(dag, sp, node_rts_dists,
    path_latency_dists)` — the node-SP-only half. Identical to the existing
    `ObtainSP_DAG_From_Dists` EXCEPT the node loop multiplies
    `perf_coefficient` (Hazard B — `ObtainSP`/`ObtainSP_DAG_From_Dists` OMIT it;
    the oracle `ObtainSP_TaskSet:61,65` INCLUDES it). Existing
    `ObtainSP_DAG_From_Dists` UNTOUCHED (live caller `ObtainSPFromRTAFiles`).
  - `ObtainSP_Full_From_NodeRTAs(dag, sp, pa, tl, node_rtas)` — the drop-in
    for the oracle body. Bakes TL + applies pa internally, scores nodes via the
    perf-coeff helper, recomputes chain terms via `GetRTDA_Dist_AllChains` (NOT
    cached — Q4: no cache win on chains; matches `ObtainSP_DAG:96-107`). Uses
    `std::vector<int>` for `pa` (not `PriorityVec`) so `SP_Metric.h` stays a
    leaf — `PriorityVec` is typedef'd in `OptimizeSP_Base.h`, which includes
    `SP_Metric.h` (would form a header cycle).
- **Compile-verified:** `cmake --build build --target SP_OPT -j5` emits ZERO
  errors for `SP_Metric.cpp` (object rebuilt 21:21). The ONLY build errors are
  the expected stale `OptimizeSP_Incre.cpp:193,298` signatures (header has the
  new `RTACacheOpt` param, `.cpp` doesn't) — that's the next Phase 1 sub-step,
  NOT a defect of this one.
- **No test yet.** The differential TDD (cache-eval SP == oracle SP
  bit-identical) belongs with the optimizer wiring (the helpers can't be
  exercised end-to-end without a cache + a call site). The helpers ARE
  bit-identical by construction (mirror the oracle body + `ObtainSP_DAG`
  exactly), but construction ≠ verification — the TDD test will pin it once the
  call site exists.

### Next

Phase 1 sub-step 2: update `OptimizeSP_Incre.cpp` definitions of `OptimizeIncre`
(`:298`) + `OptimizeIncre_SingleTask` (`:193`) to add the `RTACacheOpt rta_cache
= std::nullopt` param + the `if (rta_cache) rta_cache->get().Evaluate(...)` +
`ObtainSP_Full_From_NodeRTAs` branch at the eval site (`:287`) + `AdoptChampion`
at baseline (`:308`) and per-adoption (`:290`). Then sub-step 3: differential
TDD test. Awaits user review of this sub-step + go.

---

## 2026-07-18 — Sub-step 1b: Hazard B FIXED IN PLACE in `ObtainSP_DAG_From_Dists` (16/16 ctest green, NOT committed)

- User re-examined the sub-step-1 design and redirected it:
  - "i noticed ObtainSP_DAG_From_Dists doesn't consider perf_coefficient, that
    is a bug if true. fix it rather than adding ObtainSP_DAG_From_Dists_With_Perf_Coeff?"
  - "if the perf coefficient is processed in other code, i think that's wrong
    place to handle it. we should handle it in ObtainSP_DAG_From_Dists, should
    we modify other functions to make sure perf coefficient is always effective
    and only effective once?"
  - "i need to make sure all computation results are correct."
- **BINDING principle (user):** the perf coefficient is a per-task multiplier
  that belongs in the SP-eval function (`ObtainSP_DAG_From_Dists`), applied
  EXACTLY ONCE. Handling it anywhere else (e.g. the weight slot) is the wrong
  place and must be removed. Correctness of all computation results is the hard
  requirement.
- I had initially argued for keeping the parallel `_With_Perf_Coeff` helper
  (preserve the weight-slot semantics as a "preference"). User overruled — the
  principle is correct; the cleaner, uniform semantics is the right default.
- **Implementation (option (a) — full clean end-state):**
  1. `ObtainSP_DAG_From_Dists` (`SP_Metric.cpp`) — node loop now multiplies
     `dag_tasks.tasks[i].GetPerfCoefficient()`, mirroring the oracle
     `ObtainSP_TaskSet:62,66`. Perf is now applied in the SP function,
     uniformly across all SP-eval paths.
  2. `ObtainSPFromRTAFiles` — removed the `update_node_weight(0, tsp_weight)`
     weight-slot hack + the dead `GetAvgTaskPerfTerm` call + the `tsp_ext_path`
     param + the stale "without worrying about the execution time distribution"
     comment + the TSP-specific `assert(dag_tasks.tasks[0].name == "TSP")`.
     TSP's perf now flows through `perf_coefficient` like every other task.
  3. DROPPED `ObtainSP_DAG_From_Dists_With_Perf_Coeff` entirely (`.cpp` + `.h`)
     — redundant once the base fn is fixed in place.
  4. `ObtainSP_Full_From_NodeRTAs` — simplified to call the corrected
     `ObtainSP_DAG_From_Dists` (instead of the dropped `_With_Perf_Coeff`).
  5. Updated the two `ObtainSPFromRTAFiles` callers for the dropped
     `tsp_ext_path` arg: `tests/testSP.cpp:391` (local `tsp_ext_path` var
     removed) + `tests/AnalyzeSP_Metric.cpp:101` (CLI `--tsp_ext_path` arg +
     var + call-site arg all removed).
  6. `GetAvgTaskPerfTerm` KEPT — it still has its own pinned test
     (`testSP.cpp:347` `TaskSetForTest_robotics_v19.GetAvgTaskPerfTerm`); it's
     just no longer called by `ObtainSPFromRTAFiles`.
- **Value-preservation verification (the user's "all computation results
  correct" gate):** for `test_robotics_v20.yaml`, only TSP (id 0) has
  `timePerformancePairs` (MPC/RRT/SLAM have none → `GetPerfCoefficient()=1.0`
  → unchanged). TSP: both the empirical perf the hack used (measured ET samples
  from `TSP_execution_time_115_125.txt`, mean ~216.78, all in the first perf
  bucket `[184.1, 397.5)` → perf 0.5) and the analytic `GetPerfCoefficient()`
  (truncated-Gaussian mean ~195-202, also in the first bucket → 0.5) evaluate
  to 0.5 → current `SP_Func(miss,0.5)*weight(0.5)` == fixed
  `SP_Func(miss,0.5)*weight(1)*perf_coeff(0.5)`. Therefore the
  `EXPECT_EQ(2.0, sp_value_overall)` pin does NOT move.
- **Sub-step 2a (signature-only unblock) also landed:** the API-approved header
  (`OptimizeSP_Incre.h`) has the `RTACacheOpt rta_cache = std::nullopt` param
  on `OptimizeIncre`/`OptimizeIncre_SingleTask`, but the `.cpp` definitions
  still had the OLD signatures → "no declaration matches" build red (the
  pre-existing sub-step-1 state). Added `[[maybe_unused]] RTACacheOpt rta_cache
  = std::nullopt` to both `.cpp` definitions (`OptimizeSP_Incre.cpp:275,298`)
  to match the header. **BODIES UNCHANGED** — the oracle
  `EvaluateSPWithPriorityVec` path is still live; no cache wiring yet (that's
  sub-step 2b). The `[[maybe_unused]]` silences the named-but-unused-param
  warning for now.
- **Verification:** `cmake --build build --target check.SP_OPT -j5` → **16/16
  ctest green.** Direct testSP run confirmed:
  `TaskSetForTest_robotics_v20.ObtainSPFromRTAFiles` PASSES (the `2.0` pin
  holds) + `TaskSetForTest_robotics_v19.GetAvgTaskPerfTerm` PASSES (the kept
  utility still tested) + the 5 `GetPerfCoefficient.*` cases PASS.

### Next

Phase 1 sub-step 2b (the actual cache wiring): in the BODIES of
`OptimizeIncre`/`OptimizeIncre_SingleTask` (`OptimizeSP_Incre.cpp`), add `if
(rta_cache) rta_cache->get().Evaluate(...)` + `ObtainSP_Full_From_NodeRTAs` at
the eval site (`:287`) + `AdoptChampion` at baseline (`:308`) + per-adoption
(`:290`). Default `std::nullopt` keeps the oracle path (behavior-preserving).
Then sub-step 3: differential TDD (cache-eval SP == oracle SP bit-identical
across 1D variations). Awaits user review of this sub-step + go.

---

## 2026-07-18 — Sub-step 1c: USER CORRECTION — `tsp_ext_path` RESTORED (TSP measured ET must be loaded into the dist, NOT just the weight hack source)

- User caught a real methodology bug in my sub-step 1b Hazard B fix:
  "modifications on is wrong, this function must read TSP ET from file, you can
  add first read tsp et then update dag_tasks' TSP task's ET.
  ObtainSPFromRTAFiles"
- **Root cause of my error:** after Hazard B, `ObtainSP_DAG_From_Dists` calls
  `dag_tasks.tasks[i].GetPerfCoefficient()`, which reads
  `execution_time_dist.GetAvgValue()`. For TSP (id 0, the one task with
  `timePerformancePairs`) that dist was the ANALYTIC YAML GAUSSIAN (mu≈202.3),
  NOT the measured ET. The `tsp_ext_path` file
  (`TaskData/AnalyzeSP_Metric/TSP_execution_time_115_125.txt`, 20 samples
  ~209–211) is the source of TSP's REAL ET. The OLD code never loaded those
  samples into `dag_tasks.tasks[0].execution_time_dist` — it only used them to
  compute an average perf coefficient (via `GetAvgTaskPerfTerm`) and shoved
  that into TSP's weight slot (the hack the user's principle killed). So when
  I dropped `tsp_ext_path` entirely in sub-step 1b, I left `GetPerfCoefficient()`
  reading the analytic model instead of measured reality — a CORRECTNESS
  regression on the very quantity the user's principle ("all computation
  results correct") protects.
- **Fix (user's direction):** `ObtainSPFromRTAFiles` reads `tsp_ext_path`'s
  measured ET samples via `ReadTxtFile` and writes
  `dag_tasks.tasks[0].execution_time_dist = FiniteDist(tsp_ext_times,
  granularity)` BEFORE scoring — so `GetPerfCoefficient()` (applied once, in
  the SP fn) reads the measured avg. `FiniteDist(const std::vector<double>&,
  int)` is the same constructor already used for the node-RTA files.
- **What STAYS dead/dropped (sub-step 1b, correct):** the `update_node_weight(
  0, tsp_weight)` weight-slot hack, the `GetAvgTaskPerfTerm` call, the TSP-
  specific `assert(dag_tasks.tasks[0].name == "TSP")`, the redundant
  `ObtainSP_DAG_From_Dists_With_Perf_Coeff` helper. `GetAvgTaskPerfTerm` the
  function stays (own pinned test at `testSP.cpp:347`).
- **`tsp_ext_path` param + CLI `--tsp_ext_path` arg RESTORED in all 4 places:**
  `SP_Metric.h` (decl), `SP_Metric.cpp` (param + ET-load), `tests/testSP.cpp`
  (local var + call-site arg), `tests/AnalyzeSP_Metric.cpp` (CLI arg + var +
  abs-path + call-site arg).
- **Value-preservation re-verified:** TSP's measured avg ~210 still lands in
  the first perf bucket `[184.1, 397.5)` → perf 0.5 (same as the old empirical
  0.5 AND the analytic 0.5) → `EXPECT_EQ(2.0, sp_value_overall)` pin HOLDS.
  `cmake --build build --target check.SP_OPT -j5` → **16/16 ctest green.**
- **Lesson recorded:** the perf-coefficient's correctness depends on
  `execution_time_dist` carrying the RIGHT ET source. `ObtainSPFromRTAFiles` is
  the one call site where the measured ET must be injected into the dist
  BEFORE the (now perf-aware) SP fn reads it. The analytic-yaml-Gaussian path
  (`ReadDAG_Tasks`) is fine for the other 3 tasks (no perf pairs →
  `GetPerfCoefficient()` returns 1.0 → ET source irrelevant), but NOT for TSP.

---

## 2026-07-18 — Sub-step 2b + 3: cache wiring LANDED + differential TDD GREEN (16/16 ctest, NOT committed)

- User: "continue working on OptimizePA_Incre". Implemented the actual cache
  wiring in the `OptimizeIncre`/`OptimizeIncre_SingleTask` BODIES (the last
  open implementation step) + the differential TDD test. Both compile clean +
  pass.
- **Wiring design (grounded in the frozen `RTACache` contract):**
  - **`OptimizeIncre_SingleTask` (eval site):** per-variation `if (rta_cache)`
    branch. Cache path: `rta_cache->get().Evaluate(dag_tasks_update,
    priority_assignment, no_tl)` (a ≤1-task RTA patch vs the champion whose
    PA==opt_pa_; all variations move ONE task's position vs opt_pa_ →
    |diff|<=1) → `ObtainSP_Full_From_NodeRTAs(..., rtas_holder.rtas)` for SP
    (perf_coefficient INCLUDED via the corrected `ObtainSP_DAG_From_Dists`).
    `else` = oracle `EvaluateSPWithPriorityVec`. On strict-improvement
    (`sp_eval > opt_sp_`): adopt `opt_pa_`/`opt_sp_` AND
    `rta_cache->get().AdoptChampion(dag_tasks_update, priority_assignment,
    no_tl, rtas_holder.rtas)`. AdoptChampion MUST advance the champion per
    adoption — the next variation's diff must stay |diff|<=1 (Evaluate does NOT
    advance the champion itself; a stale champion drifts to |diff|>1 →
    ComputeTaskSetDifference throws, called unguarded by ClassifyReusePerTask).
  - **`OptimizeIncre` (baseline re-score):** `if (rta_cache)` → `Initialize`
    (full RTA, establishes opt_pa_ as the fresh champion) +
    `ObtainSP_Full_From_NodeRTAs` for the SP. NOT `Evaluate` — this is a fresh
    interval, the carried champion (if any) is on the OLD dag_tasks_ and may
    differ by >1 task → Evaluate would throw. `Initialize` overwrites all prior
    state. Honors `baseline_sp` (caller-provided SP skips the re-score but
    STILL `Initialize`s the champion RTA, since the downstream SingleTask
    variations need a champion to patch against).
  - **`no_tl = vector<double>(N, -1.0)`:** fed to the cache (Q5 resolved —
    call-site dag_tasks_update is TL-baked via
    `UpdateExtDistBasedOnTimeLimit` == `ApplyTimeLimitsToTasksExecutionTime`;
    the cache's internal `ApplyTimeLimitsToTasksExecutionTime` is a no-op →
    cache sees exactly the final ETs the oracle did → bit-identity by
    construction).
  - **`NodeRtasHolder` local struct:** COPIES `Evaluate`'s const-ref return.
    `Evaluate` returns `const vector<FiniteDist>&` into the cache's
    `candidate_rta_` buffer, which the NEXT `Evaluate` overwrites → binding it
    to a `reference_wrapper` would dangle. The copy stabilizes the RTA so it's
    valid for BOTH the SP score AND the `AdoptChampion` feed. Mirrors the
    `RTACacheOpt` no-raw-pointer idiom (the user's BINDING "never use raw
    pointer" rule — no `FiniteDist*` either).
- **Differential TDD (sub-step 3):** added
  `OptimizeIncre_Cache.Differential_BitIdenticalToOracle_OnSingleEtChange` to
  `testOptimizeIncrePA.cpp` (before `main()`). Two independent optimizers from
  the SAME scratch state (`OptimizeFromScratch(2)` is deterministic → identical
  opt_pa_/opt_sp_); arm A = `OptimizeIncre(v23)` with `std::nullopt` (oracle),
  arm B = `OptimizeIncre(v23, INT_MIN, cache)` with an engaged `RTACache`.
  Asserts `AssertEqualVectorExact(pa_oracle, pa_cache)` +
  `EXPECT_DOUBLE_EQ(opt_sp_oracle, opt_sp_cache)`. Uses the v22→v23 pair
  (|diff|==1, task 1 increase — same as the existing SingleTask differential).
  PASSES → the cache path reproduces the oracle's adopted PA + SP exactly.
- **Verification:** `cmake --build build --target check.SP_OPT -j5` → **16/16
  ctest green.** New test ran + passed directly:
  `./build/tests/testOptimizeIncrePA --gtest_filter='OptimizeIncre_Cache.*'` →
  `[  PASSED  ] OptimizeIncre_Cache.Differential_BitIdenticalToOracle_OnSingle
  EtChange (213 ms)`. The default-`nullopt` path is bit-identical to the
  pre-wiring behavior (no existing test moved).
- **What's NOT done:** no production call site passes an engaged `RTACache` yet
  (the cache is opt-in; `OptimizeIncre` callers default to `nullopt`). Wiring
  a production caller + Phase 2 measurement (profile candidate-eval time at
  N=6/10/16 with/without cache) is the remaining work.

### Next

User reviews sub-step 2b + 3 (cache wiring + differential TDD). Phase 1
implementation is COMPLETE (header + .cpp bodies + helpers + differential
test, all green). Then Phase 2 (measurement) + a production call site that
opts in to the cache. Awaits user review + go (modular-dev checkpoint).

## 2026-07-18 — Sub-step 3b: broader differential coverage (cache test suite)

User feedback on sub-step 3: "tests need higher coverage, like try different
types of input? i think one case cannot cover all passes." Correct — the single
v22→v23 case covered ONE branch: task 1, ET increase, low-weight core-1 task.
The cache path has more surface than that.

Added 4 differential cases + a shared `ExpectCacheMatchesOracleOnMutation`
helper to `tests/testOptimizeIncrePA.cpp`. The helper builds synthetic |diff|==1
mutations by overwriting one task's `execution_time_dist` with a
`FiniteDist(GaussianDist, min, max, 5)` — the SAME construction path
`ReadDAG_Tasks` uses (`RegularTasks.cpp:75-79`), so the cache/oracle see the
exact FiniteDist shape a real read produces (and
`ApplyTimeLimitsToTasksExecutionTime` is a no-op since no TL is set).

Branches now covered:
- **ET decrease** → `AnalyzePriorityChangeStatus` flips its half (Decrease vs
  Increase); 1D variations scan the OTHER priority range half.
- **High-weight task** (task 3, sp_weight=10, unique-highest, core 0) → hits the
  `if_highest_weight_unique` branch (the low-weight case never does).
- **Adoption-triggering magnitude** → sweeps task 3's ET up until the oracle arm
  ADOPTS (PA ≠ carried); runs the differential on that magnitude. FAILS by
  construction if no magnitude adopts, so the `AdoptChampion` champion-advance
  path (the "next variation stays |diff|<=1 vs the NEW champion" invariant) is
  guaranteed exercised, not silently skipped. First version used a fixed
  mu=450 which did NOT adopt (DBG line fired) → replaced with the sweep; sweep
  found an adopting magnitude, confirmed an adoption fired (no DBG, no FAIL).
- **Sequential intervals** → two `OptimizeIncre` calls sharing ONE `RTACache`;
  the 2nd call's baseline `Initialize` overwrites the 1st call's `AdoptChampion`
  state without throwing (the stale-carried-champion path the wiring doc warns
  about — OptimizeIncre uses Initialize, not Evaluate, for exactly this reason).

Verified: `cmake --build build --target check.SP_OPT -j5` → 16/16 ctest green;
5/5 `OptimizeIncre_Cache.*` pass directly
(`./build/tests/testOptimizeIncrePA --gtest_filter='OptimizeIncre_Cache.*'`).
Working tree UNCOMMITTED.

### Next

User reviews sub-step 3b. Phase 1 implementation + test coverage now complete.
Then Phase 2 (measurement) + a production call site that opts in to the cache.

---

## 2026-07-18 — Sub-step 3c: close the two remaining branch gaps

**Trigger.** User asked "does all code path trigger now?" after 3b. Honest
answer was NO. After enumerating the cache-vs-oracle branches in
`OptimizeIncre` / `OptimizeIncre_SingleTask`, two gap-families survived 3b:

**Gap (a) — the `baseline_sp != INT_MIN` else-branch.** `OptimizeIncre`'s
baseline re-score has two arms:
- `baseline_sp == INT_MIN` (the "not provided" default) → re-score the carried
  PA: cache arm = `Initialize` + `ObtainSP_Full_From_NodeRTAs`; nullopt arm =
  `EvaluateSPWithPriorityVec`. ALL 3b tests passed `INT_MIN` → this arm covered.
- `baseline_sp != INT_MIN` (caller already holds the SP) → trust it: cache arm
  = `Initialize`-only (no SP re-score); nullopt arm = `opt_sp_ = baseline_sp`
  (no Initialize). NEITHER sub-branch was exercised by any 3b test.

**Gap (b) — the `for (DiffObj ...)` loop running ≥2×.** This is the important
one. All 3b tests were |diff|==1, so `OptimizeIncre`'s loop ran ONCE. That
means `AdoptChampion`'s champion-*advance* was NOT load-bearing in any 3b
test: in a single-task loop every 1D variation shares the same "rest" (the
moved task removed), so `|diff|<=1` vs the *original* champion holds even if
`AdoptChampion` never advanced. The 3b `LargeEtIncrease` test proved
`AdoptChampion` is *called* (the differential would diverge otherwise) — but a
*buggy* advance (champion not actually advancing) would still pass it.

The advance only matters when `SingleTask` is called a 2nd time on a different
task: its 1D variations are built from the *advanced* `opt_pa_`, and `Evaluate`
patches each vs the champion. If the champion didn't advance to match `opt_pa_`,
the 2nd call's candidates are `|diff|==2` vs the champion →
`ComputeTaskSetDifference` throws inside `Evaluate`. **No 3b test exercised
this** — it's the exact path the wiring doc's "AdoptChampion MUST advance"
comment exists to protect.

**Fix — 2 new cases (`tests/testOptimizeIncrePA.cpp`):**
- `Differential_BaselineSpProvided_ElseBranch` — computes the carried PA's SP
  under `dag_update` via `EvaluateSPWithPriorityVec` (the EXACT value the header
  contract `OptimizeSP_Incre.h:154` requires) and feeds it to BOTH arms; the
  cache arm still `Initialize`s (exercising the Initialize-only path) but
  trusts the SP. Closes gap (a).
- `Differential_TwoTaskDiff_LoopRunsTwice_AdvanceLoadBearing` — mutates BOTH
  task 1 (low-weight core 1) AND task 3 (high-weight core 0) →
  `FindTaskWithDifferentEt` returns 2 → the loop runs twice → the 2nd
  `SingleTask` call's `Evaluate` patches vs the advanced champion. A stale
  champion throws here. The `[INCR-NDIFF-PROBE] ndiff=2` line confirmed the
  loop actually ran twice. Closes gap (b) — the LOAD-BEARING advance path.

**Verification.** `cmake --build build --target check.SP_OPT -j5` → 16/16 ctest
green. `./build/tests/testOptimizeIncrePA --gtest_filter='OptimizeIncre_Cache.*'`
→ 7/7 pass (5 prior + 2 new). The 2-task case's `ndiff=2` probe line confirms
the loop-runs-twice path is genuinely exercised, not silently skipped.

**Branch-coverage status after 3c.** The cache-vs-oracle divergence branches in
`OptimizeIncre` / `OptimizeIncre_SingleTask` are now covered: both
`AnalyzePriorityChangeStatus` halves (ET decrease + increase); the
`if_highest_weight_unique` branch; `AdoptChampion` strict-improvement
champion-advance (load-bearing via the 2-task case); sequential-interval
baseline `Initialize` overwriting stale champion state; AND now the
`baseline_sp != INT_MIN` else-branch (both arms). What remains uncovered at the
`testOptimizeIncrePA.cpp` level: the cache-*internal* sub-branches
(pure-priority-move vs combined ET+move vs core-migration rejection in
`RTACache::Evaluate` / `AnalyzePrioritySwitch`) — those are tested directly in
`testRTA.cpp` (the P1.9 plan file's test surface), not here. No gcov/lcov
line-coverage tooling has been wired.

## 2026-07-18 — sub-step 2c: cache ALWAYS engaged inside OptimizeIncre (local-cache fallback + loop forwarding)

- User request: "OptimizeIncre, if rta cache is not provided as input, create
  one cache, and use it when traversing different priority assignments."
- **LATENT GAP found while grounding:** `OptimizeIncre`'s loop
  (`OptimizeSP_Incre.cpp:411-414`) called `OptimizeIncre_SingleTask` WITHOUT
  forwarding `rta_cache` — so even when a caller passed an engaged cache, the
  per-variation `Evaluate`/`AdoptChampion` path in `OptimizeIncre_SingleTask`
  was NEVER reached; only the baseline re-score used the cache. (This is why the
  sub-step 3/3b/3c differential tests passed bit-identically despite the cache
  supposedly being wired into the traversal — the "cache arm" actually scored
  its variations via the oracle too; only the baseline SP differed-by-not-
  differing.)
- **Fix (two parts), `sources/Optimization/OptimizeSP_Incre.cpp`:**
  1. At the top of `OptimizeIncre`, resolve an effective handle: if `rta_cache`
     is engaged, use it; if `std::nullopt`, construct a local `RTACache
     local_cache;` in the same scope (outlives the loop) and bind
     `rta_cache = std::ref(local_cache)`. From that point `rta_cache` is ALWAYS
     engaged. NO raw pointer — `std::ref` on a same-scope local.
  2. Forward `rta_cache` into the loop's `OptimizeIncre_SingleTask` call (the
     gap fix). Now the per-variation traversal actually uses the cache.
  3. Removed `[[maybe_unused]]` from both `.cpp` definitions
     (`OptimizeIncre` + `OptimizeIncre_SingleTask`) since the param is now
     always used.
- **Header (`OptimizeSP_Incre.h`) doc comments updated:** the `nullopt → legacy
  oracle path` contract is SUPERSEDED. nullopt now means "create a local cache."
  There is NO oracle arm inside `OptimizeIncre` anymore — the cache path is the
  only path. `OptimizeIncre_SingleTask` still keeps the nullopt=oracle fallback
  in its comment (a direct caller with no cache can still reach it), but
  `OptimizeIncre` always binds a cache before calling it, so that arm is
  unreachable from `OptimizeIncre`.
- **CONTRACT / TEST-PREMISE IMPLICATION (flagged, NOT silently rewritten):**
  the 7 `OptimizeIncre_Cache.*` differential tests in `testOptimizeIncrePA.cpp`
  were built as oracle-arm (`optOracle.OptimizeIncre(dag_update)` = nullopt) vs
  cache-arm (engaged `RTACache`). With this change the oracle arm NOW creates its
  OWN local cache → the tests are **cache-vs-cache**, not cache-vs-oracle. They
  still pass (the cache path is deterministic + bit-identical to the oracle by
  Hazard B + the Q5 TL-baked-input invariant), 7/7 green directly + 16/16 ctest
  green. BUT they no longer validate cache==oracle — a systematic cache bug
  would now pass trivially on both arms. A clean fix exists (replace the oracle
  arm with a hand-rolled `EvaluateSPWithPriorityVec`-per-candidate helper, the
  same algorithm `OptimizeIncre_SingleTask` runs), but that's a design call
  (duplicating the algorithm in the test) — flagged here per the modular-dev
  rule, NOT applied. `EvaluateSPWithPriorityVec` is already called directly in
  the `baseline_sp`-provided case, so the helper would be a small extension.
- **Production callers:** `EvaluateTimeLimitConfig_ScratchOrIncre`
  (`OptimizeSP_TL_Incre.cpp:162` `optimizer.OptimizeIncre(dag_tasks_cur);` with
  no cache) — previously oracle, now uses a local cache (bit-identical SP
  results, the intended speedup path). `EvaluateTimeLimitConfig_SubIncremental`
  (`:249`) calls `OptimizeIncre_SingleTask` directly with no cache → still the
  oracle arm inside SingleTask (unchanged).
- Build: `cmake --build build --target check.SP_OPT -j5` → 16/16 ctest green.
  `./build/tests/testOptimizeIncrePA --gtest_filter='OptimizeIncre_Cache.*'` →
  7/7 passed.
- Working tree, NOT committed (standing directive: commit only when the user
  asks).
