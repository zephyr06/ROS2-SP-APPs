# P1.13 — Tasks (working checklist)

> See `goal.md` for scope and `api_design.md` for the API under discussion.
> DESIGN-FIRST: no source edits until the user signs off on the API.

---

## Phase 0 — API design (CURRENT)

- [x] File the task folder (`goal.md` / `api_design.md` / `tasks.md` /
  `dev_log.md`). ← DONE 2026-07-18.
- [x] Ground the design in the actual `OptimizeIncre` / `OptimizeIncre_SingleTask`
  loop + the frozen `RTACache` contract + the oracle `EvaluateSPWithPriorityVec`
  path. ← DONE 2026-07-18 (facts in `api_design.md` "Grounded facts").
- [x] Draft the API-shape questions Q1–Q6 with a recommendation each.
  ← DONE 2026-07-18.
- [ ] **User reviews `api_design.md` and picks:**
  - Q2 (where the cache lives): A / B (rec) / C.
  - Q3-open + Q5 (tl/dag plumbing): resolve after confirming
    `UpdateExtDistBasedOnTimeLimit` == `ApplyTimeLimitsToTasksExecutionTime`
    and whether `dag_tasks_update` arrives TL-baked at the base-class
    `OptimizeIncre`.
  - Q4 (two new `ObtainSP_*` helpers; path SP recomputed per candidate).
  - Q6 (no flag; pointer is the gate).
- [ ] Confirm the chosen API, freeze it in `api_design.md`.

## Phase 1 — Implementation (gated on Phase 0 sign-off)

- [x] **Sub-step 1 (DONE 2026-07-18, SUPERSEDED by the in-place Hazard B fix
  below):** originally added `ObtainSP_DAG_From_Dists_With_Perf_Coeff` +
  `ObtainSP_Full_From_NodeRTAs` to `SP_Metric.h/.cpp` (Hazard B as a parallel
  helper, existing `ObtainSP_DAG_From_Dists` UNTOUCHED). User re-examined and
  directed the in-place fix instead (see sub-step 1b).
- [x] **Sub-step 1b (DONE 2026-07-18, 16/16 ctest green, NOT committed) —
  Hazard B FIXED IN PLACE + `tsp_ext_path` RESTORED.** User: "i noticed
  ObtainSP_DAG_From_Dists doesn't consider perf_coefficient, that is a bug if
  true. fix it rather than adding ObtainSP_DAG_From_Dists_With_Perf_Coeff?" →
  "if the perf coefficient is processed in other code, i think that's wrong
  place to handle it. we should handle it in ObtainSP_DAG_From_Dists, should we
  modify other functions to make sure perf coefficient is always effective and
  only effective once?" → "i need to make sure all computation results are
  correct." BINDING principle: perf coefficient is a per-task multiplier that
  belongs in the SP-eval fn, applied EXACTLY ONCE. Changes (option (a) — full
  clean end-state):
  - `ObtainSP_DAG_From_Dists` node loop multiplies
    `dag_tasks.tasks[i].GetPerfCoefficient()` (mirrors oracle
    `ObtainSP_TaskSet:62,66`).
  - `ObtainSPFromRTAFiles` — removed the `update_node_weight(0, tsp_weight)`
    weight-slot hack + the dead `GetAvgTaskPerfTerm` call + the stale dist
    comment + the TSP-specific `assert`. TSP perf now flows through
    `perf_coefficient` uniformly.
  - DROPPED `ObtainSP_DAG_From_Dists_With_Perf_Coeff` (`.cpp` + `.h`) —
    redundant.
  - `ObtainSP_Full_From_NodeRTAs` simplified to call the corrected
    `ObtainSP_DAG_From_Dists`.
  - `GetAvgTaskPerfTerm` KEPT (own test at `testSP.cpp:347`).
  - **USER CORRECTION (2026-07-18): `tsp_ext_path` RESTORED.** I had dropped
    it entirely — WRONG. After Hazard B, `ObtainSP_DAG_From_Dists` calls
    `GetPerfCoefficient()`, which reads `execution_time_dist.GetAvgValue()`.
    For TSP that dist was the ANALYTIC YAML GAUSSIAN (mu≈202.3), not the
    measured ET; the `tsp_ext_path` file (`TSP_execution_time_115_125.txt`, 20
    samples ~209–211) is TSP's REAL ET. The OLD code never loaded those samples
    into `dag_tasks.tasks[0].execution_time_dist` — it only reduced them to an
    average perf coefficient for the weight slot. So dropping `tsp_ext_path`
    left `GetPerfCoefficient()` reading the wrong dist (a correctness
    regression on the very quantity the principle protects). FIX (user's
    direction): `ObtainSPFromRTAFiles` reads `tsp_ext_path`'s measured ET via
    `ReadTxtFile` and writes `dag_tasks.tasks[0].execution_time_dist =
    FiniteDist(tsp_ext_times, granularity)` BEFORE scoring. `tsp_ext_path`
    param + CLI `--tsp_ext_path` arg restored in all 4 places (`SP_Metric.h`,
    `SP_Metric.cpp`, `tests/testSP.cpp`, `tests/AnalyzeSP_Metric.cpp`).
  - VALUE-PRESERVING: TSP measured avg ~210 → first perf bucket `[184.1,
    397.5)` → perf 0.5 (== old empirical 0.5 == analytic 0.5) →
    `EXPECT_EQ(2.0, sp_value_overall)` pin holds; other 3 tasks `perf_coeff=1.0`
    → unchanged. 16/16 ctest green re-verified.
- [x] **Sub-step 2a (DONE 2026-07-18, signature-only, NOT committed):** added
  `[[maybe_unused]] RTACacheOpt rta_cache = std::nullopt` to the `.cpp`
  definitions of `OptimizeIncre`/`OptimizeIncre_SingleTask`
  (`OptimizeSP_Incre.cpp:275,298`) to match the API-approved header. BODIES
  UNCHANGED (oracle `EvaluateSPWithPriorityVec` path still live, no cache
  wiring yet). Unblocks the lib build → 16/16 ctest green.
- [x] Add the chosen cache plumbing to `OptimizeSP_Incre.h/.cpp` (per Q2
  decision). Default = legacy oracle path (behavior-preserving).
  - [x] Header (`OptimizeSP_Incre.h`): `RTACacheOpt` alias + signatures DONE
    2026-07-18 (API-approved, no raw pointers).
  - [x] `.cpp` signatures (`OptimizeSP_Incre.cpp:275,298`): `RTACacheOpt` param
    added (sub-step 2a, signature-only).
  - [x] **`.cpp` BODIES (`OptimizeSP_Incre.cpp`) — sub-step 2b DONE 2026-07-18
    (16/16 ctest green, NOT committed):**
    - `OptimizeIncre_SingleTask`: per-variation `if (rta_cache)` branch →
      `Evaluate` (≤1-task patch vs champion==opt_pa_) +
      `ObtainSP_Full_From_NodeRTAs` (perf_coefficient included) +
      `AdoptChampion` on strict-improvement (advances champion so the next
      variation stays |diff|<=1). `else` = oracle `EvaluateSPWithPriorityVec`.
    - `OptimizeIncre` baseline re-score: `if (rta_cache)` → `Initialize`
      (full RTA, establishes opt_pa_ as fresh champion — NOT `Evaluate`, which
      would throw vs a stale old-dag champion) + `ObtainSP_Full_From_NodeRTAs`
      for the SP. Honors `baseline_sp` (still `Initialize`s the champion RTA).
    - `no_tl = vector<double>(N, -1.0)` fed to the cache (Q5: call-site
      dag_tasks_update is TL-baked → `ApplyTimeLimitsToTasksExecutionTime`
      no-op → bit-identity).
    - `NodeRtasHolder` local struct COPIES `Evaluate`'s const-ref return
      (points into the cache's `candidate_rta_` buffer, which the next
      `Evaluate` overwrites) → stable for both SP scoring + `AdoptChampion`.
      Mirrors the `RTACacheOpt` no-raw-pointer idiom.
- [x] Wire the champion lifecycle in `OptimizeIncre` (adopt at baseline
  re-score via `Initialize`) + `OptimizeIncre_SingleTask` (adopt on each
  strict-improvement via `AdoptChampion`). (Q3.) DONE 2026-07-18.
- [x] **Differential TDD (sub-step 3) DONE 2026-07-18:**
  `testOptimizeIncrePA.cpp` `OptimizeIncre_Cache.Differential_BitIdentical
  ToOracle_OnSingleEtChange` — two optimizers from the same scratch state,
  `OptimizeIncre(v23)` with `nullopt` (oracle) vs an engaged `RTACache` (cache);
  asserts `opt_pa_` + `opt_sp_` bit-identical. PASSES. Uses the v22→v23 pair
  (|diff|==1, task 1 increase) so the 1D variations exercise the patch path.
- [x] Build/test: `cmake --build build --target check.SP_OPT -j5` (DEBUG) →
  16/16 ctest green. DONE 2026-07-18 (new test ran + passed directly:
  `./build/tests/testOptimizeIncrePA --gtest_filter='OptimizeIncre_Cache.*'`).
- [x] **Sub-step 3b — broader differential coverage DONE 2026-07-18
  (16/16 ctest green, NOT committed).** User: "tests need higher coverage, like
  try different types of input? i think one case cannot cover all passes." The
  single v22→v23 case exercises ONE branch (task 1, ET increase, low-weight
  core-1). Added 4 cases + a shared `ExpectCacheMatchesOracleOnMutation` helper
  (`tests/testOptimizeIncrePA.cpp`):
  - `Differential_EtDecrease_FlipsVariationHalf` — ET DECREASE on task 1 →
    `AnalyzePriorityChangeStatus` flips its half (Decrease vs Increase), so the
    1D variations scan the OTHER half of the priority range.
  - `Differential_HighWeightTask_EtIncrease` — mutate task 3 (sp_weight=10,
    unique-highest, core 0) → hits the `if_highest_weight_unique` branch the
    low-weight case never reaches.
  - `Differential_LargeEtIncrease_TriggersAdoption` — sweeps task 3's ET up
    until the oracle arm ADOPTS (returns PA ≠ carried); runs the differential
    on THAT magnitude. FAILS if no magnitude adopts → guarantees the
    `AdoptChampion` champion-advance path (next variation stays |diff|<=1 vs the
    NEW champion) is actually exercised, not silently skipped.
  - `Differential_SequentialIntervals_ReinitializeChampion` — two back-to-back
    `OptimizeIncre` calls sharing ONE `RTACache`; the 2nd call's baseline
    `Initialize` must overwrite the 1st call's `AdoptChampion` state without
    throwing (the stale-carried-champion path the wiring doc warns about).
  - Helper builds synthetic |diff|==1 mutations by overwriting one task's
    `execution_time_dist` with a `FiniteDist(GaussianDist, min, max, 5)` — the
    SAME construction path `ReadDAG_Tasks` uses (`RegularTasks.cpp:75-79`). 5/5
    cache tests pass directly; the adoption case confirmed an adoption fired.
- [x] **Sub-step 3c — close the two remaining branch gaps DONE 2026-07-18
  (16/16 ctest green, NOT committed).** User: "does all code path trigger now?"
  Honest answer was NO — two gap-families remained after 3b:
  (a) the `baseline_sp != INT_MIN` `else` branch in `OptimizeIncre`'s baseline
  re-score (cache arm = `Initialize`-only, no `ObtainSP_Full_From_NodeRTAs`
  re-score; nullopt arm = `opt_sp_ = baseline_sp`, no Initialize); every 3b
  test passed `INT_MIN`.
  (b) the `for (DiffObj ...)` loop running ≥2× — all 3b tests were |diff|==1 so
  the loop ran once, meaning `AdoptChampion`'s champion-*advance* was NOT
  load-bearing (a buggy advance would still pass). The advance only matters
  when the 2nd `SingleTask` call's variations are built from the *advanced*
  `opt_pa_` and patched vs the champion; if the champion didn't advance,
  `ComputeTaskSetDifference` throws inside `Evaluate`.
  Added 2 cases (`tests/testOptimizeIncrePA.cpp`):
  - `Differential_BaselineSpProvided_ElseBranch` — computes the carried PA's SP
    under `dag_update` via `EvaluateSPWithPriorityVec` (the EXACT value the
    header contract `OptimizeSP_Incre.h:154` requires) and feeds it to BOTH
    arms; cache arm still `Initialize`s (the Initialize-only path) but trusts
    the SP. Closes gap (a).
  - `Differential_TwoTaskDiff_LoopRunsTwice_AdvanceLoadBearing` — mutates BOTH
    task 1 AND task 3 → `FindTaskWithDifferentEt` returns 2 → loop runs twice →
    the 2nd `SingleTask` call's `Evaluate` patches vs the advanced champion;
    a stale champion throws here. `[INCR-NDIFF-PROBE] ndiff=2` confirmed the
    loop ran twice. Closes gap (b) — the LOAD-BEARING advance path. 7/7 cache
    tests pass directly.
- [x] **Sub-step 2c DONE 2026-07-18 (16/16 ctest green, NOT committed) — cache
  ALWAYS engaged inside `OptimizeIncre` (local-cache fallback + loop
  forwarding).** User: "OptimizeIncre, if rta cache is not provided as input,
  create one cache, and use it when traversing different priority assignments."
  Found + fixed a LATENT GAP: the loop (`OptimizeSP_Incre.cpp:411-414`) was
  calling `OptimizeIncre_SingleTask` WITHOUT forwarding `rta_cache`, so even an
  engaged cache never reached the per-variation `Evaluate`/`AdoptChampion` path
  (only the baseline re-score used it — explains why 3/3b/3c differentials passed
  bit-identically despite the cache "being wired"). Fix: (1) at the top of
  `OptimizeIncre`, if `rta_cache` is nullopt construct a same-scope local
  `RTACache` + bind via `std::ref` (no raw pointer) → `rta_cache` always engaged;
  (2) forward `rta_cache` into the `OptimizeIncre_SingleTask` loop call; (3) drop
  `[[maybe_unused]]` from both `.cpp` defs. Header doc comments updated: the
  `nullopt → legacy oracle path` contract is SUPERSEDED — nullopt now creates a
  local cache; there is NO oracle arm inside `OptimizeIncre` anymore (cache path
  is the only path, bit-identical to oracle by Hazard B + Q5 TL-baked-input).
  **TEST-PREMISE CAVEAT (OPEN follow-up):** the 7 `OptimizeIncre_Cache.*`
  differentials are now cache-vs-cache (the nullopt "oracle arm" creates its own
  local cache) → still 7/7 green + 16/16 ctest, but no longer validate
  cache==oracle. Clean fix = hand-rolled `EvaluateSPWithPriorityVec`-per-candidate
  oracle arm; design call, awaiting user direction (see Follow-ups).

## Phase 2 — Measurement (deferred)

- [ ] Profile candidate-eval time at N=6/10/16 with and without the cache
  (pointer on vs nullptr).
- [ ] Document the speedup; confirm no SP regression (bit-identical).

## Follow-ups (out of Phase order, surfaced 2026-07-18)

- [ ] **sub-step 2c test-premise (ACCEPTED as-is per user 2026-07-18, NOT a
  blocking gap):** the 7 `OptimizeIncre_Cache.*` differential tests are now
  cache-vs-cache (the nullopt "oracle arm" creates a local cache after 2c). They
  pass 7/7 + 16/16 ctest green but no longer validate cache==oracle — HOWEVER the
  core cache==oracle bit-identity is still validated by the DIRECT
  `OptimizeIncre_SingleTask.Differential_BitIdenticalOnSingleEtChange` test
  (`tests/testOptimizeIncrePA.cpp:381`): Path A = `OptimizeIncre` (forwards its
  local cache → SingleTask takes the cache branch) vs Path B =
  `OptimizeIncre_SingleTask` called directly with nullopt → SingleTask does NOT
  auto-create a cache (only `OptimizeIncre` does) → takes the `else` oracle
  branch (`EvaluateSPWithPriorityVec` per candidate). So `SingleTask`'s nullopt
  genuinely = oracle, and that test is a true cache-vs-oracle differential
  (passes). The 7 `OptimizeIncre_Cache.*` tests' value is BRANCH BREADTH
  (ET-decrease / high-weight / adoption-trigger / sequential-intervals /
  `baseline_sp`-provided / two-task loop), not the cache==oracle claim — that
  claim is covered by the SingleTask test. User: "that's an issue to fix for the
  unit tests, but i suppose tests cover OptimizeIncre_SingleTask is okay." Clean
  fix if ever wanted = hand-rolled `EvaluateSPWithPriorityVec`-per-candidate
  oracle arm in the 7 tests; NOT pursued (low value, duplicates the algorithm).
