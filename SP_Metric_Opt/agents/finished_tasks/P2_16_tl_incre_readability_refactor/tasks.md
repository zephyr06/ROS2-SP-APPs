# P2.16 — Tasks (working checklist)

> User-rescoped 2026-07-26: original 4 steps → **Phase A (name changes) → Phase B
> (break inheritance) → Phase C (docs)**. Original Steps 3 (trim history comments) & 4
> (extract `SumTimeLimitsExcludingSentinel`) DEFERRED until after Phase B.
> Gate = SP bit-identical (17/17 ctest, `--clean-first`). Agent `git add`s only; user commits.

## Step 1 — Delete dead `TraverseTimeLimitOptions` + stale debug print  ✅ COMMITTED
- [x] **1a–1e.** Removed decl (`OptimizeSP_TL_Incre.h`) + debug-print block (`UpdateRecords`).
  Committed as `d860aac5 "slight refactor"` (-10 lines: 3 header + 7 cpp).

---

## Phase A — Pure name changes (behavior-neutral; build must stay green after each stage)

Decisions (user, 2026-07-26):
- `EvaluateTimeLimitConfig_SubIncremental` → **`OptimizeIncreSingleTask`** (drops "TimeLimit" —
  it *runs* the incremental opt, TL is just the config; env tasks call it too, not only TL-changed).
- `EvaluateTimeLimitConfig_ScratchOrIncre` → **`CallOptimizerGivenTimeLimits`**.
- `OptimizeOneTaskTimeLimit` → **`OptimizeOneTaskWithTimeLimit`**.
- `OptimizeSingleTaskTimeLimit_Impl` → **`WalkOneTaskWithTimeLimitOptions`**.
- `Optimize_w_TL_ScratchOrIncre` → **`DispatchReoptOrIncremental`** (expand scope to
  `SimulationOrchestrator.cpp` + `testINCRTimeout.cpp` + `testIncreOpt_w_TL.cpp`).
- `PerformCoordinateDescentForTaskConfigOpt` → **REMOVE** (1 real caller `.cpp:792`; inline
  `RunIntervalDescent(Reopt,…)`; repoint test/`parameters.yaml` comment refs).
- `K` → **`beam_search_width`** in TL-subclass signatures only (base class `OptimizePA_Incre`
  keeps `K` — out of scope; pass positionally). Remove `K` where unused (`OptimizeIncreSingleTask`
  has `(void)K;` at `:171`).

**Name-proximity note:** `OptimizeIncreSingleTask` (new) sits one underscore + cap away from
base-class primitive `OptimizeIncre_SingleTask` (`OptimizeSP_Incre.{h,cpp}`) — same-instance
callability confusion. **Phase B (break inheritance) dissolves this**; Phase A accepts the residual.

### Stage A1 — `OptimizeSP_TL_Incre.{h,cpp}` internal renames + remove wrapper  ✅ COMMITTED `6d068ec9`
- [x] **A1a.** `OptimizeSingleTaskTimeLimit_Impl` → `WalkOneTaskWithTimeLimitOptions`: header
      decl + `.cpp` def + 2 call sites in `OptimizeOneTaskWithTimeLimit`.
- [x] **A1b.** `OptimizeOneTaskTimeLimit` → `OptimizeOneTaskWithTimeLimit`: header decl + `.cpp`
      def + call site in `WalkSerializedTaskQueue`.
- [x] **A1c.** `EvaluateTimeLimitConfig_SubIncremental` → `OptimizeIncreSingleTask`: header decl
      + `.cpp` def + CoutError string + 2 call sites + comment refs. **`K` param removed**
      (unused `(void)K;` dropped) + `K_cap` local + lambda arg dropped; `:363` call updated.
- [x] **A1d.** `EvaluateTimeLimitConfig_ScratchOrIncre` → `CallOptimizerGivenTimeLimits`: header
      decl + `.cpp` def + CoutError string + 2 call sites.
- [x] **A1e.** `PerformCoordinateDescentForTaskConfigOpt` REMOVED: header decl + comment block +
      `.cpp` def deleted; `RunIntervalDescent(beam_search_width, time_limits,
      IntervalDescentMode::Reopt, dag_tasks_prev_pre_tl)` inlined at the sole caller
      (`ReOptimizePeriodic`); stale `:647` comment repointed.
- [x] **A1f.** `K` → `beam_search_width` in all TL-subclass signatures (`OptimizeWithTimeLimitOptDisabled`,
      `OptimizeIncre_w_TL`, `ReOptimizePeriodic`, `Optimize_w_TL_ScratchOrIncre`, `WalkSerializedTaskQueue`,
      `PerformSerializedTaskQueueOptimization`, `RunIntervalDescent`, `SeedBaselineAndArmCache`,
      `OptimizeOneTaskWithTimeLimit`). Base-class `K` untouched (positional pass-through).
- [x] **A1g.** Build gate passed via A2h (A1 alone leaves tests broken — full green at A2h).
- [x] **A1h.** `git add` the 2 files (done with A2i as one staged set).

### Stage A2 — `tests/testIncreOpt_w_TL.cpp` test-seam + call-site renames  ✅ COMMITTED `6d068ec9`
- [x] **A2a.** `EvaluateTimeLimitConfig_ScratchOrIncre` → `CallOptimizerGivenTimeLimits`:
      overrides in `RecordingDispatcherOpt`, `StartTLStub` (×2), `StubTLWalkOptimizer` + all
      comment refs + the `:2033` recursive call (via `MakeScratchOrIncreEval`).
- [x] **A2b.** `EvaluateTimeLimitConfig_SubIncremental` → `OptimizeIncreSingleTask`: overrides in
      `CounterDispatcherSynthetic`, `ReoptFlagOnMultiTLFlexibleSynthetic`; comment refs; **`K` arg
      dropped** at override sigs + calls (mirror A1c).
- [x] **A2c.** `OptimizeOneTaskTimeLimit` → `OptimizeOneTaskWithTimeLimit`: comment ref `:1167`.
- [x] **A2d.** `OptimizeSingleTaskTimeLimit_Impl` → `WalkOneTaskWithTimeLimitOptions`: 7 direct
      call sites (`:2101`–`:2243`) + 3 comment refs (`:1934,:1991,:1992`).
- [x] **A2e.** `PerformCoordinateDescentForTaskConfigOpt` comment refs repointed to
      `RunIntervalDescent(Reopt)` at `:970,:1144,:1162,:1280,:1285,:1961,:1992`.
- [ ] **A2f.** `Optimize_w_TL_ScratchOrIncre` → `DispatchReoptOrIncremental`: **DEFERRED to
      Stage A3** (cross-file cascade — `SimulationOrchestrator.cpp` + `testINCRTimeout.cpp` +
      this file's ~10 call sites). The `.h`/`.cpp` kept the old name; renaming it cascades across
      files outside the 2-file A1/A2 scope. Folded into A3a–A3c as one cross-file unit.
- [x] **A2g.** `K` → `beam_search_width` in the test-seam override signatures (mirror A1f). Test
      *call sites* passing literal `2` left positional (per A2g guidance).
- [x] **A2h.** Build gate: `cmake --build build_test --target check.SP_OPT -j5 --clean-first`
      → **17/17 ctest green** (20.43s; `testIncreOpt_w_TL` 2.09s).
- [x] **A2i.** `git add` `testIncreOpt_w_TL.cpp` (with A1h as one staged set).

### Stage A3 — Cross-file `Optimize_w_TL_ScratchOrIncre` → `DispatchReoptOrIncremental` cascade
> Folded A2f in: the `.h`/`.cpp` declaration is renamed here (not in A1/A2) so the 2-file A1/A2
> set stays self-contained and reviewable. A3 renames the symbol in the `.h`/`.cpp` FIRST, then
> every caller across files. Single review/commit unit.
- [ ] **A3a.** `OptimizeSP_TL_Incre.{h,cpp}`: rename the decl (`:94`) + def (`:576`) — the symbol
      itself (A1f already did `K`→`beam_search_width` on its signature).
- [ ] **A3b.** `tests/testINCRTimeout.cpp`: 2 call sites (`:95,:154`) + comment refs
      (`:28,:104,:119,:154,:167`). (`K`=literal `2` at calls — leave positional.)
- [ ] **A3c.** `sources/RTDA/ImplicitCommunication/SimulationOrchestrator.cpp`: 3 call sites
      (`:325,:334,:342`) + comment refs (`:24,:286,:293`). NOT in the parallel agent's set.
- [ ] **A3d.** `tests/testIncreOpt_w_TL.cpp`: ~10 call sites (`:1299,:1310,:1343,:1345,:1347,
      :1363,:1389,:1400,:1423`) + comment refs (`:1040,:1047`).
- [ ] **A3e.** Comment-only stragglers: `tests/testScheduleSimulate.cpp:758,:803` (mentions
      `EvaluateTimeLimitConfig_ScratchOrIncre` → `CallOptimizerGivenTimeLimits` — these are ALREADY
      stale vs A1d; fix in passing) + `sources/parameters.yaml:15` (mentions
      `PerformCoordinateDescentForTaskConfigOpt` → repoint to `RunIntervalDescent` or delete).
      `parameters.yaml` is NOT in anyone's set — safe.
- [ ] **A3f.** Build gate: `cmake --build build_test --target check.SP_OPT -j5 --clean-first`
      → 17/17 ctest green.
- [ ] **A3g.** `git add` the A3 files; report for review+commit.

---

## Phase B — Break `OptimizePA_Incre_with_TimeLimits` inheritance from `OptimizePA_Incre`
- [ ] **B1.** Enumerate every inherited symbol the TL subclass uses (`OptimizeFromScratch`,
      `OptimizeIncre`, `OptimizeIncre_SingleTask`, `dag_tasks_`, `opt_sp_`, `opt_pa_`,
      `IfInitialized`, `UpdateExtDistBasedOnTimeLimit`, `sp_parameters_`, `eval_count_`,
      `res_opt_`-adjacent, etc.). Decide composition (member `OptimizePA_Incre optimizer_`) vs
      a common `OptimizePA_Base` base.
- [ ] **B2.** Plan the re-plumb (filed for user review BEFORE editing — this is structural, not
      behavior-neutral-label).
- [ ] **B3.** Execute in small TDD-pinned stages; 17/17 ctest after each.
- [ ] **B4.** `git add`; report for review+commit.

---

## Phase C — Update docs
- [ ] **C1.** `goal.md` scope section (Phase B may change which symbols are public/private).
- [ ] **C2.** `agents/overall_tasks.md` P2.16 row (coordinate with parallel agent — that file is
      in their modified set; do NOT edit concurrently).
- [ ] **C3.** Memory file `p216-tl-incre-readability-refactor.md` + `MEMORY.md` pointer.
- [ ] **C4.** Cross-link from P2.11 memory (this task consumes its deferred cleanup).

---

## Deferred (post-Phase-B, if still relevant)
- **Trim dense inline history comments** (`SeedBaselineAndArmCache:379-383`, `RunIntervalDescent`,
  `PerformSerializedTaskQueueOptimization:646-648`, stale `:647`). Comments only.
- **Extract `UpdateRecords` TL-sum tie-break** → `SumTimeLimitsExcludingSentinel`.
