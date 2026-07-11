# P0.5 — Redesign the Optimizer Iteration Process — Dev Log

> **RESOLVED 2026-07-10.** All five phases complete; code committed
> (`a8dba07f`→`7fa2e9d2`); 46 `testIncreOpt_w_TL` + 16/16 ctest green. Functional
> TL-init bug (former P0.1) subsumed by construction. Resolution recorded in
> `agents/finished_tasks/summary.md`; rationale in memory
> `p05-subsumes-tl-init-bug.md`. Entries below are the original working log, kept
> for the record.

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-08

- Task created per user (2026-07-08): redesign the optimizer iteration process
  is the #2 priority, after the TL bug (P0.1). User chose the **full scope**
  (flow + state), so this task absorbs the former P0.1b data-member refactor.
  P0.1 is now bug-fix-only (P0.1a); its `tasks.md` P0.1b section was moved here.
- Predecessor: ~~P0.1 (adopted-TL YAML fix) should land first so the redesign
  works against a file that is a faithful prior.~~ **REVERSED 2026-07-08** — see
  the re-scoping entry below: P0.5 subsumes the functional TL-init bug by
  construction; P0.1 is demoted to inspectability-only and is no longer a
  predecessor.
- **Design decided by user (2026-07-08).** User rejected the open-ended
  "propose the target shape" plan and chose the **incumbent-state** approach:
  - `res_opt_` becomes the single durable incumbent store (no parallel cache).
  - `has_incumbent_` bool replaces `prev_optimizer_.IfInitialized()` as the
    interval-0 gate (the old gate only checked `!opt_pa_.empty()`, which was the
    root of the `sp_parameters_` desync).
  - `CommitIncumbent(pa, sp, tl)` — one helper replacing the 8 scattered sync
    assignments across `UpdateRecords` (`:128-134`) and `SeedStateFromIncumbent`
    (`:457-476`); one writer → the desync class of bug is structurally
    impossible.
  - `BuildChallengerFromIncumbent()` — builds a throwaway `OptimizePA_Incre`
    from `res_opt_` (reconstructed adopted-TL DAG + carried PA + `sp_parameters_`)
    each incremental interval. The challenger is transient; the incumbent is
    durable.
  - The DAG-advance side-effect of `OptimizeIncre` (`OptimizeSP_Incre.cpp:296`)
    becomes harmless (throwaway local); the adopted TL it persists into
    `res_opt_.id2time_limit` is the real invariant, reconstructed next interval.
    The frozen-baseline propagation path simply ceases to exist.
  - Zero public-API change; blast radius = `OptimizeSP_TL_Incre.{h,cpp}` +
    `testIncreOpt_w_TL.cpp` + a comment-only touch in `OptimizeSP_Incre.cpp`.
- The full spec (problem, design, the diff-baseline invariant that must survive
  the reconstruction, the 4-phase migration, scope boundaries, open
  re-derivation questions) is recorded in **`design.md`**. `goal.md` and
  `tasks.md` were rewritten to reflect the decided design (the earlier "propose,
  do not decide" framing is superseded). `tasks.md` now carries the per-step
  checklist including the per-test rewrite spec for the 3 frozen-baseline DAG-ET
  assertions (Phase 3a) — they rewrite to observe the carried TL via `res_opt_`,
  proving the diff invariant survives the reconstruction.
- Confirmed against the current code (read `OptimizeSP_TL_Incre.{h,cpp}`,
  `OptimizeSP_Incre.{h,cpp}`, `OptimizeSP_Base.h`, the frozen-baseline test
  block `testIncreOpt_w_TL.cpp:880-1113`, and the dispatcher-routing tests):
  - The DAG-advance side-effect is `dag_tasks_ = dag_tasks_update` at
    `OptimizeSP_Incre.cpp:296`; it only propagates to the incumbent via
    `UpdateRecords`'s `prev_optimizer_ = optimizer` copy at `:134` — the fragile
    path the redesign removes.
  - 15 `prev_optimizer_.*` reads across 6 tests (`:678-720`, `:913-950`,
    `:1010-1042`, `:1062-1113`) migrate to `res_opt_`/`has_incumbent_`; the 3
    DAG-ET assertions (`:922`, `:942`, `:1106`) are the rewritten ones.
  - `eval_count_` (`:1085`, `:1098`, `:1140`, `:1151`) and `from_scratch_flags`
    (`:770+`) stay as documented test seams.
- **Scope narrowed vs the v1 goal:** the *flow-shape* re-derivation
  (INCR/REOPT/scratch dispatch, compare-and-keep, cold-start bootstrap) is
  a Phase-3d re-derivation, NOT a precondition of this design. The user's
  decision is specifically the **state** refactor. `ResourceOptResult` overlap
  collapse and `PriorityPartialPath` per-path copies are explicitly out of scope.
- **Re-scoping (2026-07-08, user).** User observed the incumbent-state design
  can also solve the TL-initialization bug, so P0.5 should be worked **first**.
  Verified against the code: the TL-init bug has two faces. (1) *Optimizer-
  internal* — `OptimizeIncre_w_TL` cold-started from the Gaussian-mean TL so the
  update side of `FindTaskWithDifferentEt`'s diff was a point dist at the
  Gaussian-mean TL while the baseline carried the adopted TL → false-flagged
  perf-pair tasks (the P1.1 `ndiff` residual). P0.5 fixes this **by
  construction**: `BuildChallengerFromIncumbent` reconstructs the adopted-TL DAG
  from `res_opt_`, so both diff sides carry the adopted TL — no YAML write
  needed. (2) *External/inspectability* — the on-disk YAML still shows the
  Gaussian; the orchestrator's `setExecutionTime(GetAvgValue())` at
  `SimulationOrchestrator.cpp:395/684` reads it. BUT at `:461-463` each job's
  `execution_time` is clamped to `res.id2time_limit` (the adopted TL) whenever a
  TL exists, so a perf-pair task's runtime ET is already governed by the adopted
  TL, not the Gaussian — the external read only matters for gaussian-only
  (TL=−1) tasks or absent traces. So the external face is debuggability, not
  runtime correctness. **Conclusion:** P0.5 subsumes the functional TL-init bug;
  P0.1 (YAML persistence) is demoted to inspectability-only and is no longer a
  predecessor. The P1.1 `ndiff` 5→~2 probe should pass from P0.5 alone. Recorded
  in memory `p05-subsumes-tl-init-bug`; `goal.md`/`design.md`/`overall_tasks.md`
  updated to reverse the P0.1→P0.5 edge.
- Not yet started. Next step is Phase 1 (additively introduce `has_incumbent_`
  + the two helpers, no call-site change), to be executed in staged
  sub-sessions per the `tasks.md` checklist.

## 2026-07-08 (implementation)

- **Phase 1a DONE.** Added `has_incumbent_` (bool, default false) + declarations
  for `CommitIncumbent(pa, sp, tl)` and `BuildChallengerFromIncumbent()` to
  `OptimizeSP_TL_Incre.h`; documented `time_limit_option_for_each_task_` as
  transient (per-call, not incumbent). No call-site change. Build clean.
- **Phase 1b DONE.** Implemented both helper bodies in `OptimizeSP_TL_Incre.cpp`
  (inserted after `SeedStateFromIncumbent`):
  - `CommitIncumbent` = the `SeedStateFromIncumbent` write-block
    (`opt_sp_`/`opt_pa_`/`res_opt_.SaveTimeLimits`/`UpdatePriorityVec`/`sp_opt`)
    **minus** the `prev_optimizer_` lines, plus `has_incumbent_ = true`. Does
    NOT touch `prev_optimizer_` (caller owns the legacy dual-write).
  - `BuildChallengerFromIncumbent` returns a fresh `OptimizePA_Incre` built from
    `UpdateExtDistBasedOnTimeLimit(dag_tasks_, ReconstructTimeLimitVecFromResOpt())`
    + `sp_parameters_`, with `opt_pa_`=`res_opt_.priority_vec` and
    `opt_sp_`=`res_opt_.sp_opt`. Throwaway by construction.
  No call-site change. Build clean.
- **BUILD-SYSTEM DISCOVERY (critical for all future test runs).** The first
  `ctest` runs after 1b reported "green" but were running a **stale Jul-7 test
  binary** — my Phase 1c test additions hit `undefined reference to
  CommitIncumbent/BuildChallengerFromIncumbent` at link, proving the binary
  predates the 1b source. Root cause: `tests/CMakeLists.txt:1` gates test
  registration on `if(CMAKE_BUILD_TYPE STREQUAL "DEBUG")` — **uppercase DEBUG**.
  The cache had `CMAKE_BUILD_TYPE=Debug` (capitalized), which does NOT match, so
  `gtsamAddTestsGlob` was silently skipped on reconfigure and `ctest` kept
  launching the old binary whose object files predated the source edits. Fix:
  reconfigure with `cmake -DCMAKE_BUILD_TYPE=DEBUG ..` (which produces
  `libSP_OPTDebug.so` — the lib the tests link), then build+run via
  `cmake --build . --target check.SP_OPT -j5`. Recorded in memory
  `sp-opt-test-build-debug-config`. Going forward: ALWAYS use `check.SP_OPT`
  under `DEBUG`; `make SP_OPT` alone builds the lib but not the test exes, and
  plain `ctest` under `Debug` runs stale binaries.
- **Phase 1c DONE (verified for real under DEBUG).** Added two focused unit
  tests to `testIncreOpt_w_TL.cpp` (after `SeedStateFromIncumbent_WritesFullFourTuple`):
  - `CommitIncumbent_WritesFourTupleAndSetsGate` — populates `res_opt_`
    (id2time_limit/sp_opt), `opt_pa_`/`opt_sp_`, sets `has_incumbent_`; asserts
    `prev_optimizer_` stays uninitialized (caller-owned dual-write contract).
  - `BuildChallengerFromIncumbent_ReconstructsAdoptedTlDag` — challenger's
    `dag_tasks_` per-task ET dist equals
    `UpdateExtDistBasedOnTimeLimit(dag_tasks, ReconstructTimeLimitVecFromResOpt())`
    (the diff baseline, made structural); `opt_pa_`/`opt_sp_` mirror `res_opt_`;
    incumbent store unchanged by the build.
  Confirmed `[ OK ]` for both via `ctest -R testIncreOpt_w_TL -V`: **44 tests**
  (was 42; +2 new). 16/16 `check.SP_OPT` green.
- Next: Phase 2 (dual-write). 2a `SeedStateFromIncumbent` calls `CommitIncumbent`
  + legacy `prev_optimizer_` writes; 2b `UpdateRecords` calls `CommitIncumbent`
  + `prev_optimizer_ = optimizer`; 2c the load-bearing flip —
  `EvaluateTimeLimitConfig_ScratchOrIncre` incremental branch replaces
  `OptimizePA_Incre optimizer = prev_optimizer_` with
  `BuildChallengerFromIncumbent()`.

## 2026-07-08 (Phase 2 — dual-write)

- **Phase 2a/2b/2c DONE** (present in working tree; verified green this session).
  The three dual-write edits landed in `OptimizeSP_TL_Incre.cpp`:
  - **2a** `SeedStateFromIncumbent` (`:465-488`): now calls `CommitIncumbent(pa,
    sp, tl)` first, then performs the legacy `prev_optimizer_` writes
    (`UpdateDAG`, `opt_pa_`, `opt_sp_`, `sp_parameters_`). `has_incumbent_` and
    `prev_optimizer_.IfInitialized()` agree after this call.
  - **2b** `UpdateRecords` `should_update` block (`:127-134`): now calls
    `CommitIncumbent(optimizer.opt_pa_, optimizer.opt_sp_, time_limits)`, then
    `prev_optimizer_ = optimizer` (full copy). Single-writer for `res_opt_`/
    `opt_*`/`has_incumbent_`; `prev_optimizer_` is the legacy mirror kept until
    Phase 3b.
  - **2c** the load-bearing flip — `EvaluateTimeLimitConfig_ScratchOrIncre`
    incremental branch (`:159-176`): `OptimizePA_Incre optimizer =
    prev_optimizer_;` → `OptimizePA_Incre optimizer =
    BuildChallengerFromIncumbent();`. The challenger is now transient; the diff
    baseline is reconstructed from `res_opt_` each interval. The `else` branch
    (no incumbent) became an explicit `CoutError` contract-violation (was
    previously the `OptimizeFromScratch` fallback) — the incremental path now
    requires a prior `CommitIncumbent`, which `SeedIncumbentBaseline` always
    establishes first.
- **Verified green under DEBUG.** `cmake --build . --target check.SP_OPT -j5` in
  `build/` (the `CMAKE_BUILD_TYPE=DEBUG` dir per memory
  `sp-opt-test-build-debug-config`): 16/16 ctest pass; `testIncreOpt_w_TL` =
  **44 tests** green. The 6 `prev_optimizer_.*`-reading tests still pass because
  2a/2b dual-write keeps `prev_optimizer_` populated.
- Next: Phase 3a — migrate the 6 `prev_optimizer_.*`-reading tests
  (`:678-720`, `:913-950`, `:1010-1042`, `:1062-1113`) to read `res_opt_` /
  `has_incumbent_` / `BuildChallengerFromIncumbent()`, with the 3 DAG-ET
  assertions rewritten to observe the carried TL via `res_opt_`. They must pass
  against the dual-write code (Phase 2) BEFORE the field is removed in 3b.

## 2026-07-08 (Phase 3a/3b/3c — applied in working tree, logged retroactively)

- **Phase 3a/3b/3c were found APPLIED in the working tree** during the
  2026-07-08 pickup, beyond where the log above stops (Phase 2). Confirmed by
  source inspection + a clean DEBUG build:
  - `OptimizeSP_TL_Incre.h` has **no `prev_optimizer_` member** (only comment
    references at `:179`/`:198`); `has_incumbent_` is the gate.
  - `SeedStateFromIncumbent` (`:465-474`) routes through `CommitIncumbent` only
    — **no** legacy dual-write of `prev_optimizer_` (2a's dual-write dropped).
  - `UpdateRecords` (`:127-141`) routes through `CommitIncumbent` only — **no**
    `prev_optimizer_ = optimizer` (2b's dual-write dropped).
  - `SeedIncumbentBaseline` (`:531`) gates on `has_incumbent_`, not
    `prev_optimizer_.IfInitialized()`.
  - `OptimizeSP_Incre.cpp:288-297` comment rewritten to the
    throwaway-challenger / `res_opt_`-carries-the-adopted-TL model (3c).
  - `testIncreOpt_w_TL.cpp`: all 4 remaining `prev_optimizer_` mentions are
    **comment-only** (no code reads the field). The 6 formerly-`prev_optimizer_`-
    reading tests now read `res_opt_` / `has_incumbent_` /
    `BuildChallengerFromIncumbent()`; the 3 DAG-ET assertions observe the
    carried TL/current DAG via `BuildChallengerFromIncumbent().dag_tasks_` +
    `res_opt_.id2time_limit` (e.g. `OptimizeIncre_AdvancesPrevOptimizerDagTasks`
    at `:992`, `PerformCoordinateDescent_AllMinusOneOnly_...` at `:1151`).
- **Verified green under DEBUG.** `cmake --build . --target check.SP_OPT -j5`
  in `build/` (`CMAKE_BUILD_TYPE=DEBUG` per memory
  `sp-opt-test-build-debug-config`): 16/16 ctest pass; `testIncreOpt_w_TL` =
  **44 tests** green. (The test names `..._AdvancesPrevOptimizerDagTasks` /
  `..._AdvancesPrevOptimizer` are kept as legacy labels; their bodies are
  migrated.)

## 2026-07-08 (Phase 3d — re-derivation)

- **Phase 3d re-derived, not assumed.** Two call sites were re-evaluated; both
  decisions **overturn** a default recorded in `design.md`.
  - **(1) Reopt cold-start (`ReOptimizePeriodic:575`,
    `InitializeTimeLimitsFromETConfig()`) — NOT a bug; KEEP as-is.**
    `design.md` §1 symptom 3 had claimed the reopt cold-start was "the same
    class of bug on the other path" as the incremental descent-start-TL bug.
    Re-derivation shows this is **wrong**: the reopt path uses
    `from_scratch=true` → `EvaluateTimeLimitConfig_ScratchOrIncre` builds a
    *fresh* `OptimizePA_Incre` and calls `OptimizeFromScratch` (NOT
    `OptimizeIncre`), so `FindTaskWithDifferentEt`'s diff **never runs** on the
    reopt path. The starting `time_limits` is just the from-scratch search's
    initial TL vector (the walk explores the full option set from there); there
    is no baseline/update diff invariant to preserve. So the reopt cold-start
    does not need to start at the carried adopted TL. Symptom 3 is
    **incremental-path only**. `design.md` §1 corrected in-place.
  - **(2) Incremental start (`OptimizeIncre_w_TL:398`,
    `ReconstructTimeLimitVecFromResOpt()` + stale-TL guard `:411-421`) — KEEP
    as-is, do NOT fold into `BuildChallengerFromIncumbent`.** `design.md` §6 Q3's
    default was "fold — the helper owns the reconstruction; the call site should
    not re-name the primitive." Re-derivation shows the call site's
    `time_limits` and the helper's internal `tl_prev` are **not the same vector**
    and serve different purposes:
    - The helper (`BuildChallengerFromIncumbent:510`) uses
      `tl_prev = ReconstructTimeLimitVecFromResOpt()` **raw** (no guard) to build
      the *baseline* DAG.
    - The call site (`OptimizeIncre_w_TL:398-421`) uses
      `ReconstructTimeLimitVecFromResOpt()` **then applies the stale-TL guard**
      (`:411-421`, forces −1 when the carried TL is no longer a member of the
      current option set) to produce the *update-side* descent start vector.
    Folding would either drop the guard from the update side (re-introducing the
    stale-TL-applied-as-point-dist hazard the guard exists to prevent) or push
    the guard into the helper (making the baseline diverge from the raw
    reconstruction). They are correctly separate. `design.md` §6 Q3 corrected
    in-place. (The guard is latent today — `CommitIncumbent` only ever writes a
    current-option TL — but the baseline-reconstruction vs. descent-start-vector
    separation is intentional, not a fold candidate.)

## 2026-07-08 (Phase 4 — verify + close)

- **Suite green (re-confirmed this session).** `cmake --build . --target
  check.SP_OPT -j5` in `build/` (DEBUG): 16/16 ctest pass;
  `testIncreOpt_w_TL` = **44 tests** green.
- **P1.1 runtime probe re-run — PASS, and stronger than predicted.** Rebuilt
  `release/tests/RunOrchestrator` (was stale Jul-7 vs the redesigned Jul-8
  source) and re-ran the INCR_P10 N=8 taskset_0 probe:
  `release/tests/RunOrchestrator <ts_dir> <out> INCR_P10 10000 1`
  (4th arg = per-interval horizon 10000 ms, per memory
  `runorchestrator-duration-arg-semantics`; NOT n_sec*1000). Trace:
  `simulation_experiments/optimizer_comparison/et_repro/p05_probe_ts0_P10/`.
  - **Pre-redesign** (`dbg_trace_ts0_new/P10`, `[INCR-ET-DBG]`):
    `call=0 ndiff=5` — tasks 0,1,2,6,7 flagged; 3 false positives (0,1,2:
    adopted TL ≠ Gaussian-mean TL).
  - **Post-redesign** (`p05_probe_ts0_P10`, `[INCR-NDIFF-PROBE]`):
    `call=0 ndiff=0` — **nothing flagged**. The 3 perf-pair false positives
    (0,1,2) vanished because both diff sides now carry the adopted TL by
    construction. Across all 302 incremental calls `ndiff` is only ever 0 or 1,
    never the spurious 5. No `CoutError`/contract-violation in the trace (the
    new hard-error `else` branch at `:176-191` is never hit — `SeedIncumbentBaseline`
    always establishes the incumbent first).
  - **Reconciliation with the corrected ground truth.** The P1.1 investigation
    summary (`investigation_summary.md` §4) predicted `ndiff` 5 → ~2 ("only
    gaussian-only tasks flag"). The actual result is `ndiff=0` at call=0 —
    **stronger** than predicted. The summary's "ground truth = 2 (tasks 6,7
    gaussian-only)" was itself based on diffing *raw Gaussians between
    intervals*. Under the redesign `BuildChallengerFromIncumbent` rebuilds the
    baseline from the **current** `dag_tasks_` each interval, so for a
    gaussian-only task (TL=−1, no point-dist applied) both diff sides are the
    *current* interval's raw Gaussian → identical → not flagged. The
    inter-interval Gaussian drift is no longer in the diff either. Only a task
    whose **adopted TL actually moved within the current descent** flags (task 5
    at call=1: 23.467→20.35). This is the more-correct semantic for the
    incremental diff: `FindTaskWithDifferentEt` flags a task iff its ET differs
    between the carried incumbent's applied DAG and the current descent's
    applied DAG — and since both are built from the current `dag_tasks_` + the
    respective TL vectors, only a real adopted-TL move flags.
  - **Conclusion:** P0.5 alone (no YAML persistence / P0.1) collapses the
    `ndiff` false-positive class. P0.1's remaining value is inspectability-only
    (the on-disk YAML still shows the Gaussian), as `p05-subsumes-tl-init-bug`
    predicted.
- Next: append the top-level milestone to `agents/dev_log.md`. (DONE — the
  milestone is in `agents/dev_log.md` under the "P0.5 incumbent-state redesign
  LANDED" heading.)

## 2026-07-08 (re-verification on resume)

- Resumed the task; re-verified the working-tree state end-to-end against the
  records (not just trusting the checkboxes):
  - `OptimizeSP_TL_Incre.h`: no `prev_optimizer_` member (only comment refs at
    `:179`/`:198`); `has_incumbent_` (`:204`) + `CommitIncumbent` (`:185`) +
    `BuildChallengerFromIncumbent` (`:187`) declarations present.
  - `OptimizeSP_TL_Incre.cpp`: single-writer routing confirmed —
    `CommitIncumbent(...)` at `:133` (UpdateRecords `should_update`) and `:473`
    (SeedStateFromIncumbent); challenger flip at `:172`
    (`BuildChallengerFromIncumbent()`); `has_incumbent_` gate at `:531`
    (SeedIncumbentBaseline). The `else` no-incumbent branch is the `CoutError`
    contract violation at `:188`.
  - Every `prev_optimizer_` mention in `tests/testIncreOpt_w_TL.cpp` and the two
    source files is **comment-only** (grep confirmed: 4 test refs + 2 source
    refs, all in prose).
  - **Build + tests green (ran this session):** `cmake --build . --target
    check.SP_OPT -j5` in `build/` (`CMAKE_BUILD_TYPE=DEBUG` per
    `sp-opt-test-build-debug-config`) → 16/16 ctest pass;
    `testIncreOpt_w_TL` = **44 tests** green.
  - **P1.1 probe trace verified on disk:**
    `simulation_experiments/optimizer_comparison/et_repro/p05_probe_ts0_P10/`
    contains `INCR_P10`/`stdout.txt`/`stderr.txt`; `ndiff=` distribution across
    the run is **27× `ndiff=0` + 275× `ndiff=1`** (302 calls total), never the
    pre-redesign `ndiff=5`. No `CoutError`/contract string in the trace.
- **Status correction:** `goal.md` line 4 said "implementation not started"
  (stale from the design phase) — fixed to "LANDED in working tree … Not yet
  committed." `tasks.md` checkboxes and `overall_tasks.md` were already correct.
- **Commit scope — OPEN ITEM (the only thing left).** The working tree is
  tangled: it bundles (a) P0.5 proper
  (`OptimizeSP_TL_Incre.{h,cpp}`, `OptimizeSP_Incre.cpp`, `testIncreOpt_w_TL.cpp`,
  the `agents/active_tasks/P0_5_*` dir, the `agents/dev_log.md` +
  `overall_tasks.md` milestone), (b) **leftover debug instrumentation removal**
  from P1.1 that overlaps P0.5's blast radius — the `[INCR-ET-DBG]` block was
  stripped from `SimulationOrchestrator.cpp:288-316` and `g_incr_et_debug_sp_dag_calls`
  was removed from `SP_Metric.{h,cpp}`, BUT the `[INCR-NDIFF-PROBE]` `cerr`
  block at `OptimizeSP_Incre.cpp:251-260` is **still in the tree** (inconsistent
  with the removal), and (c) **unrelated config/experiment churn** —
  `parameters.yaml` `debugMode: 0→1`, `p25_period_ab_config.json` test/prod mode
  edits (N lists, durations, worker counts), `.gitignore` `+build_test/`, plus
  the `P1_1_*`/`P2_3_*`/`P0_1_*` task dirs. Decision needed: commit P0.5 as its
  own focused commit (stage only the P0.5 + the matched debug-instrumentation
  removal) and leave the config/experiment churn + other task dirs for their own
  commits, OR commit the lot. Awaiting user guidance; nothing committed yet.

---

## 2026-07-09 — Phase 5 issue (5): unified `ResetIncumbentBaseline`

**User review fix (5) of 8.** The "reset `res_opt_` for each new interval"
invariant held, but via two **different implicit mechanisms** split across two
paths — the "code organization not good / repeated code" the user flagged:
- Incremental: `opt_sp_ = -1.0` sentinel at `OptimizeIncre_w_TL:375` → baseline
  eval's `UpdateRecords` force-commits → `res_opt_` overwritten.
- Reopt: `SeedIncumbentBaseline()` at `ReOptimizePeriodic:569` re-evals carried
  {pa, tl} under the new DAG + commits → `opt_sp_` = re-evaluated value.

**Decision (user, 2026-07-09):** keep both paths' behavior EXACTLY (the reopt
PA-search-at-default-TL that Issue A protected stays), but consolidate the reset
into **one explicit function** — `ResetIncumbentBaseline(bool from_scratch)` —
branched on `from_scratch`:
- `true`  → reopt reset = today's `SeedIncumbentBaseline` body (byte-identical).
- `false` → incremental reset = `opt_sp_ = -1.0` (moved from `OptimizeIncre_w_TL:375`).

**Placement:** called at the top of `PerformCoordinateDescentForTaskConfigOpt`
(before the baseline eval) AND at the top of `OptimizeWithTimeLimitOptDisabled`
(covers the disable-path bypass — that path skips the descent, so without its
own reset `res_opt_` would stay stale for the interval). `SeedIncumbentBaseline()`
call removed from `ReOptimizePeriodic:569`; `opt_sp_ = -1.0` removed from
`OptimizeIncre_w_TL:375`; `SeedIncumbentBaseline` declaration removed from the
header.

**Critical ordering invariant (preserved):** `BuildChallengerFromIncumbent`
(called inside the baseline eval) reads `res_opt_` to build the challenger. On
the incremental path `ResetIncumbentBaseline(false)` sets ONLY `opt_sp_ = -1.0`
(the gate); it must NOT touch `res_opt_`. Order: read prior from `res_opt_` →
re-eval under new DAG → `UpdateRecords` force-commits (overwrites `res_opt_`).
Matches today exactly.

**Comment trims (user directive: "if i see that kind of long code comments,
i'll just skip it"):** cut the patience / baseline / {-1}-skip / zero-work-
fallback comments in the descent, the carried-adopted-TL + edge-case-guard
comments in `OptimizeIncre_w_TL`, and the helper-block comments in
`EvaluateTimeLimitConfig_ScratchOrIncre` / `UpdateRecords` /
`SeedStateFromIncumbent` / `CommitIncumbent` / `BuildChallengerFromIncumbent` to
~1-3 lines each. Stale `SeedIncumbentBaseline` / `prev_optimizer_` comment
references in `OptimizeSP_TL_Incre.h` and `SimulationOrchestrator.cpp:300`
updated to `ResetIncumbentBaseline` / `has_incumbent_`.

**Tests:** the 2 `SeedIncumbentBaseline_*` tests rewritten to call
`ResetIncumbentBaseline(true)` (bodies/assertions unchanged — reopt branch is
byte-identical, so `ReOptimizePeriodic_AdoptsWhenDagMutationShiftsOptimum` /
`_KeepsIncumbentWhenDagUnchanged` compare-and-keep contracts are preserved).
The 2 `*_BaselineOverwritesResOptForNewInterval` test comments updated to
attribute the invariant to `ResetIncumbentBaseline` instead of the inline
sentinel.

**Out of scope (separate follow-ups, one-by-one per user):** (1) remove the
stale-TL guard `:411-421` — here only its comment was trimmed; (2) evaluate
removing `has_incumbent_`; (3) rename `time_limits` param; (4) remove dead
`any_eval_ran` / `:335-337` fallback (kept + comment trimmed here); (6) patience
consecutive-vs-total semantics; (8) persistent challenger.

**Verify:** DEBUG build (`cmake -DCMAKE_BUILD_TYPE=DEBUG ..` +
`cmake --build . --target check.SP_OPT -j5`) → **46 `testIncreOpt_w_TL` + 16/16
ctest green**. Staged with `git add` (4 files: `OptimizeSP_TL_Incre.{h,cpp}`,
`SimulationOrchestrator.cpp`, `testIncreOpt_w_TL.cpp`). **NOT committed** per
standing constraint.

---

## 2026-07-09 — Phase 5 issue (1): removed the stale-TL edge-case guard

**Issue.** `OptimizeIncre_w_TL` carried an edge-case guard (`:342-354`) that
intersected each carried TL from `ReconstructTimeLimitVecFromResOpt()` against
the current interval's `time_limit_option_for_each_task_`, forcing `-1` for any
task whose carried TL was no longer a valid option this interval (a perf pair
lost since N-1, or a cold `res_opt_`).

**User decision: remove for code simplicity.** The guard was NOT fully latent —
each interval loads a fresh DAG (`taskset_..._interval_N.yaml`), so a task CAN
lose its `timePerformancePairs` across intervals, and
`UpdateExtDistBasedOnTimeLimit` (which does NOT consult
`time_limit_option_for_each_task_`) would apply a stale carried TL as a point
dist via `GetUnitExecutionTimeDist`. The user judged the simplicity win worth
that edge-case exposure. The walk itself is still protected:
`OptimizeSingleTaskTimeLimit:208-210`'s `FindTimeLimitOptionIndex`-sentinel
skips any task whose baseline_val is not a member of the current option set —
so a stale carried TL only affects the BASELINE eval (one eval per interval),
not the search. No experiment in the current suite mutates a task's
`timePerformancePairs` across intervals, so the change is behavior-preserving
for every run/test in the repo.

**Change.** Deleted the 13-line `for` loop + `valid` scan at `:342-354` of
`OptimizeSP_TL_Incre.cpp`. Trimmed the carried-adopted-TL comment above it
(dropped the now-stale "opt_sp_=-1.0 reset ... lives in ResetIncumbentBaseline"
sentence — that reset detail belongs to issue (5), not this region).

**Verify:** DEBUG build → **46 `testIncreOpt_w_TL` + 16/16 ctest green**.
Staged with `git add` (`OptimizeSP_TL_Incre.cpp` + `tasks.md` + `dev_log.md`).
**NOT committed** per standing constraint.

## 2026-07-09 — Phase 5 issue (3): renamed `time_limits` → `starting_time_limits`

User: *"i want a more meaningful name, for example, is it the time_limits from
last interval optimization?"*

**Verified the origin (re-derived, not assumed) — it is path-dependent:**
- **Incremental** call site (`OptimizeIncre_w_TL:340`) passes
  `ReconstructTimeLimitVecFromResOpt()` = the **carried adopted TL from
  `res_opt_`** (last interval's result). User's "from last interval
  optimization" hypothesis is **correct on this path**.
- **Reopt** call site (`ReOptimizePeriodic:478`) passes
  `InitializeTimeLimitsFromETConfig()` = **Gaussian-mean-closest TL for the
  current DAG** (`Find_Close_ExecutionTime` against
  `execution_time_dist.GetAvgValue()`). A fresh start, NOT last interval.

So no name can honestly say "last interval's" — the origin differs by path.
Chose the origin-neutral **`starting_time_limits`** ("the TL vector the descent
walks from") and documented both provenances in a 4-line header comment.

**Scope decision:** renamed only THIS function's parameter. The callees it flows
into (`EvaluateTimeLimitConfig_ScratchOrIncre`, `OptimizeSingleTaskTimeLimit`,
`UpdateRecords`) receive a *candidate-being-evaluated/mutated* — a different
semantic role — so their `time_limits` params were intentionally left as-is.
Call sites unchanged (positional args; their local var name `time_limits` is
independent of the parameter name).

**Verify:** DEBUG build → **46 `testIncreOpt_w_TL` + 16/16 ctest green**.
Staged with `git add` (`OptimizeSP_TL_Incre.h` + `OptimizeSP_TL_Incre.cpp` +
`tasks.md` + `dev_log.md`). **NOT committed** per standing constraint.

---

## 2026-07-09 — Phase 5 issue (4): removed dead `any_eval_ran` + zero-work fallback

**User:** "(4) any_eval_ran is always true in
PerformCoordinateDescentForTaskConfigOpt(), so it's not useful. This zero-work
fallback is never triggered [...]"

**What:** the `if (!any_eval_ran && !dag_tasks_.tasks.empty())` block at the end
of `PerformCoordinateDescentForTaskConfigOpt` was unreachable. `any_eval_ran`
was a local declared `= true` (the baseline eval above always runs first) and
never set to false anywhere, so the guard's predicate was always false. Pure
dead code.

**Done:** deleted (1) the fallback `if`-block and (2) the `any_eval_ran` local
(both the declaration `:264` and the always-true assignment). Verified with a
grep across `sources/` + `tests/` + `*.yaml` that `any_eval_ran` had no other
references — only the 2 lines inside this function.

**Behavior:** no change — the deleted branch never executed. The all-`{-1}` case
(every task lacks timePerformancePairs) is already covered by the baseline eval
at the top of the function, which runs `OptimizeIncre`/`OptimizeFromScratch` and
commits via `UpdateRecords` → `CommitIncumbent` exactly once.

**Verify:** DEBUG build → **46 `testIncreOpt_w_TL` + 16/16 ctest green**.
Staged with `git add` (`OptimizeSP_TL_Incre.cpp` + `tasks.md` + `dev_log.md`).
**NOT committed** per standing constraint.

## 2026-07-09 — Phase 5 issue (6): simplify OptimizeSingleTaskTimeLimit patience

**What:** `OptimizeSingleTaskTimeLimit` carried a separate
`consecutive_non_improving` counter that reset on each improvement — a
CONSECUTIVE non-improvement budget, breaking when `counter > patience`.

**User's design:** drop the counter; decrement `patience` directly on failure;
no reset on improvement (a TOTAL non-improvement budget). User's literal stop
formula: "patience-- if failed, stop if patience<0 or patience==0".

**Form chosen = check-then-decrement**, not the literal formula. The literal
"decrement then stop if `<=0`" collapses patience=1 → 0 → stop on the first
non-improvement, i.e. patience=1 behaves like patience=0 and the reopt/
incremental distinction (two YAML values) is erased. Check-then-decrement
(`else if (patience == 0) break; else --patience;`) preserves `patience=N`
meaning "tolerate N non-improving steps": patience=0 breaks on first
non-improvement (identical to before), patience=1 tolerates one.

**Semantics shift — reopt path only.** For incremental (patience=0) every
formulation agrees (break on first non-improvement) → byte-identical. For reopt
(patience=1) the budget changed from CONSECUTIVE to TOTAL: a noisy-but-trending-
up region (IMP,NIP,IMP,NIP,…) that previously reset on each IMP and walked far
now stops at the 2nd NIP. Effect: shorter reopt walks (efficiency win on the
expensive `OptimizeFromScratch` evals) at bounded SP risk (each task still gets
backward+forward passes from baseline). The user accepted this tradeoff
("efficiency with potential performance loss on SP").

**Done:** rewrote the walk body (`OptimizeSP_TL_Incre.cpp:212-237`) — dropped
the counter, the reset, and the `> patience` check; replaced with
check-then-decrement. Header doc (`OptimizeSP_TL_Incre.h:140-153`) rewritten:
"total non-improvement budget, NOT reset on improvement", patience values
described as N tolerates N non-improving steps.

**Verify:** DEBUG build → **46 `testIncreOpt_w_TL` + 16/16 ctest green**.
Staged with `git add` (`OptimizeSP_TL_Incre.cpp` + `.h` + `tasks.md` +
`dev_log.md`). **NOT committed** per standing constraint.

**Open (not blocking):** the unit suite cannot surface the reopt SP delta
(positive or negative) from the consecutive→total shift — that needs an A/B
experiment on the P1.1 taskset if quantifying it matters. Deferred unless the
user wants it.

## 2026-07-10 — Phase 5 issue (8): persistent challenger EVALUATED → REJECTED

**The user's original (8):** "implementation of `BuildChallengerFromIncumbent`
is wrong — you create a new optimizer from the champion each time; what I asked
is to modify from the challenger each time, to fully utilize incremental
optimization... compare performance/efficiency in experiments and decide which
to keep. I think my design is better in efficiency, with potential SP loss."

**Discussion-first (no edits for several rounds).** The user asked for a fuller
trade-off enumeration before deciding. Mid-discussion the user independently
reconsidered their own efficiency premise and arrived at the opposite
conclusion: rebuilding from the champion gives the MINIMUM tasks-with-different-
TL when moving between tasks, whereas a persistent challenger could flag MORE.

**Code re-derivation confirms the user's revised reasoning.** Read
`UpdateRecords` (`OptimizeSP_TL_Incre.cpp:105-140`) + `OptimizeSingleTaskTimeLimit`
(`:194-239`) + `EvaluateTimeLimitConfig_ScratchOrIncre` (`:142-175`):
- `UpdateRecords` commits `res_opt_` (the champion) on EVERY adoption — strict
  SP gain OR tie-with-smaller-TL-sum. So the champion TL tracks the adopted
  working TL, not some stale earlier value.
- `OptimizeSingleTaskTimeLimit` resets `time_limits[task_idx] = best_option_val`
  (`:237`) after each pass — `best_option_val` starts as the current value and
  only updates on `IsBetterTimeLimitOption`. So on no-improvement the working TL
  for that task is unchanged → stays in sync with the champion.
- Invariant: `res_opt_` TL == working `starting_time_limits` EXCEPT for the one
  task currently being walked. Therefore `BuildChallengerFromIncumbent`'s
  baseline DAG (built from `res_opt_`) vs the candidate DAG (built from the
  working TL) differ in exactly one task → `FindTaskWithDifferentEt` flags ≤ 1
  task → `OptimizeIncre` re-searches just that task's 1D priority variations.
  This is the ideal input for incremental optimization.

**Why the persistent challenger (P) is weakly dominated within-interval.** P
keeps one `OptimizePA_Incre` member; `OptimizeIncre` advances
`challenger.dag_tasks_ = dag_tasks_update` at the end of EVERY call
(`OptimizeSP_Incre.cpp:301`) — unconditionally, adopted or not. So
`challenger.dag_tasks_` = the last-EVALUATED candidate's DAG, while the working
TL = the ADOPTED best. They diverge whenever the last exploration wasn't
adopted. Moving from task A to task B: if A's last explored option wasn't
adopted, the challenger still has A at the unadopted value, the working TL has
A at the adopted value → B's first eval flags BOTH A and B. So within-interval
`ndiff_P ≥ ndiff_U` always. P can be MORE expensive (more RTA evals), not less.

**The two SP mechanisms in P cut opposite ways (ambiguous sign):** (a) P
re-searches DAG-mutated tasks' PA cross-interval (U skips this until reopt) →
potential SP GAIN if mutations shift the optimal PA; (b) P's PA warm-start
drifts off the champion → 1D variations from a worse starting PA → potential SP
LOSS. Net SP delta is empirical, not signed.

**Decision (user, 2026-07-10): option (1) — keep the current rebuild-from-
champion design; do NOT adopt P.** Rationale: U weakly dominates within-interval
(a clean, provable invariant — minimal diff, clean PA warm-start); P's only
potential edge (cross-interval PA re-search of mutated tasks) is a separable
mechanism that can be added to U directly (e.g. flag mutated tasks at the
baseline eval) if measurement ever shows it helps — without taking on P's drift
and extra within-interval evals. The "fully utilize incremental optimization"
intent is real but cuts two ways; U is incremental along the within-interval
axis (the one that matters for the per-candidate diff), P along the cross-
interval axis. No A/B experiment run — the analysis is conclusive enough that
the user chose to skip it.

**Code change: comments only (no behavior change).** Added/strengthened the
"only one task's ET changes" guarantee at three sites:
- `BuildChallengerFromIncumbent` definition (`OptimizeSP_TL_Incre.cpp:397-402`):
  states the champion-tracks-working-TL invariant and that the diff flags only
  the walked task → perfect for `OptimizeIncre`'s diff-driven 1D re-search; a
  persistent challenger would drift to non-adopted candidates and flag extras.
- `EvaluateTimeLimitConfig_ScratchOrIncre` incremental branch call site
  (`:156-159`): tight pointer to the helper + the guarantee.
- `OptimizeSP_TL_Incre.h` incumbent-state-helpers block (`:180-187`): the
  rebuild-vs-persistent rationale at the declaration.

**Verify:** DEBUG build (`cmake --build . --target check.SP_OPT -j5` in
`build/`) → **46 `testIncreOpt_w_TL` + 16/16 ctest green** (no behavior change;
comment-only). Staged with `git add` (`OptimizeSP_TL_Incre.cpp` + `.h` +
`tasks.md` + `dev_log.md`). **NOT committed** per standing constraint.

## 2026-07-10 — Phase 5 issue (2): `has_incumbent_` EVALUATED → REMOVED

User: "i want to remove that task, possibly removing has_incumbent_, as it
doesn't really [hurt] code readability." Confirmed via structured Q&A: drop the
5d bullet AND remove the field (gate on `IfInitialized()` instead).

**Re-derivation (not assumed).** After P0.5, who writes `this->opt_pa_` on the
persistent optimizer? Traced every call site:
- `CommitIncumbent` (`OptimizeSP_TL_Incre.cpp:390`) — `opt_pa_ = pa`. ✓ single writer.
- `OptimizeFromScratch` (`OptimizeSP_Incre.cpp:127`) — sets `opt_pa_`, BUT only
  called on throwaway local challengers (`EvaluateTimeLimitConfig_ScratchOrIncre:152`),
  never on `this`.
- `OptimizeIncre` (`OptimizeSP_Incre.cpp:284`) — sets `opt_pa_`, BUT only on
  throwaway local challengers (`:161`), never on `this`.
- `BuildChallengerFromIncumbent` (`:408`) — writes `challenger.opt_pa_` (a
  different object), not `this->opt_pa_`.

So `has_incumbent_` (set only by `CommitIncumbent`) and `!opt_pa_.empty()` (set
only by `CommitIncumbent` on `this`) flip together, always. The bool carried zero
information beyond what `IfInitialized()` already reports. Its original reason —
the `prev_optimizer_.IfInitialized()` desync where `opt_pa_` could be non-empty
while `sp_parameters_` was still empty — is structurally impossible now that
`prev_optimizer_` is gone and `CommitIncumbent` (which writes `opt_pa_` from a
fully-constructed challenger's `opt_pa_`) is the single writer.

**Changes:**
- `EvaluateTimeLimitConfig_ScratchOrIncre:155` — `else if (has_incumbent_)` →
  `else if (IfInitialized())`.
- `ResetIncumbentBaseline:428` — `if (has_incumbent_)` → `if (IfInitialized())`.
- `CommitIncumbent:394` — dropped `has_incumbent_ = true;`.
- `OptimizeSP_TL_Incre.h:206` — dropped the `bool has_incumbent_ = false;` member;
  replaced its comment with a 2-liner noting the gate is `IfInitialized()` and
  why no separate bool is needed.
- `:170` CoutError message reworded ("has_incumbent_ is false" → "no incumbent
  is initialized").
- `testIncreOpt_w_TL.cpp` — 16 `opt.has_incumbent_` → `opt.IfInitialized()`;
  the stale "flips has_incumbent_" / "has_incumbent_ is true" comment phrasings
  rewritten to "establishes an incumbent" / "the incumbent is initialized".
- Comment-only refresh in 3 spots still referencing the bool by name
  (`OptimizeSP_TL_Incre.h:73`, `OptimizeSP_TL_Incre.cpp:284` and `:128`,
  `SimulationOrchestrator.cpp:299`) — "has_incumbent_ false" → "no incumbent
  carried".

**Trade-off accepted.** The bool gave mild defense-in-depth: if a future edit
set `opt_pa_` outside `CommitIncumbent`, the `CoutError` at `:168` would fire
under the bool gate but silently proceed under `!opt_pa_.empty()`. Hypothetical,
not a current bug; user judged readability a wash → simpler state wins.

**Verify:** DEBUG build (`cmake --build build --target check.SP_OPT -j5`) →
**46 `testIncreOpt_w_TL` + 16/16 ctest green**. Staged with `git add`
(`OptimizeSP_TL_Incre.{h,cpp}`, `SimulationOrchestrator.cpp`,
`testIncreOpt_w_TL.cpp`, `tasks.md`, `dev_log.md`). **NOT committed** per
standing constraint.

**5h (issue 7, efficiency) NOT touched here** — user's earlier "move efficiency
optimization into a different task" directive is handled separately (5h stays
deferred in P0.5 pending its own task move).

## 2026-07-10 — Phase 5 issue (7): 5h MOVED to P3.1 (efficiency bucket)

Per user directive ("next move the last task0.5 into efficiency optimization,
as it seems to be an efficiency optimization task"), moved 5h (issue 7, "Reuse
a single optimizer instance across `EvaluateTimeLimitConfig_ScratchOrIncre`
calls instead of rebuilding per candidate") out of P0.5 and into the deferred
efficiency bucket `active_tasks/P3_1_efficiency_optimizations/`.

It's a pure perf, not correctness, item, so it belongs in P3.1, not the P0.5
redesign. Added as a third deferred item in P3.1's `goal.md` + `tasks.md`,
with the trade-off recorded: the current rebuild-from-champion design (decided
in 5b on 2026-07-10) was chosen OVER the persistent challenger because the
champion tracks the working TL so the diff flags only the one task being
walked; a persistent challenger would drift the diff baseline to non-adopted
candidates and flag extras → potentially more RTA evals, not fewer. The "5h
entangled with (8)" note is therefore stale — (8)/5b's resolution IS the
trade-off for 5h. 5h marked `[~]` MOVED in P0.5's `tasks.md`.

**P0.5 Phase 5 is now complete:** 5a–5g done, 5h moved. All Phase-5 items
resolved. No code changed in this move (docs only).
