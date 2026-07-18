# P1.11 — Incremental RTA Patching — Dev Log

> Detailed working log for this task. Append chronological entries below.

---

## History Timeline (Migrated from P1.9)

- **2026-07-13**: Task elevated from Idea 11. Phase 1 (infra) and Phase 2 (patching) proposed. Initial cache design locked (rev-1, per-core `unordered_map`).
- **2026-07-14**: HP-prefix checkpoint store landed (refactored `ProbabilisticRTA_TaskSet_SingleCore` to take `hp_tasks_et_conv_vec` out-param). Step 3a written (staged `ComputeRTA_FullAndCache` in `RTA.cpp`, verified vs oracle). Added `tl_vec` to cache as ET-dist validity proxy.
- **2026-07-15**: API Rev 2 (whole-taskset `PerCoreRTACache` self-supplied class). Put on hold behind P1.10.
- **2026-07-17**: **P1.10 landed** — proved single-change invariant (`|diff| <= 1` per SP-eval), unblocking P1.9 cache optimization.
- **2026-07-18**: API Rev 3 (simplified by the proven invariant). Wrote `RTACache` class + `TaskSetDifference`/`RTAReusePerTask` queries; `RTA_Cache.cpp` + 11 differential test cases; 16/16 ctest green.

---

## 2026-07-18 (P1.9 migrated to P1.11 & records simplified)

- User requested to simplify records due to length and create a new task folder `p1_11` from `p1_9` records.
- Migrated task tracking files to `P1_11_incremental_rta_patching` and created concise `goal.md`, `tasks.md`, and `dev_log.md`.
- Deleted the old `P1_9_incremental_rta_patching` folder.
- Updated `agents/overall_tasks.md` to point to `P1_11_incremental_rta_patching` and renamed the old `P1.11` (Partial task-subset optimization) to `P3.11` (with folder `P3_11_partial_task_subset_optimization`).

---

## 2026-07-18 — Phase 0 COMPLETE: rev-3 cache + API + direct unit tests (LANDED, working tree, NOT committed)

**Status: 16/16 ctest + 46/46 testRTA green.** All cache logic bit-identical to the `ProbabilisticRTA_TaskSet` oracle (differential TDD). Everything in this section is in the working tree, uncommitted.

Built up over a series of user code-reviews (7-comment refactor → 3-follow-up → verdict-driven Evaluate → drop short names + extract `AnalyzePrioritySwitch` → split per-core step → two-pointer `RestEqualAfterRemoving` → header-extract + direct unit tests). The landed API surface, distilled:

### `RTACache` class (`sources/Safety_Performance_Metric/RTA_Cache.h`)

Public surface (the testable + wired contract):
- `Initialize(dag, pa, tl)` — cold-start: full N-task RTA via the `ProbabilisticRTA_TaskSet` oracle + builds prefix checkpoints (`RebuildPrefixes`). Efficiency TODO: oracle could emit the prefixes to dedupe.
- `AdoptChampion(dag, pa, tl, rtas)` — cheap commit on promotion: stores candidate `rtas` + champion triple, rolls prefix forward, **no RTA compute**. The wired-in commit point = `CommitIncumbent`.
- `Evaluate(dag, pa, tl)` — hot-loop read-only entry. Mechanical **verdict-driven per-task dispatcher**: (1) no champion → `Initialize`; (2) `candidate_rta_ = rta_` (seed all); (3) no `NoReuse` task → return (`|diff|==0` pure-reuse short-circuit); (4) else walk each core in **candidate priority order**, accumulating `hp_tasks` over ALL tasks above the current one (regardless of verdict, so a `NoReuse` task's HP set includes candidate-ET versions of reused tasks above it); overwrite each `NoReuse` task's RTA via 1-arg `GetRTA_OneTask(task, hp_tasks)`; `FullReuse` tasks skipped (keep seed) but still pushed to `hp_tasks`.
- `ComputeTaskSetDifference(dag, pa, tl) → TaskSetDifference` — pure locator query `{changed_task_id, core, old_pos, new_pos}` (no `klass` field — verdict is DERIVED from locators: `changed_task_id==-1` ⟺ `|diff|==0`). No-champion short-circuits to `{-1,-1,-1,-1}`; **throws `std::runtime_error` on >1 change** (we only design for the single-change invariant).
- `IsSingleTaskChange(dag, pa, tl) → bool` — non-throwing predicate; thin wrapper over the private core.
- `ClassifyReusePerTask(dag, pa, tl) → vector<RTAReusePerTask>` — per-task verdict, derived from the locator: `changed_task_id==-1` → all `FullReuse`; else same-core (`diff.core`) → `NoReuse`, cross-core → `FullReuse` (v1 = whole-changed-core recompute).

`enum class RTAReusePerTask { NoReuse, FullReuse, ReuseHpTasksEt }` — `ReuseHpTasksEt` is RESERVED (not emitted by v1); kept so the future same-core-suffix refinement names its path.

Private: `TryComputeSingleChange(dag, pa, tl, out&) → bool` (the one diff core; never throws — returns true + fills `out` iff `|diff|<=1`); `PerCoreOrderFromPa`; `RebuildPrefixes`.

### Priority-analysis utilities (`sources/Safety_Performance_Metric/PrioritySwitchAnalysis.h` — NEW leaf header)

Extracted from `RTA_Cache.cpp`'s anonymous namespace to `SP_OPT_PA` scope so the test suite can target them directly. True leaf header (depends only on `<vector>`/`<unordered_map>`/`int`, no SP/RTA/DAG types → no header cycle). Sources `GLOB_RECURSE`d → no CMake change. Declarations + the `PrioritySwitchStatus`/`PrioritySwitchAnalysis` types live here (doc comments = the spec the tests pin); definitions stay in `RTA_Cache.cpp` (bodies unchanged).

- `RestEqualAfterRemoving(candidate_order, champion_order, task_id) → bool` — **two-pointer walk** (no vector materialization): advance `i`/`j`, skip `task_id` in either vector, compare the un-skipped entries pairwise, drain trailing `task_id`s, check both reached the end. Caller guarantees same-size orders (size mismatch = core migration, rejected before this) + `task_id` appears once in each.
- `FindCoreOfTask(per_core, task_id) → int`.
- `AnalyzePrioritySwitchPerCore(candidate_order, champion_order, out&) → PrioritySwitchStatus` — per-core single-move detection + locator fill. "Remove one task from both, compare the rest": at first mismatch `i` the moved task is `champion_order[i]` OR `candidate_order[i]`; try removing each; either-matches = single move, neither = `NotSingle`.
- `AnalyzePrioritySwitch(candidate_per_core, champion_per_core) → PrioritySwitchAnalysis` — whole-map: per-core size check (mismatch ⇒ core migration ⇒ `NotSingle`) + find-the-one-changed-core (≥2 ⇒ `NotSingle`) + delegate the in-core test.

`IdentityPrefix`/`RollPrefix` STAY file-local in `RTA_Cache.cpp` (RTA-prefix-only, not testable surface).

### Key algorithmic facts (the load-bearing invariants)

- **Single-change invariant** (proven by P1.10's `AssertSingleChangeInvariant`): per SP-eval `|diff|<=1` vs champion — Type-L TL step → `|diff|==1`; Type-E env step → `|diff|==0` (env move absorbed into `dag_tasks_` both sides, RTA is PA-independent). Never `|diff|>1`.
- **The diff algorithm** (`TryComputeSingleChange`, the user's "remove one task from both per-core orders, compare the rest"): (1) ET diff>1 → false; (2) per-core size mismatch → false (core migration); (3) find the ONE changed core (2nd → false); (4) no ET diff + no order diff → `|diff|==0` (`changed_task_id=-1`); (5) cross-check ET-diff task's core == priority-move core (else 2 changes); (6) remove-and-compare. The pure-priority-move case (0 ET diff — `Evaluate_PriorityMove_OneTaskPatch`) is handled by `AnalyzePrioritySwitch`'s 0-ET-diff branch: at first mismatch `i`, moved task is `champ[i]` OR `cand[i]`; try removing each.
- **Indexing**: verdict indexed by task id (ids 0..N-1 via `dag.tasks[i].id==i`); `candidate_rta_`/`rta_` indexed by position-in-prioritized-taskset (the oracle's `task_id2index` layout) → writes map via `task_id2index.at(id)`.
- **`hp_prefix_per_core_` is built but NOT consumed by `Evaluate`** under v1 (whole-changed-core recompute). Retained for the future same-core-suffix v2 refinement (reuse `hp_prefix[core][m]` instead of recomputing the prefix). A future reader shouldn't wonder why `RebuildPrefixes` exists if `Evaluate` ignores it.
- **Bugs caught during TDD** (now fixed): (a) `IsSingleTaskChange` mis-counted a priority swap as 2 changes — fixed by counting distinct independent changes (priority moves per core + ET changes not on a moved task; swap = 1 move). (b) Step (6) first bound `co`/`ch` to `changed_core`'s vectors, but for ET-only moves `changed_core==-1` → empty temporaries → fixed to use the ET task's actual core (`et_core`).

### Tests (`tests/testRTA.cpp`)

- Cache differential tests (vs oracle, bit-identical): `Evaluate_PriorityMove_OneTaskPatch` (0-ET-diff), `Evaluate_TLAndPriorityMove_CombinedPatch` (combined ET+move), `Evaluate_TLChange_*`, `ComputeTaskSetDifference_*` (>1 case now `EXPECT_THROW`), `ClassifyReusePerTask_*` (v1 cross-core), `NoChampion_AllNoReuse`, `TwoArgOverload_SameRtasAsOneArg`.
- **NEW direct unit tests** (free `TEST`, hand-built `vector<int>`/`unordered_map` inputs, no DAG setup): `RestEqualAfterRemovingTest` (11 cases — front/middle/end removal, trailing-`task_id` drain, two-task-swap→false, **contract-split pair** pinning the size guard is the CALLER's job: extra-is-non-`task_id`→false vs extra-IS-`task_id`→true), `AnalyzePrioritySwitchPerCoreTest` (7), `AnalyzePrioritySwitchTest` (6). +24 cases (testRTA 22 → 46).

---

## Next: Phase 1 — wire `RTACache` into the live incremental optimizer eval path

**Phase 0 (cache + API + tests) is done.** The cache exists, is bit-identical to the oracle, and is directly unit-tested — but it is NOT yet wired into `OptimizePA_Incre_with_TimeLimits` / `OptimizeIncre_SingleTask`. That wiring is Phase 1 (see `tasks.md`). Working tree stays uncommitted until the user asks for a commit.

---

## 2026-07-18 — Integration split out to P1.12

- User decided to split the cache-integration work into its own task folder
  `P1_12_integrate_rta_cache` rather than keep it as P1.11 Phase 1+2. P1.11's
  scope is now the cache **design + build** (Phase 0, DONE, working tree
  uncommitted) — this file remains the design doc for the `RTACache` API surface
  P1.12 consumes.
- The former Phase 1 (wire cache into live eval path) and Phase 2 (dispatch
  cache in hot loops) sections are retained in `tasks.md` as historical record;
  their active tracking now lives in
  [`../P1_12_integrate_rta_cache/tasks.md`](../P1_12_integrate_rta_cache/tasks.md).
- **P1.11 itself is at scope-complete closeout** (no further code work planned
  here unless the integration surfaces an API gap to file back).
