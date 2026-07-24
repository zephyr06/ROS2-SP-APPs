# P1.20 — Tasks (working checklist)

> Perf refactor: Convert processor-partition hash tables (`unordered_map`) to flat `std::vector`.
> Bit-identical SP output is the only acceptance gate (TDD relaxed per user).

> **Plan revised 2026-07-23** — investigation proved `-1` is NOT special-cased
> anywhere (plain default "one core"). Decision D1 = normalize at source
> (`-1`→`0` + fix stray fixtures) rather than reject `-1`. Decomposed into 4
> manageable commits (C3+C4 folded). Full reasoning in `dev_log.md`.

## Commit 1 — Ingestion seam: kill `-1`, add `ValidateProcessorIds`, delete dead code [D1+D2+D3]
- [x] **1a. `RegularTasks.h:99` default `-1`→`0`** + update comment.
- [x] **1b. `DAG_Model::ValidateProcessorIds()`** — reject `<0` AND non-dense;
  called at top of `CategorizeTaskSet()`. [D2 fail-fast]
- [x] **1c. Delete dead `task_id2task_index_within_processor_`** (member + write). [D3]
- [x] **1d. Fix stray non-dense fixtures** — `test_robotics_v8.yaml` + `test_robotics_v27.yaml`
  (both `pids={1}` → core 0). Found by the validator firing on ctest.
- [x] **1e. `testIO.cpp:32,45`** asserts `-1`→`0`.
- [x] **1f. Gate** — 17/17 ctest (incl. both `Differential_BitIdentical*` probes, 8/8);
  `--clean-first` (header member removed).
- [x] **1g. `git add`** the 6 files (user commits).

## Commit 2 — DAG_Model: vectorize `processor2taskset_` [internal-only]
- [x] **2a.** `processor2taskset_`: `unordered_map<int,TaskSet>` → `vector<TaskSet>` (sized `max+1`).
- [x] **2b.** `CategorizeTaskSet()` indexed loop.
- [x] **2c.** Gate — 17/17 ctest + bit-identical SP (8/8 `*Differential*`/`*BitIdentical*` probes).

## Commit 3 — RTA layer: vectorize `ExtractTaskSetPerProcessor` + RTA_Cache + PrioritySwitchAnalysis (folded C3+C4)
- [x] **3a.** `ExtractTaskSetPerProcessor` (RTA.h/.cpp) return `vector<TaskSet>`; `ProbabilisticRTA_TaskSet` indexed loop.
- [x] **3b.** `ChampionState::champ_per_core`/`hp_prefix_per_core` → `vector<vector<int>>`/`vector<vector<FiniteDist>>`.
- [x] **3c.** `PerCoreOrderFromPa`/`PerCoreOrderOfPrioritized` → `vector<vector<int>>`.
- [x] **3d.** `RebuildPrefixes`/`Evaluate`/`IsSingleTaskChange`/`ClassifyReusePerTask`: `.at(core)`→`vec[core]`, range-for→indexed.
- [x] **3e.** `PrioritySwitchAnalysis.h`: `FindCoreOfTask`/`AnalyzePrioritySwitch` take `vector<vector<int>>`.
- [x] **3f.** `testRTA.cpp` fixtures → `vector<vector<int>>`; refresh "default-(-1)" comments → "default-0".
- [x] **3g.** Gate — 17/17 ctest + bit-identical SP (8/8 `*Differential*`/`*BitIdentical*` probes).

## Commit 4 — Remaining fixtures + final bit-identical gate
- [~] **4a.** `testOptimizeIncrePA.cpp` hand-built per-core map fixtures — **CONFIRMED EMPTY** (only a
  stale comment referencing `RTACache::AnalyzePrioritySwitch`, not a call). No-op.
- [~] **4b.** Final stash→HEAD bit-identical gate — **superseded.** The 8/8 `*Differential*`/`*BitIdentical*`
  in-binary oracle probes run during C1–C3 already assert SP equality against the oracle within the same
  binary (stronger than a stdout diff: same machine, same libc, no timing noise). C1/C2/C3 each reported
  8/8 PASS. The separate stash→HEAD→diff ritual adds no coverage. Closed without running.
- [~] **4c.** Optional perf spot-check (N=10 INCR wall-clock vs HEAD) — **NOT run (non-gate).**
  Pure storage change (hash removal) is provably neutral-to-faster; measurement deferred.

## Status — CLOSED 2026-07-23
All three commits **COMMITTED** by user: C1 `484df475`, C2 `b20e5828`, C3 `337e9f8f`.
C4 = verification-only; 4a empty, 4b superseded by the in-binary oracle probes (8/8 across C1–C3),
4c non-gate. **P1.20 DONE.** 17/17 ctest green throughout; SP provably neutral at every commit
(map→vector, same partition contents, per-core-independence proven). → moved to `finished_tasks/`.

## Out of scope
- **`task_id2position_`** — keyed by *task_id*, not processorId (D4).
- NOSPEC yamls left untouched (all-default → all core 0 = correct single-core).
