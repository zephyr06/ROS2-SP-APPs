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
- [ ] **2a.** `processor2taskset_`: `unordered_map<int,TaskSet>` → `vector<TaskSet>` (sized `max+1`).
- [ ] **2b.** `CategorizeTaskSet()` indexed loop.
- [ ] **2c.** Gate — 17/17 ctest + bit-identical SP.

## Commit 3 — RTA layer: vectorize `ExtractTaskSetPerProcessor` + RTA_Cache + PrioritySwitchAnalysis (folded C3+C4)
- [ ] **3a.** `ExtractTaskSetPerProcessor` (RTA.h/.cpp) return `vector<TaskSet>`; `ProbabilisticRTA_TaskSet` indexed loop.
- [ ] **3b.** `ChampionState::champ_per_core`/`hp_prefix_per_core` → `vector<vector<int>>`/`vector<vector<FiniteDist>>`.
- [ ] **3c.** `PerCoreOrderFromPa`/`PerCoreOrderOfPrioritized` → `vector<vector<int>>`.
- [ ] **3d.** `RebuildPrefixes`/`Evaluate`/`IsSingleTaskChange`/`ClassifyReusePerTask`: `.at(core)`→`vec[core]`, range-for→indexed.
- [ ] **3e.** `PrioritySwitchAnalysis.h`: `FindCoreOfTask`/`AnalyzePrioritySwitch` take `vector<vector<int>>`.
- [ ] **3f.** `testRTA.cpp` fixtures → `vector<vector<int>>`; refresh "default-(-1)" comments → "default-0".
- [ ] **3g.** Gate — 17/17 ctest + bit-identical SP.

## Commit 4 — Remaining fixtures + final bit-identical gate
- [ ] **4a.** `testOptimizeIncrePA.cpp` hand-built per-core map fixtures (if any).
- [ ] **4b.** Final gate: stash→HEAD→capture→restore→rebuild→diff (timing-normalized) = 0 lines.
- [ ] **4c.** Optional perf spot-check (N=10 INCR): wall-clock vs HEAD; NOT a gate.

## Out of scope
- **`task_id2position_`** — keyed by *task_id*, not processorId (D4).
- NOSPEC yamls left untouched (all-default → all core 0 = correct single-core).
