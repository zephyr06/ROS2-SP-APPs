# P1.20 — Dev Log

## 2026-07-23 — Investigation findings + Modification Plan

### The `-1` sentinel — resolved (NOT special-cased anywhere)

`RegularTasks.h:99` defaults `int processorId = -1;`. The goal.md feared this
would block a "reject negatives" validation seam. Investigation proves `-1` is
**purely a default** ("one single processor"), with NO value-based branching:

- `SimulationOrchestrator.cpp:465,668` — `processor_id >= 0` is a **caller-supplied
  filter arg** (always `>=0`, from `GetProcessorIds`), NOT a branch on the task field.
- `ExtractTaskSetPerProcessor` / `ProbabilisticRTA_TaskSet` (RTA.cpp:117-149) — `-1`
  is a plain hash key; all `-1` tasks land in one core group.
- `OptimizeSP_Base.cpp:95,99`, `OptimizeSP_Incre.cpp:76` — compare two tasks'
  `processorId` for *equality* only; `-1 == -1` is fine.

**Yaml partition is clean:** 21 SPEC files (every task specifies, already dense
0-based) vs 10 NOSPEC files (NO task specifies → all `-1`; never mixed). Production
generator `taskset_generator.py:553` assigns `t.processorId = np.argmin(core_utilizations)`
→ greedy least-loaded fill → used-core set is always `{0,…,k-1}`, NEVER gapped.
So a density check will never reject valid generator output.

### Decisions (USER, 2026-07-23)

- **D1 = normalize at source.** `RegularTasks.h` default `-1`→`0`. Update `testIO.cpp`
  asserts (`-1`→`0`). NOSPEC yamls left untouched (all-default → all core 0 = correct
  single-core). NO backward-compat shim.
- **D2 = fail-fast `ValidateProcessorIds()` in `DAG_Model` ctor.** Reject `<0` AND
  non-dense (a core id in `{0..max}` with zero tasks). Catches stray yaml at load.
- **D3 = delete dead code** `task_id2task_index_within_processor_` (DAG_Model.h:119,
  written DAG_Model.cpp:171, NEVER read). No backward-compat.
- **D4 = `task_id2position_` is OUT OF SCOPE.** It is keyed by **task_id**, not
  processorId — not a processor-partition structure.
- **D5 = gate = bit-identical SP.** No TDD red-pin required (user: "no need to follow
  TDD very strictly"); ctest 17/17 + bit-identical stdout vs HEAD is the gate.

### Blast radius (exact)

- `processor2taskset_` (DAG_Model.h:118) — written+read ONLY inside `CategorizeTaskSet()`
  (DAG_Model.cpp:161-174). NO external readers → trivial to vectorize, no API change.
- `ExtractTaskSetPerProcessor` (RTA.h:27 / RTA.cpp:117-128) — 3 call sites:
  RTA.cpp:135 (`ProbabilisticRTA_TaskSet`), RTA_Cache.cpp:222 (`RebuildPrefixes`),
  RTA_Cache.cpp:409 (`Evaluate`). Return type changes `unordered_map<int,TaskSet>`
  → `vector<TaskSet>`.
- RTA_Cache `ChampionState` (RTA_Cache.h:45-51): `champ_per_core`,
  `hp_prefix_per_core` → `vector<vector<int>>` / `vector<vector<FiniteDist>>`.
  Stays copyable (D1=(b) P1.25 reject-path backup still deep-copies correctly).
- `PerCoreOrderFromPa` / `PerCoreOrderOfPrioritized` (RTA_Cache.h:150-156) — return
  type `unordered_map<int,vector<int>>` → `vector<vector<int>>`.
- `PrioritySwitchAnalysis.h`: `FindCoreOfTask`, `AnalyzePrioritySwitch` take
  `unordered_map<int,vector<int>>` → `vector<vector<int>>`. Leaf header, unit-tested
  directly with hand-built vectors.
- `Evaluate`/`IsSingleTaskChange`/`ClassifyReusePerTask` (RTA_Cache.cpp) — replace
  `.at(core)`/range-for over map with `vec[core]`/indexed loop.

---

## Modification Plan (decomposed into manageable commits)

Each commit is independently buildable + testable. Agent `git add`s; user commits.

### Commit 1 — Ingestion seam: kill `-1`, add `ValidateProcessorIds` [D1+D2+D3]
**Files:** `RegularTasks.h`, `DAG_Model.h`, `DAG_Model.cpp`, `testIO.cpp`
1. `RegularTasks.h:99`: `int processorId = -1;` → `int processorId = 0;`. Update the
   comment (remove "-1 means…" semantics; 0 = default single core).
2. `DAG_Model`: add `ValidateProcessorIds(const TaskSet&)` — `<0` → throw; collect
   `max`, `vector<char> seen(max+1)`, every task marks its core, then assert every
   index `0..max` seen (else non-dense → throw). Call it at the top of `CategorizeTaskSet()`.
3. Delete `task_id2task_index_within_processor_` member (DAG_Model.h:119) + its write
   (DAG_Model.cpp:171). [D3 dead code]
4. `testIO.cpp:32,45`: `EXPECT_EQ(-1, …processorId)` → `EXPECT_EQ(0, …)`. Update
   the v2/v11 comment if it mentions `-1`.
5. Build + `ctest` green (this commit changes NO partition data structure yet —
   `processor2taskset_` is still a map; only the *input invariant* tightens).
   **Gate:** 17/17 ctest; bit-identical SP (no `-1` ever reaches a partition now).

### Commit 2 — DAG_Model: vectorize `processor2taskset_` [internal-only]
**Files:** `DAG_Model.h`, `DAG_Model.cpp`
1. `processor2taskset_`: `unordered_map<int,TaskSet>` → `vector<TaskSet>`.
2. `CategorizeTaskSet()`: size the vector to `max_processorId+1`; `processor2taskset_[p_id].push_back(tasks[i])`.
3. Add `GetProcessorPartition()` const accessor IF any reader needs it — but
   investigation shows NO external readers, so likely just remove the now-private
   member's exposure. (Verify no `.h` inline reader.)
4. Build + ctest green. **Gate:** bit-identical SP (pure storage change, same partition).

### Commit 3 — RTA: vectorize `ExtractTaskSetPerProcessor`
**Files:** `RTA.h`, `RTA.cpp`
1. Signature: `unordered_map<int,TaskSet>` → `vector<TaskSet>` (sized `max+1`).
2. Body: `processor_task_set[task.processorId].push_back(task)`.
3. `ProbabilisticRTA_TaskSet` (RTA.cpp:130-149): range-for over `vector<TaskSet>`.
4. Build + ctest green (3 internal call sites in RTA.cpp/RTA_Cache.cpp will break →
   fixed in Commit 4; so this commit + Commit 4 may need to land together OR this
   commit temporarily keeps a map-returning shim. **Decision: land C3+C4 as one
   logical unit** — see note below.)

### Commit 4 — RTA_Cache + PrioritySwitchAnalysis: vectorize per-core structures
**Files:** `RTA_Cache.h`, `RTA_Cache.cpp`, `PrioritySwitchAnalysis.h`
1. `ChampionState`: `champ_per_core`/`hp_prefix_per_core` → `vector<vector<int>>` /
   `vector<vector<FiniteDist>>`.
2. `PerCoreOrderFromPa`/`PerCoreOrderOfPrioritized` return `vector<vector<int>>`.
3. `RebuildPrefixes` (uses `ExtractTaskSetPerProcessor`): indexed loop.
4. `IsSingleTaskChange`/`ClassifyReusePerTask`/`Evaluate`: `.at(core)` → `vec[core]`;
   range-for over map → indexed loop over vector.
5. `PrioritySwitchAnalysis.h`: `FindCoreOfTask`/`AnalyzePrioritySwitch` take
   `vector<vector<int>>`; update the hand-built unit-test fixtures in `testRTA.cpp`.
6. Build + ctest green. **Gate:** bit-identical SP to HEAD.

> **C3+C4 landing note:** `ExtractTaskSetPerProcessor`'s 3 call sites span RTA.cpp
> (C3) and RTA_Cache.cpp (C4). Changing the return type breaks both files at once.
> Cleanest decomposition: C3 (signature+RTA.cpp caller) and C4 (RTA_Cache.cpp callers
> + struct changes) as **two commits that land back-to-back**, OR fold C3 into C4 as
> one "RTA layer vectorization" commit. Prefer two commits with a temporary
> `vector`-returning `ExtractTaskSetPerProcessor` whose RTA_Cache.cpp call sites are
> updated in C4 — build is broken between them only if built at that exact commit.
> If the user wants every commit independently green, fold C3+C4 → single "RTA layer"
> commit. **Default: fold into one commit** (keeps each commit shippable-green).

### Commit 5 — Fixtures + final gate [verification]
**Files:** `testRTA.cpp`, `testOptimizeIncrePA.cpp` (if any hand-built per-core map)
1. Update any remaining hand-built `unordered_map<int,vector<int>>` test fixtures →
   `vector<vector<int>>`.
2. Full gate: `cmake --build build_test --target check.SP_OPT --clean-first -j5`
   → 17/17 ctest + pytest; bit-identical SP probe vs HEAD (stash → build HEAD →
   capture → restore → rebuild → diff, timing-normalized).
3. Optional perf spot-check (N=10 INCR): record wall-clock vs HEAD; NOT a gate,
   just confirms no regression (hash removal should be neutral-to-faster).

---

### Commit shape summary
| # | Scope | Files | Gate |
|---|-------|-------|------|
| 1 | kill `-1` + `ValidateProcessorIds` + dead-code delete | RegularTasks.h, DAG_Model.h/.cpp, testIO.cpp | 17/17 + bit-id SP |
| 2 | vectorize `processor2taskset_` (internal) | DAG_Model.h/.cpp | 17/17 + bit-id SP |
| 3+4 (folded) | vectorize RTA layer (`ExtractTaskSetPerProcessor` + RTA_Cache + PrioritySwitchAnalysis) | RTA.h/.cpp, RTA_Cache.h/.cpp, PrioritySwitchAnalysis.h, testRTA.cpp | 17/17 + bit-id SP |
| 5 | fixtures + final bit-identical gate | testRTA.cpp, testOptimizeIncrePA.cpp | 17/17 + bit-id SP + perf spot-check |

**Total: 4 commits** (C3+C4 folded). Each is a coherent module; each gates on
bit-identical SP. Agent `git add`s after each; user commits.

---

## 2026-07-23 — Commit 1 LANDED (staged, awaiting user commit)

**Edits:**
- `RegularTasks.h:99`: `int processorId = -1;` → `int processorId = 0;` + comment
  rewrite (removed `-1` semantics; 0 = default single core; notes validation).
- `DAG_Model.h`: added `void ValidateProcessorIds() const;` decl; **deleted** dead
  member `task_id2task_index_within_processor_` (D3).
- `DAG_Model.cpp`: added `ValidateProcessorIds()` impl (reject `<0`; reject non-dense
  via `vector<char> seen(max+1)` + assert every `0..max` seen); call it at the top
  of `CategorizeTaskSet()`; removed the dead `task_id2task_index_within_processor_`
  write line.
- `testIO.cpp:32,45`: `EXPECT_EQ(-1, …processorId)` → `EXPECT_EQ(0, …)`.
- **Stray fixtures caught by the validator:** `test_robotics_v8.yaml` +
  `test_robotics_v27.yaml` both had `pids={1}` (all tasks on core 1, core 0
  missing → non-dense). Both are 2-task single-core sets mislabeled as core 1 →
  normalized to core 0 (consistent with D1). Proactively scanned all 31 TaskData
  yamls: ONLY these two were non-dense; none negative. So D2's density check
  will never reject valid generator output (greedy `np.argmin` fills 0,1,2,…).

**Build/gate:** `cmake --build build_test --target check.SP_OPT --clean-first -j5`
(header member removed → `--clean-first` per stale-`.o` rule) → **17/17 ctest**.
Direct run of the 8 `*Differential*`/`BitIdentical*` tests in testOptimizeIncrePA =
8/8 PASS. Commit 1 changes NO partition data structure (only tightens the input
invariant + deletes dead code), so SP output is provably neutral: `-1`→`0` only
affects NOSPEC yamls (all-default → all core 0 = same single-core semantics);
`ValidateProcessorIds` only throws on already-invalid fixtures (now fixed).

**Staged (agent `git add`):** `RegularTasks.h`, `DAG_Model.h`, `DAG_Model.cpp`,
`testIO.cpp`, `test_robotics_v8.yaml`, `test_robotics_v27.yaml`. Awaiting user commit.

