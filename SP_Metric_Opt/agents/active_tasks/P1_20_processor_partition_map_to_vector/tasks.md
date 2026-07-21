# P1.20 — Tasks (working checklist)

> Perf refactor. Bit-identical SP output is the only acceptance gate.
> TDD-first: pin the processorId invariant with a test BEFORE relying on it.
> Per the agent rule: `git add` only; the user commits. Work by module, commit
> by module.

## Phase 0 — Prove the invariant (no production code changes)

- [ ] **0a. Enumerate every processorId assignment site** and confirm each
  emits a non-negative int. Sources to audit: `RegularTasks.cpp:85` (YAML read),
  the testRTA fixtures (`:272-275`, `:770`), and `Gen_Taskset/lib/*.py` (the
  generator that writes the YAML). Record the distinct id sets each emits in
  `dev_log.md`. If any site can emit `-1` or a sparse set, FLAG it here — the
  vector form is blocked on that.
- [ ] **0b. Pin the invariant with a differential/property test** that, for
  every fixture the ctest suite + testRTA exercises, asserts:
  (i) every task's `processorId >= 0`, and
  (ii) the set of distinct ids == `{0, 1, ..., max(processorId)}` (dense, no
  gaps). RED→GREEN on the current tree (should be GREEN immediately — this is a
  pin, not a fix). This test is the safety net for every later phase.
- [ ] **0c. Confirm the gate is GREEN on the unmodified tree** as the baseline:
  17/17 ctest DEBUG + 56/56 testRTA. — record the exact command + timing.

## Phase 1 — `RTA_Cache` internals (the hot path)

- [ ] **1a. `champ_per_core_` + `hp_prefix_per_core_` → vector.** Change the two
  members (`RTA_Cache.h:152,160`) from `unordered_map<int, vector<...>>` to
  `vector<vector<...>>` indexed by processorId. Populate in `Initialize`/
  `AdoptChampion` (size = nb processors, derived per D1). `TryComputeSingleChange`
  reads `champ_per_core_` by index. Differential GREEN.
- [ ] **1b. `PerCoreOrderFromPa` / `PerCoreOrderOfPrioritized` return vector.**
  Change both return types (`RTA_Cache.h:170,176`) + bodies (`RTA_Cache.cpp:154,167`)
  to `vector<vector<int>>`. Update `Evaluate`'s candidate-side `per_core` build
  (`:277`) and the `ComputeTaskSetDifference` call sites. Differential GREEN.
- [ ] **1c. `Evaluate`'s `unordered_map<int, TaskSet> per_core` (`:230, :430`).**
  These two locals partition for the recompute loop; convert to vector. Check
  whether `ExtractTaskSetPerProcessor` (Phase 2) feeds them or they're built
  inline — if inline, this is independent of Phase 2. Differential GREEN.

## Phase 2 — RTA + DAG_Model public API (ripple surface)

- [ ] **2a. `ExtractTaskSetPerProcessor` → return `vector<TaskSet>`.** Change
  signature in `RTA.h:27` + body in `RTA.cpp:117-134`. Enumerate ALL callers
  first (D3) — the agent rule is no backward compat, but know the blast radius.
  Differential GREEN.
- [ ] **2b. `DAG_Model::processor2taskset_` → vector.** Change the member
  (`DAG_Model.h:118`) + `CategorizeTaskSet` build (`DAG_Model.cpp:164`). Resolve
  D4 (the `-1` sentinel policy) here — if Phase 0 found `-1` can flow in, this
  is where the filter/reject lands. Differential GREEN.

## Phase 3 — Test fixture + verify + measure

- [ ] **3a. `testRTA.cpp:1192` local `per_core` initializer** — convert to a
  vector initializer matching the new types. Keep the fixture's processorIds as
  the `[0,1]` dense pair (the Phase 0 invariant source).
- [ ] **3b. Full gate GREEN:** 17/17 ctest DEBUG + 56/56 testRTA + the P1.15
  crash suite; release build clean.
- [ ] **3c. Scalability measurement at N=6/10/16** (shares the P1.12 Phase 2
  item 5 / P1.17 3b harness): record per-candidate `Evaluate` time before/after
  this task in `dev_log.md`. The vector win is only real if it shows up here.
- [ ] **3d. Update `overall_tasks.md` + memory.** Mark P1.20 resolved;
  cross-link from P1.17 (this is a sibling perf task on the same hot path) and
  P1.18 (D2 — the `task_id → core` representation question affects the suffix-
  reuse verdict P1.18 adds).
