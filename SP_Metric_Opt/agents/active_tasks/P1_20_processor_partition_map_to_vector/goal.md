# P1.20 — Convert processor→task-set maps from hash table to vector

## The Goal
Every per-processor partition in the analytic RTA/SP path is currently a
`std::unordered_map<int, ...>` keyed by `processorId`:

- `RTA_Cache.h:152`  `champ_per_core_`            — `unordered_map<int, vector<int>>`
- `RTA_Cache.h:160`  `hp_prefix_per_core_`        — `unordered_map<int, vector<FiniteDist>>`
- `RTA_Cache.h:170,176` `PerCoreOrderFromPa` / `PerCoreOrderOfPrioritized` return types
- `RTA_Cache.cpp:50,100,101` + `PrioritySwitchAnalysis.h:40,90,91` — `FindCoreOfTask`,
  `AnalyzePrioritySwitchPerCore` signatures (take the partition by const-ref)
- `RTA_Cache.cpp:154,167,230,277,282,430` — locals built per call / per champion
- `RTA.cpp:117,119,134` + `RTA.h:27` — `ExtractTaskSetPerProcessor` returns
  `unordered_map<int, TaskSet>`
- `DAG_Model.h:118` `processor2taskset_`          — `unordered_map<int, TaskSet>`
- `tests/testRTA.cpp:1192` — a local `per_core` initializer (test fixture)

A hash table is the wrong structure here: processorIds are a dense contiguous
`[0, nb_processors)` range (Phase 0 verifies this), so an `unordered_map` pays
hash + bucket + indirection overhead per lookup for an O(1) array index that a
`std::vector` gives directly. The RTA cache's hot path — `Evaluate` and
`TryComputeSingleChange`, both called once per candidate on the serialized opt
walk — builds and probes these maps repeatedly (P1.17 catalogued the per-call
rebuilds). Replacing the map with a vector indexed by `processorId` removes the
per-access hash cost and tightens the cache's memory locality, on top of the
P1.17 redundancy work.

## Scope
- **In scope:** the processor-keyed maps listed above in `RTA_Cache.{h,cpp}`,
  `RTA.{h,cpp}` (`ExtractTaskSetPerProcessor`), `DAG_Model.{h,cpp}`
  (`processor2taskset_`), `PrioritySwitchAnalysis.h` signatures, and the
  `testRTA.cpp` fixture. The `SimulationOrchestrator` `trace_indices` maps
  (`SimulationOrchestrator.{h,cpp}`) are keyed by trace index, NOT processor —
  out of scope.
- **Out of scope:** `DAG_Model.h:119` `task_id2task_index_within_processor_` is
  keyed by *task id*, not processorId — a separate vectorization concern, not
  this task. The sim-side RunQueue-per-core partition (P1.7) is a different
  structure; mention as related, do not touch.
- **Non-negotiable gate:** bit-identical SP to the oracle on every fixture
  (differential TDD, same contract as P1.12/P1.17/P1.18). A vectorization that
  moves any SP bit is rejected. The 17/17 ctest DEBUG gate + 56/56 testRTA must
  stay GREEN.

## Why a vector is safe (the precondition Phase 0 must prove)
A `vector` indexed by `processorId` requires processorIds to be a dense
`[0, N)` range with no gaps and no negatives. `RegularTasks.h:99` declares
`int processorId = -1; // -1 means not assigned to any processor, or all` — so
the `-1` sentinel exists in the type. Phase 0 must prove that **by the time any
of the in-scope maps is built**, every task's `processorId` is `>= 0` and the
set of distinct ids is exactly `{0, 1, ..., nb_processors-1}`. If `-1` ever
flows into a partition build, the vector form would be a hazard (index -1) and
the task is blocked until that's resolved or filtered upstream.

## Design questions to resolve (filed, not answered)
- **D1.** Is there a canonical "number of processors" available where the maps
  are built, or must each site derive it as `max(processorId)+1`? `DAG_Model`
  has `processor2taskset_` (the distinct ids ARE the cores) but no explicit
  `nb_processors_` member. A vector needs the size up front; decide whether to
  add an accessor or compute locally.
- **D2.** `FindCoreOfTask` (PrioritySwitchAnalysis) currently scans the map to
  locate a task's core. With a vector, the lookup inverts: do we keep a parallel
  `task_id → core` vector, or is the scan-over-cores still acceptable (cores are
  few)? P1.18 may add a same-core-suffix verdict that reads `core` per task —
  coordinate the representation.
- **D3.** `ExtractTaskSetPerProcessor` (RTA.h) is a free function returning the
  map by value; its return type is part of the public RTA API. Changing it to
  `vector<TaskSet>` ripples to every caller — enumerate them before changing the
  signature. (Agent rule: no backward compat — change ruthlessly, but know the
  blast radius first.)
- **D4.** Does the `-1` sentinel ever reach `CategorizeTaskSet`
  (`DAG_Model.cpp:164`)? If a task is genuinely unassigned at DAG construction,
  the vector form needs a policy (reject, or assign to a synthesized core).
  Phase 0 answers this before any code change.
