# P1.20 — Dev log

## 2026-07-20 — Filed (no code)

Filed by user request: convert every processor→task-set partition in the
analytic RTA/SP path from `std::unordered_map<int, ...>` to `std::vector<...>`
indexed by `processorId`. Motivation: the maps are keyed by a dense `[0, N)`
range, so a hash table pays per-access hash/bucket/indirection cost for what is
an O(1) array index. The hot path (`RTACache::Evaluate` +
`TryComputeSingleChange`, once per candidate on the serialized opt walk) builds
and probes these repeatedly — P1.17 catalogued the per-call rebuilds; this task
attacks the *container* cost, P1.17 attacked the *rebuild* count.

### Site inventory (from the grounding grep)

Processor-keyed `unordered_map` sites in scope:

| File:line | Symbol | Container |
|---|---|---|
| `RTA_Cache.h:152` | `champ_per_core_` (member) | `unordered_map<int, vector<int>>` |
| `RTA_Cache.h:160` | `hp_prefix_per_core_` (member) | `unordered_map<int, vector<FiniteDist>>` |
| `RTA_Cache.h:170,176` | `PerCoreOrderFromPa` / `PerCoreOrderOfPrioritized` (return) | `unordered_map<int, vector<int>>` |
| `RTA_Cache.cpp:154,167` | (the two helpers' bodies) | — |
| `RTA_Cache.cpp:230,430` | `Evaluate` locals `per_core` | `unordered_map<int, TaskSet>` |
| `RTA_Cache.cpp:277,282` | candidate + champion `per_core` in `TryComputeSingleChange` | `unordered_map<int, vector<int>>` |
| `RTA.cpp:117,119,134` + `RTA.h:27` | `ExtractTaskSetPerProcessor` (free fn) | `unordered_map<int, TaskSet>` |
| `DAG_Model.h:118` | `processor2taskset_` (member) | `unordered_map<int, TaskSet>` |
| `PrioritySwitchAnalysis.h:40,90,91` + `RTA_Cache.cpp:50,100,101` | `FindCoreOfTask` / `AnalyzePrioritySwitchPerCore` (signatures) | `unordered_map<int, vector<int>>&` |
| `tests/testRTA.cpp:1192` | fixture local `per_core` | `unordered_map<int, vector<int>>` |

Out of scope (NOT processor-keyed despite the grep hit):
- `DAG_Model.h:119` `task_id2task_index_within_processor_` — keyed by *task id*.
- `SimulationOrchestrator.{h,cpp}` `trace_indices` — keyed by trace index.

### Phase 0 status (the precondition — NOT yet run)

The vector form requires processorIds to be a dense `[0, N)` range, all `>= 0`.
Evidence so far (grep only, no test run):
- `RegularTasks.h:99` — `int processorId = -1; // -1 means not assigned` → the
  sentinel EXISTS in the type. Phase 0 must prove it never reaches a partition
  build.
- `testRTA.cpp:272-275` — fixture assigns `{0,0,1,1}` (dense, non-negative). ✓
- `DAG_Model.cpp:164` `CategorizeTaskSet` — builds `processor2taskset_` straight
  from `tasks[i].processorId`; no `-1` filter. If `-1` can flow in, this is the
  hazard site (D4).

Phase 0b's property test (every fixture: `processorId >= 0` AND distinct ids ==
`{0..max}`) is the safety net for every later phase — it must land FIRST.

### Open questions (filed in goal.md, not answered)
- D1: is there a canonical `nb_processors`, or derive `max(processorId)+1`?
- D2: does `FindCoreOfTask`'s task→core lookup need a parallel vector?
- D3: `ExtractTaskSetPerProcessor` return-type change blast radius.
- D4: the `-1` sentinel policy at `CategorizeTaskSet`.

### Status
FILED. No production code touched. Awaiting Phase 0.
