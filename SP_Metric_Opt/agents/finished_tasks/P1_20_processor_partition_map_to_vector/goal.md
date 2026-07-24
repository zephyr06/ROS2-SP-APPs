# P1.20 — Convert processor→task-set maps from hash table to vector

## The Goal

Every per-processor partition in the analytic RTA/SP path is currently a `std::unordered_map<int, ...>` keyed by `processorId`:

- `RTA_Cache.h:152` `champ_per_core_` — `unordered_map<int, vector<int>>`
- `RTA_Cache.h:160` `hp_prefix_per_core_` — `unordered_map<int, vector<FiniteDist>>`
- `RTA_Cache.h:170,176` `PerCoreOrderFromPa` / `PerCoreOrderOfPrioritized` return types
- `RTA_Cache.cpp` + `PrioritySwitchAnalysis.h` — `FindCoreOfTask`, `AnalyzePrioritySwitchPerCore` signatures
- `RTA.cpp` + `RTA.h` — `ExtractTaskSetPerProcessor` returns `unordered_map<int, TaskSet>`
- `DAG_Model.h:118` `processor2taskset_` — `unordered_map<int, TaskSet>`

A hash table is the wrong container here: `processorId`s should represent a dense, 0-based contiguous range $\{0, 1, \dots, N_{\text{cores}}-1\}$. Hash tables incur per-access hashing overhead, bucket probing, dynamic node allocations, and cache line misses.

This task introduces a fast **Processor ID Validation Seam (`ValidateProcessorIds`)** during taskset ingestion to guarantee that all processor IDs form a dense, 0-based non-negative integer sequence. Once validated, all downstream partition structures are safely converted from `std::unordered_map` to $O(1)$ flat `std::vector<std::vector<T>>` indexed directly by `processorId`.

---

## Architectural Strategy: Ingestion Validation & Flat Vectorization

### 1. Ingestion Validation (`ValidateProcessorIds`)
Implement `ValidateProcessorIds(const TaskSet& tasks)` at taskset ingestion (`DAG_Model` construction / `CategorizeTaskSet` & `RTACache::Initialize`):
- **Non-negative Check**: Verify every `task.processorId >= 0` (rejects `-1` unassigned sentinels).
- **Dense Contiguous Sequence Check**: Verify that the set of distinct processor IDs equals $\{0, 1, \dots, \max(\text{processorId})\}$.
- **Fail-Fast Error**: If validation fails, throw `std::invalid_argument` / `CoutError` with detailed diagnostic text.

### 2. Flat Vectorization ($O(1)$ Direct Indexing)
Once validated at entry, downstream partition structures use `std::vector<std::vector<T>>` sized to $N_{\text{cores}} = \max(\text{processorId}) + 1$:
- `processor2taskset_`: `std::vector<TaskSet>`
- `champ_per_core_`: `std::vector<std::vector<int>>`
- `hp_prefix_per_core_`: `std::vector<std::vector<FiniteDist>>`
- `PerCoreOrderFromPa` / `PerCoreOrderOfPrioritized`: `std::vector<std::vector<int>>`
- `ExtractTaskSetPerProcessor`: Returns `std::vector<TaskSet>`

### 3. Resolution of Design Questions (D1–D4)

- **D1. Canonical `nb_processors`**: `DAG_Model` and `RTACache` store `nb_processors = processor2taskset_.size()`, eliminating redundant `max(processorId) + 1` recalculations.
- **D2. Fast `FindCoreOfTask`**: Optional flat `task_id2core_` vector (`std::vector<int>`) indexed by `task_id` allows $O(1)$ task-to-core lookups without scanning core partitions.
- **D3. `ExtractTaskSetPerProcessor` Signature**: Return type cleanly switches to `std::vector<TaskSet>`.
- **D4. `-1` Sentinel Policy**: Unassigned sentinels (`-1`) are explicitly caught and rejected at `ValidateProcessorIds` before reaching partition builders.

---

## Scope & Non-Negotiable Requirements

1. **Bit-Identical Safety Performance (SP) Output**:
   Replacing hash tables with vectors must produce exact, bit-identical SP metrics across all unit tests and simulation experiments.
2. **Strict Validation Fail-Fast**:
   Tasksets with invalid, sparse, or negative processor IDs must fail fast with descriptive errors.
