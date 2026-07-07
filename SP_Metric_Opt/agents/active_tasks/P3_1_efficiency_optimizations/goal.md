# P3.1 — Efficiency Optimizations (reference)

> Memory and algorithmic optimizations for the `INCR` scheduler and Response Time
> Analysis (`RTA`). **All live items below are deferred** (perf, not correctness)
> — see the Deferred/P3 table in `agents/overall_tasks.md` for the let-go
> rationale. This file is kept as **design reference** for the optimizations
> themselves, not an active to-do list. Pick these up only if P1.1 finds
> per-activation ET is a paper blocker.

---

## Convert `PriorityPartialPath` to pointers — DEFERRED

* **Location:** `sources/Optimization/OptimizeSP_Incre.h` (struct `PriorityPartialPath`)
* **Bottleneck:** `PriorityPartialPath` stores `DAG_Model dag_tasks` and
  `SP_Parameters sp_parameters` by value. Audsley's search loop
  (`OptimizeFromScratch`) creates and copies thousands of partial paths:
  ```cpp
  PriorityPartialPath new_path = path; // Triggers full deep copy of task graph and parameters
  ```
  This wastes CPU cycles on heap allocations and copying unchanged constants.
* **Optimization:** Convert the fields to `const DAG_Model*` and
  `const SP_Parameters*` pointers. This reduces the copy footprint to a few
  bytes and makes copy construction virtually free.
* **State:** still by-value as of 2026-07-04; the commented-out reference
  members at lines 39–40 of `OptimizeSP_Incre.h` are the leftover sketch of
  this idea.

---

## Incremental HP-task convolution $O(N^2) \to O(N)$ — DEFERRED (on hold)

* **Location:** `sources/Safety_Performance_Metric/RTA.cpp`
  (`ProbabilisticRTA_TaskSet_SingleCore`)
* **Bottleneck:** For task `i`, the algorithm calls
  `GetRTA_OneTask(tasks[i], hp_tasks)`, which performs $i$ convolutions of all
  higher-priority task execution times from scratch.
* **Optimization proposal:** Maintain a running convolved distribution
  `hp_conv` of all higher-priority tasks as we iterate through the taskset, and
  convolve task `i` with `hp_conv` once.
* **State:** ON HOLD — needs verification that dynamic preemptions and
  deadlines are evaluated correctly under merged convolutions.

---

## Reference: implemented optimizations (historical)

These are **already landed**; recorded here for "why is the code this shape"
context. The source is the source of truth.

* **Flat vector sort-coalesce for convolution** — IMPLEMENTED in
  `FiniteDist::Convolve` (`sources/Safety_Performance_Metric/Probability.cpp:71-`).
  Convolves into a flat pre-allocated `std::vector<Value_Proba>`,
  `reserve(distribution.size() * other.distribution.size())`, then sorts, then
  single-pass coalesces. Replaces the old `std::unordered_map<double,double>`
  hot path (hashing, bucket lookup, per-node heap alloc) with contiguous cache
  -friendly memory, one allocation, and FP-safe coalescing
  (`std::abs(a-b) < 1e-9`). The `unordered_map` path survives only in the
  separate `GetV_PMap` / `ConvolveWithMap` helpers (lines 6, 47), not in the
  hot `Convolve`.
* **Per-TL optimizer cache** — OBSOLETE. The `timelimit2optimizer_` cache was
  removed in P25 Commit 2 (see `agents/finished_tasks/P24_task.md` § Commit 2)
  and replaced by a single persistent `OptimizePA_Incre prev_optimizer_`
  incumbent. The incumbent-copy cost the old cache-item was trying to avoid is
  now addressed structurally (one incumbent, not a map of them).
