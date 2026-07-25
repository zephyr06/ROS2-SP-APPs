# P1.17 — Remove redundant copy / recompute ops in the RTA cache + main optimization code

## The Goal
Audit and remove copy-paste / unnecessary operations that have accumulated across
the RTA cache (`sources/Safety_Performance_Metric/RTA_Cache.{h,cpp}`) and the main
optimization code (`sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}`,
`sources/Optimization/OptimizeSP_Incre.{h,cpp}`, and the SP-metric primitives in
`sources/Safety_Performance_Metric/RTA.{cpp}`). The focus is on:

1. **Unnecessary copies** of `FiniteDist` / `TaskSet` / `PriorityVec` / `RTACache`
   that are taken defensively and then discarded, or taken when a const-ref /
   move would do.
2. **Redundant recomputes** of the same quantity within one evaluation path
   (e.g., the candidate DAG being TL-baked + priority-sorted more than once; the
   per-core order map rebuilt by multiple callees on the same inputs).
3. **Copy-pasted bodies** between `RTACache::Evaluate`,
   `RTACache::ComputeTaskSetDifference`/`ClassifyReusePerTask`, and the oracle
   `ProbabilisticRTA_TaskSet` path that can be factored into shared helpers
   (without changing the bit-identical-to-oracle contract).

This is a **refactor + perf** task (P1 priority: touches a hot path), NOT a
correctness task. The non-negotiable invariant: **bit-identical SP output to the
current committed code on every fixture** (differential TDD, same gate as
P1.12). Behavior-preserving by construction; if a candidate change moves any SP
bit, it is rejected.

## Scope boundary
- **In scope:** the cache internals + the TL/INCR optimizer eval path that
  consumes the cache.
- **Out of scope (separate tasks):** extending `ClassifyReusePerTask` to NEW
  reuse types = **P1.18**; the cache's `AdoptChampion` prefix-rollback already
  shipped in P1.16; dead `ifTimeout` pruning (P1.14 follow-up, noted there).
- C++ only this task (the Python harness is P1.15's domain).

## Why now
The P1.12 read-side swap + the P1.16 backup/revert + the P1.12 item-1b task-id
reindex all layered copies on top of copies (the `Evaluate` body now bakes the
candidate DAG, bakes the champion DAG, builds `task_id2index`,
`PerCoreOrderFromPa`, `ExtractTaskSetPerProcessor`, AND takes a full
`RTACache cache_backup` per `EvaluateTimeLimitConfig_SubIncremental` call). Now
that the crash (P1.16) and the harness (P1.15) are settled, this is the cleanup
pass that makes the cache's runtime cost honest before the P1.12 Phase 2
scalability measurement at N=6/10/16 — measuring a cache that still carries
defensive copies would understate the win.
