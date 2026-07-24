# P1.22 — Investigate RTA Cache Transaction Performance Slowdown

## The Goal
Investigate why the `RTACache::Transaction` (Lazy Copy-on-Write) abstraction introduced in P1.21 resulted in a slight performance slowdown compared to the prior eager stack-backup approach during incremental optimization, and engineer high-performance optimizations to eliminate the overhead while keeping a clean API.

---

## Background & Initial Findings

During P1.21, the eager stack copy (`RTACache cache_backup = rta_cache_;`) was replaced with a scoped `RTACache::Transaction` using Lazy Copy-on-Write (CoW). The theoretical design assumption was that candidate step rejections without champion updates would dominate, making most sub-incremental walk steps zero-copy ($O(1)$).

However, empirical runtime profiling revealed a slight performance regression.

---

## Detailed Hypotheses for the Slowdown

### 1. High Frequency of Mid-Walk `AdoptChampion` Calls (Invalid Zero-Copy Premise)
- **Hypothesis**: The lazy snapshot assumption ("champion updates are rare") does not hold during `OptimizeIncre_SingleTask`.
- **Mechanism**: `OptimizeIncre_SingleTask` searches 1D priority variations within *every* sub-incremental step. Whenever a 1D priority move improves SP locally, `OptimizeIncre_SingleTask` calls `AdoptChampion` to advance the baseline so subsequent variations satisfy the $|\text{diff}| \le 1$ invariant.
- **Consequence**: Local priority improvements occur frequently during 1D search, so `AdoptChampion` fires in almost every sub-incremental step. The lazy snapshot is triggered ~90–95% of the time anyway, failing to provide zero-copy savings.

### 2. Heap Memory Allocation Churn (`std::make_unique<ChampionState>`)
- **Hypothesis**: Replacing stack allocation with dynamic heap allocation introduces noticeable memory allocator overhead (`malloc`/`free`) and cache line invalidations.
- **Mechanism**: Prior to P1.21, `RTACache cache_backup = rta_cache_;` allocated the backup object on the stack. P1.21 uses `snapshot_ = std::make_unique<ChampionState>(CaptureChampionState());`, which allocates a new `ChampionState` object on the heap on every snapshot trigger, followed by `snapshot_.reset()` (`free`) on scope exit.
- **Consequence**: Running dynamic heap allocations/deallocations thousands of times per second during optimization walks adds significant allocator latency.

### 3. Redundant Mid-Walk Bakes & Convolutions (`RebuildPrefixes`)
- **Hypothesis**: Intermediate `AdoptChampion` calls inside `OptimizeIncre_SingleTask` pay unnecessary computation costs for rejected walk steps.
- **Mechanism**: Each `AdoptChampion` call invokes `BakeChampionForms`, which executes `RebuildPrefixes` (re-rolling all per-core execution-time convolutions). If the outer step is ultimately rejected by `UpdateRecords`, all those intermediate per-core convolutions are discarded upon `RollbackTransaction()`.

### 4. Pointer Indirection & Branching Overhead
- **Hypothesis**: Checking transaction state (`in_transaction_`, `has_snapshot_`) and dereferencing `std::unique_ptr` adds instruction overhead on hot evaluation paths.
- **Mechanism**: Extra branches in `AdoptChampion`, `Initialize`, and `Evaluate`, combined with pointer indirection through `snapshot_`.

---

## Proposed Remediation & Optimization Plan

1. **Eliminate Heap Allocation**: Replace `std::unique_ptr<ChampionState>` with an inline/reusable `ChampionState` buffer inside `RTACache` (or `std::optional<ChampionState>`) to ensure zero dynamic heap allocation per transaction.
2. **Buffer Capacity Reuse**: Implement `CaptureChampionState` such that vector capacities (`rta`, `champ_prioritized`, `champ_per_core`, `hp_prefix_per_core`) are retained across transactions rather than re-allocated from scratch.
3. **Profile & Benchmark**: Run targeted micro-benchmarks comparing:
   - Baseline (eager stack copy)
   - Current P1.21 (lazy heap `unique_ptr`)
   - Optimized zero-heap allocation transaction buffer.
