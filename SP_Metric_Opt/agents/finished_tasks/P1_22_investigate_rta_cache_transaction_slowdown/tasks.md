# P1.22 — Tasks (working checklist)

> Goal: Investigate and resolve the performance slowdown caused by RTA cache transaction abstraction.

> **CLOSED 2026-07-23 — SUPERSEDED by P1.25.** The "does the transaction pay
> for itself" debate is moot: P1.25 removed the transaction layer entirely
> (kept the RTA cache), restoring the `1217d227` eager (b) backup on the reject
> path. P1.22's findings stand as the motivation: 80.3% of captures thrown away
> by rollback (530/660); the original "+5.02%@N=10 slowdown" premise was a
> timer-scoping artifact (P1.23: corrected timer → cache ~36% *faster*, not
> slower). The temporary `TransactionCounters` instrumentation + the
> `p1_22_counter_probe.json` probe config were reverted/deleted by P1.25
> Phases 0b/0c/2b. No further P1.22 work. See
> [[p125-remove-rta-cache-transaction]], [[p123-ab-rta-cache-speedup]].

## Phase 1 — Investigation & Profiling Setup

- [x] **1a. Profile Transaction Trigger Frequency & Heap Overhead** (2026-07-23)
  - Added temporary `TransactionCounters` + `atexit` stderr dump in `RTA_Cache.cpp`
    (UNCOMMITTED, to revert). Probe config `p1_22_counter_probe.json` (INCR_Reopt_10
    only, N={10,16}, 3 tasksets, 300s, 1 worker). See dev_log 2026-07-23.
  - **Hypothesis 1 OVER-estimated.** Lazy snapshots fire on **~40%** of opened tx
    (N=10: 38%, N=16: 42%), NOT 90–95%. 60% of tx (reject-without-adopt) pay ZERO
    copy — the lazy premise HOLDS. AdoptChampion fires in-tx on 89% of adopt calls;
    mean 0.82 in-tx adopts/opened tx. Heap alloc is only ~110/run → NOT the cost.
- [ ] **1b. Measure Memory Allocation Cost**
  - Profile the execution time of `std::make_unique<ChampionState>` dynamic heap allocations vs pre-allocated/stack buffers.

### Decision needed (2026-07-23, before Phase 2)
Frequency data (1a) rules out heap churn and confirms the lazy-COW premise
holds (~40% capture, NOT 90–95%; 60% zero-copy). BUT the original
"+5.02%@N=10 transaction slowdown" reading may be a **timer-scoping artifact**
— P1.23 ([[p123-ab-rta-cache-speedup]]) found the old timer bracketed all of
`RunSimulation` (RTDA + I/O), washing the optimizer delta; under the corrected
timer (`cc9aa0ce`) the cache arm was ~36% *faster*, not slower. Options:
- **(A)** corrected-timer A/B: HEAD-with-Transaction vs pre-Transaction `ecbed896` → verify a regression exists at all.
- **(B)** Phase 1b: release-build timing profile to locate cost (deep-copy on capture/restore vs redundant `RebuildPrefixes` on rolled-back in-tx adopts).
- **(C)** treat frequency data as sufficient; reconsider whether P1.22 stays open.

NOTE: `RTA_Cache.cpp` instrumentation + `p1_22_counter_probe.json` are
TEMPORARY/uncommitted — revert before any commit.

## Phase 2 — Optimization Implementation

- [ ] **2a. Eliminate Dynamic Heap Allocation**
  - Replace `std::unique_ptr<ChampionState>` in `RTACache` with a persistent, reusable `ChampionState` buffer (or `std::optional<ChampionState>`).
- [ ] **2b. Reuse Internal Vector Capacities**
  - Modify `CaptureChampionState` to assign/copy into existing vector capacities instead of constructing new vectors per snapshot.
- [ ] **2c. Evaluate Deferred Mid-Walk Prefix Rebuilding**
  - Investigate if `RebuildPrefixes` can be deferred or streamlined during intermediate 1D search adopts.

## Phase 3 — Verification & Benchmarking

- [ ] **3a. Run Full Test Gate**
  - Ensure all 17 C++ tests (`ctest`) and Python unit/integration tests pass (bit-identical SP results).
- [ ] **3b. Benchmark Optimization Gains**
  - Compare execution timing before and after eliminating heap allocation churn.
