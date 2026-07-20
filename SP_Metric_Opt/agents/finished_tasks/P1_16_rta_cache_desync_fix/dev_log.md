# P1.16 — Dev Log

## 2026-07-19 — Initial Analysis & Proposal

### The Crash Mechanism
We investigated why the C++ binary aborts (exit code 134) under `INCR_Reopt_5/10/30/60` on tasksets 0, 3, 6, 7, and 8.
The crash originates from `RTACache::ComputeTaskSetDifference(...)` throwing `std::runtime_error` because `TryComputeSingleChange` returns `false` (candidate differs from champion by >1 task).

During `PerformSerializedTaskQueueOptimization`, we iterate through a queue of tasks. For each task:
1. `EvaluateTimeLimitConfig_SubIncremental` is called.
2. It builds `challenger` from `res_opt_` (priority $PA_{old}$, time limits $TL_{old}$).
3. It calls `challenger.OptimizeIncre_SingleTask(..., std::ref(rta_cache_))`.
4. Inside `OptimizeIncre_SingleTask`, we check priority variations for the task. If a variation gets a better SP, it speculatively calls `AdoptChampion` on the cache, updating the cache's champion to `PA_new_A` (and the trial baked DAG of task A).
5. When `OptimizeIncre_SingleTask` returns, we call `UpdateRecords`. If this configuration is rejected by `UpdateRecords` (e.g. SP not strictly higher or tie-breaker not met), `CommitIncumbent` is **not** called.
6. Thus, `res_opt_` remains in the old state ($PA_{old}$, $TL_{old}$), but the cache champion retains the speculatively adopted state ($PA_{new\_A}$ + task A's trial DAG). They are now desynchronized.
7. On the next task walk (task B), `EvaluateTimeLimitConfig_SubIncremental` builds the challenger from `res_opt_` ($PA_{old}$) and calls `rta_cache_.Evaluate(dag_tasks_cur, PA_old, TL_new_B)`.
8. The cache compares the Candidate `(dag_tasks_cur, PA_old, TL_new_B)` vs the Champion `(dag_tasks_A_trial, PA_new_A, no_tl)`.
9. The candidate and champion differ in BOTH task A's ET, task B's ET, and task A's priority. This represents a multi-task change ($et\_diff.size() == 2$), which violates the single-change invariant, causing a throw and abort.

### Proposal: Reverting Cache on Rejection
We compared two ways to handle this:
* **Option A (Graceful Degradation)**: Modify the cache `Evaluate` to fall back to a full RTA computation (`Initialize`) when `!IsSingleTaskChange(...)`. While this prevents crashes, it is slow ($0.5\text{ms}$ to $2\text{ms}$ per full RTA) and since rejected walk steps are very common in search algorithms, we will suffer from frequent cache misses.
* **Option B (Cache Backup & Revert)**: Since copying the `RTACache` state is extremely fast (only around 50 `FiniteDist` objects of bounded size, taking $1\text{--}5\mu\text{s}$), we can backup `rta_cache_` at the start of `EvaluateTimeLimitConfig_SubIncremental` and restore it if `UpdateRecords` rejects the configuration. This keeps the cache hit rate near $100\%$ and completely avoids desynchronization.

We choose **Option B** as it is highly efficient and clean.
