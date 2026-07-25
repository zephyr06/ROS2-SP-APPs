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

## Reuse one challenger across `EvaluateTimeLimitConfig_ScratchOrIncre` calls — DEFERRED

* **Location:** `sources/Optimization/OptimizeSP_TL_Incre.cpp`
  (`EvaluateTimeLimitConfig_ScratchOrIncre`, `:142`; the incremental branch
  `:160` calls `BuildChallengerFromIncumbent()`, defined at `:402`).
* **Bottleneck:** Each incremental candidate constructs a FRESH
  `OptimizePA_Incre` challenger from `res_opt_` (the champion) via
  `BuildChallengerFromIncumbent` — discarding the previous challenger's PA
  search state. The TL coordinate-descent walk
  (`PerformCoordinateDescentForTaskConfigOpt` → `OptimizeSingleTaskTimeLimit`)
  evaluates one candidate per TL step, so a long walk rebuilds the challenger
  many times within one interval.
* **Optimization:** Keep a persistent challenger optimizer and modify it in
  place each candidate (true incremental PA search — reuse the search state,
  not just the adopted TL), instead of rebuilding from `res_opt_` every call.
* **Trade-off / why deferred:** Could save PA-search work, BUT a persistent
  challenger would advance `dag_tasks_` to the last-evaluated (possibly
  non-adopted) candidate each call, drifting the diff baseline off the adopted
  working TL and flagging EXTRA tasks (the explored-but-not-adopted previous
  task) → potentially MORE RTA evals, not fewer. The current rebuild-from-
  champion design was chosen over the persistent challenger (P0.5 Phase-5
  issue 5b, decided 2026-07-10) precisely because the champion tracks the
  working TL so the diff flags only the one task being walked — the perfect
  case for incremental optimization. Net: rebuild weakly dominates within-
  interval; the persistent challenger's only potential edge (cross-interval PA
  re-search of DAG-mutated tasks) is a separable mechanism that could be
  added to the rebuild design if measurement ever shows it helps.
* **State:** deferred. Originally P0.5 Phase-5 issue 5h ("Reuse a single
  optimizer instance across `EvaluateTimeLimitConfig_ScratchOrIncre` calls
  instead of rebuilding per candidate. Efficiency."), moved here 2026-07-10
  as a perf, not correctness, item. Pick up only if profiling shows the
  challenger rebuild is a runtime blocker.

---

## Reference: implemented optimizations (historical)

These are **already landed**; recorded here for "why is the code this shape"
context. The source is the source of truth.

* **Incremental HP-task convolution $O(N^2) \to O(N)$** — IMPLEMENTED in
  `sources/Safety_Performance_Metric/RTA.cpp` (`ProbabilisticRTA_TaskSet_SingleCore`).
  Maintains a running convolved distribution `hp_tasks_et_conv` of all higher-priority
  tasks as we iterate through the sorted taskset, calling the 3-argument version of
  `GetRTA_OneTask` to convolve the current task with the running distribution.
  Correctness verified by unit tests.
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

---

## Proposed New Optimization Ideas

### 1. Incremental and Memoized RTA (Processor-Level & Priority-Level)
* **Concept**: Avoid running full RTA on all processors and all tasks when only a single task configuration or priority changes.
* **Mechanism**:
  * **Processor Isolation**: RTA is completely independent across processors. If a task limit or priority changes on CPU $A$, do not re-evaluate tasks on CPU $B$.
  * **Priority Prefix Reuse**: If task $T_i$'s priority is changed (e.g., during 1D search variations), tasks with priority higher than both the old and new position of $T_i$ are unaffected. Cache the running convolved HP task execution time distribution to warm-start RTA from the first affected priority index.
* **Expected Impact**: **High**. Eliminates up to 80-90% of convolutions during 1D search.

### 2. RTA Short-Circuiting for Guaranteed Deadline Misses
* **Concept**: If a task's response time is guaranteed to miss its deadline, stop convolving.
* **Mechanism**: If `rta_cur.min_time > task_curr.deadline` at any point during preemption resolution, its safety-performance (SP) metric is already guaranteed to be $0$. We can immediately stop convolving further preemptions for this task.
* **Expected Impact**: **Medium**. Saves CPU time when evaluating highly unschedulable task configurations.

### 3. Avoiding Sort in Convolution (Sorted Merge)
* **Concept**: Convolving two sorted distributions of size $N$ and $M$ produces a set of values that can be generated/merged in sorted order without using `std::sort` on $N \times M$ elements.
* **Mechanism**: Use a multi-way merge (via a min-heap of size $N$) to generate convolved elements in sorted order, or a two-pointer merge if one of the distributions is very small (e.g., single preemption).
* **Expected Impact**: **Medium**. Reduces the sorting overhead of $O(G^2 \log G^2)$ to $O(G^2 \log G)$ or $O(G^2)$.

### 4. Low-Probability Tail Pruning during Convolution
* **Concept**: Filter out state combinations with negligible probability (e.g., $< 10^{-12}$) before sorting and coalescing.
* **Expected Impact**: **Low-Medium**. Reduces the number of elements processed by `std::sort` and `Coalesce`.
