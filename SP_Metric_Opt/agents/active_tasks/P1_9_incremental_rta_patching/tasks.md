# P1.9 — Tasks (working checklist)

> See `goal.md` for scope. **Perf, not correctness** — every patch must be
> bit-identical to the full recompute (TDD vs the oracle). Target regime:
> larger N (N=10/N=16). Per `agent_coding_rules.md`: one small sub-task at a
> time, ask the user to review + commit after each.

## Locked design decisions (2026-07-13)

1. **`hp_tasks_et_conv_vec[i]`** = convolution of the **higher-priority
   tasks' ET distributions** `[0, i)` on a core — i.e. the rolling
   `hp_tasks_et_conv` (`RTA.cpp:73`) snapshotted at priority index `i`.
   `hp_tasks_et_conv_vec[0]` = `FiniteDist({Value_Proba(0, 1.0)})` (empty HP
   set). The 3-arg `GetRTA_OneTask` (`RTA.cpp:44`) already consumes exactly
   this value, so a patched suffix gets `ResolvePreemptionsAndCompress` for
   free.
2. **Cache compute/patch are free functions** in `RTA.h`/`RTA.cpp`, NOT class
   methods. Each takes the cache by reference and mutates it in place.
3. **One cache per interval optimization, never crosses intervals.** The cache
   is a member of `OptimizePA_Incre_with_TimeLimits` (the TL-walk owner), reset
   at the start of each interval. **Implement the no-PA-change (TL) path
   first**; the priority-move (PA-change) path comes second.
4. **The cache replaces the existing RTA path outright** — no knob, no fallback.
   `ProbabilisticRTA_TaskSet` (`RTA.cpp:100`) is replaced by
   `ComputeRTA_FullAndCache` on the baseline path and by the patchers on the
   candidate paths. Correctness is gated by the differential unit tests, not by
   keeping a slow path alive.

---

## The cache value type (pure data, no PA-search state)

```cpp
// RTA.h, alongside ProbabilisticRTA_TaskSet.
struct PerCoreRTACache {
    int processor_id = -1;
    // sorted_task_ids[i] = task id at priority index i on this core
    // (sorted by pa_vec priority, descending HP-first). ET dists are read
    // from the live DAG, so a stale DAG_ET invalidates the whole cache.
    std::vector<int> sorted_task_ids;
    // rta[i] = RTA distribution of sorted_task_ids[i]. Length == sorted_task_ids.size().
    std::vector<FiniteDist> rta;
    // hp_tasks_et_conv_vec[i] = HP-ET convolution of sorted_task_ids[0..i).
    //   hp_tasks_et_conv_vec[0] == FiniteDist({Value_Proba(0, 1.0)}).
    //   hp_tasks_et_conv_vec[i] is exactly what the 3-arg GetRTA_OneTask consumes.
    std::vector<FiniteDist> hp_tasks_et_conv_vec;
};
```

Memoized **output**, not search state — that's the property that keeps it off
the P0.5 rejected-challenger drift hazard (`goal.md` §"Cache lifetime"). It's a
pure function of `(per-core taskset, per-core priority order, per-core ET
dists)`.

## The free functions (RTA.h / RTA.cpp)

```cpp
// Full compute, populates `cache` (one PerCoreRTACache per processorId) AND
// returns the flat rta vector (same shape ProbabilisticRTA_TaskSet returned).
// Replaces the bare ProbabilisticRTA_TaskSet call on the baseline/descent path.
std::vector<FiniteDist> ComputeRTA_FullAndCache(
    const DAG_Model& dag,
    const PriorityVec& pa_vec,
    const TimeLimitVec& tl_vec,
    std::unordered_map<int, PerCoreRTACache>& cache);

// TL patch (Loop B, no PA change): one task's TL changed at priority position
// `priority_position` on `core`. Reuses hp_tasks_et_conv_vec[priority_position],
// recomputes rta[priority_position..n) via the 3-arg GetRTA_OneTask, returns
// other cores' cached rta untouched. Mutates `cache[core]` in place.
std::vector<FiniteDist> PatchRTA_OneTaskTL(
    int changed_task_id, int core, int priority_position,
    const DAG_Model& dag,
    std::unordered_map<int, PerCoreRTACache>& cache);

// Priority-move patch (Loop A, PA change): one task moved old_pos→new_pos on
// `core`. Reuses hp_tasks_et_conv_vec[min(old_pos,new_pos)], replays the suffix
// in the new sorted order. The O(N²)→O(k) payoff. Mutates `cache[core]` in
// place.
std::vector<FiniteDist> PatchRTA_PriorityMove(
    int moved_task_id, int core, int old_pos, int new_pos,
    const DAG_Model& dag,
    std::unordered_map<int, PerCoreRTACache>& cache);
```

## Cache storage + validity

```cpp
class OptimizePA_Incre_with_TimeLimits {
    // ...existing members...
    // One per interval. Reset (cleared) at the start of each interval's
    // optimization — never carries across intervals.
    std::unordered_map<int, PerCoreRTACache> per_core_rta_cache_;
};
```

Validity is an explicit predicate `CacheIsValidFor(core, dag, pa_vec, tl_vec)`,
not a hash:
- `cache[core].sorted_task_ids` matches the per-core tasks sorted by the
  current `pa_vec`; **and**
- every cached core-task's ET dist in `dag` equals what it was at cache time (a
  TL change to a *different* core does NOT invalidate this core; a TL change to
  *this* core invalidates only the suffix — handled by the patcher, not a full
  flush); **and**
- the cache was built this interval (interval reset ⇒ full flush).

---

## Phase 0 — measure before claiming

- [ ] **Micro-benchmark**: `ProbabilisticRTA_TaskSet_SingleCore` cost vs
      tasks-per-core M at M=3,5,8 (N=6/10/16 with 2 balanced cores), and the
      split of convolve-cost vs `ResolvePreemptionsAndCompress` while-loop
      cost. Built DEBUG (`cmake -DCMAKE_BUILD_TYPE=DEBUG ..` +
      `cmake --build . --target check.SP_OPT -j5`; lib is
      `libSP_OPTDebug.so`). Confirms convolve-count is the dominant term
      before investing in the cache. Creates `tests/benchConvolve.cpp` (the
      ghost entry in git status — file does not yet exist on disk).

## Phase 1 — the cache infrastructure (two sub-tasks)

- [x] **HP-prefix checkpoint store** (the patching primitive) — **LANDED
      2026-07-13**. In `ProbabilisticRTA_TaskSet_SingleCore` (`RTA.cpp`),
      instead of letting `hp_tasks_et_conv` be a single rolling value that
      gets discarded, snapshot it into `hp_tasks_et_conv_vec[i]` = HP-prefix
      convolution of tasks `[0, i)` at the top of each iteration `i`. The
      3-arg `GetRTA_OneTask` already consumes this value. TDD-verified:
      `HpTasksEtConvVec_MatchesRollingValue` (independently replays the rolling
      value, bit-identical) + `TwoArgOverload_SameRtasAsOneArg` (2-arg
      returns bit-identical `rtas` to 1-arg); the 6 pre-existing pinned-`rtas`
      tests are the behavior-preservation oracle (1-arg now delegates to
      2-arg). `check.SP_OPT` 16/16 green. Pure refactor — no behavior change
      to any returned `rtas`. Implementation: 2-arg overload
      `(tasks, hp_tasks_et_conv_vec&)` holds the logic; 1-arg is a thin
      wrapper. See `dev_log.md` §2026-07-13 (step 2).
- [ ] **Per-core RTA cache** (the storage + the full-compute-and-cache entry
      point). Add the `PerCoreRTACache` struct + the `ComputeRTA_FullAndCache`
      free function. `ComputeRTA_FullAndCache` does what
      `ProbabilisticRTA_TaskSet` (`RTA.cpp:100-119`) does today — partition
      by `processorId` via `ExtractTaskSetPerProcessor` (`:87`), run
      `SingleCore` per core — but **retains** each core's `rta` vector,
      `sorted_task_ids`, and `hp_tasks_et_conv_vec` into `cache[core]`. Wire it into
      the baseline/descent eval path, replacing the bare
      `ProbabilisticRTA_TaskSet` call. TDD: `ComputeRTA_FullAndCache` returns
      bit-identical `FiniteDist`s to the old `ProbabilisticRTA_TaskSet` on
      the same input (differential test; the old function is the oracle —
      kept only long enough to verify, then removed per decision 4).

## Phase 2 — dispatch the cache in the two hot loops

- [ ] **TL patch dispatch** (Loop B, the no-PA-change path — **implement
      first**). Wire `PatchRTA_OneTaskTL` into the TL walk
      (`OptimizeSingleTaskTimeLimit`, `OptimizeSP_TL_Incre.cpp:194-239`,
      driven by `PerformCoordinateDescentForTaskConfigOpt` `:241-281`). On a
      TL change to the task at priority position `p` on core A: reuse
      `cache[A].hp_tasks_et_conv_vec[p]`, recompute `rta[p..n)` via the 3-arg
      `GetRTA_OneTask`, return cached `rta` for all other cores. Low risk —
      no reordering, HP-prefix structure unchanged. TDD: differential vs
      `ComputeRTA_FullAndCache` (full recompute) across a sweep of TL moves
      (every task, every TL option, both directions); bit-identical.
- [ ] **Priority-move patch dispatch** (Loop A, the PA-change path — the
      O(N²) payoff). Wire `PatchRTA_PriorityMove` into the 1D priority loop
      (`OptimizeIncre`, `OptimizeSP_Incre.cpp:269-287`; variations from
      `FindPriorityVec1D_Variations` `:180-212`). On a priority move of one
      task from `old_pos` to `new_pos` on core A: reuse
      `cache[A].hp_tasks_et_conv_vec[min(old_pos, new_pos)]`, replay the suffix
      `[min(old_pos,new_pos), n)` in the new sorted order. Higher risk —
      reordering changes which tasks are HP for which; the TDD oracle (full
      recompute) is the safety net. TDD: differential vs
      `ComputeRTA_FullAndCache` across a sweep of priority moves (every
      task, every target position in the search half); bit-identical.
- [ ] **End-to-end scalability measurement**. Re-run the Phase-0 micro-bench
      with patching wired in, at N=6/10/16. Confirm the ~3-4× candidate-cost
      reduction predicted in `dev_log.md` §2026-07-13 on the dominant O(N²)
      term. Record actual numbers (revise the estimate if
      `ResolvePreemptionsAndCompress` dominates instead).

## Build order (descriptive, no opaque labels)

1. Phase 0 micro-bench — measure first, settles convolve vs
   `ResolvePreemptionsAndCompress` as the dominant term.
2. HP-prefix checkpoint store — smallest, pure refactor of `RTA.cpp:58-85`;
   the primitive both patchers reuse.
3. Per-core RTA cache — `PerCoreRTACache` + `ComputeRTA_FullAndCache`,
   replaces `ProbabilisticRTA_TaskSet` on the baseline path.
4. TL patch dispatch (Loop B, no PA change) — the path the user wants first;
   low risk.
5. Priority-move patch dispatch (Loop A, PA change) — the O(N²) payoff.
6. End-to-end measurement at N=6/10/16.

Each step is one review-and-commit cycle per `agent_coding_rules.md`.

## Done when
- [ ] All of: HP-prefix checkpoint store, per-core RTA cache, TL patch
      dispatch, priority-move patch dispatch — landed with
      `cmake --build . --target check.SP_OPT -j5` green + `ctest` 16/16, each
      patch TDD-verified bit-identical to the full-recompute oracle; **and**
      the end-to-end measurement records the actual N=6/10/16 speedup (or
      documents why it fell short). The old `ProbabilisticRTA_TaskSet` /
      `ProbabilisticRTA_TaskSet_SingleCore` direct-call path is removed
      (replaced outright per decision 4), not left as a dead fallback.
