# P1.9 — Incremental RTA Patching (priority-prefix reuse + per-core cache)

> Implements **Idea 11** of `P1_1_efficiency_optimizations/idea_queue.md`
> (the live, priority-prefix half of Idea 2). Elevated from P1.1's deferred
> bucket to an active task on 2026-07-13: the re-evaluated impact (see
> `dev_log.md` §2026-07-13) shows the O(N²) 1D priority loop dominates at
> larger N (N=10/N=16), and per-processor skip + prefix reuse compound to a
> real scalability win there. **Perf, not correctness** — patching must be
> bit-identical to the full recomputation (TDD-verified against the oracle).
>
> Scope target: **scalability improvements at larger N such as N=10 or
> N=16** (the regime where this earns its cost). The current 2-core balanced
> generator topology (`N_CORES=2`, greedy ~N/2 per core) is the baseline.

---

## Why this exists (the lever)

Every SP candidate the optimizer evaluates pays **one full N-task RTA**:
`EvaluateTimeLimitConfig_ScratchOrIncre` → `EvaluateSPWithPriorityVec`
(`OptimizeSP_Base.cpp:148`) → `ObtainSP_DAG` (`SP_Metric.cpp:89`) →
`ObtainSP_TaskSet` (`:53`) → `ProbabilisticRTA_TaskSet` (`RTA.cpp:100`) →
`ProbabilisticRTA_TaskSet_SingleCore` (`RTA.cpp:58`).

Two hot loops change **ONE task per step** but today recompute the full RTA:

- **Loop B — the TL walk** (`OptimizeSingleTaskTimeLimit`,
  `OptimizeSP_TL_Incre.cpp:194-239`, driven by
  `PerformCoordinateDescentForTaskConfigOpt` `:241-281`): each step changes
  one task's TL → only that task's RTA and lower-priority tasks that include
  it as HP change. O(N) candidates/interval.
- **Loop A — the 1D priority loop** (`OptimizeIncre`,
  `OptimizeSP_Incre.cpp:269-287`; variations from
  `FindPriorityVec1D_Variations` `:180-212`): each variation moves one task
  to a new priority position. Tasks at priority above **both** old and new
  position have an identical HP set → identical RTA; only the moved task and
  tasks between old/new positions change. `FindPriorityVec1D_Variations`
  generates up to **N candidates per changed task** → this is the **O(N²)
  term** and the dominant cost at larger N.

The existing `ProbabilisticRTA_TaskSet_SingleCore` (`RTA.cpp:58-85`)
**already maintains** a running HP-prefix convolution `hp_tasks_et_conv`
(`:73`) as it iterates the sorted taskset — but it **discards the
intermediate prefix values**. The 3-argument `GetRTA_OneTask` overload
(`RTA.cpp:44-56`) already accepts a precomputed `hp_tasks_et_conv` and folds
preemption resolution (`ResolvePreemptionsAndCompress`, `:7-28`) in for
free. So the patching primitive exists; what's missing is (1) a place to
**store** the prefix checkpoints and per-core RTA vectors between
candidates, and (2) the dispatch to reuse them.

`ProbabilisticRTA_TaskSet` (`RTA.cpp:100-119`) **already partitions** tasks
by `processorId` via `ExtractTaskSetPerProcessor` (`:87`) and runs
`ProbabilisticRTA_TaskSet_SingleCore` per core independently. So
cross-core isolation is structurally free — a change on core A leaves core
B's entire RTA vector unchanged.

---

## Two sub-tasks (Phase 1), then Phase 2

### Sub-task 1a — Per-core RTA cache

* **What**: Cache the per-core RTA vectors (`std::vector<FiniteDist>`) and
  the per-core sorted taskset from one candidate eval, keyed on the
  `(dag, priority-vec, time-limit-vec)` identity that produced them.
* **Why**: `ProbabilisticRTA_TaskSet` (`RTA.cpp:100`) already computes
  per-core. When a TL/priority change lands on core A, **core B's RTA is
  byte-identical** to the cached value — return it untouched, skip the
  entire `ProbabilisticRTA_TaskSet_SingleCore` call for that core. With
  `N_CORES=2` and greedy balanced assignment (`taskset_generator.py:546-554`,
  ~N/2 per core), that's ~N/2 tasks' RTA skipped per candidate.
* **Correctness guard**: the cache is valid iff the priority-vec, the
  TL-vec, AND the ET dists for that core's tasks are all unchanged. Any
  multi-task diff, any ET change on a cached core, or a new interval
  (DAG_ET change) must invalidate. **Bit-identical** to a full recompute by
  construction (it *is* the cached recompute result).

### Sub-task 1b — HP-prefix checkpoint store

* **What**: In `ProbabilisticRTA_TaskSet_SingleCore`, instead of letting
  `hp_tasks_et_conv` (`RTA.cpp:73`) be a single rolling value, snapshot the
  prefix convolution **at each priority index** `i` into a vector
  `hp_tasks_et_conv_vec[i]` = HP-prefix convolution of tasks `[0, i)`. The
  3-arg `GetRTA_OneTask` (`RTA.cpp:44`) already consumes exactly such a value.
* **Why**: This is the primitive both loops patch against. Loop A reuses
  `hp_tasks_et_conv_vec[min(old, new)]` and recomputes only the suffix
  `[min(old,new), n)` in the new order. Loop B reuses
  `hp_tasks_et_conv_vec[p]` for the changed task's position `p` and
  recomputes `[p, n)`. Turns a candidate from O(N) convolves into O(k)
  where k = affected suffix length.
* **Correctness guard**: the checkpoints are a function of the sorted
  taskset's ET dists only; invalidate together with the per-core cache.
  Bit-identical to the rolling value (it's the same convolution, just
  retained).

### Phase 2 — Dispatch the cache in the two hot loops

* **Loop B patch (`PatchOneTaskTL`)** — low risk: a TL change alters one
  task's ET dist in place; HP-prefix structure is unchanged, so reuse
  `hp_tasks_et_conv_vec[p]` and recompute `[p, n)`. No reordering. Pairs with
  1a's per-core skip (untouched cores return cached).
* **Loop A patch (`PatchPriorityMove`)** — the O(N²) payoff: a priority
  move reorders the sorted-per-core taskset. Reuse
  `hp_tasks_et_conv_vec[min(old, new)]`, then replay the suffix in the new
  order. Higher risk (reordering changes which tasks are HP for which); the
  TDD oracle (full recompute) is the safety net.

A **cross-cutting micro-benchmark** (M=3/5/8 SingleCore RTA, built DEBUG)
measures the actual per-candidate cost *before* claiming the win, settling
whether convolve-count or `ResolvePreemptionsAndCompress`'s while-loop is
the dominant term. The task docs gate P1.1 perf work on profiling; this
bench is that profiling.

---

## Cache lifetime & invalidation (the design constraint)

The cache must live in `OptimizePA_Incre_with_TimeLimits` (the TL-walk
owner), **not** in the throwaway challenger built by
`BuildChallengerFromIncumbent` (`OptimizeSP_TL_Incre.cpp:394`) — the P0.5
redesign rebuilds the challenger from `res_opt_` each
`EvaluateTimeLimitConfig_ScratchOrIncre` call, so any state held in the
challenger is discarded. The cache is **pure distribution memoization** (no
PA-search state), so it does NOT carry the rejected-challenger drift hazard
documented in P1.1 `goal.md` §3 — *provided* it is invalidated on:
1. PA change the patch can't handle (e.g. a move that changes the HP set of
   a cached prefix);
2. multi-task diff (`FindTaskWithDifferentEt` flags >1 task);
3. new interval (DAG_ET change).

**Cache implementation design is locked (2026-07-13).** Four decisions:
(1) `hp_tasks_et_conv_vec[i]` = HP-tasks'-ET convolution `[0, i)` on a core,
consumed verbatim by the 3-arg `GetRTA_OneTask`; (2) cache compute/patch
are **free functions** in `RTA.h`/`RTA.cpp` taking the cache by reference
and mutating in place (not class methods); (3) **one cache per interval**,
never crosses intervals, held by `OptimizePA_Incre_with_TimeLimits`,
no-PA-change (TL) path implemented first; (4) **replaces the existing RTA
path outright** — no knob, no fallback, correctness gated by differential
unit tests. Full design, the `PerCoreRTACache` value type, the three free
functions, and the named build order live in `tasks.md`.

---

## Out of scope / non-goals

- **NOT** a correctness change. Patching must reproduce the full-recompute
  result to the bit; the TDD differential vs the oracle is the acceptance
  gate.
- Does **not** change the generator's `N_CORES` topology or the priority-
  assignment algorithm — only the cost of evaluating a candidate.
- Does **not** touch the persistent-challenger redesign (P1.1 §3 / P0.5 5h)
  — that remains deferred.
- Does **not** implement Idea 13 (multi-fidelity coarse search) or Idea 15
  (patience-bounded 1D walk) — separate ideas in `idea_queue.md`.
