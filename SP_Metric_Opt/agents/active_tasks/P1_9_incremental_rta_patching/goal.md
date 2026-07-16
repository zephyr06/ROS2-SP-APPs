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
* **Validity design (refined 2026-07-14)**: the cache stores two things per
  core — `sorted_task_ids` (the per-core priority ORDER, which is the
  SUFFICIENT proxy for pa_vec; RTA cares about order, not priority values) and
  `tl_vec` (the per-core time limits). `tl_vec` is a SUFFICIENT proxy for
  ET-dist validity because within an interval ET dists mutate ONLY via TL
  (`ApplyTimeLimitsToTasksExecutionTime` → `GetUnitExecutionTimeDist(tl)`, a
  deterministic point mass) — so `stored_tl[i] == current_tl[i]` ⟺ ET-dist
  unchanged (tl==-1 = the immutable base Gaussian, which never moves mid-walk).
  The one exception, the one-time WCET ablation
  (`ApplyWCETAblationIfRequired`, `OptimizeSP_TL_Incre.cpp:487`), is a setup
  boundary covered by the interval reset. So `pa_vec` is NOT stored
  (redundant with `sorted_task_ids`); `tl_vec` IS stored. Full design in
  `tasks.md`.

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

**Cache implementation design is locked (2026-07-13, refined 2026-07-14).**
Four decisions: (1) `hp_tasks_et_conv_vec[i]` = HP-tasks'-ET convolution
`[0, i)` on a core, consumed verbatim by the 3-arg `GetRTA_OneTask`;
(2) cache compute/patch are **free functions** taking the cache by reference
and mutating in place (not class methods) — now in a dedicated
`RTA_Cache.h`/`RTA_Cache.cpp` (refined 2026-07-14: living in `RTA.h`/`.cpp`
caused a header cycle that forced the as-built signature divergence; a
dedicated leaf header breaks it and lets the locked `(dag, pa_vec, tl_vec,
cache)` signature stand); (3) **one cache per interval**, never crosses
intervals, held by `OptimizePA_Incre_with_TimeLimits`
(`EvaluateSPWithPriorityVec` is a per-candidate free fn, can't hold the
member), no-PA-change (TL) path implemented first; (4) **replaces the
existing RTA path outright** — no knob, no fallback, correctness gated by
differential unit tests. **Validity refinement (2026-07-14):** the cache
stores `sorted_task_ids` (sufficient pa_vec proxy — per-core order, not
values) + `tl_vec` (sufficient ET-dist proxy within an interval, since ET
dists mutate only via TL). Full design, the `PerCoreRTACache` value type,
the three free functions, and the named build order live in `tasks.md`.

**API revision (2026-07-15, pre-commit of 3c):** decision (2) is refined —
`PerCoreRTACache` is now a **class** (private data + read accessors + query
helpers + a `Populate` build method), not a bare struct, to centralize the
sorted_task_ids↔rta↔tl_vec↔hp_tasks_et_conv_vec alignment invariant. The
cache-level entry points (`ComputeRTA_FullAndCache`, `ClassifyReuse`, the
patchers) remain **free functions** taking the cache by reference. The role-2
reuse query is redesigned: the per-core `CacheReuseInfo`/`AnalyzeCacheReuse`/
`CacheConsistentWith` surface is replaced by a **per-task `enum class
RTAReuseClass` vector** from `ClassifyReuse(cache, dag, changed_task_ids)`
(v0 = same-processor check; `RecomputeWithHpPrefix` declared but not produced
until the prefix refinement paired with the patchers). `CacheConsistentWith` is
dropped (derivable: all tasks `RtaReuse`). Full record in `tasks.md` §"API
revision" + `dev_log.md` §2026-07-15 (API revision). The step-3c scope, the
baseline-only 3b decision, Hazard A/B, and the DECIDED required-
`PerCoreRTACache&` seam on `OptimizeIncre` are UNCHANGED.

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
- **Step 3b is BASELINE-ONLY** (2026-07-14 scope decision): the cache is wired
  into the incremental optimizer's eval path and `ProbabilisticRTA_TaskSet` is
  retired from THAT path only. The shared free-fn path
  (`EvaluateSPWithPriorityVec` → `ObtainSP_DAG` → `ObtainSP_TaskSet` →
  `ProbabilisticRTA_TaskSet`) stays for `OptimizePA_BF` / `OptimizeSP_TL_BF`
  until step 7's end-to-end review. Decision 4's "no fallback, replaced
  outright" applies to the incremental path at 3b; full retirement everywhere
  is deferred. The cache is BUILT per candidate at 3b (full compute + populate);
  reuse (patching) is steps 4/5. Full call-chain map + wiring shape + the two
  integration hazards (A: cache owner on derived class vs PA loop on base class
  with a sliced inner optimizer → seam threads the cache by parameter;
  B: `ObtainSP_DAG_From_Dists` omits `perf_coefficient` from the node term → the
  assembly inlines it, NOT a bare `ObtainSP_DAG_From_Dists` call) in `tasks.md`
  §"Step 3b design". **NEXT (gated on user commit of 3c + OK to code): implement
  3b** — add `per_core_rta_cache_` member (on the DERIVED owner, NOT base — Hazard A)
  + the cache-aware eval seam (**DECIDED 2026-07-15 user: a REQUIRED `PerCoreRTACache&`
  param on `OptimizeIncre` — always active, no separate fn, no optional pointer; the
  required ref satisfies `agent_coding_rules.md` L3/L10**) + the perf_coefficient-corrected
  assembly + the differential test (fixture must include a perf-pair task + a
  chain, else Hazard B hides). **Scope note (2026-07-15): the seam's primary
  target is `OptimizeIncre` `:239`/`:279` (the per-candidate O(N²) sites);
  `OptimizeFromScratch`'s beam search (`UpdateSP`→`GetRTA_OneTask`) bypasses
  `ProbabilisticRTA_TaskSet` and is NOT cache-replaceable — only its single `:136`
  final eval is, which is secondary (signature unchanged at 3b).**

---

## Extra Ideas & Design Refinements (Proposed 2026-07-15)

The following refinements are documented as potential design enhancements to be integrated alongside or after step 3b:

1. **Unified Caching API (`EvaluateRTA_WithCache`)**: Merges full compute, TL patching, priority patching, and exact-match reuse into a single robust entry point. The cache automatically determines status per core, simplifying the optimizer integration and eliminating manual index tracking.
2. **Zero-Copy Order Derivation**: Optimizes `AnalyzeCacheReuse`'s internal candidate sorting by working with task ID primitives rather than allocating and copying full `Task` structures and `FiniteDist` arrays.
3. **Allocation-Free Flat RTA Rebuilding**: Flattens cache distributions to flat priority-aligned RTA output in O(N) using task-ID indexed arrays instead of hash-maps.
4. **Speculative Cache Copying**: Ensures speculative evaluations do not affect the champion's cache by using the "scratch copy on spec, swap on accept" pattern (extremely cheap at ~16KB data size).

---

## STATUS: ON HOLD (2026-07-15) — API rev 2 designed, not implemented

The user reshaped the cache API a second time (rev 2 — **self-supplied cache**:
whole-taskset, `rta_` flat by task id, stores `pa_` + a `dag_tasks` copy;
`Initialize`/`UpdateFullCache`/`CheckTaskSetRTAReuse`/`GetRTA_OneTask` members
+ free `EvaluateRTA_WithCache`; `tl` threaded directly at the seam — option
(a), verified in scope at `OptimizeSP_TL_Incre.cpp:142-146`/`:161`), then
**put `RTA_Cache` on hold** to resolve a more-important issue first. **No code
written for rev 2** — design-only. Full record + 5 open questions for the user
in `tasks.md` §"API revision 2" + `dev_log.md` §2026-07-15 (API rev 2). The
staged 3c code on disk still carries the rev-1 surface (uncommitted).

**Resume sequence:** answer Q1–Q5 → write `RTA_Cache.h` to the locked rev-2
surface → implement step 3b (required `PerCoreRTACache&` + `tl` seam on
`OptimizeIncre`, `per_core_rta_cache_` member on the derived owner — now ONE
whole-taskset object, NOT the rev-1 per-core map, so the §"Step 3b design"
wiring shape is STALE w.r.t. rev 2 and must be rewritten) → steps 4/5/7.

