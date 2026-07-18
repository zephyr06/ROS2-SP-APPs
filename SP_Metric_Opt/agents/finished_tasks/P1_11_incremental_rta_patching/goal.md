# P1.11 — Incremental RTA Patching (priority-prefix reuse + per-core cache)

> Implements **Idea 11** of `P1_1_efficiency_optimizations/idea_queue.md`.
> Gated on P1.10 (proven single-change invariant). Target regime: larger N.
>
> **Scope narrowed 2026-07-18:** P1.11 now owns the cache **design + build**
> (Phase 0, DONE, working tree uncommitted). The **integration** of this cache
> into the live incremental optimizer eval path (former Phase 1 + Phase 2) was
> split out to a new task, **P1.12** — see
> [`../P1_12_integrate_rta_cache/`](../P1_12_integrate_rta_cache/). This file
> remains the design doc for the `RTACache` API surface P1.12 consumes.

---

## Why this exists (the lever)

Every SP candidate the optimizer evaluates pays one full N-task RTA. Two hot loops change only one task per step:
- **Loop B (TL walk)**: changes one task's TL.
- **Loop A (1D priority walk)**: moves one task to a new priority position (up to N variations per changed task, the $O(N^2)$ term).

RTA is partitioned by `processorId` and computed per-core independently. When a task on core A changes:
1. Core B's RTA is untouched and can be skipped (per-core skip).
2. On core A, higher-priority tasks' ET convolution (`hp_tasks_et_conv`) is unaffected. We can reuse the prefix convolution checkpoint and only convolve the suffix tasks (prefix reuse).

---

## The Rev-3 Single-Champion Cache Design

The cache exploits the **proven** P1.10 single-change invariant ($|diff| \le 1$ vs the champion):
- **Type-L (TL step) / Priority Move**: $|diff| == 1$.
- **Type-E (env step)**: $|diff| == 0$ (same DAG, only PA varies). Since RTA is PA-independent, we reuse the RTA verbatim.

### Core class `RTACache` (in `RTA_Cache.h`)
- **`Initialize(dag, pa, tl)`**: Cold-start. Computes full N-task RTA, stores the champion triple, and builds prefix checkpoints.
- **`AdoptChampion(dag, pa, tl, rtas)`**: Cheap commit on promotion. Stores candidate `rtas`, triple, and rolls prefix forward (no RTA compute). Called inside `CommitIncumbent`.
- **`Evaluate(dag, pa, tl)`**: Read-only hot-loop entry. Internally dispatches on the diff:
  - `FullReuse` ($|diff| == 0$): return champion RTA.
  - `ReuseHpTasksEt` ($|diff| == 1$): reuse HP-prefix at `min(old_pos, new_pos)` on its core, recompute suffix in candidate order.
  - `NoReuse` (fallback): full recompute.
- **`ComputeTaskSetDifference(dag, pa, tl)`**: Pure diff query returning `TaskSetDifference` (locators + classification).
- **`ClassifyReusePerTask(dag, pa, tl)`**: Returns `vector<RTAReusePerTask>` for fine-grained per-task tracking.
- **`IsSingleTaskChange(dag, pa, tl)`**: Boolean predicate for checking invariant validity.

---

## Out of Scope / Non-goals

- No change to the generator's topology or baseline priority assignment.
- Correctness is paramount: patching must be bit-identical to the full recomputation (verified by differential TDD tests).
