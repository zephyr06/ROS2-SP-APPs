# P1.13 — RTA Cache for Priority Optimization (standalone, base-class)

> See `api_design.md` for the API under discussion (THIS TASK = design first;
> implementation only after the user signs off on the API).
> Split out as a STANDALONE task 2026-07-18 at the user's direction:
> "actually, this can be a standalone implementation / task. let's focus on
> this, don't worry about optimizeSP_TL_Incre.h, just try to add rta cache
> into the priority optimization."

## Scope (one sentence)

Wire the P1.11 `RTACache` into the **base** priority optimizer
`OptimizePA_Incre` (`sources/Optimization/OptimizeSP_Incre.{h,cpp}`) so that
`OptimizeIncre` reuses RTA across its 1D priority-assignment variations
instead of calling the full `ProbabilisticRTA_TaskSet` per candidate. **NOT**
touching `OptimizeSP_TL_Incre.h` / the TL path / the serialized walk — that
remains P1.12's scope. P1.12's 2a write-side (already landed in the working
tree) is independent and unaffected.

## Why standalone / why now

P1.12's staging confined cache work to the **derived** class
(`OptimizePA_Incre_with_TimeLimits`) and deferred Hazard A — threading a
`RTACache&` into the **base** `OptimizePA_Incre` so `OptimizeIncre_SingleTask`
can reach a cache — to "3/N". But the base-class integration is a
self-contained win that does NOT depend on the TL/serialized machinery:

- `OptimizeIncre` (`OptimizeSP_Incre.cpp:298`) loops over `FindTaskWithDifferentEt`
  diff tasks and calls `OptimizeIncre_SingleTask` per task.
- `OptimizeIncre_SingleTask` (`:275`) generates `FindPriorityVec1D_Variations`
  for ONE task and scores each via `EvaluateSPWithPriorityVec`
  (`:287`) → `ObtainSP_DAG` → `ObtainSP_TaskSet` → `ProbabilisticRTA_TaskSet`
  (a FULL N-task RTA per candidate).
- Each variation moves exactly ONE task's priority position (the 1D
  generator). Relative to the carried PA, that is a **single priority
  move** = `|diff|<=1` against a champion whose PA == the carried PA. That
  is precisely the case `RTACache::Evaluate` serves (patch the suffix from
  the moved task, reuse cross-core RTA verbatim).

So the base-class `OptimizeIncre` loop is a natural, invariant-holding
client of the cache — independent of P1.10's serialized walk and P1.12's
TL-side wiring. Filing it as its own task lets it land (and be A/B-measured)
on its own.

## What `OptimizeFromScratch` does NOT get the cache (user's call)

`OptimizeFromScratch` (`OptimizeSP_Incre.cpp:74`) builds partial priority paths
bottom-up via `PriorityPartialPath::AssignAndUpdateSP` — each partial path is
a DIFFERENT prefix of the priority assignment, growing by one task per level.
Successive `AssignAndUpdateSP` calls differ by appending one task AND
re-shuffling the unassigned remainder's prospective SP — the candidate taskset
differs from any single "champion" by more than one task's priority position
(it's a constructive search, not a 1-move perturbation of a fixed PA). The
single-change invariant does NOT hold → the cache cannot serve it without
falling back to full RTA every call (= no win, just overhead). User decision:
**`OptimizeFromScratch` stays cache-free.** (See `api_design.md` Q1.)

## Correctness gate

Bit-identical SP output to the `EvaluateSPWithPriorityVec` oracle
(differential TDD), exactly like P1.11 Phase 0 / P1.12 2b. The cache's
`Evaluate` returns the SAME flat RTA vector `ProbabilisticRTA_TaskSet` would,
so once the SP-assembly helper multiplies `perf_coefficient` (Hazard B, see
`api_design.md` Q4), the assembled SP is bit-identical. No gate verdict moves,
no A/B regression — only the per-candidate RTA cost drops.

## Out of scope (explicitly)

- `OptimizeSP_TL_Incre.{h,cpp}` — P1.12's domain. The P1.12 2a write-side
  already in the working tree is untouched.
- `OptimizeFromScratch` cache integration (see above).
- The TL walk (`OptimizeSingleTaskTimeLimit`) dispatch — P1.12 Phase 2.
- Any change to `RTA_Cache.h` / `RTA_Cache.cpp` — the cache API is FROZEN
  (P1.11 Phase 0 DONE). This task CONSUMES the frozen API; it does not extend
  it.

## Status

**FILED 2026-07-18 (design only, NO code yet).** `api_design.md` drafted;
awaits user review of the API before any source edit.
