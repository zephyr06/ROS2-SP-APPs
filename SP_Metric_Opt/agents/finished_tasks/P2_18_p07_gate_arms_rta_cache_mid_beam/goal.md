# P2.18 — P0.7 Important-Task Gate Arms the RTA Cache Mid From-Scratch Beam

> Filed 2026-08-01, surfaced by the `compare_against_bf` N=4 re-run AFTER P2.17's
> gate-bypass fix landed. This is a **distinct, new bug** — NOT P2.17 (P2.17's fix
> works: seed=1040, INCR_Reopt_10 finished OK on the crashed taskset) and NOT
> P0.6's `ComputeSafeFallback` loud-fail (that path never threw — the witness
> INCR_Reopt_10 ran it and finished). It is a **P0.7-gate regression** in how the
> gate sources its RTA.

## The Goal

The P0.7 during-walk gate (trigger b-i) sources its challenger RTA from
`rta_cache_.Evaluate(...)`, called inside `UpdateRecords`
(`OptimizeSP_TL_Incre.cpp:223`) whenever `enable_fallback_use_` is ON (the prod
default). That `Evaluate` call **arms the cache**: on the first call with no
champion it runs `RTACache::Initialize`, which **adopts a champion** (bakes the
4 champion forms + `champion_.rta`).

In the **Reopt** branch of `SeedBaselineAndArmCache` (`OptimizeSP_TL_Incre.cpp:520-533`)
the intended invariant is:

> `ResetIncumbentBaseline(true)` clears the cache + disarms
> (`rta_cache_active_=false`); the from-scratch beam at line 521 runs DISARMED so
> no champion is adopted; the cache is re-armed at line 529, and line 532's
> `Evaluate` is a safe `Initialize`.

The P0.7 gate **broke that invariant**: the beam's `UpdateRecords` calls
`rta_cache_.Evaluate` (gated by `enable_fallback_use_`, NOT by
`rta_cache_active_`) → `Initialize` adopts a champion mid-beam. A *later* beam
step commits a `{pa, tl}` that differs from that champion by **more than one
task** (a from-scratch beam explores multi-task changes, not the 1-task walk the
cache serves) → `RTACache::ComputeTaskSetDifference` throws `std::runtime_error`
(`RTA_Cache.cpp:358`, the `|diff|>1` single-change invariant) → uncaught →
`std::terminate` → SIGABRT.

## Verified evidence

Reproduced deterministically (debug + release builds) on the crashed arm
`compare_against_bf` N=4 taskset_2 / `INCR_WCET`. Source-level backtrace
(debug build, `catch throw`):

```
RTACache::ComputeTaskSetDifference  throws std::runtime_error   RTA_Cache.cpp:358
  <- ClassifyReusePerTask                                       RTA_Cache.cpp:376
  <- Evaluate                                                   RTA_Cache.cpp:465
  <- SeedBaselineAndArmCache  (mode = Reopt)                    OptimizeSP_TL_Incre.cpp:532
  <- RunIntervalDescent                                         OptimizeSP_TL_Incre.cpp:558
  <- ReOptimizePeriodic                                         OptimizeSP_TL_Incre.cpp:1209
  <- Optimize_w_TL_ScratchOrIncre  (interval 0)                 OptimizeSP_TL_Incre.cpp:749
```

- `run.log` is 0 bytes — the abort precedes any log flush.
- `INCR_Reopt_10` finished OK on the same taskset_2 → `ComputeSafeFallback`
  (P0.6) did NOT throw → this is NOT the P2.17 / P0.6 loud-fail signature.
- `INCR_NO_FALLBACK` was still RUNNING when the harness aborted (no witness
  either way, but it shares the same `UpdateRecords` gate path when the flag is
  ON; `INCR_NO_FALLBACK` flips the flag OFF so its gate is inert — consistent
  with it not crashing).
- `git blame` on `UpdateRecords:214-223` confirms the `rta_cache_.Evaluate`
  gate call is the P0.7/P0.6-gate code (commits `500665d5`, `c87ae0d4`,
  `a2be3876`).

## Why INCR_WCET trips it and INCR_Reopt_10 did not (on this taskset)

INCR_WCET sets `GlobalVariables::use_wcet_execution_time=true`, changing the ET
distribution the gate RTA sees. With WCET ETs the from-scratch beam's first
committed `{pa, tl}` and a later step's `{pa, tl}` differ by >1 task → throw.
INCR_Reopt_10 (mean ET) happened to commit a champion whose next beam step was
still ≤1-different on this taskset — luck of the draws, not a real safety
difference. **It is a latent bug exposed by the ET distribution**, not by the
WCET feature. The same path crashes `measure_p07_penalty` N=6 and any INCR-family
arm with the gate ON once a taskset produces a >1-diff beam commit sequence.

## Why it surfaced now

Before P0.7, `UpdateRecords` never called `Evaluate` in the from-scratch path
(the gate did not exist), so the cache stayed empty through the beam and
`SeedBaselineAndArmCache:532`'s `Evaluate` was a safe `Initialize`. P0.7's gate
added the `Evaluate` call at `UpdateRecords:223` as the gate's RTA source,
inadvertently arming the cache mid-beam and breaking the "disarmed beam"
invariant that `SeedBaselineAndArmCache`'s comments rely on.

## Open decisions (settle with user before any code)

- **D1 — Fix shape:** how should the gate source its RTA without arming the
  cache mid-beam? Two candidate directions (to be presented concretely in the
  plan before the user picks):
  - (a) **Guard the existing call:** in `UpdateRecords`, only call
    `rta_cache_.Evaluate` for the gate when it is safe
    (`HasChampion()` + `IsSingleTaskChange()`); otherwise compute the gate RTA
    standalone (the same self-contained path `AdoptFallbackIfUnschedulable`
    already uses — fresh RTAs, no cache). Minimal, local to one call site.
  - (b) **Decouple the gate from the cache entirely:** give the gate its own RTA
    computation that never touches `rta_cache_`. Cleaner separation; slightly
    larger change.
- **D2 — Should the from-scratch beam's `UpdateRecords` gate even run?** The
  gate's purpose is to reject *unsafe SP-better challengers during the walk*.
  The from-scratch beam (Reopt) is a fresh search, not a 1-task walk. Whether
  the gate belongs there at all — vs only in the incremental walk — is a design
  question. (P0.7's intent: the gate guards ANY accepted incumbent, so it likely
  should stay; but the RTA sourcing must be cache-free there.)

## Non-goals

- Changing P0.7's triggers or the `enable_fallback_use_` flag default (sound; ON).
- Changing P2.17's gate-bypass fix (working; seed=1040 confirms it).
- Changing P0.6's `ComputeSafeFallback` or worst-case-DAG certificate.
- Changing P0.8's gate logic.
- The WCET ET distribution (it merely exposed the bug; not the cause).
