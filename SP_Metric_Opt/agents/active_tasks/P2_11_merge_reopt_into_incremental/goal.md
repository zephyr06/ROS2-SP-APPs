# P2.11 — Merge Re-Optimization Into the Incremental Path

## The Goal

Replace the current re-optimization path with a design where **re-opt is the
standard incremental path plus two small, easily-interpretable deltas** — instead
of a separate, parameterized descent with a 7-branch mode flag. The motivation
(verbatim from the user): the current reopt path "is too complicated with many
changes that cannot be easily interpreted." This task merges the two paths as
maximally as possible.

### The two deltas (the user's design, verbatim intent)

Re-optimization follows the **same path as incremental optimization** except:

1. **Interval 0 — RM-Fast init.** At interval 0 there is no carried incumbent,
   so the first solution is initialized as **RM-Fast** (Rate-Monotonic priority
   assignment + smallest time limits), NOT a from-scratch beam search.
2. **Interval start — one re-opt step seeds the champion, then incremental walk
   with patience+1.** At the start of each new interval, perform ONE
   re-optimization on the new DAG **adopting the time limits from the last
   interval**; the result of this single re-optimization becomes the champion.
   After this extra step, do the **same as the standard incremental
   optimization**, except **patience is increased by 1** compared to the
   standard incremental patience.

The end state: re-opt and incremental share ONE descent body and ONE walk arm;
the only deltas are (i) patience 0 vs (0+1), and (ii) the one upfront re-opt step
(re-opt) vs the dedicated re-score (incremental).

## Why it matters now

Two threads converge here:

- **P2.10** (prerequisite, in working tree) already merged the duplicated Type-L
  walk body into one shared `OptimizeOneTaskTimeLimit` and renamed the
  confusable functions. The structural unification is half-done; this task
  finishes it at the descent-body level.
- **P2.9** added the `ReoptimizationUseSubIncrementalWalk` flag (default OFF) that
  routes the reopt TL walk through the sub-incremental arm. The user's evaluation
  (recorded in `dev_log.md`) found that **~90% of this design is already the
  P2.9 flag-on path**: `ResetIncumbentBaseline(true)` adopts last interval's TLs,
  `EvaluateTimeLimitConfig_PAReopt(from_scratch=true)` is the one upfront
  re-opt step, and `OptimizeOneTaskTimeLimit` with patience 1 is the
  incremental walk with patience+1. The ONE genuine delta beyond P2.9 flag-on is
  whether re-opt also adopts the **Type-E** (env-changed-task) entries from the
  serialized queue, which the current reopt path skips.

So this task = "turn P2.9's flag on by default + delete the legacy full-beam arm +
(optionally) add Type-E to the reopt queue," packaged as the single canonical
reopt path.

## The design (the merge)

Two readings of "do the same as the standard incremental optimization":

- **(a) Full merge (maximal).** Re-opt adopts the Type-E + Type-L serialized
  queue (`BuildSerializedTaskQueue`), like incremental, AND uses the
  sub-incremental walk arm with patience 1. This adds targeted 1D patches on
  env-changed tasks to the reopt path — strictly more targeted work than current
  reopt. Requires plumbing `dag_prev_pre_tl` into the reopt entry so
  `BuildSerializedTaskQueue` can detect Type-E (diff champion DAG vs new DAG).
- **(b) Loose merge.** Re-opt keeps Type-L only, uses the sub-incremental arm +
  patience 1. This is exactly the P2.9 flag-on path — zero new code beyond
  flipping the flag default ON and deleting the legacy arm.

**Chosen: (a)** — it is the maximal merge the user asked for and the more
interesting experiment. (b) is the fallback if the Type-E plumbing proves to
break the `|diff|<=1` cache contract.

### Unified descent body

One `RunIntervalDescent(K, tl, mode, dag_prev_pre_tl)` replaces
`RunIncrementalTLDescent` + `RunReoptTLDescent`. `mode ∈ {Incremental, Reopt}`
selects:

| step | Incremental | Reopt |
|---|---|---|
| baseline seed | dedicated re-score of carried {pa,tl} | `GlobalBeam(from_scratch=true)` (the one upfront re-opt step) |
| queue | E+L serialized | E+L serialized (reading (a)) |
| walk arm | `SubIncremental` | `SubIncremental` |
| patience | `IncrementalTimeLimitSearchPatience` (0) | +1 (1) |
| cache active | yes | yes (same as incremental) |

The reopt legacy arm `WalkOneTaskTimeLimit_FullBeam` and the P2.9 flag are
**deleted** — reopt always uses the sub-incremental walk. `EvaluateTimeLimitConfig_PAReopt`
stays, but ONLY for the one upfront baseline re-opt step in reopt mode.

## Guardrails (what this task is NOT)

- **NOT a behavior-preserving refactor.** This is a **new reopt algorithm**. It
  is NOT covered by P2.10's bit-identity gate. It needs its own A/B experiment
  (current-default-reopt vs new-merged-reopt), which the user already intends:
  "we'll run exp to evaluate performance."
- The user's working hypothesis: this design may perform **similarly to current
  frequent re-opt**, which in current experiments is sometimes **worse than pure
  incremental**. The likely non-eliminated cause is **TL thrashing** (patience-1
  allows a non-improving step that doesn't pay off next interval). The likely
  IMPROVEMENT over current reopt is **Type-E blindness removal** (current reopt
  ignores env-changed tasks; reading (a) adds them). Net effect is empirical.
- **Cache contract must hold.** The one upfront re-opt step (`GlobalBeam`) is a
  >1 change (memoryless `OptimizeFromScratch`) and must NOT route through
  `EvaluateTimeLimitConfig_SingleTaskPatch` / `rta_cache_` the way the incremental
  walk does. `rta_cache_active_` must stay FALSE during the seed step (the current
  reopt path already does this); it is armed only for the subsequent walk.
- **Incumbent safety.** The carried {pa,tl} is the compare-guard baseline;
  `UpdateRecords`' strict-SP guard (tie-break lower TL-sum) adopts only
  strictly-better, so the upfront `GlobalBeam` cannot regress past the carried
  champion. This mitigates the incumbent-degradation risk (cause 1 above).
- **Prerequisite: P2.10 must land first** (shared arm + renamed symbols). P2.10's
  Phase 5a bit-identity gate (flag OFF) is the clean baseline this task changes
  from.

## Open questions / decisions

- **(a) vs (b)** — confirm before implementing. (a) is chosen above; verify the
  Type-E plumbing preserves `|diff|<=1` for the Type-E entries first.
- **RM-Fast at interval 0** — current interval-0 fallback seeds Gaussian-mean TL
  (`InitializeTimeLimitsFromETConfig`). "RM-Fast" may mean strict smallest-TL
  (`SmallestTimeLimitVec`). Confirm which.
- **Patience source** — reopt patience = `IncrementalTimeLimitSearchPatience + 1`
  (currently 0+1=1, matching today's `ReoptimizationTimeLimitSearchPatience`). If
  the incremental patience is ever raised, reopt follows automatically.
