# P1.11 — Partial Task-Subset Optimization (X% per interval, weight-biased + fair)

> A perf/feature follow-on to [[P1.10]] (the serialized E+L queue, now in
> `finished_tasks/`). P1.10 walks **every** env-changed (Type-E) + TL-flexible
> (Type-L) task each interval. This task adds a tuning knob to walk only **X%**
> of them per interval, selecting the subset **weight-biased** (high-weight tasks
> optimized more often) but **fair** (no task starved — every task is
> re-optimized within a bounded number of intervals). **X = 1.0 is the current
> prod behavior**, bit-identical — the ablation anchor.
>
> This is the "Extension idea" recorded in P1.10's `goal.md` (§"Per-interval
> task subset with cycling"), now elevated to its own task with the user's
> percentage-based parameterization (2026-07-17).
>
> **Perf, not correctness** — but it changes *which* tasks get re-searched each
> interval, so it changes result *quality* (a skipped task's priority sits stale
> one cycle). The TDD gate is "X=1.0 bit-identical to current; X<1.0 SP no worse
> than a stated tolerance on a fixed taskset" + the coverage guarantee proven by
> trace.

---

## The problem (why this exists)

Every incremental interval, `PerformSerializedTaskQueueOptimization`
(`OptimizeSP_TL_Incre.cpp:381`) builds the merged E+L queue
(`BuildSerializedTaskQueue` `:411`, weight-desc sorted) and walks **every**
entry (`:419`). Per-interval SP-eval cost = O(|E| + |L|) queue steps, each
Type-L step internally walking its TL option set. At larger N this is the
dominant per-interval cost.

The user's hypothesis: **not every task needs re-optimizing every interval.**
A TL-flexible task whose TL/priority was just adopted is near-optimal for one
interval; an env-changed task whose ET moved slightly may not need an immediate
re-search. If we optimize only X% of the queue per interval — cycling *which*
X% across intervals so nothing drifts too long — per-interval cost drops to
O(X·(|E|+|L|)) at the cost of slower per-task response to drift.

---

## The design

### The knob

One config: `IncrementalTaskOptimizationPercentage` (a `double` in (0, 1],
default 1.0). Per interval, the subset size is `K = ⌈X · |queue|⌉` (ceil so
X>0, |queue|≥1 ⇒ K≥1; X=1.0 ⇒ K=|queue| ⇒ full queue ⇒ **bit-identical** to
current). Following the `ReoptimizationPeriod` / patience-knob convention, it
is a **required** `parameters.yaml` key (no fallback default at load — fail
loudly if missing, matching `agent_coding_rules.md`'s "raise if important config
is not passed"), with a range check `(0, 1]` that rejects X≤0 or X>1 at startup.

### The selection policy (weight-biased + fair, single knob)

Two buckets fill the K slots, then the subset is walked in the queue's existing
weight-desc order:

1. **Stale bucket (the fairness guarantee):** tasks whose
   `intervals_since_last_optimized ≥ CoverageHorizon(X)` are **force-included**,
   regardless of weight. This is the backstop that prevents starvation.
2. **Weight bucket (the bias):** the remaining `K − |stale|` slots go to the
   **highest-weight** tasks not already in the stale bucket (i.e. the first
   `K − |stale|` non-stale entries of the weight-desc queue).

The subset is emitted in the queue's original weight-desc order (build the
selected-id set, then filter the queue in order) — so the walk semantics are
unchanged from P1.10, just over fewer entries.

### The coverage horizon (derived, not a knob)

`CoverageHorizon(X) = ⌈1/X⌉` (static, pure function of X). A task unselected
for `⌈1/X⌉` consecutive intervals enters the stale bucket on the next. So
**every task is re-optimized within `⌈1/X⌉ + 1` intervals** of last being
selected. For the user's example:

- **X = 0.5 → horizon = ⌈2⌉ = 2 → coverage within 3 intervals.** ✓ matches the
  user's "if we only optimize half of tasks each interval, all tasks optimized
  at least once after 3 intervals."

Trace (N=4, K=2, weights W1>W2>W3>W4, fresh staleness=0):

| Interval | Stale bucket | Weight bucket | Selected | W1 W2 W3 W4 staleness after |
|----------|--------------|---------------|----------|-----------------------------|
| 1 | {} (all 0) | {W1,W2} | {W1,W2} | 0 0 1 1 |
| 2 | {} (W3,W4=1) | {W1,W2} | {W1,W2} | 0 0 2 2 |
| 3 | {W3,W4} (≥2) | {} (K−2=0) | {W3,W4} | 1 1 0 0 |
| 4 | {} | {W1,W2} | {W1,W2} | 0 0 1 1 |

W3,W4 optimized at interval 3 — within 3 intervals. ✓ High-weight W1,W2
optimized ~2× per 3 intervals; low-weight W3,W4 1× per 3 — **weight-biased**.
✓

### The algorithm (pseudocode)

```
SelectTaskSubset(queue, X, staleness):           # free fn, pure (testable)
  N = queue.size()
  if X >= 1.0 or N == 0:                          # bit-identical no-op
    return queue                                  #   (staleness UNTOUCHED)
  K = ceil(X * N)                                 # >= 1
  horizon = ceil(1.0 / X)                         # CoverageHorizon(X)

  stale_ids = { e.task_id for e in queue
                if staleness.get(e.task_id, 0) >= horizon }
  weight_slots = max(0, K - len(stale_ids))
  weight_ids = []
  for e in queue:                                 # queue is weight-desc
    if e.task_id in stale_ids: continue
    weight_ids.append(e.task_id)
    if len(weight_ids) >= weight_slots: break
  selected_ids = stale_ids | set(weight_ids)

  subset = [ e for e in queue if e.task_id in selected_ids ]   # weight-desc order

  for e in queue:                                 # update staleness
    staleness[e.task_id] = 0 if e.task_id in selected_ids \
                           else staleness.get(e.task_id, 0) + 1
  return subset
```

### Config + state + signatures

**`sources/Utils/Parameters.h` / `.cpp`** (match the patience-knob convention):
```cpp
// P1.11: fraction of the serialized E+L queue optimized per incremental
// interval. 1.0 = full queue (current prod behavior, bit-identical). <1.0 =
// weight-biased + fair subset (see SelectTaskSubset). Range (0, 1].
extern double IncrementalTaskOptimizationPercentage;
```
```cpp
double IncrementalTaskOptimizationPercentage =
    loaded_doc["IncrementalTaskOptimizationPercentage"].as<double>();
// + a static-init range check (mirroring _exportDefaultsSet's shape but
// REJECTING out-of-range via CoutError, not falling back): X in (0, 1].
```
Ship `parameters.yaml` with `IncrementalTaskOptimizationPercentage: 1.0` (= current behavior).

**`sources/Optimization/OptimizeSP_TL_Incre.h`** — free function + member + state, mirroring P1.10's "extract the pure logic as a free function for unit-testability" pattern (cf. `FindTimeLimitOptionIndex` / `IsBetterTimeLimitOption`, `OptimizeSP_TL_Incre.h:24-38`):
```cpp
// P1.11: the coverage horizon (stale threshold) derived from X. Pure function
// of the percentage. A task unselected this many consecutive intervals is
// force-included next → every task re-optimized within horizon+1 intervals.
int CoverageHorizon(double optimization_percentage);

// P1.11: pure subset-selection logic (no optimizer state) — unit-testable in
// isolation, mirroring the P1.10 free-fn pattern. `staleness` is read AND
// updated in place (selected → 0, unselected → +1). Returns the selected
// entries in the queue's weight-desc order. X>=1.0 → returns `queue` unmodified
// and leaves `staleness` untouched (the bit-identical no-op).
std::vector<SerializedTaskQueueEntry> SelectTaskSubset(
    const std::vector<SerializedTaskQueueEntry>& queue,
    double optimization_percentage,
    std::unordered_map<int, int>& staleness);
```
```cpp
class OptimizePA_Incre_with_TimeLimits : public OptimizePA_Incre {
  // ...
  // P1.11: per-task intervals-since-last-optimized, the fairness-backstop
  // state. Persistent across intervals (incr_optimizer_ is the orchestrator's
  // stable member, SimulationOrchestrator.h:71). Cleared by the from-scratch
  // reopt path (ResetIncumbentBaseline(from_scratch=true)) — a reopt re-searches
  // every task, so all are fresh afterwards.
  std::unordered_map<int, int> intervals_since_last_optimized_;
};
```

**Hook in `PerformSerializedTaskQueueOptimization`** (`OptimizeSP_TL_Incre.cpp:381`), after `BuildSerializedTaskQueue` (`:411`), before the walk (`:419`):
```cpp
std::vector<SerializedTaskQueueEntry> queue =
    BuildSerializedTaskQueue(dag_tasks_prev_pre_tl);
// P1.11: walk a weight-biased + fair X% subset (1.0 = full queue, no-op).
queue = SelectTaskSubset(
    queue, GlobalVariables::IncrementalTaskOptimizationPercentage,
    intervals_since_last_optimized_);
```

**Staleness clear on reopt** — in `ResetIncumbentBaseline(bool from_scratch)`
(header `:262-268`): `if (from_scratch) intervals_since_last_optimized_.clear();`.
A from-scratch reopt (`ReOptimizePeriodic`) re-searches every priority → all
tasks fresh. Also covers interval-0 bootstrap (fresh optimizer, map already
empty → clear is a no-op). This is the "cover everything" safety net P1.10's
extension idea called out.

---

## Edge cases (handled)

1. **X = 1.0** → early-return full queue, staleness untouched → bit-identical. ✓
2. **Empty queue (|E|+|L| = 0)** → early-return empty → walk is a no-op. ✓
3. **K rounds up** → `ceil(X·N)` ≥ 1 for X>0, N≥1; never 0. Clamped to ≤ N by X≤1. ✓
4. **|stale| > K** (many tasks go stale at once) → stale bucket fills/exceeds K,
   weight bucket empty, subset = all stale (> K). **The guarantee wins over the
   budget** — the budget is a soft target this interval. Rare (only when K is
   set very low relative to N and tasks go stale in bulk). Documented, not
   clamped — clamping would violate the coverage guarantee.
5. **First interval / unseen task** → `staleness.get(id, 0) = 0` → not stale →
   eligible for weight bucket. Pure top-K by weight on interval 1. ✓
6. **Task absent from this interval's queue** (neither env-changed nor
   TL-flexible) → not considered; staleness keeps accruing; on reappearance, if
   ≥ horizon, force-included (re-search is cheap + harmless — the sub-incremental
   confirms the frozen priority). Simple + never wrong. (Eligibility-aware
   staleness is a noted refinement, not the first cut.)
7. **Reopt interval** → `ResetIncumbentBaseline(true)` clears staleness → next
   incremental interval starts fresh. ✓

---

## The X=1.0 correctness anchor (TDD gate)

X=1.0 MUST be bit-identical to the current prod path. Mechanism: `SelectTaskSubset`
early-returns the full queue *and leaves `staleness` untouched* when X≥1.0, so
the walk sees the identical queue in the identical order with no side effects →
identical `opt_pa_` / `opt_sp_` / `res_opt_`. The regression test
(`PerformSerializedTaskQueueOptimization_X1p0_BitIdenticalToBaseline`) pins this
on a fixed taskset. **This is the load-bearing correctness property** — if it
breaks, the knob is not safe to ship at 1.0.

---

## TDD plan (red → green, in `tests/testIncreOpt_w_TL.cpp`)

Following the P1.10 test style (free-fn tests + integration tests):

1. `SelectTaskSubset_FullPercentageReturnsAllTasks` — X=1.0 → returns the full
   queue, order preserved, `staleness` unchanged. (Bit-identical anchor, unit.)
2. `SelectTaskSubset_HalfSelectsHalfByWeight` — X=0.5, N=10, distinct weights →
   K=5, the 5 highest-weight selected; staleness: selected→0, unselected→+1. (Unit.)
3. `SelectTaskSubset_StaleTasksForceIncluded` — a low-weight task with
   staleness ≥ horizon is force-included even though it's below the weight
   cut. (The fairness guarantee, unit.)
4. `SelectTaskSubset_CoverageWithinHorizon` — drive 5 intervals at X=0.5 on a
   fixed queue, assert every task_id was selected ≥1× within ⌈1/X⌉+1=3
   intervals. (The user's "3 intervals" property, traced. Unit, deterministic
   by feeding a fixed queue + reading the staleness map back.)
5. `SelectTaskSubset_PreservesWeightDescOrder` — the returned subset is in the
   input queue's order (weight-desc), not stale-then-weight. (Unit.)
6. `SelectTaskSubset_ClampsAndRejects` — X>1.0 treated as 1.0 (full queue,
   defensive); X=0.0 / negative rejected at config load (Parameters range
   check), not here. (Unit + a Parameters-load test if the range check lands.)
7. `PerformSerializedTaskQueueOptimization_X1p0_BitIdenticalToBaseline` — full
   optimizer run with X=1.0 vs the pre-P1.11 baseline on a fixed taskset →
   identical `opt_pa_` + `opt_sp_`. (Integration, regression anchor.)
8. `PerformSerializedTaskQueueOptimization_SubsetPreservesSingleChangeInvariant`
   — X=0.5, debugMode on, run the serialized optimization, assert
   `AssertSingleChangeInvariant` never throws (a subset is still single-task
   steps — P1.9's premise is preserved). (Integration, debugMode.)
9. `ReOptimizePeriodic_ClearsStaleness` — populate `intervals_since_last_optimized_`
   with non-zero values, run a from-scratch reopt, assert the map is empty
   afterwards. (Integration — the backstop reset.)

---

## A/B evaluation

Sweep X ∈ {0.25, 0.5, 0.75, 1.0} as A/B arms (1.0 = control). Measures:
- **Quality:** the eval-suite gates (Q1/Q2/Q3/E1/E3, all INCR-only post-P2.5)
  — SP must not degrade beyond a stated tolerance as X drops.
- **Perf:** SP-evals per interval (and/or wall-clock per interval) — expected
  ~linear in X. The hypothesis: X=0.5 roughly halves per-interval cost at a
  small SP cost (the stale-task lag), with the coverage guarantee bounding the
  quality loss.

The user runs the A/B (as always). Config arm plumbing mirrors the existing
`INCR_Reopt_X` arms — a per-arm override of
`IncrementalTaskOptimizationPercentage` (or separate arms `INCR_Subset_X`).
Arm-naming detail deferred to the user (open question D5).

---

## Relationship to P1.9 (the RTA cache)

P1.11 and P1.9 both touch `PerformSerializedTaskQueueOptimization`'s vicinity
but at **different points**: P1.11 hooks **before the walk** (subset
selection); P1.9 hooks **inside the eval** (`EvaluateTimeLimitConfig_SubIncremental`,
a `PerCoreRTACache&` seam on `OptimizeIncre`). They are largely independent,
but both edit the same function — **land one before the other, not concurrently.**
P1.11 is smaller and self-contained (one free fn + one member + one config +
one hook line); P1.9 is the bigger cache refactor. Recommended sequence is an
open question (D6) for the user — P1.11 first gives an immediate perf win +
A/B data that may inform whether P1.9's cache is still worth its cost.

**The single-change invariant is preserved** by P1.11 (subset = fewer
single-task steps, each still |diff|≤1), so P1.9's rev-2 cache premise
(`{ReuseAll at |diff|==0, PatchOneTask at |diff|==1}`) is unaffected.

---

## Out of scope / non-goals

- **NOT** a correctness change to the per-task search — the sub-incremental
  primitive (`OptimizeIncre_SingleTask`), the TL walk, and the single-change
  invariant are unchanged. P1.11 only selects *which* tasks to walk.
- Does **not** change the generator, simulator, SP metric, or `ReOptimizePeriodic`'s
  from-scratch descent (reopt still re-searches ALL priorities every
  `ReoptimizationPeriod`-th interval — the coverage backstop).
- Does **not** select by env-change **magnitude** for Type-E (P1.10's extension
  idea mentioned "biggest ET mover first") — the user's request is weight +
  fairness only. Magnitude is a noted future refinement.
- Does **not** apply to the reopt/from-scratch path — that path restructures
  everything by design; only the incremental path's queue is subsetted.
- No **continuous** weight-biasing (DRR / score formula) in the first cut — the
  two-bucket design is binary (top-K weight vs stale-rest) but satisfies
  "higher weight → selected more often." A continuous variant is open question D4.

---

## Open design questions (for the user)

- **D1 — config default vs required.** Proposed: required `parameters.yaml`
  key `IncrementalTaskOptimizationPercentage`, shipped at 1.0, range-checked
  (0,1] at load (rejects out-of-range loudly, no fallback — matches the
  patience-knob convention, not the EXPORT_DETAIL_LEVEL fallback convention).
  OK?
- **D2 — coverage horizon formula.** Proposed `⌈1/X⌉` (X=0.5 → 3-interval
  coverage, matching your example). Alternative `⌈1/X⌉ + 1` is more
  conservative. The trace in §"The coverage horizon" validates `⌈1/X⌉`. OK?
- **D3 — staleness for absent tasks.** Proposed: staleness accrues for a task
  even when absent from the queue (simple; on reappearance a stale task is
  force-included, re-search is harmless). Alternative: accrue only when
  eligible-but-unselected. Proposed is simpler + never wrong; OK with the
  occasional unnecessary re-search on reappearance?
- **D4 — binary vs continuous weight bias.** Proposed: two-bucket (top-K weight
  + stale-rest) — binary. Alternative: a continuous score
  `weight + λ·staleness` (smoother, but adds a λ knob or a derived λ). Proposed
  is simpler (single knob X). Prefer binary for the first cut?
- **D5 — A/B arm shape.** Separate `INCR_Subset_25/50/75` arms (each pins X),
  or a per-arm override of the one config? Naming deferred.
- **D6 — sequencing vs P1.9.** P1.11 first (smaller, immediate perf win +
  A/B data), or P1.9 first (the bigger cache refactor, unblocked now)?
- **D7 — subset walk order.** Proposed: preserve the queue's weight-desc order
  within the subset (build selected-id set, filter queue in order). Alternative:
  walk stale-bucket first (most-overdue first). Proposed matches P1.10's walk
  semantics; OK?

---

## STATUS: FILED 2026-07-17 (design only; no code yet)

All design decisions are PROPOSED (D1–D7 open) pending a user decisions pass.
No code written. Next = user resolves D1–D7 → TDD plan → implement the free fn
+ config + member + hook → A/B. P1.10 (the serialized queue this builds on) is
COMPLETE + committed (in `finished_tasks/`); P1.9 is UNBLOCKED but independent
of this task's landing order (D6).
