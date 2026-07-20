# P1.13 — API Design (under discussion; NO implementation yet)

> Goal: agree the API surface for wiring the FROZEN P1.11 `RTACache` into the
> BASE priority optimizer `OptimizePA_Incre` (`sources/Optimization/OptimizeSP_Incre.{h,cpp}`).
> The cache API itself (`RTACache::Initialize/AdoptChampion/Evaluate/...`) is
> FROZEN (P1.11 Phase 0 DONE) — this task CONSUMES it, does not change it.
> Questions Q1–Q6 below are the open API-shape decisions. Implementation
> starts only after the user signs off.

## Grounded facts (verified 2026-07-18)

**The loop we're speeding up** — `OptimizeIncre_SingleTask`
(`OptimizeSP_Incre.cpp:275-296`):
```cpp
std::vector<PriorityVec> pa_vec_variations = FindPriorityVec1D_Variations(
    opt_pa_, task_id,
    AnalyzePriorityChangeStatus(sp_parameters_, task_id, et_increased));
for (const PriorityVec& priority_assignment : pa_vec_variations) {
    double sp_eval = EvaluateSPWithPriorityVec(
        dag_tasks_update, sp_parameters_, priority_assignment);   // <-- full RTA per candidate
    if (sp_eval > opt_sp_) { opt_sp_ = sp_eval; opt_pa_ = priority_assignment; }
}
```
Each `priority_assignment` differs from `opt_pa_` (the carried PA) by exactly
ONE task's priority position — the 1D generator moves only `task_id`. That is
a **single priority move** vs a champion whose PA == `opt_pa_`. → `|diff|<=1`,
the exact case `RTACache::Evaluate` serves.

**`OptimizeIncre`** (`:298-360`): scores the carried PA once as the baseline
(`:308-311`), then loops the diff tasks calling `OptimizeIncre_SingleTask`
per task. Between tasks, `opt_pa_` may have moved (a prior task's variation
was adopted) → the champion must advance per adoption for the next task's
diff to stay `|diff|<=1`.

**Oracle path** — `EvaluateSPWithPriorityVec` (`OptimizeSP_Base.cpp:148-158`):
`UpdateTaskSetPriorities` (sorts tasks by PA, `:28-40`) → `ObtainSP_DAG` →
`ObtainSP_TaskSet` (`SP_Metric.cpp:53-67`) → `ProbabilisticRTA_TaskSet` (full
N-task RTA, partitions by `processorId`). `ObtainSP_TaskSet` multiplies
`perf_coefficient` (`:61, :65`).

**Frozen cache contract** (`RTA_Cache.h`):
- `Evaluate(dag, pa, tl) -> const vector<FiniteDist>&` — candidate RTA via
  ≤1-task patch vs champion; seeds `candidate_rta_=rta_`, patches NoReuse
  tasks; does NOT mutate champion; falls back to `Initialize` if no champion.
- `AdoptChampion(dag, pa, tl, rtas)` — cheap commit (stores caller `rtas` +
  rebuilds HP-prefixes, NO RTA); advances champion.
- `Initialize(dag, pa, tl)` — expensive full RTA (self-bootstrap).
- **NONE take `sp_parameters`** — callers pass RAW `dag` + `tl`; cache bakes
  TLs internally via `ApplyTimeLimitsToTasksExecutionTime`.
- `ComputeTaskSetDifference` THROWS on `|diff|>1`; `Evaluate` calls it
  UNGUARDED via `ClassifyReusePerTask` → throws on `|diff|>1`.

**Hazard B (perf_coefficient) — VERIFIED:** the oracle multiplies
`perf_coefficient` (`ObtainSP_TaskSet:61,65`). The cache returns only the RTA
vector; the SP-assembly from RTA must multiply `perf_coefficient` too, else
NOT bit-identical. Existing `ObtainSP_DAG_From_Dists` (`SP_Metric.cpp:129-149`)
calls `ObtainSP` (`:11-15`) which OMITS `perf_coefficient` → cannot reuse it
verbatim. It also has a LIVE caller (`ObtainSPFromRTAFiles:221`) → must NOT
be modified. → need a new helper (Q4).

---

## Q1 — Does `OptimizeFromScratch` get the cache?

**Decision (user-stated): NO.** `OptimizeFromScratch` (`:74`) is a
constructive beam search — each partial path appends one task and re-scores
the unassigned remainder; candidates differ from any fixed champion by >1
task. The single-change invariant does not hold → cache would fall back to
full RTA every call (pure overhead). Stays cache-free.

**API consequence:** the cache is an `OptimizeIncre`-family concern, NOT a
base-class member of `OptimizePA_Incre` that both methods would read. See Q2.

---

## Q2 — Where does the `RTACache` LIVE? (member vs threaded ref)

Three options. This is the central API-shape decision.

### Option A — `RTACache rta_cache_` member on `OptimizePA_Incre` (base class)

```cpp
class OptimizePA_Incre : public OptimimizePA_Base {
   private:
    RTACache rta_cache_;   // NEW
    ...
};
```
- `OptimizeIncre_SingleTask` reads `rta_cache_` directly (it's a method on
  the same class) → **no `RTACache&` threading**, Hazard A vanishes.
- `OptimizeIncre` adopts at each `opt_pa_` move so the next task's diff stays
  `|diff|<=1`.
- `OptimizeFromScratch` simply never touches `rta_cache_` → stays cold, no
  overhead (Q1).
- **Cost:** the cache lives on EVERY `OptimizePA_Incre` instance, including
  the throwaway challengers `BuildChallengerFromIncumbent` rebuilds each
  interval (those are `OptimizePA_Incre` by value). A throwaway's cache is
  dead state (constructed empty, never read) → wasted `sizeof(RTACache)` per
  challenger, but no RTA cost. Acceptable? `RTACache` is a few vectors +
  an `unordered_map` — default-construct is cheap (empty). Probably fine.
- **P1.12 interaction:** P1.12 2a added `RTACache rta_cache_` to the DERIVED
  `OptimizePA_Incre_with_TimeLimits`. If the base now owns one too, the
  derived has TWO (`OptimizePA_Incre::rta_cache_` + its own). That's
  confusing + wasteful. → if we pick A, P1.12's derived-class member should
  be REMOVED and P1.12 re-routed to read the base-class cache. That's a P1.12
  change, but it UNIFIES the cache (one per optimizer, both paths). **Or:**
  P1.12's 2a stays as-is (its own member, gated by `rta_cache_active_`) and
  P1.13's base-class member is a SEPARATE cache used only by `OptimizeIncre`.
  Two caches on the derived object. Works but duplicative.

### Option B — `RTACache&` threaded through `OptimizeIncre` / `OptimizeIncre_SingleTask`

```cpp
PriorityVec OptimizeIncre(const DAG_Model& dag_tasks_update,
                          double baseline_sp = INT_MIN,
                          RTACache* rta_cache = nullptr);   // NEW opt-in
PriorityVec OptimizeIncre_SingleTask(const DAG_Model& dag_tasks_update,
                                     int task_id, bool et_increased,
                                     RTACache* rta_cache = nullptr);  // NEW
```
- Caller owns the cache; passes `nullptr` = legacy oracle path (bit-identical,
  no behavior change for existing callers). **Opt-in → behavior-preserving by
  default**, differential TDD just flips the pointer on in one test.
- `OptimizeIncre_SingleTask` uses `rta_cache->Evaluate(...)` if non-null, else
  `EvaluateSPWithPriorityVec`.
- **No member on base or derived** → no throwaway waste, no P1.12
  double-cache collision.
- **Cost:** a pointer arg on two signatures + the `if (rta_cache)` branch at
  the eval site. Existing callers (`OptimizeSP_TL_Incre.cpp:131, :164, :252,
  :344` via `OptimizeIncre`/`OptimizeIncre_SingleTask`) pass nothing → get
  `nullptr` → unchanged. P1.12's call sites can pass `&rta_cache_` later if
  unified.
- **Hazard A:** the pointer IS the threading — base-class methods reach a
  caller-owned cache via the arg. Resolved.
- This mirrors the codebase's own `baseline_sp` opt-in pattern (default
  `INT_MIN` = "not provided" → legacy path).

### Option C — `RTACache&` threaded + stored on base ONLY when constructed with one

A hybrid: a `RTACache* rta_cache_ = nullptr;` member on base (defaults null),
settable via a constructor or setter. `OptimizeIncre_SingleTask` reads the
member. Combines A's "no arg threading" with B's "opt-in".

**Recommendation: Option B.** It's the smallest API change (two defaulted
pointer args), behavior-preserving by default (nullptr → oracle), avoids the
P1.12 double-cache collision and the throwaway-waste question, and matches
the `baseline_sp` opt-in idiom already on `OptimizeIncre`. The `if` branch at
the eval site is one line. **Awaiting user pick.**

---

## Q3 — When does the champion get ADOPTED inside `OptimizeIncre`?

The champion must track `opt_pa_` so the next variation's diff stays
`|diff|<=1`. Two sub-decisions:

**(a) Bootstrap:** the first eval in an `OptimizeIncre` call needs a champion
whose PA == the carried `opt_pa_`. Options:
- **(a1)** `OptimizeIncre`'s baseline re-score (`:308-311`) is ALREADY scoring
  the carried PA — adopt it as champion there (one `Initialize` or
  `AdoptChampion` if a prior champion's rtas are reusable). Then every
  variation in every `OptimizeIncre_SingleTask` call diffs against the
  carried PA → `|diff|<=1`. **Cleanest.**
- **(a2)** lazy: the first `OptimizeIncre_SingleTask` variation triggers
  `Evaluate` on a cold cache → `Initialize` (full RTA) → adopts. More
  implicit, harder to reason about.

**(b) Per-adoption:** inside the `OptimizeIncre_SingleTask` adopt block
(`:290-293`, when `sp_eval > opt_sp_`), the adopted variation becomes the new
`opt_pa_` → it MUST become the champion too, else the NEXT variation diffs
against the OLD champion (a 2-move diff → throw). So:
```cpp
if (sp_eval > opt_sp_) {
    opt_sp_ = sp_eval;
    opt_pa_ = priority_assignment;
    if (rta_cache) rta_cache->AdoptChampion(dag_tasks_update, opt_pa_, tl, rtas);
}
```
where `rtas` is the vector `Evaluate` just returned for this candidate.

**Open:** is the `tl` for `OptimizeIncre` always the carried adopted TL
(`ReconstructTimeLimitVecFromResOpt`)? In the base-class `OptimizeIncre`
context there's no `res_opt_` (that's derived). The base class has
`dag_tasks_` only. → **Q3-open:** what `tl` does `OptimizeIncre` pass the
cache? Likely the all-`-1` vector (no TL baking — base `OptimizeIncre` is
called on a DAG whose ETs are already final, NOT TL-baked). Need to confirm
the call-site DAG is pre- or post-TL. (See Q5.)

**Recommendation: (a1) + (b) as above; resolve Q3-open via Q5.**

---

## Q4 — The SP-assembly helper (Hazard B)

`Evaluate` returns `const vector<FiniteDist>&` (the RTA vector). The oracle
assembles SP via `ObtainSP_TaskSet` (`:53-67`):
```cpp
sp_overall += SP_Func(ddl_miss, thresholds_node[task_id]) *
              weights_node[task_id] * perf_coefficient;   // <-- perf_coefficient!
```
We need a helper that takes the cache's RTA vector + the DAG + sp_parameters
and reproduces EXACTLY that, so cache-eval SP == oracle SP bit-identical.

**Proposed** (new, in `SP_Metric.h/.cpp`):
```cpp
// Assemble the node-level SP from a precomputed RTA vector (flat by task id),
// multiplying perf_coefficient — bit-identical to ObtainSP_TaskSet's node
// loop. The caller supplies the RTA (e.g. from RTACache::Evaluate); this does
// NO RTA work. Path-level SP (chains) is added by the DAG-level wrapper below.
double ObtainSP_DAG_From_Dists_With_Perf_Coeff(
    const DAG_Model& dag_tasks,
    const SP_Parameters& sp_parameters,
    const std::vector<FiniteDist>& node_rtas);

// Full SP = node SP (above) + path SP (chains). Path SP does NOT depend on
// the RTA cache (chain reaction-time dists are separate, GetRTDA_Dist_AllChains)
// — reuses the oracle's path loop verbatim.
double ObtainSP_Full_From_NodeRTAs(
    const DAG_Model& dag_tasks,
    const SP_Parameters& sp_parameters,
    const std::vector<FiniteDist>& node_rtas);
```
- `ObtainSP_DAG_From_Dists_With_Perf_Coeff` = the node loop of
  `ObtainSP_TaskSet` (`:57-66`) lifted to take a precomputed RTA vector,
  INCLUDING `perf_coefficient` (the existing `ObtainSP_DAG_From_Dists` OMITS
  it — Hazard B — so a NEW helper, the existing one is NOT modified, its live
  caller `ObtainSPFromRTAFiles:221` stays intact).
- `ObtainSP_Full_From_NodeRTAs` = node SP (above) + the path/chain loop from
  `ObtainSP_DAG` (`:96-107`), unchanged. The path SP uses
  `GetRTDA_Dist_AllChains` (chain reaction times, NOT the node RTA cache) so
  it's recomputed as the oracle does — bit-identical.
- **Naming:** matches the codebase's `ObtainSP_*` family. "With_Perf_Coeff"
  is explicit because the omission is the hazard. (User pref: reuse existing
  symbol names — this is a NEW symbol, not a rename, so no conflict.)

**Open Q4:** does the cache-eval path need to recompute the PATH SP every
candidate, or can it be reused too? Path SP depends on chain reaction-time
dists (`GetRTDA_Dist_AllChains`), which depend on the full taskset RTA — a
priority move changes the RTA → changes the path SP. So path SP MUST be
recomputed per candidate (no cache win there). The win is ONLY on the node
RTA. → `ObtainSP_Full_From_NodeRTAs` recomputes path SP every call. Confirm
acceptable (it's what the oracle does too — no regression, just no speedup
on the path portion).

---

## Q5 — Is the `OptimizeIncre` call-site DAG TL-baked? (decides the `tl` arg)

The cache bakes TLs internally via `ApplyTimeLimitsToTasksExecutionTime`
(`SP_Metric.cpp:70-80`): `tl[i]==-1` → keep base dist; else → unit dist at
`tl[i]`. For the cache to be bit-identical to the oracle, the `tl` we pass
must reproduce whatever ETs the oracle's `EvaluateSPWithPriorityVec` saw.

**Two call contexts for `OptimizeIncre`:**
1. **From `EvaluateTimeLimitConfig_ScratchOrIncre`** (`OptimizeSP_TL_Incre.cpp:155-164`,
   the incremental branch): builds `dag_tasks_cur` via
   `UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)` (`:145-146`) —
   TL-BAKED — then calls `optimizer.OptimizeIncre(dag_tasks_cur)`. Here the
   ETs are already TL-baked INTO the DAG. → pass `tl` = all-`-1` (cache bakes
   nothing; the DAG is already final). OR pass the real `tl` and the raw
   (pre-bake) DAG. **Must pick one and be consistent** — the cache's
   `ApplyTimeLimitsToTasksExecutionTime` must see the SAME ETs the oracle did.
2. **From `OptimizeIncre_w_TL` / serialized path:** similar, TL-baked DAG
   passed in.

**Key invariant to verify:** `UpdateExtDistBasedOnTimeLimit` (TL-bake) MUST
be the SAME operation as `ApplyTimeLimitsToTasksExecutionTime` (cache internal
bake), else passing all-`-1` `tl` + the baked DAG vs passing the real `tl` +
the raw DAG diverge. Memory says they're the same op (P1.12 2b notes:
"`UpdateExtDistBasedOnTimeLimit` = same op as cache's internal
`ApplyTimeLimitsToTasksExecutionTime`") — **re-verify before implementing.**

**Recommendation:** pass the RAW (pre-bake) `dag_tasks_` + the real `tl`
vector to the cache, matching P1.12's frozen-contract call pattern
(`Evaluate(dag, pa, tl)` with raw dag + tl). This makes the cache's bake
explicit and matches the P1.12 2b plan. But the base `OptimizeIncre` receives
an already-baked `dag_tasks_update` — so it would need the RAW dag + `tl`
threaded in too, OR we pass the baked DAG + all-`-1` `tl`. **Q5 = pick the
plumbing.** Affects whether `OptimizeIncre` needs a new `tl` arg.

---

## Q6 — Gating: opt-in pointer (Q2-B) already gates. Need a flag too?

If Q2 = Option B (pointer, nullptr → oracle), the pointer IS the gate — no
separate `rta_cache_active_` flag needed (unlike P1.12 2a, which needed the
flag because the cache was an always-present member shared with the reopt
path). Here, the reopt path (`OptimizeFromScratch`) simply doesn't get the
pointer passed → no cache, no throw risk, no regression.

**Recommendation: no flag.** The pointer opt-in is the gate. Simpler than
P1.12 2a's flag (justified there by the shared-`CommitIncumbent` reopt
collision, which does NOT exist here because `OptimizeFromScratch` is a
separate method, not sharing `OptimizeIncre`'s writer).

---

## Summary of recommended API (pending user sign-off)

```cpp
// OptimizeSP_Incre.h — two signatures gain an opt-in RTACache* (nullptr = oracle)
class OptimizePA_Incre : public OptimimizePA_Base {
    PriorityVec OptimizeIncre(const DAG_Model& dag_tasks_update,
                              double baseline_sp = INT_MIN,
                              RTACache* rta_cache = nullptr);          // Q2-B, Q6
    PriorityVec OptimizeIncre_SingleTask(const DAG_Model& dag_tasks_update,
                                         int task_id, bool et_increased,
                                         RTACache* rta_cache = nullptr); // Q2-B, Q6
    // OptimizeFromScratch — UNCHANGED (Q1, no cache)
};

// SP_Metric.h/.cpp — two new helpers (Hazard B, Q4). Existing helpers untouched.
double ObtainSP_DAG_From_Dists_With_Perf_Coeff(
    const DAG_Model&, const SP_Parameters&,
    const std::vector<FiniteDist>& node_rtas);
double ObtainSP_Full_From_NodeRTAs(
    const DAG_Model&, const SP_Parameters&,
    const std::vector<FiniteDist>& node_rtas);
```
- Behavior-preserving by default (nullptr → oracle, all existing callers
  unchanged).
- Differential TDD: one test passes a real `RTACache*`, asserts cache-eval SP
  == oracle SP bit-identical across the 1D variations.
- Champion lifecycle (Q3): adopt at `OptimizeIncre` baseline re-score (a1) +
  at each `OptimizeIncre_SingleTask` adoption (b).
- `tl` plumbing (Q5): pass raw dag + real `tl` — requires confirming the
  base-class `OptimizeIncre` can see the raw dag + tl, OR pass baked dag +
  all-`-1` tl. **Open.**

## Open questions for the user

- **Q2:** A (base member), B (threaded pointer, recommended), or C (hybrid)?
- **Q3-open + Q5:** what `tl`/dag does base `OptimizeIncre` pass the cache?
  (Resolves once we confirm whether `dag_tasks_update` arrives TL-baked and
  whether `UpdateExtDistBasedOnTimeLimit` == `ApplyTimeLimitsToTasksExecutionTime`.)
- **Q4:** OK with two new `ObtainSP_*` helpers (node + full), existing
  `ObtainSP_DAG_From_Dists` left untouched? And accept that path SP is
  recomputed per candidate (no cache win on chains)?
- **Q6:** confirm no flag needed (pointer is the gate).
