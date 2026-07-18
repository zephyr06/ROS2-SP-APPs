# P1.10 — API Design Proposal (signatures + purposes, NO implementation yet)

> Phase = API design only. Per the user's "we'll first focus on API design … propose
> function and sub-function signature design, and purpose of each sub-function first."
> No source code is written here; this is a proposal for a decisions pass. It embeds
> proposed answers to **D1, D2 (partially), D3 (partially), D4 (proposed)**; **D5**
> stays deferred. Grounded in the 2026-07-15 source re-read (see `goal.md`
> "Code-grounded findings").
>
> **2026-07-16 update:** **D1 sub-point 1 is DECIDED + LANDED** (`33b2270c`) — the
> redundant `:238-239`-duplicate eval is removed at the generator via a default
> `exclude_opt_pa=true` on `FindPriorityVec1D_Variations` (skip the carried-position
> variation), bit-identical for the existing `OptimizeIncre`. This **inverts**
> refinement 2's earlier "drop `:238-239`" proposal (keep `:238-239` as baseline, drop
> the carried-pos variation instead). The D1 *main* question (new method vs
> fast-path) is also DECIDED (see `tasks.md` / `dev_log.md`).
>
> **2026-07-17 update:** **D2, D3, D4 DECIDED** (this session; D5 still open). D2 =
> **`FindTaskWithDifferentEt` UNCHANGED** — the caller of the Type-E diff prepares
> the two diff-side DAGs so TL-flexible tasks carry identical ET on both sides, after
> which the unchanged function reports only genuinely env-changed static tasks. This
> **retires the "New state" member** (`dag_tasks_prev_interval_pre_tl_`) proposed
> below: the caller-ensures-same-ET property holds by construction at the capture
> site (both sides pre-TL), so no retained member is needed — see refinement 1 + the
> D2 correction block below. D3 = sort the merged E+L queue by **task weight
> (descending)**; D4 = **running-adopted single champion** (`res_opt_`).

## What this proposes (the shape, restated)

At each new interval, **explicitly separate** the two task types, **sort them
together** into one merged queue, and walk the queue serially calling a single
**sub-incremental** primitive per task:

- **Type E (env-changed ET)** → the NEW capability: re-search that one task's 1D
  priority under the new env (today the incremental path never does this — env
  cancels in `FindTaskWithDifferentEt`, priorities frozen).
- **Type L (TL-flexible)** → the existing trial-and-error TL walk, but each TL step
  calls the sub-incremental (drops the redundant `:238-239` duplicate eval), NOT
  the full `OptimizeIncre` entry.

"Separately" = distinct step *handlers* per type. "Sort together" = one merged,
heuristically-sorted queue both types feed into.

## Grounding refinements discovered while designing (sharpen the premise)

1. **`dag_tasks_` (the OWNER member) is always pre-TL.** TLs are applied transiently
   via `UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)` into a fresh
   `dag_tasks_cur`; the member is never mutated by TL. Only `OptimizeIncre_w_TL :313`
   (`dag_tasks_ = dag_tasks_update`) mutates it, absorbing the new interval's env.
   ⇒ Type-E is a diff of two **pre-TL env DAGs** = `FindTaskWithDifferentEt(prev,
   cur)`. **No new diff function needed** — reuse `FindTaskWithDifferentEt` with new
   state. This SIMPLIFIES D2. **(D2 correction, 2026-07-17 — see the block below: the
   "new state" is a LOCAL capture, NOT a retained member.)**
2. **The carried-position variation duplicates `:238-239` — D1 sub-point 1 DECIDED
   2026-07-16 (inverts this section's earlier proposal).**
   `FindPriorityVec1D_Variations` always emits `i == old_priority_index`, which
   reconstructs the carried PA; its eval under `dag_tasks_cur` IS what `:238-239`
   computes (`opt_sp_ = EvaluateSPWithPriorityVec(dag_tasks_update, ..., opt_pa_)`).
   One redundant eval per changed task. **DECIDED mechanism:** give the generator a
   default param `bool exclude_opt_pa = true` (`OptimizeSP_Incre.h:75` / `.cpp:180`)
   that skips emitting the carried-pos variation. This is the **inverse** of this
   section's earlier "sub-incremental drops `:238-239`" proposal: **keep `:238-239`**
   as the baseline (it is the genuine new-env SP — `goal.md` finding 5: NOT redundant at
   baseline), **drop the carried-pos variation** instead. Same net **−1 SP-eval per
   call**, but uniformly correct: the carried-pos variation duplicates `:238-239` at
   BOTH baseline and TL steps (so always skipping it is right), whereas `:238-239` is
   only redundant at TL steps (so dropping it would be conditional). The generator is
   the single right place — it knows `old_priority_index` from the input `pa_vec`
   before any mutation. **Bit-identical for the existing `OptimizeIncre`** (production
   caller at `OptimizeSP_Incre.cpp:273-277` inherits `true`): the carried-pos eval gives
   SP_base, and the adopt test at `OptimizeSP_Incre.cpp:282` is strict `>`
   (`if (sp_eval > opt_sp_)`), so an equal SP never displaces `opt_sp_`. LANDED in
   `33b2270c`. See `dev_log.md` 2026-07-16.
3. **Type-E carried-position eval is NOT redundant.** `res_opt_.sp_opt` is under the
   OLD env; under the new env the carried PA's SP is new. So for Type-E ALL variation
   evals (carried included) are genuine new work — Type-E is +N evals where today
   there are 0 (the capability didn't exist). For Type-L the carried-position eval is
   also new (trial TL ≠ committed); only the `:238-239` duplicate is dropped.

## D2 correction (2026-07-17) — `FindTaskWithDifferentEt` UNCHANGED, caller normalizes

> **⚠ AMENDED 2026-07-17 (same session, later) — see "D2 amendment" block below.**
> The "caller normalizes / both diff sides equal ET for TL-flexible tasks" premise
> rested on TL-flexible dists being bit-equal on both sides. They are NOT robustly so:
> `FiniteDist::operator!=` is 10%-relative `approx_equal` (`Probability.cpp:415-417`),
> and a TL-flexible task's `execution_time_dist` is built from the raw mu/min/max YAML
> fields at read (`RegularTasks.cpp:75-79`, no adopted-TL override). So the premise is
> tolerance-dependent → fragile. D2 is AMENDED to a **structural filter**
> (`FindEnvTaskWithDifferentEt` = `FindTaskWithDifferentEt` MINUS TL-flexible tasks);
> `FindTaskWithDifferentEt` stays unchanged ONLY for the live TL-walk `:282` caller.
> The text below is the pre-amendment record, kept for provenance.

> **Authoritative (user correction 2026-07-17):** "`FindTaskWithDifferentEt`'s
> implementation does its job. The issue is that the **caller** of
> `FindTaskWithDifferentEt` should make sure `dag_tasks_updated` and `dag_tasks` have
> the same ET for tasks with flexible ET." This **overtURNS the earlier idea of adding
> a mask/exclusion parameter** to `FindTaskWithDifferentEt`. The function's contract —
> "diff `execution_time_dist`, report movers with direction" — is correct and stays
> exactly as written (`OptimizeSP_Incre.cpp:140-155`, no signature/behavior change).
> `OptimizeIncre`'s call site at `.cpp:282` stays exactly as-is (no mask arg). Base
> `OptimizeIncre` remains TL-unaware; TL-awareness is the caller's responsibility.

**The caller-ensures-same-ET property holds by construction at the capture site**, so
the normalization is NOT an extra pass — it is a consequence of capturing two
**pre-TL** env DAGs:

- `dag_tasks_prev` = `dag_tasks_` captured **before** the `:313` (or `:452`) absorb.
  At that point `dag_tasks_` is the previous interval's pre-TL env DAG (TLs went into
  transient `dag_tasks_cur` last interval, never the member).
- `dag_tasks_cur` = `dag_tasks_update` (the orchestrator-passed new pre-TL env DAG).
- Both sides pre-TL → under the generator invariant "TL-flexible tasks have no env
  dependence by design," every TL-flexible task's `execution_time_dist` is **equal on
  both sides** → `FindTaskWithDifferentEt` never flags them → the surviving diff is
  pure env (exactly the Type-E set, with direction). No mask, no normalization pass.

This **retires the "New state" member** (`dag_tasks_prev_interval_pre_tl_`) proposed
elsewhere in this doc: the prev-DAG is a **local** captured at the top of the entry
(`OptimizeIncre_w_TL` / `ReOptimizePeriodic`), used for the one Type-E diff, then
discarded — not a retained member. (If a future caller wants Type-E across a boundary
the local can't see, *that* caller owns retaining what it needs; the base path stays
local-only.)

**Sub-point (open, carried from the prior D2 sharpening):** the local prev-DAG
capture must happen at BOTH entry points that absorb a new env DAG —
`OptimizeIncre_w_TL :313` AND `ReOptimizePeriodic :452` (identical absorb). Capturing
at the top of each entry handles both for free. If only the incremental entry
captures, the first post-reopt interval's Type-E diff compares against a stale
pre-reopt prev-DAG → accept Type-E suppressed on that one interval, or capture in
both. (D5 will decide whether `ReOptimizePeriodic` is in scope at all; if D5 =
incremental-only first cut, only the `:313` site needs the capture.)

## D2 amendment (2026-07-17, later same session) — structural filter supersedes "caller normalizes"

> Supersedes the "D2 correction" block above. The user was investigating whether
> v19/v21 TSP's `execution_time_dist` actually compares unequal via `operator!=` in
> `FindTaskWithDifferentEt` (and whether `ScaleToInteger` mutates it differently). The
> investigation found the premise fragile, so the user directed: *"rename the method as
> `FindEnvTaskWithDifferentEt`, and explicitly filter out tasks with TL options in the
> implementation."* Implemented + tested this session (NOT committed; see `dev_log.md`
> 2026-07-17 D2 AMENDED).

**The fragile premise:** `FiniteDist::operator!=` is `!operator==`, and `operator==`
is `approx_equal(other, 1e-1)` — a **10%-relative tolerance** (`Probability.cpp:415-417`).
A TL-flexible task's `execution_time_dist` is `FiniteDist(gauss, min, max, granularity)`
built from the raw `execution_time_mu/min/max/sigma` YAML fields at read
(`RegularTasks.cpp:75-79`) — perf-pair tasks are NOT special-cased on read (no override
to the adopted TL). So the raw YAML fields flow into the dist, and equality across
intervals is tolerance-dependent: a TL-flexible task's dist can compare unequal for
TL-induced (perf-pair grid) reasons, not env. Relying on the caller to equalize those
dists holds only while they stay within 10% of each other.

**The amendment — two new free functions, `FindTaskWithDifferentEt` kept unchanged:**

```cpp
// Task IDs with TL freedom (non-empty timePerformancePairs — the perf-pair grid).
// Mirrors the {-1}-sentinel test in RecordTimeLimitOptions (OptimizeSP_TL_BF.cpp:29-30).
std::vector<int> FindTasksWithFlexibleTimeLimits(const DAG_Model& dag_tasks);

// Type-E (env-changed) diff: FindTaskWithDifferentEt(prev, cur) MINUS TL-flexible
// tasks. The env signal survives without depending on bit-equal perf-pair dists.
std::vector<DiffObj> FindEnvTaskWithDifferentEt(
    const DAG_Model& dag_tasks, const DAG_Model& dag_tasks_updated);
```

- **`FindEnvTaskWithDifferentEt`** is what the Phase 2 Type-E local-capture diff calls
  (`FindEnvTaskWithDifferentEt(dag_tasks_prev_pre_tl, dag_tasks_)`), NOT the raw
  `FindTaskWithDifferentEt`. Section B's "reuse `FindTaskWithDifferentEt` UNCHANGED"
  is superseded: reuse the new filtered function instead.
- **`FindTaskWithDifferentEt` stays unchanged** for ONE caller only: `OptimizeIncre`'s
  `.cpp:282` site on the **live TL-walk path**. There the diff MUST flag the TL-walked
  (TL-flexible) task so `OptimizeIncre` re-searches its 1D priority each TL step
  (`EvaluateTimeLimitConfig_ScratchOrIncre` `from_scratch=false` →
  `BuildChallengerFromIncumbent` → `OptimizeIncre`). Filtering there would empty the
  diff and stop the mid-walk priority re-search — a behavior change to the live
  incremental path. A single filtered function cannot serve both the Type-E diff
  (filter TL-flexible) and the TL walk (flag the TL-walked TL-flexible task) → two
  functions. The TL walk migrates to `OptimizeIncre_SingleTask(task_id, …)` in Phase 2
  (D5), retiring `:282`'s TL-walk usage — but that is Phase 2, not this step.

## Decided answers to the D-questions (2026-07-17)

- **D1 → DECIDED** new method `OptimizeIncre_SingleTask` (NOT a `|diff|==1` fast-path
  inside `OptimizeIncre`). The serialized loop already KNOWS which task changed (it is
  walking that task), so recomputing `FindTaskWithDifferentEt` is wasted work. A
  direct `OptimizeIncre_SingleTask(dag, task_id, et_increased)` is cleaner than a
  fast-path that still calls the diff. LANDED (see `dev_log.md` 2026-07-16 + `tasks.md`
  Phase 1).
- **D2 → DECIDED** `FindTaskWithDifferentEt` **UNCHANGED**; the caller prepares the two
  diff-side DAGs so TL-flexible tasks carry identical ET on both sides. Comparison
  key stays `execution_time_dist !=` with `GetAvgValue()` direction (the function's
  existing logic). The caller-ensures-same-ET property holds by construction at the
  capture site (both sides pre-TL), so NO new state member — a LOCAL prev-DAG capture
  at the top of the entry (before the `:313`/`:452` absorb) is the whole mechanism.
  See the "D2 correction" block above. (Sub-point open: capture at both entries vs
  incremental-only — coupled to D5.)
- **D3 → DECIDED** sort the merged E+L queue by **task weight, descending** (simple,
  uniform key; no E/L tier). High-weight tasks optimized first. (Earlier "reuse
  `TaskSortingHeuristic`" proposal is superseded by the user's "first use a simple
  sorting function based on tasks' weights" — `TaskSortingHeuristic` is a
  weight/threshold/id composite; the first cut uses weight alone. A future cut may
  reintroduce the full heuristic.)
- **D4 → DECIDED** **running-adopted single champion** — `res_opt_` (the P0.5 single
  durable incumbent: PA/TL/SP). Seeded at interval start from the previous interval's
  committed results; each serialized step compares-and-keeps via the existing
  `UpdateRecords`/`CommitIncumbent` (`OptimizeSP_TL_Incre.cpp:105`/`:379`); new
  champion vs keep-old treated identically (the strict-`>` adopt test governs).
  Matches the existing `UpdateRecords` pattern. (Alt: interval-start champion — each
  step diffs against the original — NOT adopted.)
- **D5 → DECIDED 2026-07-17: incremental-only first cut.** The serialized loop's
  primitive (`OptimizeIncre_SingleTask`) is warm-started 1D — maps cleanly onto the
  incremental path (replace `OptimizeIncre` with `OptimizeIncre_SingleTask`), does
  NOT fit reopt. Reopt (`ReOptimizePeriodic`) is deliberately **memoryless about the
  PA**: each candidate eval calls `OptimizeFromScratch(K)` (full beam, empty start),
  not `OptimizeIncre`; the champion is the *yardstick* (compare-and-keep), not the
  search's start. Plugging the warm-started primitive into reopt would make reopt
  warm-started → defeating its escape-incumbent purpose. That's a **semantic change**
  to reopt, not a loop rewrite. Additionally **Type-E does not apply to reopt** —
  reopt re-searches ALL priorities every candidate (`OptimizeFromScratch`), so
  nothing is frozen; Type-E is purely an incremental-path concept. ⇒ First cut =
  rewrite the incremental path only; `ReOptimizePeriodic` stays memoryless full-beam.
- **D2 sub-point DISSOLVED 2026-07-17 (was coupled to D5): capture at `:313` only is
  correct across the reopt boundary for free.** Premise corrected: reopt updates
  `dag_tasks_` to its own env at `:452` just as incremental does at `:313` (both are
  the ONLY writers of the `dag_tasks_` member). So at the start of any interval T,
  before its own absorb, `dag_tasks_` already holds interval T-1's env (whichever
  path T-1 took) → `dag_tasks_prev_pre_tl = dag_tasks_` captured before `:313` gives
  T-1's env, and the Type-E diff T-1→T is correct across the reopt boundary. NOT
  stale, NOT suppressed. The earlier "stale pre-reopt snapshot" framing was based on
  the wrong premise that reopt doesn't update `dag_tasks_`.

## New state — NONE (retired 2026-07-17 by the D2 correction)

The D2 correction retires the previously-proposed retained member
`dag_tasks_prev_interval_pre_tl_`. The prev-DAG is a **local** captured at the top of
the entry (`OptimizeIncre_w_TL` and, if D5 in-scope, `ReOptimizePeriodic`), used for
the one Type-E diff, then discarded:

```cpp
// LOCAL (in the entry body), NOT a member:
DAG_Model dag_tasks_prev_pre_tl = dag_tasks_;   // before the :313 / :452 absorb
dag_tasks_ = dag_tasks_update;                  // the existing absorb
// ... later, the Type-E diff:
std::vector<DiffObj> type_e =
    FindTaskWithDifferentEt(dag_tasks_prev_pre_tl, dag_tasks_);
```

`dag_tasks_` is always pre-TL (refinement 1), so `dag_tasks_prev_pre_tl` is a clean
pre-TL env DAG; `dag_tasks_update` is the new pre-TL env DAG; both pre-TL →
TL-flexible tasks equal on both sides → the unchanged `FindTaskWithDifferentEt`
reports only env movers. No member, no mask, no normalization pass.

**Across the reopt boundary (D2 sub-point DISSOLVED 2026-07-17):** capturing at
`:313` only is correct even when interval T-1 was a reopt. Reopt updates `dag_tasks_`
to its own env at `:452` (the OTHER writer of the member), identical in kind to the
incremental `:313` absorb. So at the start of any interval T, before its own absorb,
`dag_tasks_` holds T-1's env regardless of which path T-1 took → the `:313` capture
gives T-1's env → the T-1→T diff is correct across the boundary. (Reopt itself
doesn't compute Type-E — D5 — so `:452` needs no capture.)

## Function signatures + purposes

### A. Sub-incremental primitive — `OptimizePA_Incre::OptimizeIncre_SingleTask`
*(base class, `OptimizeSP_Incre.h/.cpp`, sibling to `OptimizeIncre`)*

```cpp
// Re-search ONE task's 1D priority position against `dag_tasks_update`, warm-started
// from the incumbent opt_pa_/opt_sp_. The |diff|==1 specialization of OptimizeIncre's
// :269 loop body: instead of computing FindTaskWithDifferentEt and looping, the
// caller already knows the single changed task.
//
// Purpose of each step:
//  - Baseline opt_sp_ = EvaluateSPWithPriorityVec(dag_tasks_update, sp_parameters_,
//    opt_pa_) — KEEP this (mirrors OptimizeIncre's :238-239). It is the genuine
//    carried-PA SP under the new env (NOT redundant — goal.md finding 5). The
//    redundant duplicate is removed one level down, at the generator.
//  - Generate FindPriorityVec1D_Variations(opt_pa_, task_id, AnalyzePriorityChangeStatus(
//    sp_parameters_, task_id, et_increased)) — INHERITING the new default
//    exclude_opt_pa=true, so the generator SKIPS emitting the carried-position
//    variation (i == old_priority_index). That variation would reconstruct opt_pa_
//    and re-evaluate it to the baseline SP just computed — the duplicate. The
//    emitted set is the SAME genuine candidate set OptimizeIncre's :269 loop uses
//    minus that one no-op. (D1 sub-point 1, DECIDED 2026-07-16; see refinement 2.)
//  - Evaluate each emitted variation via EvaluateSPWithPriorityVec(dag_tasks_update,
//    ..., variation); keep the best (strictly greater SP; ApproxEqualSP ties keep
//    the existing opt_pa_, mirroring OptimizeIncre's `>` adopt at :282).
//  - Advance dag_tasks_ = dag_tasks_update (mirrors :300).
// `et_increased`: direction of the ET change, drives the variation pruning via
//    AnalyzePriorityChangeStatus. Caller computes it (Type-E: avg-ET delta vs last
//    interval; Type-L: OPEN — proposed = actual avg-ET delta ET(trial TL) −
//    ET(committed TL) read from the perf-pair, NOT the TL-sign heuristic
//    (sharpening 1, dev_log.md 2026-07-16 — the sign is a monotonicity assumption
//    that prunes the wrong half if it breaks; the actual delta is always available).
//    REQUIRED (no default) — a wrong direction prunes the wrong half and can miss
//    the optimum.
// Returns the (possibly updated) opt_pa_; mutates opt_pa_/opt_sp_ in place.
PriorityVec OptimizeIncre_SingleTask(const DAG_Model& dag_tasks_update,
                                     int task_id, bool et_increased);
```

### B. Type-E set — reuse `FindTaskWithDifferentEt` UNCHANGED (NO new function)

```cpp
// Type-E = tasks whose pre-TL execution_time_dist moved vs last interval. Computed
// by REUSING FindTaskWithDifferentEt — UNCHANGED — against the LOCAL prev-DAG:
//   FindTaskWithDifferentEt(dag_tasks_prev_pre_tl, dag_tasks_)
// (D2, DECIDED 2026-07-17: FindTaskWithDifferentEt's implementation does its job; the
// CALLER ensures both diff sides have identical ET for TL-flexible tasks. That holds
// by construction here because both sides are pre-TL env DAGs — refinement 1 — so
// under "TL-flexible tasks have no env dependence," TL-flexible ETs are equal on both
// sides and the function never flags them. The surviving diff is exactly the Type-E
// set, with direction (DiffObj.increase). No new diff function, no mask, no member.)
//
// dag_tasks_prev_pre_tl is the LOCAL captured before the :313/:452 absorb (see
// "New state — NONE" above). The function's .cpp:140-155 body and .h:59 decl are
// untouched; OptimizeIncre's call site at .cpp:282 is untouched.
```

### C. Type-L set — `CollectTLFlexibleTaskIds` (small helper, derived)

```cpp
// On OptimizePA_Incre_with_TimeLimits. Returns task ids with TL freedom, i.e. those
// whose time_limit_option_for_each_task_[id] is NOT the {-1}-only sentinel. These
// are the tasks PerformCoordinateDescentForTaskConfigOpt walks today (the :266-267
// skip is the inverse filter). Pure query over existing state; no mutation.
std::vector<int> CollectTLFlexibleTaskIds() const;
```

### D. Merged + sorted queue — `BuildSerializedTaskQueue` (derived)

```cpp
struct SerializedTaskQueueEntry {
    int task_id;
    enum class Kind { EnvChanged, TLFlexible } kind;
};

// Merge the Type-E set (B) and the Type-L set (C) into one queue, sort TOGETHER by
// task WEIGHT DESCENDING (D3, DECIDED 2026-07-17 — simple, uniform key; no E/L tier;
// high-weight first). Each entry carries its Kind so the loop can dispatch to the
// right step handler. Returns by value; called once per interval at the top of the
// serialized loop.
//
// DEDUP POLICY DECIDED 2026-07-17: a task is NOT permitted to be both env-changed
// (Type-E) AND TL-flexible (Type-L). By generator design TL-flexible tasks have NO
// env dependence, so the two sets are disjoint by construction; if a task appears
// in BOTH at runtime, RaiseError (this case is not considered in this project — it
// signals a generator/contract violation, not an optimization choice). No silent
// winner-pick; the build/queue step hard-fails instead.
std::vector<SerializedTaskQueueEntry> BuildSerializedTaskQueue();
```

### E. Sub-incremental eval entry — `EvaluateTimeLimitConfig_SubIncremental`
*(derived, `OptimizeSP_TL_Incre.h/.cpp`, sibling to `EvaluateTimeLimitConfig_ScratchOrIncre`)*

```cpp
// The shared SP-eval entry for BOTH step kinds. Mirrors
// EvaluateTimeLimitConfig_ScratchOrIncre's incremental branch but calls
// OptimizeIncre_SingleTask instead of OptimizeIncre.
//
// Purpose of each step:
//  - dag_tasks_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)
//    (same as :145-146).
//  - challenger = BuildChallengerFromIncumbent() (opt_pa_/opt_sp_ seeded from
//    res_opt_; dag_tasks_ = dag_with_tl_prev) — same as :160.
//  - challenger.OptimizeIncre_SingleTask(dag_tasks_cur, task_idx, et_increased)
//    — the sub-incremental (A). The redundant carried-pos duplicate is already
//    removed one level down: A's FindPriorityVec1D_Variations call inherits
//    exclude_opt_pa=true (D1 sub-point 1, DECIDED), so the carried-position
//    variation is never emitted. A still computes the baseline opt_sp_ via
//    :238-239 (genuine new-env SP, kept).
//  - current_sp = challenger.opt_sp_; UpdateRecords(challenger, time_limits)
//    — same compare-and-keep as :162-163.
// `task_idx`: the one task whose ET differs from the champion (Type-E: env-changed
//    task, time_limits = committed TL; Type-L: TL-walked task, time_limits = trial
//    TL). `et_increased`: REQUIRED, caller-supplied direction.
// Virtual (mirrors :107) so the walk can be unit-tested with a TL→SP stub.
virtual double EvaluateTimeLimitConfig_SubIncremental(
    int K, const std::vector<double>& time_limits, size_t task_idx, bool et_increased);
```

### F. Serialized loop driver — `PerformSerializedTaskQueueOptimization` (derived)

```cpp
// Replaces PerformCoordinateDescentForTaskConfigOpt's body for the INCREMENTAL path.
// Purpose of each step:
//  - ResetIncumbentBaseline(/*from_scratch=*/false) — same :258 gate reset
//    (opt_sp_=-1.0 so the first UpdateRecords force-commits).
//  - queue = BuildSerializedTaskQueue() (D).
//  - Baseline eval DECIDED 2026-07-17 = (b) a DEDICATED RE-SCORE, NOT
//    EvaluateTimeLimitConfig_ScratchOrIncre. The baseline has NO single changed
//    task (ndiff==0): it just re-scores the champion's carried {pa,tl} under the
//    new env DAG to seed opt_sp_ for the serialized walk. Using
//    EvaluateTimeLimitConfig_ScratchOrIncre would route the baseline through the
//    incremental branch → OptimizeIncre → which performs PRIORITY OPTIMIZATION on
//    env-changed tasks BEFORE the queue's sorted order is honored. That violates
//    the proposal's core shape ("sort tasks, then optimize in order"): the baseline
//    must NOT optimize anything, only re-score. So the baseline =
//    EvaluateSPWithPriorityVec(UpdateExtDistBasedOnTimeLimit(dag_tasks_, committed_tl),
//    sp_parameters_, opt_pa_) directly (no OptimizeIncre, no BuildChallengerFromIncumbent
//    rebuild); seeds opt_sp_, then the queue walk does all optimization in D3 order.
//  - For each entry in queue (D4 = running-adopted champion):
//      * EnvChanged → EvaluateTimeLimitConfig_SubIncremental(K, committed_tl,
//        entry.task_id, env_et_increased). One sub-incremental re-search; no TL walk.
//      * TLFlexible  → OptimizeSingleTaskTimeLimit_SubIncremental(...) (G), the
//        patience-bounded outward TL walk whose each step calls (E).
//  - Champion mutation between steps = sequential (D4): each step's UpdateRecords
//    adopts into res_opt_, the next step's BuildChallengerFromIncumbent sees it.
void PerformSerializedTaskQueueOptimization(
    int K, std::vector<double>& starting_time_limits);
```

### G. Type-L step — `OptimizeSingleTaskTimeLimit_SubIncremental` + extracted `..._Impl`
*(derived; refactor of `OptimizeSingleTaskTimeLimit` so the walk body is eval-injected)*

```cpp
// Eval-injected core walk body, extracted from OptimizeSingleTaskTimeLimit (:194-239)
// so the serialized Type-L step can inject the sub-incremental eval (E) — skipping
// the redundant :238-239 — while reusing the IDENTICAL patience-bounded outward
// walk. `eval` is called per TL candidate with the trial time_limits (K bound in the
// closure). task_idx/et_increased for the sub-incremental are captured by the
// caller's binding (fixed for the whole walk of one task). This extraction is the
// Type-L REFACTOR that must NOT change behavior (tasks.md Phase 1 gate).
double OptimizeSingleTaskTimeLimit_Impl(
    size_t task_idx, std::vector<double>& time_limits,
    double current_sp, double baseline_val, int step, int patience,
    std::function<double(const std::vector<double>&)> eval);

// Existing behavior preserved: thin wrapper binding eval to
// EvaluateTimeLimitConfig_ScratchOrIncre (from_scratch threaded via capture).
// Untouched external contract; the BF/oracle and reopt paths keep using this.
double OptimizeSingleTaskTimeLimit(size_t task_idx, int K,
    std::vector<double>& time_limits, double current_sp, double baseline_val,
    int step, bool from_scratch, int patience);

// Serialized Type-L step: same walk, eval bound to
// EvaluateTimeLimitConfig_SubIncremental (E) with this task's task_idx +
// et_increased captured. et_increased per step = sign of (trial TL − committed TL)
// (larger TL → larger ET); computed inside the eval binding.
double OptimizeSingleTaskTimeLimit_SubIncremental(
    size_t task_idx, int K, std::vector<double>& time_limits,
    double current_sp, double baseline_val, int step, int patience,
    bool et_increased_baseline);
```

## How E and L are "sorted together, optimized separately"

- **Sorted together:** `BuildSerializedTaskQueue` (D) merges B+C and sorts the merged
  list by `TaskSortingHeuristic` — one ordering, both types interleaved by weight.
- **Optimized separately:** the loop (F) dispatches each entry by `Kind` to its own
  handler — EnvChanged → single sub-incremental re-search (E with committed TL);
  TLFlexible → the TL walk (G) whose steps call E with trial TLs. The two handlers
  share the sub-incremental primitive (A) and the eval entry (E); they differ only in
  what `time_limits` they feed (committed vs trial) and whether they walk.

## The single-change invariant (P1.9's unblock condition)

**CORRECTED 2026-07-17 (Phase 3 instrumentation):** the earlier claim below that
the Type-E step yields `|diff|==1` was **wrong** — it was not updated when the D2
premise correction / amendment landed. The instrumentation
(`AssertSingleChangeInvariant` in `EvaluateTimeLimitConfig_SubIncremental`,
debugMode-gated) empirically proved a Type-E step yields `|diff|==0`, not 1. The
honest invariant is `|diff|<=1`:

Every SP-eval in the serialized path changes **at most** one task's ET vs the champion:
- **Type-E step:** the env move was absorbed into `dag_tasks_` at the `:313` absorb
  BEFORE `BuildChallengerFromIncumbent` built the champion DAG, so it is on BOTH diff
  sides and cancels (`goal.md` premise-correction finding #1). The candidate DAG
  (`dag_tasks_` + committed TL) == the champion DAG (`dag_tasks_` + committed TL) →
  `|diff|==0`. The re-search is still meaningful: the carried PA may be stale under
  the new env, and `opt_sp_` is re-scored fresh under `dag_tasks_cur` so the strict-`>`
  adopt test measures variations against the genuine new-env SP. **Same DAG, only the
  re-searched PA varies** → for P1.9 this is *full RTA reuse* (not even a patch).
- **Type-L TL step:** one task's trial TL moved → `|diff|==1` (already holds today,
  `goal.md` finding 3) flagging the walked task.
- **Baseline:** ndiff==0 (no re-search), the genuine new-env re-score — NOT a patch
  candidate, so the invariant (which scopes the patcher) doesn't apply to it.

⇒ P1.9's rev-2 cache collapses to: `|diff|==0` → full RTA reuse (Type-E + baseline),
`|diff|==1` → single-task RTA patch (Type-L). **Never** a multi-task patch. P1.10's
Phase 3 burden = prove `|diff|<=1` holds at every serialized SP-eval — DONE 2026-07-17
(`AssertSingleChangeInvariant` + `SerializedIncremental_SingleChangeInvariant` TDD;
see `dev_log.md` §2026-07-17 "Phase 3"). *(Pre-correction record: the text below
claimed Type-E = `|diff|==1` "by construction"; superseded by the finding above.)*

> Every SP-eval in the serialized path changes exactly one task's ET vs the champion:
> - Type-E step: one env-changed task, committed TL → `dag_tasks_cur` differs from the
>   champion by that one task's env-moved ET. `|diff|==1` by construction.
> - Type-L TL step: one task's trial TL moved → `|diff|==1` (already holds today,
>   `goal.md` finding 3).
> - Baseline: ndiff==0 (no re-search), the genuine new-env re-score — NOT a patch
>   candidate, so the invariant (which scopes the patcher) doesn't apply to it.
>
> ⇒ P1.9's rev-2 cache collapses to a single-task patch every queue step. P1.10's new
> burden = prove the Type-E step preserves `|diff|==1` (Phase 3, `tasks.md`).
> *(SUPERSEDED 2026-07-17 — see correction above; Type-E is `|diff|==0`, not 1.)*

## Open decision points this proposal rests on (for the user)

> **D1 sub-point 1 DECIDED + LANDED 2026-07-16** (`33b2270c`): the redundant
> `:238-239`-duplicate eval is removed at the generator via a default
> `exclude_opt_pa=true` on `FindPriorityVec1D_Variations` (skip the carried-position
> variation), bit-identical for `OptimizeIncre`. Applies to both the existing
> `OptimizeIncre` and `OptimizeIncre_SingleTask`.

1. **D1 (main)**: **DECIDED 2026-07-16** — new `OptimizeIncre_SingleTask` method (NOT
   a `|diff|==1` fast-path). Caller knows the task. LANDED (see `dev_log.md` /
   `tasks.md` Phase 1).
2. **D2**: **DECIDED 2026-07-17** — `FindTaskWithDifferentEt` UNCHANGED; caller
   ensures both diff sides have identical ET for TL-flexible tasks (holds by
   construction at the pre-TL capture site). No mask, no retained member — a LOCAL
   prev-DAG capture before the `:313`/`:452` absorb. **Sub-point still open** (coupled
   to D5): capture at BOTH entries vs incremental-only. See "D2 correction" block.
3. **D3**: **DECIDED 2026-07-17** — sort the merged E+L queue by task weight
   descending (simple, uniform; no E/L tier). High-weight first. (Full
   `TaskSortingHeuristic` deferred to a future cut.)
4. **D4**: **DECIDED 2026-07-17** — running-adopted single champion (`res_opt_`),
   seeded from the previous interval's committed results; sequential compare-and-keep
   via `UpdateRecords`/`CommitIncumbent`.
5. **Dedup policy** (D's note): **DECIDED 2026-07-17** — a task is NOT allowed to be
   both env-changed (Type-E) AND TL-flexible (Type-L). The two sets are disjoint by
   generator design (TL-flexible tasks have no env dependence); if a task appears in
   BOTH at runtime, **RaiseError** (not a winner-pick — this case is not considered in
   this project; it signals a contract violation). If a winner were ever needed, TL-flexible
   wins (user direction), but the chosen behavior is to hard-fail rather than pick.
6. **Baseline eval** (F's): **DECIDED 2026-07-17 = (b) dedicated re-score, NOT
   `EvaluateTimeLimitConfig_ScratchOrIncre`.** The baseline re-scores the champion's
   carried {pa,tl} under the new env DAG to seed `opt_sp_`; it must NOT optimize.
   Routing it through `EvaluateTimeLimitConfig_ScratchOrIncre` would call `OptimizeIncre`,
   which performs priority optimization on env-changed tasks BEFORE the queue's sorted
   order is honored — violating "sort tasks, then optimize in order." Baseline =
   `EvaluateSPWithPriorityVec(UpdateExtDistBasedOnTimeLimit(dag_tasks_, committed_tl),
   sp_parameters_, opt_pa_)` directly (no `OptimizeIncre`, no `BuildChallengerFromIncumbent`);
   the queue walk then does all optimization in D3 weight order.
7. **D5**: **DECIDED 2026-07-17 — incremental-only first cut.** The primitive is
   warm-started 1D; it fits the incremental path but NOT reopt (which is deliberately
   memoryless about the PA — each candidate eval calls `OptimizeFromScratch(K)`, not
   `OptimizeIncre`; the champion is the yardstick, not the start). Plugging the
   warm-started primitive into reopt would make reopt warm-started → defeating its
   escape-incumbent purpose = a semantic change, not a loop rewrite. AND Type-E does
   not apply to reopt (it re-searches all PAs every candidate → nothing frozen).
   ⇒ First cut = incremental path only; `ReOptimizePeriodic` stays memoryless
   full-beam. (This also DISSOLVES the D2 sub-point — see #2.)

## Extension idea (filed 2026-07-15, NOT part of the first cut)

**Per-interval task subset with cycling.** See `goal.md` "Extension idea" section.
Instead of serializing over the FULL E+L queue every interval, select a SUBSET of
size ≤ K_subset each interval (heuristic: top-K by `TaskSortingHeuristic` weight,
and/or by env-change magnitude for Type-E), and CYCLE which subset is optimized
across intervals (round-robin over the full task set / rotating window) so every task
is re-optimized over a few intervals. Bounds per-interval SP-eval cost to
O(K_subset) instead of O(|E|+|L|), at the cost of slower per-task response to env
drift (a task that drifts right after being skipped sits stale one cycle). Pairs with
`ReOptimizePeriodic` from-scratch reopt as the periodic "cover everything" backstop.
Naturally complements the serialized loop — the queue (D) is already a sorted list;
take the first K_subset and rotate the start offset each interval. Implement AFTER
the core serialized loop lands + the single-change invariant is proven.
