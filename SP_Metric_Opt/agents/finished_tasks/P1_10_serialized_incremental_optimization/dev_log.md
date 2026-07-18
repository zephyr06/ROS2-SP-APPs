# P1.10 Serialized Single-Task Incremental Optimization — Dev Log
> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-15 — Task filed (design only; grounds the redesign in source)

User filed a new architectural redesign of the per-interval optimizer search
loop, and explicitly stated this is the "more-important issue" that P1.9
(`RTA_Cache`) is ON HOLD behind — so this task **answers P1.9's open Q5**
(what is P1.9 blocked behind?). No code written; design recorded in `goal.md` +
`tasks.md`.

### User's design (3 points)

1. **Sub-incremental optimizer**: restrict the incremental optimizer to the
   case where **only one task's ET changes**, performing **one** priority
   optimization — a sub-incremental of the existing `OptimizeIncre`.
2. **Serialized interval search**: at each new interval, build the initial
   champion from last interval's result, then go through the two task types one
   by one (env-changed-ET tasks + TL-flexible-option tasks), via a sorting
   heuristic. Type-E (env-changed ET) → sub-incremental optimizer (the perfect
   single-change fit). Type-L (TL-flexible) → existing trial-and-error TL walk,
   but assuming all other tasks' ET unchanged vs the champion → also
   sub-incremental. The critical distinction: **no repeated multi-task
   incremental optimization of env-changed tasks** — serialize → fewer SP-evals.
3. **Cache revisit after**: once only-one-task-changes-per-eval is guaranteed,
   P1.9's cache design simplifies — revisit then. (P1.9 stays ON HOLD until
   this lands.)

### Code grounding (verified against source before recording)

- **Existing incremental PA search** = `OptimizePA_Incre::OptimizeIncre`
  (`OptimizeSP_Incre.cpp:233`). It already loops over `tasks_with_diff_et`
  (from `FindTaskWithDifferentEt` `:140`, returns `vector<DiffObj>` with
  `{task_id, increase}`) and runs `FindPriorityVec1D_Variations` (`:180`) per
  changed task — so the **sub-incremental optimizer is the `|diff|==1`
  specialization** of this loop body (`:269-287`). The loop body is already
  single-task-shaped (`FindPriorityVec1D_Variations(opt_pa_, task_id, ...)`);
  extraction to a `SingleTask` method is mechanical.
- **TL coordinate-descent walk** = `PerformCoordinateDescentForTaskConfigOpt`
  (`:241`) → `OptimizeSingleTaskTimeLimit` (`:194`), patience-bounded outward
  walk over `time_limit_option_for_each_task_` (from `RecordTimeLimitOptions`),
  calling `EvaluateTimeLimitConfig_ScratchOrIncre` (`:224`) per TL step. Already
  serialized over tasks (the `for (size_t idx : sorted_indices)` at `:263`),
  ordered by `TaskSortingHeuristic` (`OptimizeSP_TL_Incre.h:51`, weight-desc
  then threshold-desc).
- **The SP-eval entry the design wants to reduce** =
  `EvaluateTimeLimitConfig_ScratchOrIncre` (`OptimizeSP_TL_Incre.cpp:142`),
  which (incremental branch `:160`/`:161`) rebuilds a throwaway challenger
  (`BuildChallengerFromIncumbent` `:394`) and calls `OptimizeIncre`. So today
  every TL step re-runs the **full multi-task** `OptimizeIncre` — the redundancy
  the redesign removes.
- **Interval DAG (env changes)** arrives as `dag_tasks_update` threaded into
  `OptimizeIncre_w_TL` (`:311`) / `ReOptimizePeriodic` (`:446`); the champion
  warm-start source is `res_opt_` via `ReconstructTimeLimitVecFromResOpt`
  (`:337`) + `BuildChallengerFromIncumbent` (`:394`).

### Open design questions recorded for the user (D1–D5, in `goal.md`)

- **D1** sub-incremental API: new `OptimizeIncre_SingleTask` vs `|diff|==1`
  fast-path.
- **D2** Type-E definition: env-only ET changes (pre-TL DAG diff) vs raw
  `FindTaskWithDifferentEt` (which also flags TL-induced point-mass changes).
  This is the subtle one — a Type-L task's TL step IS a one-task ET change by
  construction, so "env-changed by environment" must be distinguished from
  "changed by our TL choice."
- **D3** merged-queue sorting heuristic: reuse `TaskSortingHeuristic` vs new
  E/L-interleaving heuristic (e.g. env-changed first).
- **D4** champion baseline between queue steps: interval-start vs
  running-adopted.
- **D5** reopt path scope: incremental-only first cut vs also `ReOptimizePeriodic`.

### Relationship to P1.9

P1.9's open **Q5** ("is the more-important issue logged anywhere?") → **YES,
here, as P1.10.** P1.9 STATUS stays ON HOLD; resume sequence updated: P1.10
lands + proves the single-change invariant → P1.9 rev-2 cache simplifies
(single-task patch every call) → then resume P1.9. The P1.9 §"Step 3b design"
wiring and the rev-2 `CheckTaskSetRTAReuse` surface are both expected to
simplify once P1.10's invariant holds; do NOT touch P1.9 until then.

### Next

Decisions pass on D1–D5 (user) → TDD plan → Phase 1 (sub-incremental
optimizer) → Phase 2 (serialized loop) → Phase 3 (prove invariant, unblock
P1.9).

## 2026-07-15 — Premise correction (source re-read rewrites the win story)

User asked: *"in my understanding of the code, env-dependent tasks' priorities
are optimized earlier than TL-tasks at the baseline `EvaluateTimeLimitConfig_ScratchOrIncre`
call — do you agree?"* I traced the actual incremental path to answer, and the
trace overturned the original P1.10 problem framing. No code changed; this is a
docs correction of the task's own premise.

### Finding (verified against source)

The original `goal.md` claimed the incremental path *"re-runs the multi-task
incremental PA search on every single TL step"* and that *"OptimizeIncre
re-walks every env-changed task on every TL step."* **Both are wrong.** The
trace (`OptimizeSP_TL_Incre.cpp:142-175, 388-402`; `OptimizeSP_Incre.cpp:233-301`):

- Both diff sides in `FindTaskWithDifferentEt` derive from the **same member
  `dag_tasks_`** (the new interval's env DAG). `EvaluateTimeLimitConfig_ScratchOrIncre
  :145-146` builds `dag_tasks_cur` from `dag_tasks_` + trial TL;
  `BuildChallengerFromIncumbent :394-402` builds the challenger from the same
  `dag_tasks_` + committed `tl_prev`. Env change is in BOTH → **cancels** →
  env-changed tasks are never flagged, never re-searched. Only re-scored
  (`OptimizeSP_Incre.cpp:238-239`) with frozen `res_opt_.priority_vec`.
- At the **baseline** eval, `starting_time_limits == ReconstructTimeLimitVecFromResOpt()`
  (set `OptimizeIncre_w_TL :324`) == the challenger's `tl_prev` → `dag_tasks_cur
  == dag_with_tl_prev` → **ndiff == 0** → the `:269` re-search loop runs zero
  iterations → **nothing is optimized**; carried `{pa, tl}` only re-scored +
  force-committed. INCR-NDIFF-PROBE (`:251-267`) empirically confirms `ndiff 5 → 0`.
- At each **TL step**, `time_limits[task_idx]` differs from `tl_prev[task_idx]`
  → **ndiff == 1** (just the walked task) → `:269` re-searches that one task's
  1D priority. Comment at `:388-393` states this explicitly.

So **`|diff|==1` already holds** throughout the incremental TL walk — the
sub-incremental `|diff|==1` case is the *existing* TL-step behavior, not a new
one P1.10 introduces.

### What this changes in the design

The design *shape* (sub-incremental + serialized Type-E/Type-L queue) is
unchanged; what changes is the *win story*:

- **Type-E steps are a quality ADDITION, not a reduction.** The incremental path
  has **zero** env-changed-task re-search today (frozen priorities). P1.10 adds a
  single per-task re-search in its own queue slot. Was 0, becomes 1.
- **Type-L steps are a perf SIMPLIFICATION.** Each TL step today pays a redundant
  full-RTA re-score at `OptimizeSP_Incre.cpp:238-239` (the challenger's `opt_sp_`
  was already `res_opt_.sp_opt` at `:400`, its DAG == committed DAG) + a
  throwaway challenger rebuild. A sub-incremental that trusts the committed
  `opt_sp_` and goes straight to the 1D variations skips that re-score per TL
  step. (Baseline re-score is NOT redundant — it computes the new env's SP.)
- Net SP-eval count: Type-L down, Type-E up from 0; whether the net is a
  reduction depends on TL-option-set sizes vs env-changed-task count — settle
  empirically in Phase 2.

### Doc edits applied

- `goal.md`: rewrote "The problem" + "The new idea" sections to state the two
  real gaps (quality: frozen env-changed priorities; perf: redundant `:238-239`
  re-score + challenger rebuild); added a "Code-grounded findings (2026-07-15
  premise correction)" section with the 5 load-bearing facts; **revised D2**
  (Type-E cannot come from `FindTaskWithDifferentEt` — needs a dedicated
  cross-interval pre-TL DAG diff; last interval's pre-TL DAG isn't currently
  retained, `OptimizeIncre_w_TL :313` overwrites `dag_tasks_` in place → new
  state to add); updated STATUS block.
- `tasks.md`: Phase 0 D2 + Phase 1/2/3 reframed (Type-L = refactor/behavior-
  preserving; Type-E = new capability + new state to store last interval's
  pre-TL DAG; Phase 3 notes the invariant already holds in the TL walk, new
  burden is the Type-E step staying `|diff|==1`).

### Relationship to P1.9 unchanged

P1.9 stays ON HOLD behind P1.10. The single-change invariant (`|diff|==1` at
every SP-eval) — which P1.9's rev-2 cache relies on — **already holds in the
current incremental TL walk**; P1.10's job is to (a) add the Type-E re-search
while *preserving* it, and (b) cut the redundant Type-L re-score. P1.9's resume
condition (P1.10 lands + invariant proven) is unaffected by the premise
correction.

## 2026-07-15 — API design proposal + extension idea filed (design only, no code)

User redirected the next step from implementation to **API design first**: *"we'll
first focus on API design. so instead of adding all the code implementation, i want
you to propose function and sub-function signature design, and purpose of each
sub-function first."* Two directives: (1) explicitly separate env-dependent tasks
and TL-tasks, sort them together, then call incremental optimization; (2) record a
new extension idea (per-interval task subset with cycling). Both recorded; **no
source code written**.

### Extension idea filed

`goal.md` "Extension idea" section: **per-interval task subset with cycling** —
select ≤ K_subset tasks each interval (top-K by `TaskSortingHeuristic` weight and/or
env-change magnitude), cycle which subset across intervals (rotating start offset),
so every task is re-optimized over ⌈N/K_subset⌉ intervals. Bounds per-interval SP-eval
cost to O(K_subset) instead of O(|E|+|L|); `ReOptimizePeriodic` remains the periodic
"cover everything" backstop. Strictly ≥ status-quo quality (off-cycle tasks are frozen,
same as today's incremental path). **Implement AFTER** the core serialized loop +
invariant proof. Fits the serialized design naturally — the queue is already a sorted
list, subset = first K_subset with rotating offset.

### API design doc (`api_design.md`, new)

Proposed signatures + purposes for 7 functions + 1 state member. Embeds proposed
answers to D1/D2(partial)/D3(partial)/D4 (D5 deferred) for the decisions pass.

**Two grounding refinements discovered while designing (sharpen the premise, don't
change the shape):**

1. **`dag_tasks_` (the OWNER member) is always pre-TL.** TLs go into the transient
   `dag_tasks_cur = UpdateExtDistBasedOnTimeLimit(dag_tasks_, time_limits)` (`:145`),
   never the member; only `OptimizeIncre_w_TL :313` (`dag_tasks_ = dag_tasks_update`)
   mutates the member (absorbing the new interval's env). ⇒ Type-E = a diff of two
   **pre-TL env DAGs** = `FindTaskWithDifferentEt(prev, cur)` — **reuse the existing
   function with new STATE, no new diff function.** This **SIMPLIFIES D2**
   (`goal.md` finding 1 + D2 said "Type-E cannot come from `FindTaskWithDifferentEt`";
   that's true for the *incremental-path* `FindTaskWithDifferentEt(challenger.dag_tasks_,
   dag_tasks_cur)` where both sides derive from the same `dag_tasks_` → cancels. But
   the cross-interval `FindTaskWithDifferentEt(dag_tasks_prev_interval_pre_tl_,
   dag_tasks_)` is exactly the pre-TL env diff Type-E needs — reuse, don't rewrite).
   The only new thing is the **retained pre-TL DAG snapshot** `dag_tasks_prev_interval_pre_tl_`
   (refreshed at the top of `OptimizeIncre_w_TL` before the `:313` absorb).

2. **The redundant `:238-239` is a DUPLICATE, not a recompute of `res_opt_.sp_opt`.**
   `FindPriorityVec1D_Variations` always emits `i == old_priority_index`, which
   reconstructs the carried PA; its eval under `dag_tasks_cur` IS what `:238-239`
   computes. So `:238-239` duplicates one variation eval. The sub-incremental drops
   `:238-239` and keeps the carried-position variation → same candidate set, **−1
   SP-eval per call**. This sharpens `goal.md` finding 5's mechanism (its
   *conclusion* — `:238-239` is redundant at TL steps and the sub-incremental drops
   it — is unchanged). NOTE: for Type-E the carried-position eval is NOT redundant
   (env changed → carried PA's SP under new env is new); only Type-L's `:238-239`
   duplicate is dropped.

### Proposed function set (signatures only — see `api_design.md` for full purpose-of-each-step)

- **A. `OptimizePA_Incre::OptimizeIncre_SingleTask(dag_tasks_update, task_id, et_increased)`**
  — base-class sub-incremental: the `|diff|==1` specialization of `OptimizeIncre`'s
  `:269` loop body. Skips `:238-239` (the duplicate), generates `FindPriorityVec1D_Variations`
  for the ONE caller-known task, evaluates each, keeps best. Answers **D1** (new method
  vs fast-path; proposed = new method, since the loop already knows the task).
- **B. Type-E = reuse `FindTaskWithDifferentEt(dag_tasks_prev_interval_pre_tl_, dag_tasks_)`**
  — no new function (D2 simplification). Comparison key = existing `execution_time_dist !=`
  with `GetAvgValue()` direction.
- **C. `CollectTLFlexibleTaskIds()`** — derived helper; task ids whose
  `time_limit_option_for_each_task_[id]` isn't the `{-1}`-only sentinel.
- **D. `BuildSerializedTaskQueue()` → `vector<SerializedTaskQueueEntry{task_id, Kind}>`**
  — merges B+C, sorts TOGETHER by `TaskSortingHeuristic` (no E/L tier). Answers **D3**
  (proposed = uniform sort; fallback = E-before-L tier). Open: dedup policy (task both
  env-changed and TL-flexible → EnvChanged wins, proposed).
- **E. `EvaluateTimeLimitConfig_SubIncremental(K, time_limits, task_idx, et_increased)`**
  — derived shared eval entry for both kinds; mirrors `:142`'s incremental branch but
  calls `OptimizeIncre_SingleTask` instead of `OptimizeIncre`. Virtual (mirrors `:107`)
  for stub-based unit tests.
- **F. `PerformSerializedTaskQueueOptimization(K, starting_time_limits)`** — derived
  loop driver replacing `PerformCoordinateDescentForTaskConfigOpt`'s body for the
  incremental path. Baseline = reuse existing `EvaluateTimeLimitConfig_ScratchOrIncre`
  (its `:238-239` is NOT redundant there — computes the new env's SP). Answers **D4**
  (proposed = running-adopted champion via `UpdateRecords`/`CommitIncumbent`).
- **G. `OptimizeSingleTaskTimeLimit_Impl(...)` + `OptimizeSingleTaskTimeLimit_SubIncremental(...)`**
  — eval-injected extraction of `OptimizeSingleTaskTimeLimit :194-239` so the Type-L
  step can inject the sub-incremental eval (E). The existing `OptimizeSingleTaskTimeLimit`
  stays as a thin wrapper (BF/oracle + reopt paths untouched). This extraction is the
  Type-L REFACTOR that must NOT change behavior (Phase 1 gate).

**New state:** `dag_tasks_prev_interval_pre_tl_` (DAG_Model, on
`OptimizePA_Incre_with_TimeLimits`) — pre-TL env DAG snapshot from the end of the
previous interval, retained for the Type-E diff. Interval-scoped, not part of the
incumbent.

### Open decision points resting on the proposal (for the user)

D1 (new method vs fast-path), D2 snapshot timing, D3 (uniform sort vs E-tier),
D4 (running vs interval-start champion), dedup policy (E∩L task), baseline eval
choice, D5 (incremental-only vs also reopt). Full list in `api_design.md`'s "Open
decision points" section.

### Next

Decisions pass on the open points → TDD plan → Phase 1 (sub-incremental primitive +
Type-L extraction, behavior-preserving) → Phase 2 (serialized loop + Type-E new
capability + new state) → Phase 3 (prove invariant, unblock P1.9). **No source code
until the decisions pass.**

## 2026-07-16 — D1 sub-point 1 DECIDED + staged (default-param excludes the carried PA)

The user decided the first concrete sub-point of D1 — how the sub-incremental removes
the redundant `:238-239`-duplicate SP-eval. Direction (verbatim): *"use my suggestion,
update unit tests, add a default parameter which is not include opt_pa_ in function
signature."* Then: *"don't work on code editing yet, first update all design decisions
into related task md file"* — so this entry records the decision before finishing the
code. It supersedes both the caller-side guard I proposed earlier and `api_design.md`
refinement 2's "drop `:238-239`" mechanism (see below).

### The decision

`FindPriorityVec1D_Variations` (`OptimizeSP_Incre.h:75` / `.cpp:180`) gains a default
param `bool exclude_opt_pa = true`. When true, the generator skips emitting the
variation that re-inserts `task_id` at its **carried position** (`i ==
old_priority_index`). That variation reconstructs `pa_vec` exactly, so scoring it
re-evaluates the incumbent's carried PA — the duplicate of the `:238-239` baseline
score. Skipping it removes one SP-eval per changed task, for BOTH the existing
`OptimizeIncre` (inherits the default `true`) and the future `OptimizeIncre_SingleTask`.

### Why this mechanism (inverts refinement 2; same net −1 SP-eval, but cleaner)

`api_design.md` refinement 2 proposed the sub-incremental **"drop `:238-239` and keep
the carried-position variation."** The chosen decision is the **inverse**: **keep
`:238-239`** (it is the baseline `opt_sp_ = EvaluateSPWithPriorityVec(dag_tasks_update,
..., opt_pa_)`) and **drop the carried-position variation** via the generator default.
Same net −1 SP-eval per call, but cleaner because:

- `:238-239` is **NOT uniformly redundant.** At the **baseline** eval it computes the
  genuine new-env SP (`goal.md` finding 5: "`:238-239` is NOT redundant [at baseline]").
  Only at **TL steps** is the carried-pos variation a duplicate of it. "Drop `:238-239`"
  would be *conditional* (keep at baseline, drop at TL steps).
- The carried-pos variation is **always** a duplicate of `:238-239` (baseline AND TL
  step — the carried PA under `dag_tasks_update` is `:238-239`'s result either way), so
  **always** skipping it is uniformly correct. The generator is the single right place:
  it knows `old_priority_index` from the input `pa_vec` before any mutation.

### Bit-identity for the existing `OptimizeIncre` (Phase 1 gate, verified)

The production caller at `OptimizeSP_Incre.cpp:273-277` inherits the new default `true`
→ it also stops emitting the carried-pos variation. **Bit-identical** to prior behavior:
the carried-pos variation evaluates to SP_base (the value `:238-239` just assigned to
`opt_sp_`), and the adopt test at `:282` is strict (`if (sp_eval > opt_sp_)`) — an equal
SP never displaces `opt_sp_`, whether or not an earlier variation raised it. So
`OptimizeIncre`'s **adopted result is unchanged**; only its eval count drops by 1 per
changed task. The future `OptimizeIncre_SingleTask` inherits the same default + argument.

### Bug this fixes in my earlier caller-side-guard proposal

I first proposed guarding inside the `:269` variation loop:
`if (GetProrityIndex(priority_assignment, task_id) == GetProrityIndex(opt_pa_, task_id)) continue;`.
The user caught the flaw: *"if opt_pa_ is updated under the way, then your code doesn't
work?"* — correct, because `:284` (`opt_pa_ = priority_assignment`) mutates `opt_pa_` on
adoption, so `GetProrityIndex(opt_pa_, task_id)` drifts to the adopted position and the
guard is unstable. The default-param approach is stable by construction:
`old_priority_index` is computed from the **input** `pa_vec` (immutable for the call),
before the loop and before any mutation.

### Staged code state (transparency — code work PAUSED per user)

- **Applied (NOT built/tested/committed):** `OptimizeSP_Incre.h:75` — declaration +
  `bool exclude_opt_pa = true` default + comment; `OptimizeSP_Incre.cpp:180` — definition
  takes `bool exclude_opt_pa` (no default at the definition site) +
  `if (exclude_opt_pa && i == old_priority_index) continue;` inside the `for (i = lb..ub)`
  loop with a comment.
- **NOT applied:** the 4 unit tests in `tests/testOptimizeIncrePA.cpp:161-198`
  (`FindPriorityVec1D_Variations`, `_increase`, `_increase2`, `_decrease`) assert the OLD
  full-range contract and will FAIL under the new default (carried element gone, size −1).
  Plan: pass `false` to preserve the full-range contract those tests cover, + add a new
  test for the default-`true` exclude behavior.
- **NOT done:** `cmake --build build --target check.SP_OPT -j5` + `ctest` 16/16 (DEBUG
  build) — the Phase 1 gate. Deferred until the user resumes code work.

### What this decides / leaves open

- **DECIDED (D1 sub-point 1):** the redundant `:238-239`-duplicate eval is removed at
  the **generator** level via `exclude_opt_pa=true` (default), bit-identical for
  `OptimizeIncre`. Applies to both `OptimizeIncre` and the future `OptimizeIncre_SingleTask`.
- **STILL OPEN:** D1 main (new `OptimizeIncre_SingleTask` method vs `|diff|==1` fast-path);
  D2 (Type-E definition + the `dag_tasks_prev_interval_pre_tl_` snapshot, incl. whether it
  must also cover the `ReOptimizePeriodic` path — see sharpening 2); D3 (merged-queue
  sort); D4 (champion mutation); dedup policy; D5 (reopt scope, deferred).

### Two sharpenings raised for confirmation (NOT yet decisions)

1. **Type-L `et_increased` direction.** `api_design.md` G proposes `et_increased` =
   sign(trial TL − committed TL) ("larger TL → larger ET"). Sharpening for the user to
   confirm: the **actual avg-ET delta** (ET at the trial TL vs ET at the committed TL,
   both read from the perf-pair) is the ground truth and is always available — the
   TL-sign is a monotonicity *assumption* that, if it ever breaks, prunes the wrong half
   and can miss the optimum. Prefer the actual delta. (Type-E's `et_increased` = avg-ET
   delta vs last interval is already the actual delta.)
2. **D2 snapshot must cover the reopt path.** `dag_tasks_prev_interval_pre_tl_` is
   proposed refreshed at the top of `OptimizeIncre_w_TL` before the `:313` absorb. But
   `ReOptimizePeriodic` is a separate entry that also absorbs a new env DAG (and runs
   from-scratch reopt). After a reopt interval, the next interval's Type-E diff must
   compare against the reopt's absorbed env, not a stale pre-reopt snapshot — so the
   refresh site must account for both entry paths (or Type-E is suppressed on the first
   post-reopt interval). Confirm with the user.

### Next

User resumes code work → Edit 3 (unit tests) → build + `ctest` 16/16 (Phase 1 gate) →
continue the D1–D5 decisions pass (D1 main, D2, D3, D4, dedup; D5 deferred) → TDD plan →
Phase 1 (`OptimizeIncre_SingleTask` primitive + Type-L extraction, behavior-preserving)
→ Phase 2 (serialized loop + Type-E + new state) → Phase 3 (prove invariant, unblock P1.9).

## 2026-07-16 — D1 sub-point 1 LANDED + TESTED (the repeated SP call is real)

User: "first work on removing the repeated SP call in
`OptimizePA_Incre::OptimizeIncre`, if it really exists."

### Premise verified (it really exists)

`OptimizeIncre` (`OptimizeSP_Incre.cpp:238-307`) does:

- `:243-244` — baseline: `opt_sp_ = EvaluateSPWithPriorityVec(dag_tasks_update,
  sp_parameters_, opt_pa_)`. Genuine new-env SP (new DAG, incumbent PA) — the
  reference the variations are measured against. NOT redundant.
- `:279-282` — `FindPriorityVec1D_Variations(opt_pa_, task_id, ...)` generates the
  candidate PAs.
- `:284-285` — re-evals each candidate.

The duplicate: in all three `PriorityChangeStatus` cases `old_priority_index ∈
[lb, ub]`, so the generator's `for (i = lb; i <= ub; i++)` loop always emits the
variation at `i == old_priority_index`, which re-inserts `task_id` at its existing
position → reconstructs `opt_pa_` exactly → its `:284` eval equals the `:243-244`
baseline. **One redundant SP-eval per changed task.** Confirmed by re-reading
`FindPriorityVec1D_Variations` (`:180-217`): the carried position is always in range.

### Removal mechanism (the staged default-param fix, now tested)

`exclude_opt_pa=true` default on `FindPriorityVec1D_Variations` (`.h:75` / `.cpp:182,
211`) skips emitting the carried-position variation. The production caller
`OptimizeIncre:279-282` inherits the new default → no longer re-evals `opt_pa_` in
the inner loop.

### Bit-identity (verified by the green gate)

`opt_sp_` is always kept in sync with `Eval(dag_tasks_update, opt_pa_)`: set at
`:243-244`, and on every adoption `:287-289` sets `opt_sp_=sp_eval` AND
`opt_pa_=priority_assignment` together. So mid-loop, the carried-pos variation's eval
always equals the current `opt_sp_`, and the strict `>` adopt test (`:287`) never
adopts an equal SP. Dropping the carried-pos variation changes no `opt_sp_`/`opt_pa_`
outcome — only the eval count drops by 1 per changed task. (This also sidesteps the
unstable caller-side guard the user caught: `GetProrityIndex(opt_pa_,…)` would drift
when `:284` mutates `opt_pa_` on adoption; the default-param approach computes
`old_priority_index` from the immutable input `pa_vec` before any mutation.)

### Tests landed

- The 4 old full-range unit tests (`tests/testOptimizeIncrePA.cpp:161-198`) renamed
  `*_full_range` and pinned to `exclude_opt_pa=false` — they document the range logic
  the default-true path skips (the range logic still exists).
- + 1 new test `FindPriorityVec1D_Variations_excludes_carried_pa`: asserts the
  default-true exclude behavior — sizes are full-range −1 in each
  `PriorityChangeStatus`, and no emitted PA equals the carried PA. (Catches the
  OpenToAll/Increase/Increase2/Decrease cases, including the Increase-on-task-0
  degenerate case where the full range is the carried position alone → 0 emitted.)

### Gate (Phase 1 bit-identity)

`cmake --build build --target check.SP_OPT -j5` (DEBUG, uppercase — confirmed
`CMAKE_BUILD_TYPE:STRING=DEBUG`, `libSP_OPTDebug.so`) + `ctest` 16/16 GREEN. Notably:
`testOptimizeIncrePA` (#9, incl. the e2e `GetPriorityAssignments_IncrementalOpt`
result assertion `:250-261`) + `testIncreOpt_w_TL` (#7, the production TL caller
inheriting the new default) both pass unchanged → `OptimizeIncre`'s adopted results
are bit-identical.

### State

LANDED + TESTED in the working tree, NOT committed (user commits). D1 sub-point 1
done. D1 main + D2/D3/D4/dedup/D5 still open for the decisions pass. P1.9 stays ON
HOLD behind P1.10.

---

## 2026-07-16 — D1 main DECIDED + primitive extracted (OptimizeIncre_SingleTask)

### Decision (Option A)

D1 main settled: new method `OptimizeIncre_SingleTask(dag_tasks_update, task_id,
et_increased)` on base `OptimizePA_Incre` — the `|diff|==1` specialization =
extracted former `:274-292` loop body. `OptimizeIncre` refactored to CALL it in
its loop (dedup, bit-identical). TL Type-L/Type-E handlers (later, in
`OptimizeSP_TL_Incre`) call the same primitive.

**Why A over B (reuse `OptimizeIncre` as-is):** B works for Type-L but CANNOT do
Type-E — env changes cancel in `FindTaskWithDifferentEt`'s diff (both sides derive
from the same `dag_tasks_` member) → returns 0 → nothing re-searched → env-changed
priorities stay frozen. Type-E is the primary quality win; only A's explicit
`task_id` param can express "re-search task X though the diff says nothing."

### Contract (user-refined 2026-07-16)

1. **`baseline_sp` is an OPTIONAL arg on `OptimizeIncre`** (default `INT_MIN` =
   "not provided" — reuses the existing uninitialized sentinel from
   `OptimizeSP_Base.h:59` constructor). Not provided → score the carried PA at the
   former `:243-244`. Provided → skip that eval. If provided it MUST equal
   `EvaluateSPWithPriorityVec(dag_tasks_update, sp_parameters_, opt_pa_)` for the
   exact `dag_tasks_update` + `opt_pa_`, else the strict-`>` adopt test compares
   against a wrong seed. **The primitive does NOT take `baseline_sp`** — it TRUSTS
   the `opt_sp_` member (caller-set): `OptimizeIncre` scores it at `:243-244`; a TL
   handler seeds it via `BuildChallengerFromIncumbent`; for Type-E it injects a
   fresh new-env baseline into the member before calling.
2. **`dag_tasks_` advance (`:305`) is orchestrator-owned.** The primitive does NOT
   advance; `OptimizeIncre` keeps `:305` for bit-identity (inert under P0.5 — the
   challenger is rebuilt each step); TL handlers own their own advance.

### Implementation (`OptimizeSP_Incre.h` / `.cpp`)

- `.h`: `OptimizeIncre` gains the `double baseline_sp = INT_MIN` default; new
  `OptimizeIncre_SingleTask(dag_tasks_update, task_id, et_increased)` decl with a
  contract comment (trusts `opt_sp_`, no `dag_tasks_` advance).
- `.cpp`: `OptimizeIncre_SingleTask` = the extracted `:274-292` body verbatim
  (`FindPriorityVec1D_Variations` + `EvaluateSPWithPriorityVec` + strict-`>` adopt,
  mutates `opt_pa_`/`opt_sp_` in place, returns `opt_pa_`). `OptimizeIncre`'s loop
  becomes `OptimizeIncre_SingleTask(dag_tasks_update, d.task_id, d.increase)`; the
  baseline seed becomes the `baseline_sp == INT_MIN ? Eval(...) : baseline_sp`
  ternary. The `[INCR-NDIFF-PROBE]` debug seam and `:305` advance are unchanged.

### Bit-identity gate (verified)

`cmake --build build --target check.SP_OPT -j5` (DEBUG) + `ctest` 16/16 GREEN.
`testOptimizeIncrePA` (#9, incl. the e2e `GetPriorityAssignments_IncrementalOpt`
result assertion) + `testIncreOpt_w_TL` (#7, the production TL caller inheriting
the refactor) both pass unchanged → `OptimizeIncre`'s adopted results are
bit-identical. (The loop `OptimizeIncre → OptimizeIncre_SingleTask → …` threads
the same `opt_pa_`/`opt_sp_` member state as the old inlined body, so the only
behavioral change is the already-committed `exclude_opt_pa=true` from `33b2270c`.)

### Differential test (the Phase 1 TDD gate)

New test `OptimizeIncre_SingleTask.Differential_BitIdenticalOnSingleEtChange`
(`tests/testOptimizeIncrePA.cpp`): on the v22→v23 pair (a confirmed `|diff|==1`
case — task 1, increase, per the `FindTaskWithDifferentEt` test), two independent
optimizers from the same deterministic `OptimizeFromScratch(2)` state. Path A =
full `OptimizeIncre(dag_update)`. Path B = primitive alone with the SAME baseline
seed `OptimizeIncre` would compute (`optB.opt_sp_ = Eval(dag_update, optB.opt_pa_)`
then `OptimizeIncre_SingleTask`). Asserts `pa_full == pa_primitive` and
`optA.opt_sp_ == optB.opt_sp_` (exact double). Proves the extraction is
behavior-preserving — a refactor, not new behavior. PASSES.

### Future idea (user, NOT first cut)

Replace the exhaustive half-range priority insertion with a trial-and-error
priority walk (mirroring `OptimizeSingleTaskTimeLimit`'s patience-bounded outward
TL walk). Caveat: priority-position→SP is non-monotonic (multi-modal — moving task
X up reduces X's interference but increases it for the jumped-over task, so
aggregate SP can move either way), so a greedy walk risks local optima that
exhaustive finds; modest saving at N=4–10 (~2–5 evals/task). No signature impact —
the primitive's variation-generation stays an internal swappable detail.

### State

LANDED + TESTED in the working tree, NOT committed (user commits). **D1 (sub-point
1 + main) fully done.** D2/D3/D4/dedup/baseline/D5 + the Type-L `et_increased`
direction sharpening (actual avg-ET delta, not TL-sign heuristic) still open for
the decisions pass. P1.9 stays ON HOLD behind P1.10.

## 2026-07-17 — D2/D3/D4 decisions pass (decision-only, no code)

Per constraint (a) "API design first / no source code until the decisions pass": D1
(sub-point 1 + main) was already done; this session resolved D2, D3, D4. D5 remains
open (the next decision). No source code written.

### D2 — `FindTaskWithDifferentEt` UNCHANGED; caller normalizes (user correction)

Authoritative user correction: "`FindTaskWithDifferentEt`'s implementation does its
job. The issue is that the **caller** of `FindTaskWithDifferentEt` should make sure
`dag_tasks_updated` and `dag_tasks` have the same ET for tasks with flexible ET."

This overtURNS the earlier idea (this session, pre-correction) of adding a
mask/exclusion parameter to `FindTaskWithDifferentEt`. Concretely:

- `FindTaskWithDifferentEt` (`OptimizeSP_Incre.cpp:140-155`, `.h:59`) is UNCHANGED —
  no signature change, no behavior change, no new param.
- `OptimizeIncre`'s call site at `.cpp:282` stays exactly as-is (no mask arg).
- Base `OptimizeIncre` stays TL-unaware (user-stated boundary: "base optimizeincre is
  not designed to be aware of TL, this is the responsibility of the caller of
  optimizeincre"). TL-awareness is the caller's job.

The caller-ensures-same-ET property holds **by construction at the capture site**,
not via an extra normalization pass:

- `dag_tasks_prev_pre_tl` = `dag_tasks_` captured BEFORE the `:313` (or `:452`)
  absorb. At that point `dag_tasks_` is the previous interval's pre-TL env DAG (TLs
  went into transient `dag_tasks_cur` last interval, never the member — verified:
  `UpdateExtDistBasedOnTimeLimit` returns a fresh local `:145-146`;
  `BuildChallengerFromIncumbent` does the same `:397`; nothing mutates `dag_tasks_`'s
  `execution_time_dist` in place for TL-flexible tasks).
- `dag_tasks_cur` = `dag_tasks_update` (orchestrator-passed new pre-TL env DAG).
- Both sides pre-TL → under the generator invariant "TL-flexible tasks have no env
  dependence by design," every TL-flexible task's `execution_time_dist` is EQUAL on
  both sides → `FindTaskWithDifferentEt` never flags them → the surviving diff is
  pure env (exactly the Type-E set, with direction).

**This RETIRES the previously-proposed retained member `dag_tasks_prev_interval_pre_tl_`**
(the "New state" section of `api_design.md`). The prev-DAG is a LOCAL captured at the
top of the entry, used for the one Type-E diff, then discarded — not a retained
member. Cleaner than the mask approach: zero blast radius (no signature to defend, no
test fixtures to update — the `FindTaskWithDifferentEt` tests at
`tests/testOptimizeIncrePA.cpp:147-160` and `:423-451` stay green unchanged), base
`OptimizeIncre` untouched.

**Verified along the way (code-grounded, read-only):** the existing TL walk already
satisfies "TL-flexible tasks match on both diff sides" by construction —
`OptimizeSingleTaskTimeLimit` changes ONE task's TL per step, so
`FindTaskWithDifferentEt(challenger.dag_tasks_, dag_tasks_cur)` flags only the walked
task. So the correction requires NO change to the existing walk; only the NEW
serialized Type-E computation must capture the local prev-DAG at the pre-TL site.

**Sub-point still OPEN (coupled to D5):** the local capture must happen at BOTH
absorb entries (`OptimizeIncre_w_TL :313` AND `ReOptimizePeriodic :452`, identical
absorb) for the first post-reopt interval's Type-E diff to not compare against a
stale pre-reopt prev-DAG. If D5 = incremental-only first cut, only `:313` is captured
and the first post-reopt interval suppresses Type-E (acceptable per the open
sharpening). D5 decides this.

### D3 — sort merged E+L queue by task weight descending

User: "for sorting, we'll first use a simple sorting function based on tasks'
weights." Uniform key (no E/L tier); high-weight tasks optimized first. Supersedes
the earlier "reuse `TaskSortingHeuristic`" proposal (a weight/threshold/id composite);
weight alone for the first cut, full heuristic may return in a future cut.

### D4 — running-adopted single champion (`res_opt_`)

`res_opt_` (the P0.5 single durable incumbent: PA/TL/SP) is the initial champion at
interval start, seeded from the previous interval's committed results. Each
serialized step compares-and-keeps via the existing `UpdateRecords`/`CommitIncumbent`
(`OptimizeSP_TL_Incre.cpp:105`/`:379`); new champion vs keep-old treated identically
(the strict-`>` adopt test governs). Matches the existing pattern. Interval-start
champion (each step diffs against the original) NOT adopted.

### State

D1 (sub-point 1 + main) done + LANDED (D1-main staged not committed; user commits).
D2/D3/D4 DECIDED this session (recorded in `tasks.md` + `api_design.md`). D5 OPEN
(the next decision). Phase 2 source code (Type-E local capture, serialized loop,
caller normalization) BLOCKED until D5 passes. P1.9 stays ON HOLD behind P1.10.

### D5 — incremental-only first cut (2026-07-17)

User clarified the seed model and asked whether there's a further issue. Verified
against source (`OptimizeSP_TL_Incre.cpp`):

- Incremental (`OptimizeIncre_w_TL` :311) and reopt (`ReOptimizePeriodic` :445) DO
  share the champion seed: both absorb `dag_tasks_ = dag_tasks_update` identically
  (:313 / :452), both seed `time_limits = ReconstructTimeLimitVecFromResOpt()`
  (:324 / :471, reopt falls back to `InitializeTimeLimitsFromETConfig` at interval 0),
  both call the same descent driver `PerformCoordinateDescentForTaskConfigOpt`.
  The user's "both share the same champion config" is correct — for the TL seed.

- The DIFFERENCE is what each candidate eval does to the PA, via `from_scratch`:
  - Incremental (`from_scratch=false`): `EvaluateTimeLimitConfig_ScratchOrIncre`
    builds a challenger via `BuildChallengerFromIncumbent` (:160) and calls
    `OptimizeIncre` (:161) — WARM-STARTED 1D: start from champion's PA, re-search
    only the changed task's priority.
  - Reopt (`from_scratch=true`): `EvaluateTimeLimitConfig_ScratchOrIncre` builds a
    FRESH `OptimizePA_Incre` (:151) and calls `OptimizeFromScratch(K)` (:152) —
    MEMORYLESS full-beam: ignore champion's PA, re-search the whole assignment. The
    champion is the YARDSTICK (compare-and-keep via `UpdateRecords`), not the start.

So in reopt the champion is the seed for the TL walk + the yardstick, but NOT the
seed for the PA search. That is the point of reopt: periodically escape incumbent
structure. The serialized loop's primitive (`OptimizeIncre_SingleTask`) is warm-
started 1D — it maps onto the incremental path but does NOT fit reopt's memoryless
PA search. Plugging it into reopt would make reopt warm-started → a SEMANTIC change
to what reopt is, not a loop rewrite.

Additionally Type-E does not apply to reopt at all: Type-E is "an env-changed task
whose priority is frozen and needs re-search," but reopt re-searches ALL priorities
every candidate via `OptimizeFromScratch` → nothing frozen → Type-E already handled
(over-handled). Type-E is purely an incremental-path concept.

⇒ D5 = incremental-only first cut. `ReOptimizePeriodic` stays memoryless full-beam
(the periodic "cover everything" backstop; also the role the extension idea casts
for it). Serialize reopt later ONLY if A/B shows the post-reopt interval hurts AND
we're willing to change reopt's character.

### D2 sub-point DISSOLVED (premise correction, 2026-07-17)

Earlier D2 sharpening worried that "if only :313 captures the prev-DAG, the first
post-reopt interval's Type-E diff compares against a stale pre-reopt snapshot."
WRONG premise. Verified: reopt updates `dag_tasks_` to its own env at :452, just as
incremental does at :313 (both are the ONLY writers of the `dag_tasks_` member;
`BuildChallengerFromIncumbent` :394-402, `UpdateExtDistBasedOnTimeLimit` :145-146,
`ResetIncumbentBaseline` :416-443 all use fresh LOCALs, never mutate the member).
So at the start of any interval T, before its own absorb, `dag_tasks_` already
holds interval T-1's env (whichever path T-1 took) → `dag_tasks_prev_pre_tl =
dag_tasks_` captured before :313 gives T-1's env → the Type-E diff T-1→T is correct
across the reopt boundary for free. NOT stale, NOT suppressed. (And reopt doesn't
compute Type-E anyway — D5.) ⇒ capture at :313 only; the "capture at both entries"
sub-point is dissolved.

### State (after D5)

ALL FIVE D-questions DECIDED (D1 LANDED, D2/D3/D4/D5 decided this session). D2
sub-point dissolved. Remaining OPEN (non-D, lower-stakes): dedup policy (#5), baseline
eval choice (#6). Phase 2 source (Type-E local capture at :313, serialized loop on
the incremental path) now UNBLOCKED — but per constraint (a) await explicit go from
the user before writing source. P1.9 still ON HOLD behind P1.10.

### Remaining non-D open points DECIDED 2026-07-17 (#5 dedup, #6 baseline eval)

Both remaining non-D open points closed (decision-only, no code; constraint (a)).

**#5 Dedup policy — RaiseError, NOT a winner-pick.** User: "raise an error if a task
is both env dep and TL flexible. i don't consider this case in this project." By
generator design TL-flexible tasks have NO env dependence, so the Type-E and Type-L
sets are DISJOINT BY CONSTRUCTION — a task appearing in both is a contract violation,
not an optimization choice. So `BuildSerializedTaskQueue` (D) hard-fails
(`RaiseError`/`CoutError`) if a task_id is in both sets; it does NOT silently pick a
winner. User added: "if you really want to pick a winner, let TL fle win" — noted as
the tiebreak direction IF a winner were ever needed, but the chosen behavior is to
fail loud. Replaces the earlier "EnvChanged wins, skip TL slot" proposed default.

**#6 Baseline eval — (b) dedicated re-score, NOT `EvaluateTimeLimitConfig_ScratchOrIncre`.**
User: "don't re-use evaluateTimeLimitConfig_ScratchOrIncre, as it performs priority
optimization for env tasks, which violates our proposal to first sort tasks then
optimize in order. use (b) only re-score without optimization." This OVERTURNS the
earlier proposed default (a) = reuse the existing eval entry. Rationale: the baseline
has ndiff==0 (no single changed task) — its only job is to seed the champion's
`opt_sp_` by re-scoring the carried `{pa, tl}` under the new env DAG. But
`EvaluateTimeLimitConfig_ScratchOrIncre`'s incremental branch calls `OptimizeIncre`,
which performs PRIORITY OPTIMIZATION on env-changed tasks (`OptimizeIncre_SingleTask`
over `FindTaskWithDifferentEt`) — that optimization happens BEFORE the queue's D3
weight-sorted order is honored, violating the proposal's core shape ("sort tasks,
then optimize in order"). So the baseline is a direct re-score:
`EvaluateSPWithPriorityVec(UpdateExtDistBasedOnTimeLimit(dag_tasks_, committed_tl),
sp_parameters_, opt_pa_)` — no `OptimizeIncre`, no `BuildChallengerFromIncumbent`
rebuild. The queue walk (F) then does ALL optimization, in D3 order.

### State (after #5 + #6)

ALL DECISION POINTS CLOSED: D1 landed; D2/D3/D4/D5 decided; #5 dedup + #6 baseline
eval decided. No open design questions remain. Phase 2 source (Type-E local capture
at `:313`, serialized loop on the incremental path, caller normalization,
`BuildSerializedTaskQueue` with the RaiseError guard, dedicated baseline re-score) now
fully UNBLOCKED — per constraint (a) await explicit go from the user before writing
source. P1.9 still ON HOLD behind P1.10.

## 2026-07-17 — D2 AMENDED: filtered `FindEnvTaskWithDifferentEt` + helper landed

The 2026-07-17 D2 decision ("`FindTaskWithDifferentEt` UNCHANGED; the CALLER ensures
both diff sides have identical ET for TL-flexible tasks") rested on a premise the user
was investigating this session: that a TL-flexible task's `execution_time_dist` is
**bit-equal on both pre-TL diff sides** so `FindTaskWithDifferentEt` never flags it.
The investigation overturned that premise's robustness, so D2 is AMENDED — a structural
filter is added instead of relying on the caller to equalize ET.

### The investigation finding (code-grounded)

- `FiniteDist::operator!=` is `!operator==`, and `operator==` is
  `approx_equal(other, 1e-1)` — a **10%-relative tolerance**
  (`Probability.cpp:415-417`). Equality of a TL-flexible task's dist is therefore
  tolerance-dependent, NOT exact.
- A TL-flexible task's `execution_time_dist` IS built from the raw
  `execution_time_mu/min/max/sigma` YAML fields at read (`RegularTasks.cpp:75-79`):
  `FiniteDist(gauss, min, max, granularity)`. There is NO read-time override to the
  adopted TL for perf-pair tasks — they are not special-cased on read. So the raw YAML
  fields DO flow into `execution_time_dist`, and (per the 10% tolerance) can compare
  unequal across intervals for TL-induced (perf-pair grid) reasons, not env reasons.
- This makes the D2 "caller normalizes" property fragile: it holds only when the
  perf-pair dists happen to fall within 10% of each other on both sides. A structural
  filter (drop TL-flexible tasks from the diff) removes the dependence on that
  tolerance entirely.

### What landed (in the working tree, NOT committed — user commits)

Two new free functions in `OptimizeSP_Incre.h` / `.cpp` (sibling to
`FindTaskWithDifferentEt`, NOT replacing it):

- **`FindTasksWithFlexibleTimeLimits(const DAG_Model&)` → `std::vector<int>`** — task
  IDs with a non-empty `timePerformancePairs` (the perf-pair grid). Mirrors the
  `{-1}`-sentinel test in `RecordTimeLimitOptions` (`OptimizeSP_TL_BF.cpp:29-30`):
  tasks WITHOUT pairs get the `{-1}`-only option set (no TL freedom); tasks WITH pairs
  are the TL-flexible set the serialized Type-L step walks. Pure query; no mutation.
- **`FindEnvTaskWithDifferentEt(const DAG_Model&, const DAG_Model&)` → `std::vector<DiffObj>`**
  — the Type-E (env-changed) diff: `FindTaskWithDifferentEt(prev, cur)` MINUS the
  TL-flexible set. The env signal survives cleanly without depending on bit-equal
  perf-pair dists.

`FindTaskWithDifferentEt` itself is **UNCHANGED** — kept for `OptimizeIncre`'s
`.cpp:282` call site on the **live TL-walk path** (`EvaluateTimeLimitConfig_ScratchOrIncre`
`from_scratch=false` → `BuildChallengerFromIncumbent` → `OptimizeIncre`). There the
diff MUST keep flagging the TL-walked (TL-flexible) task so `OptimizeIncre` re-searches
its 1D priority each TL step; filtering there would empty the diff and stop the
mid-walk priority re-search — a behavior change to the live incremental path. A single
filtered function cannot serve both the Type-E diff (filter TL-flexible) and the TL
walk (flag the TL-walked TL-flexible task), so the two are separate functions. The TL
walk migrates to `OptimizeIncre_SingleTask(task_id, …)` in Phase 2 (D5), at which point
`:282`'s TL-walk usage of `FindTaskWithDifferentEt` is retired — but that is Phase 2,
not this step.

### Tests landed (3 new cases in `tests/testOptimizeIncrePA.cpp`)

- `FindTasksWithFlexibleTimeLimits` — v19: only TSP (task 0) has
  `performance_records_time` → flexible set `{0}`.
- `FindEnvTaskWithDifferentEt_filtersTLFlexible` — v19→v21 moves two dists: TSP
  (task 0, TL-flexible, 1500.9→400.9) and SLAM (task 3, NOT TL-flexible, 2853→285).
  Full diff = `{0,3}`; env filter drops the TL-flexible TSP → `{3}` (decrease). The
  differential proves the filter removes the perf-pair mover and keeps the env mover.
- `FindEnvTaskWithDifferentEt_noopWhenFlaggedNotTLFlexible` — v22→v23 moves MPC
  (task 1, not TL-flexible): full == env (the filter never drops a non-TL-flexible
  mover — no over-filtering). v22→v24 (no movers): both empty.

### Gate (Phase 1 bit-identity)

`cmake --build build --target check.SP_OPT -j5` (DEBUG, uppercase — `libSP_OPTDebug.so`)
+ `ctest` 16/16 GREEN. The 3 new cases pass explicitly (not skipped). `testOptimizeIncrePA`
(#9) + `testIncreOpt_w_TL` (#7, the production TL caller — `FindTaskWithDifferentEt`
unchanged) both green unchanged → no behavior change to the live incremental path.

### State

LANDED + TESTED in the working tree, NOT committed (user commits). **D2 AMENDED** (the
2026-07-17 "caller normalizes" decision is superseded by a structural filter, because
`FiniteDist::operator!=` is 10%-relative `approx_equal` — the caller-equal-ET premise
is fragile). `api_design.md` + `tasks.md` D2 records marked superseded below. Phase 2
(Type-E local capture at `:313` using `FindEnvTaskWithDifferentEt`, serialized loop,
`BuildSerializedTaskQueue` with the RaiseError guard, dedicated baseline re-score) still
UNBLOCKED — await explicit go. P1.9 still ON HOLD behind P1.10.

## 2026-07-17 — Phase 1 D1-main extraction LANDED + COMMITTED (`3d2f9b28`)

Re-verified after the context summary that the D1-main extraction is **already
committed**, not just staged. Commit `3d2f9b28` ("refactor optimizeSP_Incre,
introduce single-task optimization sub function", 2026-07-17 18:42) contains:

- `OptimizeIncre_SingleTask(dag_tasks_update, task_id, et_increased)` extracted
  from `OptimizeIncre`'s former `:274-292` loop body (`OptimizeSP_Incre.cpp`). It
  TRUSTS `opt_sp_` (caller-set), generates the 1D variations via
  `FindPriorityVec1D_Variations` (+ `AnalyzePriorityChangeStatus`), scores each
  with `EvaluateSPWithPriorityVec`, adopts on strict `>`, mutates `opt_pa_`/
  `opt_sp_` in place, does NOT advance `dag_tasks_`. Comment: "Bit-identical to
  the former `:274-292` loop body."
- `OptimizeIncre` gained the optional `baseline_sp` arg (default `INT_MIN` =
  score the carried PA under the new env, the former `:243-244`; provided → skip
  the re-score) and its `:274` loop now DELEGATES to `OptimizeIncre_SingleTask`
  instead of inlining the body.
- The differential TDD test `OptimizeIncre_SingleTask.Differential_BitIdenticalOnSingleEtChange`
  (`tests/testOptimizeIncrePA.cpp:381`) — on the v22→v23 `|diff|==1` pair (task 1,
  increase), two optimizers seeded identically via `OptimizeFromScratch(2)`: path A
  = full `OptimizeIncre(dag_update)`, path B = primitive alone with the SAME
  baseline seed. Asserts `pa_full == pa_primitive` (exact) and
  `optA.opt_sp_ == optB.opt_sp_` (DOUBLE_EQ). Proves the extraction is
  behavior-preserving (refactor, not new behavior — Type-L).

### Gate (Phase 1 re-verified 2026-07-17)

`cmake --build build --target check.SP_OPT -j5` (DEBUG) + `ctest` 16/16 GREEN on
HEAD `3d2f9b28` + the uncommitted D2-amendment working tree together.
`testOptimizeIncrePA` (#9, incl. the differential test) + `testIncreOpt_w_TL`
(#7, the production TL caller) both PASS unchanged → `OptimizeIncre`'s adopted
results are bit-identical pre/post extraction; only the eval count drops by 1
per changed task (the `exclude_opt_pa=true` carry-position skip from `33b2270c`).

### Phase 1 status: COMPLETE. `tasks.md` Phase 1 checkboxes updated to `[x]`.

Phase 2 (the serialized interval search loop) remains the only outstanding P1.10
work — still UNBLOCKED design-wise but **awaiting explicit user go** per
constraint (a). The D2-amendment functions (`FindTasksWithFlexibleTimeLimits` +
`FindEnvTaskWithDifferentEt`) + their 3 tests remain uncommitted in the working
tree (landed-ahead-of-use; no caller until Phase 2).


## 2026-07-17 (later) — Phase 2 IMPLEMENTED + TDD GREEN (serialized loop)

Phase 2 (the serialized interval search) is now IMPLEMENTED in the working tree,
behind a compile-time dev flag, with a passing differential TDD test. **NOT
committed** (pending user go / review). 16/16 ctest green with the flag OFF
(legacy path bit-identical) AND with the flag ON (new path reaches the same
result on the differential case).

### What landed (sources/Optimization/OptimizeSP_TL_Incre.h + .cpp)

1. **Type-L eval injection (task #1, G):** `OptimizeSingleTaskTimeLimit` split
   into a thin 7-arg wrapper (binds `eval` to `EvaluateTimeLimitConfig_ScratchOrIncre`,
   capturing `K`/`from_scratch`) + `OptimizeSingleTaskTimeLimit_Impl` (the
   patience-bounded outward walk core taking a `std::function<double(const
   std::vector<double>&)> eval`). Behavior-preserving — the wrapper reproduces
   the former inline `EvaluateTimeLimitConfig_ScratchOrIncre` call. The
   serialized Type-L step reuses `_Impl` with a sub-incremental eval.
2. **`EvaluateTimeLimitConfig_SubIncremental` (task #2, E):** mirrors the
   incremental branch of `EvaluateTimeLimitConfig_ScratchOrIncre` but calls
   `OptimizeIncre_SingleTask(task_idx, et_increased)` instead of `OptimizeIncre`.
   Virtual (for stub tests). `K` unused (the primitive re-searches one task's
   1D positions — no beam).
3. **`CollectTLFlexibleTaskIds` (C) + `BuildSerializedTaskQueue` (task #3, D):**
   the merged E+L queue. Type-E via `FindEnvTaskWithDifferentEt(prev_pre_tl,
   dag_tasks_)`, Type-L via `CollectTLFlexibleTaskIds`, sorted together by task
   WEIGHT DESCENDING (D3, `std::stable_sort`). `CoutError` on E∩L overlap (#5).
4. **`PerformSerializedTaskQueueOptimization` (task #4, F):** the incremental-
   path driver. `ResetIncumbentBaseline(false)`; DEDICATED baseline re-score (#6
   — `EvaluateSPWithPriorityVec(UpdateExtDistBasedOnTimeLimit(dag_tasks_,
   committed_tl), sp_parameters_, opt_pa_)` directly, NO `OptimizeIncre`/challenger
   rebuild — must not optimize before the queue's sorted order is honored);
   `BuildSerializedTaskQueue`; per-entry dispatch — EnvChanged →
   `EvaluateTimeLimitConfig_SubIncremental` with committed TL + env direction;
   TLFlexible → `OptimizeSingleTaskTimeLimit_Impl` (backward then forward from
   the same origin) with eval bound to SubIncremental (et_up = trial TL >
   committed TL). Working TL refreshed from `res_opt_` after each step.
5. **`use_serialized_incremental_opt` flag** (`sources/Utils/Parameters.h/.cpp`):
   compile-time dev flag, default `false`, NOT YAML-backed (avoids forcing a new
   key across every config; opt-in until the invariant is proven + A/B'd).
   Wired in `OptimizeIncre_w_TL` — captures `dag_tasks_prev_pre_tl = dag_tasks_`
   BEFORE the `:313` absorb (the T-1 env DAG for the Type-E diff), then routes
   to F (flag on) or the legacy `PerformCoordinateDescentForTaskConfigOpt` (off).

### The bug the TDD test caught (and the fix)

The differential test `TaskSetForTest_robotics_v19.SerializedIncremental_NoWorseThanLegacy`
(bootstrap both optimizers via `ReOptimizePeriodic(v19)`, warm-start with v21;
assert serialized SP ≥ legacy + both reach TSP TL=1000) FAILED first: the
serialized walk stopped at TL=600 (SP 12.5296) instead of climbing to 1000
(SP 12.9296).

**Root cause:** `OptimizeIncre_SingleTask` TRUSTS `opt_sp_` as its baseline (it
does NOT re-score the carried PA — that's the caller's job, mirroring
`OptimizeIncre`'s `:308-311`). `BuildChallengerFromIncumbent` seeds
`challenger.opt_sp_` from `res_opt_.sp_opt` — the champion's SP at the
CHAMPION's TL. But `dag_tasks_cur` carries the TRIAL TL, so that `opt_sp_` is
STALE w.r.t. the candidate DAG. The primitive's strict-`>` adopt test then
measures priority variations against a stale baseline: a TL step that improves
SP *via the carried PA itself* (no priority variation beats the stale champion
SP) is missed → the walk sees "no improvement" → patience=0 → early break.

**Fix (in E):** re-score the carried PA under `dag_tasks_cur` to seed
`challenger.opt_sp_` with the genuine new-TL baseline BEFORE calling
`OptimizeIncre_SingleTask` — exactly what `OptimizeIncre` does at `:308-311`.
This is the ONE re-score the sub-incremental path keeps (the redundant
carried-POSITION *variation* is still dropped at the generator via
`exclude_opt_pa=true`). Uniformly correct for both Type-L (trial TL ≠ champion
TL) and Type-E (env ≠ champion env, TL same). After the fix: test GREEN,
serialized path reaches TL=1000, SP 12.9296 == legacy.

### Gate

`cmake --build build --target check.SP_OPT -j5` (DEBUG) + `ctest` 16/16 GREEN.
`testIncreOpt_w_TL` now 50 tests (was 49; +1 differential). Flag OFF → legacy
bit-identical; flag ON → differential passes. `testOptimizeIncrePA` unchanged.

### Status

Phase 2 functionally COMPLETE + TDD green (NOT committed). Phase 3 (instrument
the single-change invariant `|diff|==1` at every SP-eval; unblock P1.9) remains.
The flag stays default-OFF in production until Phase 3 + an A/B show the
serialized path is safe + beneficial.


## 2026-07-17 (latest) — Phase 3 IMPLEMENTED + TDD GREEN (single-change invariant proven; P1.9 UNBLOCKED)

Phase 3 (the single-change invariant proof — P1.9's unblock condition) is now
DONE: the invariant is instrumented (debugMode-gated) + proven by a passing TDD
test. **P1.9 is unblocked** (its STATUS flipped ON HOLD → resume). All P1.10
design work is closed; the only remaining P1.10 item is committing the Phase 2 +
Phase 3 working tree (user's call) + the eventual A/B.

### The instrumentation landed (`sources/Optimization/OptimizeSP_TL_Incre.h` + `.cpp`)

- **`AssertSingleChangeInvariant(champion_dag, candidate_dag, task_idx)`**
  (private const method on `OptimizePA_Incre_with_TimeLimits`), called inside
  `EvaluateTimeLimitConfig_SubIncremental` right after `BuildChallengerFromIncumbent`
  + `dag_tasks_cur` are both in hand. Computes
  `FindTaskWithDifferentEt(champion.dag_tasks_, dag_tasks_cur)` and enforces:
  - `|diff|==0` → OK (Type-E: env move cancels — see correction below).
  - `|diff|==1` flagging `task_idx` → OK (Type-L: the walked task).
  - `|diff|>1`, or `|diff|==1` flagging a *different* task → `CoutError` (champion
    drift — multiple ET changes in one eval, which would break the sub-incremental's
    single-change premise + P1.9's single-task patch).
  Gated on `GlobalVariables::debugMode` → cost-free in production.

### The correction the instrumentation surfaced (IMPORTANT — supersedes api_design)

The Phase 3 instrumentation **caught a real contradiction in the design docs on its
first run**. The original assertion demanded `|diff|==1` (flagging `task_idx`) at
EVERY serialized SP-eval, per `api_design.md`'s "single-change invariant" section
("Type-E step … `|diff|==1` by construction"). The first build threw:

```
P1.10 single-change invariant violated: serialized step for task 3 produced
|diff|=0 (flagged tasks: []).
```

This was a Type-E step (task 3 = SLAM, the env-only mover in v19→v21). The throw is
**correct behavior, not a bug** — it exposed that the "Type-E = `|diff|==1`" claim was
never updated when the D2 premise correction / amendment landed. Tracing it:

- `BuildChallengerFromIncumbent` builds the champion DAG from `dag_tasks_` + the
  committed TL (`ReconstructTimeLimitVecFromResOpt`).
- A Type-E step passes `starting_time_limits` = the committed TL (NO TL walk — the
  env move is the change). So `dag_tasks_cur` = `dag_tasks_` + committed TL.
- → champion DAG == candidate DAG → `FindTaskWithDifferentEt` = empty → **`|diff|==0`**.

This is exactly `goal.md`'s premise-correction finding #1 ("both diff sides derive
from the same `dag_tasks_` → env changes cancel"). The env move was absorbed into
`dag_tasks_` at the `:313` absorb BEFORE the champion was built, so it is on BOTH
diff sides and cancels. `api_design.md` line 457-458 ("Type-E … `|diff|==1` by
construction") was stale — it predates the premise correction and should have read
`|diff|==0`. **The instrumentation is what surfaced this** — the value of the Phase 3
TDD gate, not a defect in the path.

**Fix:** the invariant is `|diff|<=1`, not `|diff|==1`. Type-E = 0 (same DAG, only
the re-searched PA varies), Type-L = 1 (the walked task). Corrected in:
- `AssertSingleChangeInvariant` (accepts `|diff|==0` and `|diff|==1`-on-`task_idx`;
  rejects everything else).
- Its header + `.cpp` doc comments.
- `api_design.md` "The single-change invariant" (pre-correction text marked SUPERSEDED).
- `tasks.md` Phase 3 (correction block at the top).
- This dev_log entry.

### The TDD proof (`tests/testIncreOpt_w_TL.cpp`)

`TaskSetForTest_robotics_v19.SerializedIncremental_SingleChangeInvariant`:
- Forces `GlobalVariables::debugMode = 1` (arms the assertion — the proof would be
  vacuous with it off), restores it at the end.
- Bootstraps via `ReOptimizePeriodic(v19)`, then `OptimizeIncre_w_TL(v21)`. v19→v21
  moves BOTH the TL-flexible TSP task 0 (1500.9→400.9) AND the env-only SLAM task 3
  (2853→285) — per the D2 `FindEnvTaskWithDifferentEt` tests — so the queue walk
  exercises BOTH a Type-L step (TSP) and a Type-E step (SLAM).
- If ANY eval violated `|diff|<=1`, `AssertSingleChangeInvariant` throws → the test
  aborts (FAIL). Reaching the final `EXPECT_GT(res.sp_opt, 0.0)` means the invariant
  held at every SP-eval across both step kinds. PASSING (406 ms).

The pre-existing `SerializedIncremental_NoWorseThanLegacy` test also runs with
debugMode armed (`parameters.yaml` has `debugMode: 1`) → it doubles as a second
invariant witness on the single-TSP TL-walk case (Type-L only there).

### Gate

`cmake --build build --target check.SP_OPT -j5` (DEBUG, `libSP_OPTDebug.so`) +
`ctest` 16/16 GREEN. `testIncreOpt_w_TL` now 51 tests (was 50; +1 invariant proof).
`testOptimizeIncrePA` unchanged (20 tests). Flag OFF → legacy bit-identical; flag ON
→ both serialized tests pass with the invariant assertion ARMED.

### P1.9 unblocked

P1.9's STATUS (`agents/active_tasks/P1_9_incremental_rta_patching/goal.md`) flipped
ON HOLD → resume. The cache simplification P1.10 guarantees is now precise:
- `|diff|==0` (Type-E step + baseline re-score) → **full RTA reuse** (same DAG, only
  the PA varies — the RTA doesn't depend on the PA at all, so the cached RTA is
  reused as-is; not even a patch).
- `|diff|==1` (Type-L TL step) → **single-task RTA patch** (one task's ET moved).
- Never a multi-task patch. `RTAReuseClass` collapses to a binary
  {ReuseAll, PatchOneTask}; the rev-2 `CheckTaskSetRTAReuse` +
  `EvaluateRTA_WithCache` surface simplifies accordingly.

### Status (P1.10)

**Phase 3 COMPLETE.** All P1.10 design decisions closed (D1–D5 + #5 + #6), all three
phases implemented + TDD green. The single-change invariant (`|diff|<=1` at every
serialized SP-eval) is PROVEN. P1.9 is unblocked. **NOT committed** — the Phase 2 +
Phase 3 working tree (serialized loop, `use_serialized_incremental_opt` flag,
`AssertSingleChangeInvariant`, +1 test) is staged pending user review/commit. The
flag stays default-OFF in production until an A/B shows the serialized path is safe +
beneficial (separate from the invariant proof, which is done).
