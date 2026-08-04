# §8 — QoS Budget + Priority Collaborative Optimization

> Draft: `section8_task_config_opt.tex`. Status: **COMPLETE REWRITE planned**
> (supersedes the prior row-by-row plan; that plan kept the stale δ-radius
> framing and only promoted the `\agent` note — both discarded). See
> `overall_revision_plan.md` for conventions. Source of code truth:
> `sketch_optimization.md` (PW.1.2), esp. §env-task + §coord-descent.

## High-level guidance (user instruction)

§7 optimized priority only, with QoS budgets `\boldsymbol{\mathcal{Q}}` held fixed.
**§8 lifts that restriction: it incorporates QoS into PA optimization and performs
collaborative PA+QoS optimization.** The section is therefore **very different
from the existing §8 draft** (which framed QoS optimization as an isolated
δ-radius local search). The new §8 follows the **unified incremental optimization
framework** that the code actually runs:

1. **Sort tasks by SP weight** (descending).
2. **Iterate tasks one-by-one** through the serialized queue.
3. **Per-task dispatch** — for each task:
   - if it is a **QoS-budget (TL) task** (an anytime algorithm with a time-limit
     grid) → run the **trial-and-error QoS walk** (patience-bounded outward
     coordinate descent over the TL grid);
   - otherwise (it is an **environment-dependent task** whose ET changed) → call
     the **incremental PA solver** (±1 priority move, the §7.3 mechanism).

That per-task dispatch is the final algorithm §8 presents. **Major motivation of
this design: utilize incremental optimization + the RTA cache** (§7.4) — each step
changes exactly one task, so every SP evaluation is a cache hit (the `|diff| ≤ 1`
invariant), making collaborative PA+QoS optimization feasible online.

§8 needs a **complete rewrite**. It first presents this **overall flow** (the
serialized queue + per-task dispatch), then introduces **each step one-by-one**:
the task serialization, the QoS-budget trial-and-error walk, and the env-task
incremental PA move.

## Implementation log (Stage 1 .tex edits — applied, awaiting user review)

The complete §8 rewrite was applied to `section8_task_config_opt.tex` (2026-08-02).
The prior draft (two-strategy / δ-radius framing + four unresolved review-note
wrappers `\sen`×2 / `\agent` / `\rkwprev` / `\Sen`) was **replaced wholesale** with
the unified-loop presentation per the high-level guidance. Code-truth locators
re-verified against current source before writing prose:

- **§8.0 (lede + collaborative problem statement)** — REWRITE DONE. New lede
  states §8 lifts §7's QoS-fixed restriction and solves the joint problem
  `eq:joint_pa_qos` (`\max_{\mathcal{A},\boldsymbol{\mathcal{Q}}} SP`). Organizing
  idea: a QoS-budget task is a special env-dependent task (flexing `\mathcal{Q}_i`
  changes `\extDist{i}`, same as an env change), so ONE incremental framework
  handles both. Stated motivation: structuring each step to change exactly one
  task makes every SP eval a cache reuse hit (§7.4 invariant) → collaborative
  optimization feasible online. Deleted: "brute-force vs incremental" two-strategy
  framing, "smooth assumption" (content change 2 — dynamic/continuous framing),
  `eq: incremental_configuration` (δ-radius), δ=150 TSP Example, all four review-
  note wrappers.
- **§8.1 (overall flow)** — ADD DONE. `section_overall_flow`. 4-step enumerate:
  (1) build merged queue of Type-E + Type-L; (2) sort by SP weight desc; (3) per-
  task dispatch (Type-L → §8.3 QoS walk, Type-E → §8.4 incremental PA move);
  (4) adopt each step as new champion so next task's challenger differs by exactly
  one task. Motivation stated: every step satisfies `|diff|≤1` → every eval is a
  cache hit, no from-scratch RTA repeated within the walk. Matches
  `BuildSerializedTaskQueue` + `WalkSerializedTaskQueue`
  (`OptimizeSP_TL_Incre.cpp:436-484` / `:486-514`).
- **§8.2 (task serialization)** — ADD DONE. `section_task_serialization`. Defines
  Type-E (env-changed, carries `et_increased` direction) + Type-L (QoS-budget /
  anytime, discrete TL grid). States they are DISJOINT by construction (QoS-budget
  task's ET is governed by budget not env → cannot also be Type-E), matching the
  hard-fail at `:449`. Sort key = SP weight desc, stable (equal-weight E/L keep
  insertion order), matching `stable_sort` `:478-482`. States the queue uses
  weight-desc ONLY (NOT the full `TaskSortingHeuristic` weight→Θ→ID — that is the
  PA-only path's sort, `:11-42`, NOT this queue's). Initial `\boldsymbol{\mathcal{Q}}^{(0)}`
  = grid option closest to pre-optimization ET, matching
  `InitializeTimeLimitsFromETConfig` `:611` (`Find_Close_ExecutionTime`).
- **§8.3 (QoS walk)** — ADD DONE. `section_qos_walk`. Real algorithm: walk TL grid
  outward from current option, backward pass (`step=-1`) then forward (`step=+1`),
  halt at grid boundary or patience exhaustion; patience = total non-improvement
  budget (NOT reset on improvement); backward-first = resource-aware tie-break
  (smaller TL preferred on SP ties). Patience mode-selected (incremental vs
  reoptimization). Each trial TL evaluated through the incremental priority solver
  (`OptimizeIncreSingleTask`, `et_up = sign(trial−committed)`), cache-routed.
  Matches `OptimizeOneTaskWithTimeLimit` (`:696`) + `WalkOneTaskWithTimeLimitOptions`
  (`:647-694`). TSP example reframed: grid `{100,200,300,500}` from 300 tries
  200,100 then 500 — NOT "δ=150 → 2 candidates." Deleted δ-radius framing entirely.
- **§8.4 (env-task incremental PA move)** — ADD DONE. `section_env_pa_move`.
  Type-E dispatch calls §7.3 incremental priority solver directly (NO QoS walk —
  task has no TL grid). Re-searches priority position over one half, half selected
  by the four-scenario rule (§7.3) from carried direction; adopts best strictly-
  improving position. Cross-links §7.3 (`section_increment_pa`) for detail, does
  NOT duplicate the table. Matches `OptimizeIncreSingleTask` (`WalkSerializedTaskQueue:502`).
- **§8.5 (complexity + single-change property)** — RESOLVE DONE. Per-pass
  complexity `O(M·N)` (M = TL options per QoS task, N = queue tasks) vs `O(M^N)`
  exhaustive; full RTA-inclusive analysis deferred to §10
  (`section_complexity_analysis` — label VERIFIED in `section10_complexity.tex:3`,
  ref fixed from the stale `section: complexity`). Single-change property stated as
  STRUCTURAL (each loop iteration adjusts exactly one task; committed before next
  iteration → every eval `|diff|≤1` → cache reuse lemmas apply). NO theorem+proof
  (out of scope for T-ASE; would require formalizing the cache's move model).
  Deleted `\rkwprev`/`\Sen`/`\agent` wrappers.

LaTeX verified: 2/2 begin/end (1 `equation` + 1 `enumerate`); all `\ref` targets
exist (`section_priority_opt` §7, `section_et_model` §6, `section_rta_cache` +
`section_increment_pa` §7, `def_task_config` §5, `eq_important_task_constraint` §6,
`section_complexity_analysis` §10). QoS reframe (convention #6) subsumed — new
prose written in QoS-budget language from the start; the staged glyph-only §8
reframe diff is discarded in favor of this rewrite. Pre-existing repo-wide macro
breakage (`\extDist`, `\rtDist`, `\hptasks` undefined in `main.tex`) NOT in §8
scope (PW.2 preamble fix).

## Code truth (verified locators — re-grep before writing prose, lines may drift)

- **Top-level entry:** `OptimizeIncre_w_TL` (`OptimizeSP_TL_Incre.cpp:864`) →
  `RunIntervalDescent` (`:570`) → `BuildSerializedTaskQueue` (`:436`) +
  `WalkSerializedTaskQueue` (`:486`).
- **Task serialization:** `BuildSerializedTaskQueue` (`:436-484`) builds TWO
  **disjoint** sets — **Type-E** = `FindEnvTaskWithDifferentEt` (env-changed,
  `OptimizeSP_Incre.cpp:176`); **Type-L** = `CollectTLFlexibleTaskIds` (QoS-budget
  tasks). Sort key = **SP weight descending**, `stable_sort` (`:478-482`) —
  weight-desc ONLY (stable: equal-weight Type-E/Type-L keep insertion order). The
  comment "mirrors `TaskSortingHeuristic`" refers to the primary key only.
  `TaskSortingHeuristic` itself (`OptimizeSP_TL_Incre.cpp:11-42`, used by the PA-only
  path, NOT this queue) is weight-desc → **threshold (Θ_i) asc** → ID asc. (Old plan
  claimed "deadline asc" — WRONG, it's threshold asc. State weight-desc for the
  queue; mention the full heuristic only if §8.2 also describes the PA-only sort.)
- **Per-task dispatch:** `WalkSerializedTaskQueue` (`:486-514`) — THE dispatch:
  - `Kind::EnvChanged` → `OptimizeIncreSingleTask` (incremental PA solver, **no**
    QoS walk) `:502`;
  - `Kind::TLFlexible` → `OptimizeOneTaskWithTimeLimit` (trial-and-error QoS walk)
    `:508`.
- **QoS trial-and-error walk:** `OptimizeOneTaskWithTimeLimit` (`:696`) wraps
  `WalkOneTaskWithTimeLimitOptions` (`:647-694`) — **full TL grid, NO δ cap**;
  backward pass (`step=-1`) then forward pass (`step=+1`) from the current TL
  index; **patience** = total non-improvement budget (no reset on improvement;
  `:667-689`); stops at grid boundary or patience exhaustion; resource-aware
  tie-break toward smaller TL (via the backward-first ordering). Patience value is
  mode-selected: `IncrementalTimeLimitSearchPatience` vs
  `ReoptimizationTimeLimitSearchPatience` (`parameters.yaml`).
- **Env-task incremental PA:** `OptimizeIncreSingleTask` — the §7.3 ±1 priority
  move applied to the one env-changed task (carries `et_increased` direction).
- **RTA cache integration:** the walk's `eval` lambda routes every trial TL through
  the cache (`OptimizeIncreSingleTask` → `Evaluate`); each step changes exactly one
  task → `|diff| ≤ 1` holds → every eval is a reuse hit. Cache armed/disarmed via
  `rta_cache_active_` (`SeedBaselineAndArmCache` `:516`); `CommitIncumbent` adopts
  the champion after each step.
- **Convergence wrapper (offline only):** `OptimizeIncre_w_TL_UntilConvergence`
  (`:889`) loops `OptimizeIncre_w_TL` until a pass fails to strictly improve
  `opt_sp_` (`ApproxEqualSP` 1e-3, NO cap); called ONLY by `ComputeSafeFallback`
  (`:1002`, offline worst-case-DAG certification), NOT the online walk. Mention
  only if §8 covers the offline safety path; otherwise defer to §13.

## Subsection rows (new §8 structure)

### §8.0 Lede + collaborative problem statement — REWRITE
- **draft claim:** current lede (7-17) presents "two strategies (brute-force +
  incremental)" for QoS optimization in isolation, framed by the stale "smooth
  assumption." Two unresolved `\sen` notes (4-5) direct the reframing but were never
  implemented.
- **code reality:** QoS optimization is NOT a separate problem; it is fused with PA
  optimization in `OptimizePA_Incre_with_TimeLimits`. The `\sen` directive (4-5) IS
  the correct reframing: a QoS-budget task is treated as a *special kind of
  env-dependent task* — flexing its QoS budget is modeled as an ET change, so the
  same incremental machinery handles both.
- **action:** **REWRITE** the lede to implement the `\sen` reframing (delete the
  `\sen` notes, promote content into body): §8 lifts §7's QoS-fixed restriction and
  performs **collaborative PA+QoS optimization**. State the joint problem
  (`\max_{\mathcal{A},\boldsymbol{\mathcal{Q}}} \textbf{SP}`). Introduce the
  organizing idea: a QoS-budget task is a special env-dependent task, so ONE
  incremental framework handles both QoS flex and ET change. Delete the
  "brute-force vs incremental" two-strategy framing and the "smooth assumption"
  (content change 2 — use dynamic/continuous framing). **DELETE** the stale
  `eq: incremental_configuration` (δ-radius) and the δ=150 TSP Example entirely
  (they describe an algorithm the code does not run).

### §8.1 Overall flow: serialized queue + per-task dispatch — ADD (new body)
- **draft claim:** no such description exists; the algorithm was buried in an
  `\agent` margin note (37-49).
- **code reality:** the unified loop = `BuildSerializedTaskQueue` →
  `WalkSerializedTaskQueue` (locators above).
- **action:** **ADD** the overall flow as the section's spine. Present it as a
  high-level procedure (numbered list or Algorithm 2): (1) build the merged queue
  of env-changed (Type-E) + QoS-flexible (Type-L) tasks; (2) sort by SP weight
  descending; (3) for each task, dispatch by kind — QoS task → trial-and-error walk;
  env task → incremental PA move; (4) each adoption updates the champion, so the
  next task's eval sees the new incumbent. **State the motivation explicitly:**
  structuring the loop so each step changes exactly one task makes every SP
  evaluation an RTA-cache hit (`|diff| ≤ 1`), which is what makes collaborative
  optimization feasible online. Forward-reference §7.4 (cache) and §7.3 (the ±1 PA
  move reused for env tasks).

### §8.2 Task serialization — ADD
- **draft claim:** not present (was inside the `\agent` note as "prioritized task
  ordering: weight desc, deadline asc, ID asc").
- **code reality:** `BuildSerializedTaskQueue` (`:436-484`); weight-desc stable
  sort; Type-E and Type-L are disjoint by construction (a task in both is a
  contract violation, hard-fail `:449`).
- **action:** **ADD** a subsection: define Type-E (env-dependent, ET changed) and
  Type-L (QoS-budget / anytime) tasks; state they are disjoint; state the sort key
  (SP weight descending — the queue uses weight-desc stable ONLY; the full
  `TaskSortingHeuristic` weight→Θ→ID is the PA-only path's sort, not this queue's);
  justify weight-desc (optimize the highest-impact task first so its adoption
  informs the rest). Promote the relevant content from the old `\agent` note (init
  `\boldsymbol{\mathcal{Q}}^{(0)}` from the closest valid TL option —
  `InitializeTimeLimitsFromETConfig` `:611`).

### §8.3 QoS-budget trial-and-error walk — ADD (replaces δ-radius framing)
- **draft claim:** `eq: incremental_configuration` δ-radius local search +
  δ=150 TSP Example. STALE — code does not use δ.
- **code reality:** `OptimizeOneTaskWithTimeLimit` + `WalkOneTaskWithTimeLimitOptions`
  (`:647-694`) — patience-bounded outward coordinate descent over the FULL TL grid,
  no δ cap (locators above).
- **action:** **ADD** the real algorithm. For a QoS-budget task, walk its TL-option
  grid outward from the current budget: backward pass then forward pass, evaluating
  SP at each option (via the cache-routed eval), keeping the best; the walk halts at
  the grid boundary or when the **patience** budget (non-improving steps) is
  exhausted. State the tie-break (smaller TL preferred, via backward-first
  ordering — conserves compute). Keep the TSP anytime-algo example BUT reframe it:
  the TL grid is `[100,200,300,500]`; from a current budget of 300 the walk tries
  200,100 (backward) then 400.../500 (forward), not "δ=150 → 2 candidates." **DELETE**
  `eq: incremental_configuration` and the δ=150 framing entirely.

### §8.4 Env-task incremental PA move — ADD (cross-link to §7.3)
- **draft claim:** not present as a §8 step (the four-scenario table lives in §7.3).
- **code reality:** `OptimizeIncreSingleTask` — the §7.3 ±1 priority move applied to
  the env-changed task, carrying the `et_increased` direction.
- **action:** **ADD** a short subsection: for an env-dependent task (ET changed
  between intervals), the dispatch calls the §7.3 incremental PA solver (±1 priority
  move, four scenarios) — NO QoS walk. This is the unification the `\sen` directive
  asked for: both task kinds flow through one serialized loop, differing only in the
  per-task handler. Cross-link §7.3 for the four-scenario detail; do not duplicate it.

### §8.5 Complexity + the single-change property — RESOLVE (delete `\rkwprev`/`\Sen`/`\agent`)
- **draft claim:** `\rkwprev` (51) asks "can we prove anything?"; `\Sen` (52-53)
  defers complexity to §10 and says quality is "very hard"; `\agent` (56-58) claims
  the 1-task-ET-diff property and says "add a theorem."
- **code reality:** complexity IS in §10. The 1-task-per-step property is enforced
  *structurally* by the loop (each iteration changes exactly one task) and asserted
  by the RTA cache (`ComputeTaskSetDifference` throws on `|diff|>1`,
  `RTA_Cache.cpp:358`). It is a stated property backed by a structural invariant,
  NOT a theorem-with-proof.
- **action:** **RESOLVE** the three notes: delete the `\rkwprev`/`\Sen`/`\agent`
  wrappers; state the per-pass complexity as `O(M·N)` (M = TL options per QoS task,
  N = tasks in the queue) with a one-line argument and a forward-pointer to §10 for
  full analysis; state the **single-change property** — each step changes exactly one
  task, so every SP evaluation is an RTA-cache reuse hit — as a stated
  property/observation backed by the cache's `|diff| ≤ 1` invariant (cite the
  invariant, §7.4). Do **NOT** add a formal theorem+proof (out of scope for T-ASE;
  would require formalizing the cache's move model).

## QoS reframe note (content change 6)

§8 is heavily affected by the QoS reframe (already staged as a glyph-only change):
section title, lede, `eq: incremental_configuration`, TSP Example, and the
`\agent` note all reframe to **QoS budget** language with symbol
`\boldsymbol{\mathcal{Q}}` / `\mathcal{Q}_i`. The complete rewrite above **subsumes**
that staged reframe — the new prose is written in QoS-budget language from the
start. The staged §8 QoS-reframe diff can be discarded in favor of this rewrite
(commit the rewrite, not the glyph-swap-on-stale-framing).

## What is REMOVED from the prior §8 plan (superseded)

The prior row-by-row plan (lede/env-task reframing; δ-radius replacement; `\agent`
promotion; complexity resolution) is **superseded** by the five subsection rows
above. The prior plan kept the section's two-strategy / δ-radius *structure* and
only patched within it; the user instruction replaces the structure entirely with
the unified-loop presentation. Specific discards:
- "two strategies (brute-force + incremental)" framing → REMOVED (§8.0 rewrite).
- `eq: incremental_configuration` (δ-radius) + δ=150 Example → DELETED (§8.3).
- "smooth assumption" → DELETED, replaced by dynamic/continuous framing (§8.0).
- promoting the `\agent` note *in place* → REPLACED by promoting its *content* into
  the new §8.1/§8.2/§8.3 body structure.
