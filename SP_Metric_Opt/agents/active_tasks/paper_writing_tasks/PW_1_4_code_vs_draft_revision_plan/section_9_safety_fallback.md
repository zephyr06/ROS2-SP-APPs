# §9 — Safety Fallback (Important-Task Guarantee)

> Draft: **NEW SECTION** (no existing `.tex`; carved out of the guarantee content
> the old §9-software-impl and §13.3 allude to). Status: **ADD (new section)**.
> See `overall_revision_plan.md` for conventions. Source of code truth:
> `sketch_fallback.md` (PW.1.3). The spine = the user's three questions: **(1) why
> we need a safe fallback, (2) how to find one, (3) when to trigger it.**

## High-level guidance (user instruction)

New §9 dedicated to the **safety fallback** for important tasks. It answers, in
order:

1. **Why** we need a safe fallback.
2. **How** to find a safe fallback.
3. **When** to trigger the safe fallback.

The section states the **important-task safety-performance guarantee** as a
*self-guarantee* of the framework (NOT conditional on a user-supplied ET upper
bound — that prior framing is DRIFT, see §13.3 / §11 rows). The guarantee
predicate is **Option A**: every important task satisfies `Pr(r_i > D_i) ≤ Θ_i`
at every shipped interval. Scope = important tasks only (top-50% by `sp_weight`);
non-important tasks' schedulability is traded for SP.

This section is the **methodology home** for the guarantee. §13.3 (analysis)
becomes the *discussion* of it; §11 (limitations) drops the now-answered
"how to provide a hard guarantee" `\sen` note. Cross-link, don't duplicate.

## Code truth (verified locators — re-grep before writing prose, lines may drift)

- **Important-task label (the guarantee's scope):** `Task::is_important`
  (`RegularTasks.h:111`, `bool`, default `false`) — a **generation-time**
  property, top-50% by `sp_weight` (`IMPORTANT_TASK_RATIO`=0.5,
  `generation_config_parser.py:373`; Python marking `taskset_generator.py:566-581`).
  Persisted to YAML, read back by C++ (`RegularTasks.cpp:87`). One source of truth
  for every consumer.
- **Gate predicate (the guarantee itself):** `ImportantTasksMeetThresholds`
  (`SP_Metric.cpp:208-239`) — bakes candidate TL into ET dist
  (`ApplyTimeLimitsToTasksExecutionTime`), applies candidate PA
  (`UpdateTaskSetPriorities`), then for each **important** task rejects when
  `GetDDL_MissProbability(node_rtas[i], deadline) > threshold`. Enforced constraint
  = `Pr(r_i > D_i) ≤ Θ_i` (Option A). Two overloads: contract (`:208`, pre-
  materialized RTAs, used by the during-walk gate) + self-contained (`:245`,
  derives RTAs fresh, used by `ComputeSafeFallback`'s loud-fail + the BF gate).
- **(2) HOW — offline safe-fallback artifact:** `ComputeSafeFallback`
  (`OptimizeSP_TL_Incre.cpp:1002-1076`), runs **once** before the sim loop in
  `RunSimulation` (`SimulationOrchestrator.cpp:321-323`). On a **throwaway sibling**
  optimizer (`fallback_solver`) so the live incumbent is never seeded from it;
  forces real ET + TL-opt-on + `enable_fallback_use_=true` via `GlobalVariables`
  save/restore; seeds at DM PA + TL ≤ `et_mean` (`SeedTimeLimitsAtOrBelowEtMean`
  `:631`); runs the **offline convergence loop** `OptimizeIncre_w_TL_UntilConvergence`
  (`:889-897`, offline ONLY, no cap, stops on no strict `opt_sp_` gain,
  `ApproxEqualSP` 1e-3); then **loud-fail re-gates** the final result via
  `ImportantTasksMeetThresholds` — a miss = seed infeasible on the worst-case DAG
  (raising TL worsens interference, not walk-fixable) → **throw**, do NOT store.
  On pass, stores `safe_fallback_` (`:1088` `AdoptSafeFallbackAsIncumbent`).
- **(2) HOW — the worst-case DAG it certifies on:** `BuildDAGForObtainSafeFallBAckAcrossIntervals`
  (`WorstCaseDAG.cpp:33-80`): fuses every interval's DAG into one. **Non-perf task**
  dist = point mass at `max(execution_time_max)` across intervals (stochastic
  dominance). **Perf (TL-flexible) task** dist = point mass at its MINIMUM TL option
  (least-interference seed). Structural equality across intervals enforced
  (`TaskStructureMatches`) — only ET dist differs. Certified-on-worst-case ⇒
  interval-independent (the certified `{PA,TL}` is safe under any interval's ET).
- **(2) HOW — BF is gated too (P0.10):** `AdoptRmFastFallbackIfUnschedulable`
  (`OptimizeFallback.cpp:22-57`) called **inside**
  `OptimizePA_with_TimeLimitsStatus::Optimize()` (`OptimizeSP_TL_BF.cpp:73`) so
  **every** BF caller is gated. BF-fails-gate → swap in
  `RateMonotonicFastGroupLocked` (`OptimizeFallback.cpp:12-16`: RM + important-first
  group lock + smallest-grid TL, via shared `BuildPriorityPlan`). RM-Fast also
  fails → **throw** ("no safe solution exists — regenerate"). BF has no live RTA
  cache → uses the self-contained `ImportantTasksMeetThresholds` overload.
- **(3) WHEN — three runtime triggers (P0.7), all gated by master flag
  `enable_fallback_use_`** (default `true`=shipped; `false`=measurement baseline),
  `OptimizeSP_TL_Incre.h:469`. Entry points `Optimize_w_TL_ScratchOrIncre`
  (INCR/INCR_Reopt dispatcher) + `OptimizePureIncremental` (INCR_NO_REOPT):
  - **(a) ET-jump short-circuit** — `SkipOptOnETJump`→`DetectETJump`
    (`OptimizeSP_TL_Incre.cpp:131-151, 1078-1086`): trips on the **first** task whose
    new/old avg-ET ratio `≥ 1.5` (`ratio_threshold` default `1.5`; decreased ET
    never trips). On trip, skip the walk + adopt the safe fallback directly
    (`AdoptSafeFallbackAsIncumbent`, re-scored under current dag). Runs **before**
    `AbsorbUpdatedDAG` so `dag_tasks_` is still the saved old dag.
  - **(b-i) During-walk gate** — inside `UpdateRecords` (`:220`, gate at `:238-258`):
    on a would-beat (SP-better candidate), commit **only if** every important task's
    `ddl_miss_chance ≤ threshold` (`ImportantTasksMeetThresholds`, reading the
    challenger's final RTA from the cache). Reject → `during_walk_reject_count++` +
    `return false` (no commit, champion reverted). **Only the incremental path**
    (`rta_cache_active_`); the reopt from-scratch beam runs disarmed, gated post-hoc
    by (b-ii).
  - **(b-ii) Post-walk backstop** — `AdoptFallbackIfUnschedulable` (`:1106-1191`):
    after the walk, re-gate the **final** `res_opt_`. FAIL → adopt the safe fallback,
    record the worst important-task violator as culprit, then **re-verify** the
    adopted fallback (certified on the worst-case DAG which dominates every interval
    → it MUST pass; a second failure = **certificate violation** → throw). PASS →
    **keep the walk result** — schedulability decides, **not** SP (fallback not
    adopted even if it would have higher global SP).
- **Master flag / opt-out:** `enable_fallback_use_` (`OptimizeSP_TL_Incre.h:469`,
  default `true`). `false` = the "without fallback" measurement arm used in the A/B
  (≈1% SP penalty at N=[4,6], see P0.7).

## Subsection rows (new §9 structure)

### §9.0 Lede + the guarantee statement — ADD
- **draft claim:** no dedicated lede; the guarantee is scattered (§13.3 frames it
  conditionally; §11 `\sen` flags it as open; old §9-software-impl mentions the
  fallback path as a forward-link).
- **code reality:** the framework **self-guarantees** `Pr(r_i > D_i) ≤ Θ_i` for
  every important task, every shipped interval — via offline certification on a
  worst-case DAG + three runtime triggers + a gated offline BF. No user-supplied ET
  upper bound required.
- **action:** **ADD** a lede stating the guarantee precisely: scope = important
  tasks (top-50% by `sp_weight`, §5/§6 definition); predicate = Option A
  (`Pr(r_i > D_i) ≤ Θ_i`); the framework self-discovers and enforces it, never
  silently ships infeasible (no-safe-solution → loud-fail → regenerate). Frame the
  three questions the section answers: why / how / when. **DELETE** the
  "conditional on a known ET upper bound" framing wherever it appears (§13.3 row
  owns that rewrite; this section states the self-guarantee as fact).
- **Ryan:** 1.4 (soften "guarantee" + assumptions/regimes) — the self-guarantee IS
  the precise regime; state the scope (important tasks) + the loud-fail semantics
  explicitly so "guarantee" is not an overclaim.

### §9.1 Why we need a safe fallback — ADD
- **draft claim:** not present as a dedicated argument.
- **code reality:** two pressures motivate it. (i) **The online optimizer trades
  schedulability for SP** — the SP metric rewards lower miss probability across all
  tasks weighted by `sp_weight`, so a move that raises global SP can still break an
  important task's deadline; without a hard constraint the shipped `{PA,TL}` could
  be SP-better but important-task-infeasible. (ii) **The environment is dynamic, not
  smooth** (content change 2) — ET jumps across intervals (`DetectETJump`) can
  invalidate a warm-started incumbent faster than the incremental walk can repair
  it; the framework must have a certified floor to fall back to. The guarantee is
  scoped to important tasks (not all) precisely so it is **strong but achievable**:
  non-important tasks' schedulability is the SP trade currency.
- **action:** **ADD** the motivation: (i) SP-vs-schedulability tension on important
  tasks; (ii) dynamic environment / ET-jump invalidation. State the scoping choice
  (important tasks only) and why it is both strong (hard constraint on the critical
  tasks) and achievable (non-important tasks absorb the SP/sched trade). Forward-
  reference §6 (SP metric + important-task definition) and the dynamic-environment
  framing (content change 2).
- **Ryan:** (indirect) 1.4 — the "why" is the regime justification for the
  guarantee's scope.

### §9.2 How to find a safe fallback — ADD
- **draft claim:** not present; §13.3 only gestures at "a stronger guarantee."
- **code reality:** `ComputeSafeFallback` on the cross-interval worst-case DAG
  (locators above). Three sub-points to present:
  1. **The worst-case DAG** (`BuildDAGForObtainSafeFallBAckAcrossIntervals`) — fuses
     every interval; non-perf task = point mass at cross-interval `max(ET_max)`
     (stochastic dominance); perf task = point mass at its MIN TL option. Certified
     on this ⇒ interval-independent.
  2. **The offline compute** — throwaway sibling optimizer (live incumbent never
     seeded from fallback); DM PA + TL ≤ `et_mean` seed; offline convergence loop
     `OptimizeIncre_w_TL_UntilConvergence` (offline ONLY, no cap, monotonicity-
     guaranteed termination over a finite config space); **loud-fail re-gate** on the
     final result — a miss means the seed itself is infeasible on the worst-case DAG
     (raising TL worsens interference) → throw, do NOT store.
  3. **The BF is gated too** — `AdoptRmFastFallbackIfUnschedulable` inside BF's
     `Optimize()`; on BF-fail swap in `RateMonotonicFastGroupLocked` (RM +
     important-first group lock + smallest-grid TL); double-fail → throw. The
     guarantee is a property of the framework's *output*, not specific to the
     incremental optimizer.
- **action:** **ADD** the three sub-points as the "how." State the **certification
  principle**: compute once offline on a stochastically-dominating DAG ⇒ the
  certified `{PA,TL}` is safe under any single interval's actual ET ⇒ the runtime
  triggers only *adopt* the precomputed artifact, never re-derive it (bounds online
  cost). State the **loud-fail semantics**: no-safe-solution → throw → regenerate,
  never silently ship infeasible. Cross-link §7.4 (RTA cache, used by the
  convergence loop) and §7.3 (DM seed). Keep the convergence-loop detail light
  (one paragraph: offline-only, no cap, monotonicity) — full analysis in §10.
- **Ryan:** 1.4 (assumptions/regimes) — the worst-case DAG is built from per-
  interval ET dists the framework already collects; explicitly NOT a user-supplied
  upper bound.

### §9.3 When to trigger the safe fallback — ADD
- **draft claim:** not present.
- **code reality:** three runtime triggers (locators above), all gated by
  `enable_fallback_use_`:
  - **(a) ET-jump short-circuit** — `DetectETJump` (ratio ≥ 1.5) → skip the walk,
    adopt the fallback directly. *When:* a sudden ET change invalidates the
    warm-started incumbent before the walk can repair it.
  - **(b-i) During-walk gate** — `UpdateRecords` rejects an SP-better candidate that
    breaks an important task's `Pr(r_i > D_i) ≤ Θ_i`. *When:* an in-walk move would
    leave the gate-feasible region; the champion is reverted in place.
  - **(b-ii) Post-walk backstop** — `AdoptFallbackIfUnschedulable` re-gates the
    final `res_opt_`; FAIL → adopt fallback + re-verify (must pass; second fail =
    certificate violation → throw). PASS → keep the walk result (schedulability
    decides, NOT SP). *When:* the walk's final result (esp. the reopt from-scratch
    beam, which runs (b-i) disarmed) needs a post-hoc gate.
- **action:** **ADD** the three triggers as the "when." For each: the *condition*
  that fires it, the *action* it takes, and *which path* it covers ((b-i) =
  incremental only; (b-ii) = all paths incl. reopt beam; (a) = pre-walk). State the
  ordering: (a) is checked first (pre-walk), (b-i) during the walk, (b-ii) post-walk
  as the last-resort backstop. State the **master flag** `enable_fallback_use_`
  (default ON; OFF = measurement baseline for the A/B). Note the A/B result (≈1% SP
  penalty at N=[4,6], P0.7) as the cost of the guarantee — defer detail to §13/§15.
- **Ryan:** (none directly) — but the trigger ordering + the "schedulability
  decides, not SP" rule (b-ii PASS case) is the precise behavior that makes the
  guarantee honest (Ryan 1.4 regimes).

### §9.4 Complexity + the loud-fail contract — ADD (brief)
- **draft claim:** not present.
- **code reality:** offline cost = one `ComputeSafeFallback` run (convergence loop,
  bounded by finite config space — full analysis §10); online cost = the three
  triggers adopt a precomputed artifact (no re-derivation) + (b-i) reuses the RTA
  cache (§7.4). The loud-fail contract: every gate failure either falls back to a
  certified artifact or throws; the framework never ships a `{PA,TL}` that fails
  `Pr(r_i > D_i) ≤ Θ_i` for an important task.
- **action:** **ADD** a brief closing: offline cost is one-time (certify once);
  online cost is adoption-only (bounded) + cache-reuse; the loud-fail contract is
  the guarantee's enforcement guarantee. Forward-reference §10 for the convergence-
  loop complexity. Keep to one short subsection.
- **Ryan:** (indirect) 1.4 — the cost statement makes the "online feasibility"
  claim honest.

## QoS reframe note (content change 6)

§9 introduces no new optimization-variable symbols (the fallback is a `{PA,TL}`
artifact certified under Option A). Where it references the QoS budget, use
`\boldsymbol{\mathcal{Q}}` / `\mathcal{Q}_i` consistently (the perf-task MIN-TL
seed is `\mathcal{Q}_i = \min(\text{TL grid})`). No equation changes.

## Placement + renumbering (structural, cross-links PW.2)

**This new section displaces the current §9 (software impl) and cascades.** The
placement decision is the user's ("we'll add a new section, it should be section
9"), but the renumber mechanics touch `main.tex` + 7 `.tex` files + 7 plan files +
every cross-`\ref`. Concretely:

- Current §9 `section9_software_impl.tex` → §10; §10 complexity → §11; §11
  limitations → §12; §12 real-exp → §13; §13 real-exp-analysis → §14; §14 simu-exp
  → §15; §15 simu-exp-analysis → §16; §16 conclusion → §17.
- OR (PW.2 owns this): software-impl may merge/fold per Ryan Cat-2.4, in which case
  the cascade differs. **Defer the final numbering to PW.2's `reorg_plan.md`** (not
  yet started); this section's *content* is fixed regardless of its final number.

**For this plan file:** treat §9 as the safety-fallback section's working number;
the renumber is a PW.2/PW.3 execution concern, NOT a content concern. The existing
`section_9_software_impl.md` plan file KEEPS its content (software impl) under
whatever number PW.2 assigns; only its *number* changes. Flag the renumber in
`overall_revision_plan.md` section index (see that file's edit below).

## What this section is NOT

- NOT a theorem-with-proof. The guarantee is a stated property backed by (i) the
  offline certification on a stochastically-dominating DAG + (ii) the three
  runtime triggers + (iii) the loud-fail contract. State it as a property/
  observation, not a formal theorem (out of scope for T-ASE; would require
  formalizing the worst-case-DAG dominance + the cache's move model).
- NOT the discussion. §13.3 (rewritten) is the *discussion* of the guarantee (A/B
  cost, when it fires); this section is the *methodology*. Cross-link, don't
  duplicate.
- NOT the limitations framing. §11's `\sen` "how to provide a hard guarantee" note
  is now **answered** by this section; the note is deleted/resolved (§11 row owns
  that).
