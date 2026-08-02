# §7 — Priority Assignments Optimization

> Draft: `section7_pa_opt.tex`. Status: **RE-PLANNED** (was DONE; superseded by the
> PA-only scoping below). See `overall_revision_plan.md` for conventions.
> Source of code truth: `sketch_optimization.md` (PW.1.2).

## High-level guidance (user instruction)

§7 and §8 together introduce the main methodology. **§7 is ONLY about priority
optimization with QoS budgets held fixed.** Therefore §7 needs an explicit
**partial problem description** up front that scopes `\boldsymbol{\mathcal{Q}}`
as fixed, so §7's optimality/heuristic claims are unambiguous about what is held
constant. §7 then presents the three PA algorithms in order of decreasing cost:
**brute-force → modified-Audsley + beam search → incremental**. Finally, §7 adds
a **NEW subsection introducing the RTA cache** — the data structure that makes
the incremental algorithms in §7.3 (and the unified loop in §8) fast enough for
online use, and whose single-change invariant the §8 collaborative loop exploits.

## Subsection rows

### §7.0 Lede + partial problem statement (QoS fixed) — ADD
- **draft claim:** current lede (5-17) jumps straight into "online priority
  assignment algorithms aimed at improving the SP metric," then states "we assume
  that the QoS budgets `$\boldsymbol{\mathcal{Q}}$` are fixed" (16) as a one-liner
  mid-section. No partial problem statement; the QoS-fixed scoping is buried.
- **code reality:** the PA-only optimizers (`OptimizePA_BF`, `OptimizeFromScratch`,
  `OptimizeIncre`) all hold TL/QoS fixed and optimize only the priority vector
  `opt_pa_`; QoS optimization is a separate optimizer (`OptimizePA_Incre_with_TimeLimits`).
- **action:** **ADD** a partial problem statement at the top of §7: given task set
  `\boldsymbol{\tau}` with **fixed** QoS budgets `\boldsymbol{\mathcal{Q}}`, find
  the priority assignment `\mathcal{A}` maximizing `\textbf{SP}(\mathcal{A}; \textbf{E})`.
  State explicitly that QoS budgets are held fixed *throughout §7* and that §8 lifts
  this restriction for collaborative PA+QoS optimization. Promote the existing
  "QoS budgets are fixed" sentence (16) into this lede. Replace the undefined
  `\configs` macro with `\boldsymbol{\mathcal{Q}}` (convention #6).
- **Ryan:** (indirect) 1.4 assumptions/regimes — make the QoS-fixed regime explicit.

### §7.1 Brute-Force Priority Assignments — VERIFY
- **label `section_bf_pa`** (20).
- **draft claim:** enumerate all `n!` orderings, select optimal; impractical online
  due to factorial growth.
- **code reality:** `OptimizePA_BF`/`IterateAllPAs` (`OptimizeSP_BF.cpp`) enumerates
  all permutations. Matches. BF *is* optimal (correct to say so here — the
  "heuristic not optimal" fix applies to §7.2, not §7.1).
- **action:** **VERIFY** — keep. Optional: note BF is the offline optimality
  reference used in ablations (Ryan 2.2 optimality-gap study) if PW.4 adds that.

### §7.2 Audsley + Beam Search — FIX (Ryan 1.2 + 1.4)
- **label `section_audsley_pa`** (28), `alg:modified_audsley` (66).
- **draft claim:** Audsley is `O(n^2)` (33), "provably optimal" for schedulability
  under conditions (37, qualified); modified version uses beam width `m`,
  "heuristic-based" (62 ✓); Algorithm 1 output labeled "Optimal priority
  assignment" (70), comment "lowest priority task appears first" (72).
- **code reality:** `OptimizeFromScratch(int K)` with K=`beam_search_width`
  (`OptimizeSP_Incre.cpp:100`) is the modified-Audsley+beam; it is a **heuristic**
  (beam is not exhaustive). Code never claims optimality for the modified version.
- **action:** **FIX** Algorithm 1 pseudocode (Ryan 1.2): (a) shrink `\taskpool` as
  tasks are assigned (else duplicate assignments possible — 74-86 never remove τ_i
  from pool); (b) copy partial-assignment vectors before Push (the
  `p.Push(τ_i); ∂pa_cur.Push(p); p.Pop(τ_i)` pattern at 79-81 aliases/corrupts —
  push a *copy*); (c) define the objective used by `SelectTop` (currently
  undefined — "lowest SP loss", state it); (d) relabel output "Optimal priority
  assignment"→"Selected priority assignment" (Ryan 1.4). **FIX** the
  "lowest-priority-first" construction (72) vs the Definition that higher-priority
  tasks appear first — reconcile the ordering convention (Ryan 1.2). Keep
  "heuristic-based" (62). **SOFTEN** "provably optimal" (37) — it's qualified to
  Audsley-for-schedulability, but re-check the qualifier is unambiguous so readers
  don't infer the *modified* algo is optimal.
- **Ryan:** 1.2 (Algorithm 1 pseudocode bugs); 1.4 (relabel heuristic not "optimal").

### §7.3 Incremental Priority Assignment Optimization — FIX (smoothness) + ADD (DM seed)
- **label `section_increment_pa`** (123).
- **draft claim** (126): environment changes "continuously" → only some tasks' ET
  varies significantly → adjust those tasks' priority by ≤1 level, keep others'
  relative order. Four scenarios (142-147): important+ET↑→raise priority;
  important+ET↓→no change; not-important+ET↑→lower priority; not-important+ET↓→no
  change. "≤ one level" ✓.
- **code reality:** `OptimizeIncre` (`OptimizeSP_Incre.cpp`) + `FindTaskWithDifferentEt`
  find the changed-ET task and adjust ±1. The premise is "typically only a small
  number of tasks' ETs change per interval," NOT smoothness. The seed PA is
  **Deadline Monotonic + important-first group lock** (`BuildPriorityPlan` with
  `GroupLock::kImportantFirst`, `PriorityBuilders.{h,cpp}`) — the draft does NOT
  mention this seed.
- **action:** **FIX** the "continuously"/smoothness premise (126): reframe as
  "because typically only a small number of tasks' ETs change between intervals, …"
  (content change 2). **ADD** the DM seed + important-first group lock as the
  starting point for the incremental walk (1-2 sentences, sourced from
  `sketch_optimization.md` §seed). Keep the four scenarios (match code). **VERIFY**
  "≤ one level" matches the ±1 incremental move. Forward-reference §8: this ±1
  PA move is the **env-task branch** of the unified collaborative loop.
- **Ryan:** (indirect) 1.4 assumptions/regimes — the incremental premise is
  empirical, state so.

### §7.4 RTA Cache — ADD (NEW subsection)
- **draft claim:** no RTA-cache subsection exists. The cache is mentioned nowhere
  in §7 despite underpinning §7.3's online feasibility.
- **code reality:** `RTACache` class (`Safety_Performance_Metric/RTA_Cache.h:61`,
  `RTA_Cache.cpp`). Public API: `Initialize` (full compute, `:70`), `AdoptChampion`
  (promote candidate, `:78`), `Evaluate` (patch a candidate that differs by ≤1 task
  vs champion and return its RTA, reusing per-task prefix work, `:90`),
  `ComputeTaskSetDifference` (the throwing boundary — throws on `|diff|>1` at
  `RTA_Cache.cpp:358`). Reuse verdict driven by `ClassifyReusePerTask`
  (FullReuse vs narrowed per-task). Transaction RAII (P1.21) was **REMOVED** (P1.25);
  current model = eager save/restore via the `rta_cache_active_` flag +
  `ResetIncumbentBaseline`. Measured ~36% faster at N=10, SP bit-identical (P1.23).
- **action:** **ADD** a new subsection after §7.3 introducing the RTA cache as the
  enabler of online incremental optimization. Cover: (1) *why* — a single SP
  evaluation is at least linear in n; incremental opt repeats many such evals, so
  naive recompute is too costly for online use; (2) *key idea* — when a candidate
  PA/TL differs from the current champion by at most one task (the single-change
  invariant `|diff| ≤ 1`), only that task's RTA and the lower-priority tasks on its
  core need recomputation; all other tasks reuse the champion's RTA prefix; (3)
  *API sketch* — `Initialize` (full), `Evaluate` (incremental patch, ≤1 diff),
  `AdoptChampion` (commit a candidate as the new champion); (4) *the invariant* —
  `Evaluate` assumes `|diff| ≤ 1`; `ComputeTaskSetDifference` is the throwing
  boundary that asserts it. Forward-reference §8: the collaborative loop is
  *structured around* this invariant (each step changes exactly one task, so every
  eval is a cache hit). Keep it concise (this is methodology, not a cache paper);
  defer micro-architecture (per-core HP-prefix checkpoints, ClassifyReusePerTask
  rule A/B) to §9 software impl.
- **Ryan:** (none directly) — but this is the substance that makes the §7.3/§8
  "online" claim honest (Ryan 1.4 regimes).

## QoS reframe note (content change 6)

§7.0/§7.1 says "we assume that task configurations `$\configs$` are fixed." Under
the QoS reframe + symbol rename: the prose becomes "QoS budgets
`$\boldsymbol{\mathcal{Q}}$` are fixed," and the undefined `\configs` macro is
replaced (or defined as `\boldsymbol{\mathcal{Q}}`). No equation change in §7.
This is folded into the §7.0 partial-problem-statement ADD above.
