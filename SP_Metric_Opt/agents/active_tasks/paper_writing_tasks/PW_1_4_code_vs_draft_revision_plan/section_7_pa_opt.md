# §7 — Priority Assignments Optimization

> Draft: `section7_pa_opt.tex`. Status: row-by-row DONE.
> See `overall_revision_plan.md` for conventions.

## High-level change

Update PA methods to match code (PW.1.2). Algorithm 1 pseudocode has multiple
Ryan 1.2 bugs (pool-shrink, copy-before-push, SelectTop objective, output label,
ordering convention); the "optimal" label must be softened to "heuristic" (Ryan
1.4); the smoothness premise must be reframed (content change 2); and the DM seed
+ important-first group lock — the actual starting point the draft omits — must
be added. §7.1 BF legitimately stays "optimal"; only §7.2's modified version is a
heuristic.

## Subsection rows

### §7.1 Brute-Force Priority Assignments — VERIFY
- **label `section_bf_pa`** (16).
- **draft claim:** enumerate all `n!` orderings, select optimal; impractical
  online due to factorial growth.
- **code reality:** `OptimizePA_BF`/`IterateAllPAs` (`OptimizeSP_BF.cpp:5-44`)
  enumerates all permutations. Matches. BF *is* optimal (correct to say so here
  — the "heuristic not optimal" fix applies to §7.2, not §7.1).
- **action:** **VERIFY** — keep. Optional: note BF is the offline optimality
  reference used in ablations (Ryan 2.2 optimality-gap study) if PW.4 adds that.

### §7.2 Audsley + Beam Search — FIX (Ryan 1.2 + 1.4)
- **label `section_audsley_pa`** (21), `alg:modified_audsley` (46).
- **draft claim:** Audsley is `O(n^2)` (22), "provably optimal" for
  schedulability under conditions (24, qualified); modified version uses beam
  width `m`, "heuristic-based" (43 ✓); Algorithm 1 output labeled "Optimal
  priority assignment" (50), comment "lowest priority task appears first" (52).
- **code reality:** `OptimizeFromScratch(int K)` with K=2
  (`OptimizeSP_Incre.cpp:100-164`) is the modified-Audsley+beam; it is a
  **heuristic** (beam is not exhaustive). Code never claims optimality for the
  modified version.
- **action:** **FIX** Algorithm 1 pseudocode (Ryan 1.2): (a) shrink `\taskpool`
  as tasks are assigned (else duplicate assignments possible — 54-66 never remove
  τ_i from pool); (b) copy partial-assignment vectors before Push (the
  `p.Push(τ_i); ∂pa_cur.Push(p); p.Pop(τ_i)` pattern at 59-61 aliases/corrupts —
  push a *copy*); (c) define the objective used by `SelectTop` (currently
  undefined — "lowest SP loss", state it); (d) relabel output "Optimal priority
  assignment"→"Selected priority assignment" (Ryan 1.4). **FIX** the
  "lowest-priority-first" construction (52) vs the Definition that higher-priority
  tasks appear first — reconcile the ordering convention (Ryan 1.2). Keep
  "heuristic-based" (43). **SOFTEN** "provably optimal" (24) — it's qualified to
  Audsley-for-schedulability, but re-check the qualifier is unambiguous so readers
  don't infer the *modified* algo is optimal.
- **Ryan:** 1.2 (Algorithm 1 pseudocode bugs); 1.4 (relabel heuristic not
  "optimal").

### §7.3 Incremental Priority Assignment Optimization — FIX (smoothness) + ADD (DM seed)
- **label `section_increment_pa`** (98).
- **draft claim** (100): environment changes "continuously" → only some tasks' ET
  varies significantly → adjust those tasks' priority by ≤1 level, keep others'
  relative order. Four scenarios (111-117): important+ET↑→raise priority;
  important+ET↓→no change; not-important+ET↑→lower priority; not-important+ET↓→no
  change. "≤ one level" (102) ✓.
- **code reality:** `OptimizeIncre` (`OptimizeSP_Incre.cpp:367-462`) +
  `FindTaskWithDifferentEt` find the changed-ET task and adjust ±1. The premise is
  "only one task's ET typically changed per interval," NOT smoothness. The seed
  PA is **Deadline Monotonic + important-first group lock**
  (`DeadlineMonotonicPriorityVec` `:920-936`, `BuildPriorityPlan` with
  `GroupLock::kImportantFirst`) — the draft does NOT mention this seed.
- **action:** **FIX** the "continuously"/smoothness premise (100): reframe as
  "because typically only a small number of tasks' ETs change between intervals,
  …" (match §6.9 fix, content change 2). **ADD** the DM seed + important-first
  group lock as the starting point for the incremental walk (1-2 sentences,
  sourced from `sketch_optimization.md` §seed). Keep the four scenarios (match
  code). **VERIFY** "≤ one level" matches the ±1 incremental move. Cross-link §8
  env-task reframing (`FindEnvTaskWithDifferentEt` unifies TL-flex with ET-change
  handling).
- **Ryan:** (indirect) 1.4 assumptions/regimes — the incremental premise is
  empirical, state so.

## QoS reframe note (content change 6)

§7.1 (`section7_pa_opt.tex:16`) says "we assume that task configurations
`$\configs$` are fixed." Under the QoS reframe + symbol rename: the prose
becomes "QoS budgets `$\boldsymbol{\mathcal{Q}}$` are fixed," and the undefined
`\configs` macro is replaced (or defined as `\boldsymbol{\mathcal{Q}}`). No
equation change in §7. The `$\configs$` uses at 16 (and the §6.7 itemize rows
that parallel it) are the only §7 touchpoints.
