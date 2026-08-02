# PW.3 — Methodology Rewrite (Sections 6–8)

**Priority:** P0 (first real writing; where the four content changes concentrate)
**Status:** not started
**Depends on:** **PW.1.1, PW.1.2, PW.1.3** (hard — must match the sketch slices), **PW.1.4** (hard — executes its `revision_plan.md` row-by-row), PW.2 (re-org plan)

## Goal

Rewrite the three methodology sections — `section6_sp_opt_problem.tex`,
`section7_pa_opt.tex`, `section8_task_config_opt.tex` — so they describe the
framework **as implemented** (per the PW.1 sketch slices: `sketch_foundations.md` /
`sketch_optimization.md` / `sketch_fallback.md`) and apply the
four high-level content changes, plus the reviewer's Category-1 math/pseudocode
corrections. This is the first and most important `.tex` editing task.

## Entry point

Inputs: the PW.1 sketch slices (`PW_1_1_foundations_and_framework/sketch_foundations.md`,
`PW_1_2_priority_and_config_optimization/sketch_optimization.md`,
`PW_1_3_important_task_fallback_conv/sketch_fallback.md`) and the row-by-row
`PW_1_4_code_vs_draft_revision_plan/revision_plan.md`,
`PW_2_section_reorg_merge/reorg_plan.md`, the current
`paper_sections/full_paper_sections/section{6,7,8}_*.tex`, and Ryan's review
(Category 1.2, 1.4). Edit the `.tex` in place.

## Section 6 — SP optimization problem (primary target)

- **Remove GP.** Delete the Gaussian-process Example and the GPR prediction
  equation (`eq: gpr_predict1`, ≈lines 27–38). The `\sen` note there already
  argues GP should be de-emphasized and that the sampling-based predictor
  implements the idea. Replace with the actual predictor: sliding-window ET
  sampling + Gaussian-distribution fit (cross-ref section 9 `§predict_ET_exp`).
  Keep the **Gaussian distribution** assumption (it is the distribution model,
  not the GP predictor).
- **Reframe the environment** as dynamic and continuous: tasks' ET changes
  across reoptimization intervals; the framework re-optimizes per interval.
- **Update the optimization framework** description to match the code: modified
  Audsley + beam search (heuristic, not "optimal"), incremental ±1-priority
  adjustment, TL coordinate descent, DM seed + important-first group lock,
  `OptimizeIncre_w_TL_UntilConvergence`, RTA cache. Drop any framing that
  implies exhaustive optimality.
- **Add the important-task safety-performance guarantee.** State: important
  tasks (top-50% by `sp_weight`) are guaranteed a safe fallback — the framework
  builds a worst-case DAG across intervals, seeds an RM/DM-fast group-locked
  plan, and a during-walk gate rejects moves that make an important task
  unschedulable; on walk failure the fallback is adopted. This is the new claim;
  ground every sentence in PW.1.
- **Ryan Category 1.2 math fixes:**
  - Repair the safety definition & Θ_i threshold convention — make Definition,
    safety function, Example 2, and the (later) experiment table mutually
    consistent. Take Option A or B per PW.1's finding on how the code reads Θ_i.
  - Fix the SP-metric equation (≈line 478): unmatched parenthesis; index clash
    `Σ_i w_i` → `Σ_j w_j`.
  - Correct Example 2 arithmetic (≈lines 500–503): two performance coefficients
    swapped; safety coefficient `−0.1` vs defined `−0.01`.
  - Fix the SP interpretability claim (≈line 528): "threshold 0.9 ⇒ both safety
    and performance ≥ 0.9" is false for the weighted sum — state correctly or
    remove.
  - Define `Normalize()` precisely (domain, min/max, saturation).
  - Fix RTA definitions: `hp(i)` = strictly higher priority; response-time init
    must include the task's own `C_i` (≈lines 405, 413).

## Section 7 — Priority assignment optimization

- Update brute-force, modified-Audsley + beam-search, and incremental-PA
  descriptions to match PW.1 (DM seed, important-first group lock, the four
  ±1-priority scenarios).
- **Ryan Category 1.2 / 1.4:** Correct Algorithm 1 (`alg:modified_audsley`,
  ≈lines 634–657) pseudocode: shrink the task pool as tasks are assigned; copy
  partial-assignment vectors before pushing (Pop aliases otherwise); define the
  `SelectTop` objective; relabel "Optimal" → "Selected"; reconcile
  lowest-priority-first construction with the Definition's higher-priority-first.
- **Ryan Category 1.4:** Relabel modified-Audsley + beam search as a
  **heuristic**, not "optimal" (beam search is not exhaustive). Soften
  "guarantee" language except where the important-task fallback actually
  provides one; add the assumptions/regimes statement (provable bounds need a
  known/upper-bounded ET distribution; unknown-environment = empirical/adaptive
  only — the paper already concedes this ≈lines 1224–1225).

## Section 8 — Task configuration (TL) optimization

- Apply PW.2's reframing: TL-optimizable tasks as a special kind of
  env-dependent task → trigger incremental optimization after assuming their ET
  changed (the `FindEnvTaskWithDifferentEt` path). Resolve the `\sen` note at
  the top of the file.
- Update the sequential coordinate-descent description (4 steps: initialization,
  prioritized ordering, coordinate descent, resource-aware tie-break) to match
  the code; confirm the O(M^N) → O(M·N) complexity claim and the
  challenger-from-champion → 1-task-ET-diff property (the `\agent` note asks for
  a theorem — decide proof vs. stated property).
- Keep `eq: incremental_configuration` (`‖λ − λ^(k)‖ ≤ δ`); confirm δ → 3
  candidates matches code/config.

## Done when

- Sections 6, 7, 8 contain no GP/GPR content and no "optimal"-as-exhaustive
  claim; the environment is framed as dynamic/continuous; the optimization
  framework matches the PW.1 sketch slices; the important-task safe-fallback
  guarantee is stated and grounded.
- All Ryan Category-1.2 math/pseudocode items for sections 6–8 are resolved
  (checkable against `review_from_ryan_from_pdf_files.md`).
- A diff read against the PW.1 sketch slices shows no claim the sketches do not
  support.

## Out of scope

- Sections 1–5, 9–16 (PW.4).
- The experiment-table Θ_i numbers (PW.4, possibly a re-run — decided by PW.1's
  Option A/B finding).
- New formal proofs/theory (Ryan Category 2.4 — deferred unless trivial).
