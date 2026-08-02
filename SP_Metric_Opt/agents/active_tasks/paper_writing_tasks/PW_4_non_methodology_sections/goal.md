# PW.4 — Non-Methodology Sections (1–5, 9–16)

**Priority:** P1 (after PW.3 settles terminology + the guarantee statement)
**Status:** not started
**Depends on:** PW.1.1, PW.1.2, PW.1.3, PW.1.4, PW.3

## Goal

Update every non-methodology section so it is consistent with the rewritten
methodology (PW.3) and the algorithm sketch (PW.1), and apply the remaining
Ryan Category-1 fixes. Several items here are pure text-only corrections
(typos, escaping, deletions) and can be picked off quickly; the heavier items
are the results sections and the headline-number consolidation.

## Entry point

Inputs: the PW.1 sketch slices (`PW_1_1_foundations_and_framework/sketch_foundations.md`,
`PW_1_2_priority_and_config_optimization/sketch_optimization.md`,
`PW_1_3_important_task_fallback_conv/sketch_fallback.md`) and
`PW_1_4_code_vs_draft_revision_plan/revision_plan.md`, the rewritten
sections 6–8 (PW.3), `PW_2_section_reorg_merge/reorg_plan.md`, and Ryan's
review. Edit the `.tex` in place per section below.

## Per-section work

**Section 1 — Introduction.** Remove the Gaussian-process contribution
(≈line 59: "We develop theoretical models based on Gaussian processes…").
Consolidate the headline improvement number (currently "20–50%" here; 15–50% /
20–40% elsewhere — pick one from the prod data). Soften "to the best of our
knowledge, first work…" framing.

**Section 2 — Related work.** Soften "to the best of our knowledge, this is the
first work…" (≈line 26) — it shares a sentence with a citation to the
smARTflight precedent; reframe as "an adaptive SP-aware scheduling framework."
(Category 2.5, optional: add the related-work comparison table — Maxim RTNS
2011, Díaz RTSS 2002, RED RTSS 2023, Buttazzo elastic, DMAC ECRTS 2019,
stochastic-DAG JSA 2024, ROS 2 RT scheduling.)

**Section 3 — Background.** Light pass for terminology consistency with PW.3.

**Section 4 — Overview.** Update the framework figure / description to match
the code (per PW.1): per-interval collect→predict→optimize→update; note DM seed
and the fallback path in the overview diagram if present.

**Section 5 — System model.** Keep the Gaussian-distribution assumption for ET
(≈line 54) — it is the distribution model, not GP. Ensure the env-dependent /
TL-optimizable task definition agrees with PW.2's reframing and PW.3's usage.

**Section 9 — Software & implementation.** Already largely accurate (the
sliding-window predictor is described in `§predict_ET_exp`). Light updates: DM
priority building (was RM), the important-task fallback in the scheduler path,
`OptimizeIncre_w_TL_UntilConvergence`. Confirm RRT period vs ET is sane (Ryan
Category 1.3: period 10 ms vs ET 1–3 s is impossible — fix the configuration).

**Section 10 — Complexity.** Update to match the code: brute-force exponential;
modified Audsley O(m·N²) RTA calls; incremental O(N·O_RTA); RTA-cache speedup.

**Section 11 — Limitations.** Remove the GPR mention (≈line 13: "This project
uses Gaussian Process Regression (GPR)…") — reframe `§relax_smooth_assumption`
around the sampling/sliding-window predictor. Apply PW.2's relocation decision
(appendix vs. condensed-in-body) and fix cross-references. Remove/correct the
polar-coordinate footnote (Ryan Category 1.4: "the math basically remains the
same" is false under nonlinear polar transform) — one-line deletion.

**Sections 12–15 — Results (real + simulation).** Apply PW.2's merge
(12+13 → one; 14+15 → one). Ryan Category 1.3: fix the simulation covariance
matrix (ρ_x,c, ρ_y,c ∈ [−1,1] independent → PSD only if ρ²_x,c + ρ²_y,c ≤ 1,
≈21% invalid; constrain correlations, use a nonnegative distribution).
Consolidate the headline number to the single value chosen in Section 1.
(Category 2.1, optional: report task-level ground truth — per-task miss rates,
RT CDFs, SLAM error, path length, MPC tracking error, scheduler overhead.)

**Section 16 — Conclusion.** Consolidate the headline number ("20% to 40%" →
the single chosen value). Ensure the guarantee claim matches PW.3's wording.

**Cross-cutting (main.tex).** Ryan Category 1.1: fix Table I `\ith{i}` escaping
(≈line 228); proofread the confirmed typos — `works works` (162), `an robot`
(128, 211), `aerocrafts` (160), `preferablely` (571), `parital` (638),
`dentoes` (415), `pikes` (1365), `temporarily→temporally` (212, 356). Add the
T-ASE-mandated "Note to Practitioners" (Ryan Category 1.5).

## Done when

- No GP/GPR content remains anywhere outside the methodology (which PW.3 already
  cleared) — specifically section 1 contributions, section 11 limitations.
- The headline improvement number is identical in sections 1, 12–15, and 16.
- All Ryan Category-1.1 (typos/escaping) and 1.5 (Note to Practitioners) items
  are resolved; Category-1.3 results-config items (RRT, covariance) resolved.
- Terminology (Θ_i convention, "heuristic" vs "optimal", guarantee wording)
  matches PW.3 throughout.

## Out of scope

- Methodology sections 6–8 (PW.3).
- Re-running experiments (only needed if PW.1 picks Θ_i Option A, which inverts
  table numbers — flag to user if so; otherwise text-only).
- Category-2 items beyond the cheap folds noted above (adaptive baseline,
  high-fidelity scenario, formal theory, sensitivity sweeps) — deferred unless
  the user escalates them.
