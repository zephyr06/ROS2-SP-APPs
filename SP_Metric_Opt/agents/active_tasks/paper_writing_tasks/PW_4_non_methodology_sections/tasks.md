# PW.4 — Tasks (working checklist)

> See `goal.md` for scope. Depends on PW.1 + PW.3 (terminology + guarantee
> wording must be settled first).

## Section 1 (Introduction)
- [ ] Remove GP contribution (≈line 59)
- [ ] Consolidate headline number (currently "20–50%"; single value from prod data)
- [ ] Soften "to the best of our knowledge, first work…"

## Section 2 (Related work)
- [ ] Soften "first work…" (≈line 26); reframe as "an adaptive SP-aware scheduling framework" (shares sentence with smARTflight cite)
- [ ] (opt, Cat 2.5) related-work comparison table

## Sections 3–5 (Background / Overview / System model)
- [ ] §3 light terminology pass
- [ ] §4 update framework figure/description to match PW.1 (per-interval collect→predict→optimize→update; DM seed; fallback path)
- [ ] §5 keep Gaussian-distribution assumption (≈line 54); align env-dependent/TL-optimizable def with PW.2/PW.3

## Section 9 (Software & implementation)
- [ ] Light updates: DM priority building (was RM), important-task fallback in scheduler path, `OptimizeIncre_w_TL_UntilConvergence`
- [ ] Ryan 1.3: fix RRT config (period 10 ms vs ET 1–3 s impossible)

## Section 10 (Complexity)
- [ ] Update to match code (BF exponential; modified Audsley O(m·N²); incremental O(N·O_RTA); RTA-cache speedup)

## Section 11 (Limitations)
- [ ] Remove GPR mention (≈line 13); reframe `§relax_smooth_assumption` around sampling/sliding-window predictor
- [ ] Apply PW.2 relocation decision (appendix vs. condensed-in-body); fix cross-refs
- [ ] Ryan 1.4: remove/correct polar-coordinate footnote (one-line deletion)

## Sections 12–15 (Results — real + simulation)
- [ ] Apply PW.2 merge (12+13 → one; 14+15 → one)
- [ ] Ryan 1.3: fix simulation covariance matrix (constrain ρ; nonnegative distribution)
- [ ] Consolidate headline number (single value from §1)
- [ ] (opt, Cat 2.1) report task-level ground truth (per-task miss rates, RT CDFs, SLAM error, path length, MPC tracking error, scheduler overhead)

### Section 12 (Simulation) — figure + text refresh [IN PROGRESS 2026-08-14]
- [x] Remove stale figures (cpu_util box, per-scheduler box plots CFS/RM_Fast/RM_Slow/BR/INCR)
- [x] Copy 7 figure PDFs into `pictures/simulation/` (semantic `sim_*.pdf` names)
- [x] Rewrite §12 into 4 comparison sub-sections, each with a conclusion:
      vs. optimal (BF, N=4/6/8) / vs. baselines + scalability (N=4–16) / ablation / fallback
- [x] Fold `\agent{}` notes into text (precise-RTA caveat; INCR_WCET reasoning verified+folded)
- [x] Verify INCR_WCET reasoning: pessimistic ET → pessimistic RTA → gate over-rejects
      larger QoS budgets → smaller budgets → lower SP. CONFIRMED by data (gap widens w/ N).
- [x] Figure sizing rule: max ONE figure per column for sim results. Fig 8
      (`fig_sim_sp_baselines`) was `figure*` @ `0.8\textwidth` → too large; FIXED to
      `figure` @ `\columnwidth`. Subfigure pairs (Fig 7/10) kept — each subfigure
      `0.48\textwidth` ≈ one column.
- [x] Unify "Compared Methods" §11↔§12: §11 "Scheduling algorithms in experiments" is
      now the CANONICAL method list (`\label{section: exp_methods}`); §12.2 reduced to a
      one-¶ reference of it (same 5 methods: CFS/DM_FAST/DM_SLOW/BF/INCR) + sim-only
      notes (INCR_Reopt_10; ablation ptr §12.5). RM→DM rename across §11 prose + captions
      (DM-Fast/DM-Slow everywhere; under D_i=T_i, DM≡RM). §8 `def_dm_fast` referenced
      instead of re-derived. `\agent{}`/`\Sen{}` annotations dropped from §11 method list.
      Figure PDFs still have "RM_Fast"/"RM_Slow" legends baked in → user regenerates later.
- [x] Remove the four `\paragraph{Conclusion.}` blocks from §12 (§12.3/4/5/6); kept the
      concluding text as plain closing sentences/¶ (no label).
- [ ] Verify fallback-figure description matches `fig_fallback_rejection_ratio` content
- [x] Replace §12.3 BF comparison figures (Fig `figure*` w/ 2 subfigures:
      `sim_sp_vs_optimal.pdf` + `sim_exec_time_vs_optimal.pdf`) with a single-column
      `table` (Table IV, `\label{tab_sim_vs_optimal}`) — more space-efficient. Columns:
      N, BF SP, INCR SP, SP Gap (vs BF), BF Time (s), INCR Time (s), Speedup (vs BF).
      Data from `compare_against_bf_run_prod_...tasks4x6x8/evaluation_report.json`:
      N=4/6/8 → BF SP 0.933/0.792/0.758, INCR SP 0.932/0.779/0.742, gap 0.2/1.7/2.1%,
      BF time 0.094/32.28/423.5 s, INCR time 0.0028/0.0048/0.0117 s, speedup
      3.4×10¹/6.7×10³/3.6×10⁴. Prose updated to ref Table~\ref{tab_sim_vs_optimal}.
      Removed the §12.3 `figure*` (one fewer full-width float → less end-of-doc drift).
      Unused PDFs `sim_sp_vs_optimal.pdf`/`sim_exec_time_vs_optimal.pdf` left in
      `pictures/simulation/` (not `\includegraphics`'d; user may delete/regen).
- [x] Re-verify all figures before references after table conversion (latexmk clean,
      21 pp): Fig 10 (fallback, last fig) at L2545, R EFERENCES at L2565 — comfortable
      gap, no figure in/after the bibliography. Figures renumbered (§12.3 figure*
      removed → old Fig 8-11 → new Fig 7-10).
- [x] Build-check the paper (latexmk -g): clean, 21 pp, no undefined refs / dup labels
      (`\ref{section: exp_methods}`, `\ref{def_dm_fast}` both resolve).
- [x] All figures before the references section. Fig 10/11 (§12 ablation/fallback floats)
      had drifted into the MIDDLE of the references (between [9] and [10]) because the
      `figure*`/`figure` floats deferred to end-of-doc past `\bibliography`. FIXED: added
      `\usepackage{placeins}` + `\FloatBarrier` before `\bibliographystyle` in `main.tex`.
      Verified via pdftotext: Fig 10 (L2541) + Fig 11 (L2577) now precede REFERENCES (L2581),
      refs [1]–[56] contiguous. Also fixed 3 empty `[]` table float specifiers
      (§4/§5/§11) → `[htbp]` (cleared "No positions in optional float specifier" warnings).
- Sources: `paper_full_run` (main, N=4–16, 1s budget) + `compare_against_bf` (optimality, N=4/6/8, 500s BF budget)

## Section 16 (Conclusion)
- [ ] Consolidate headline number ("20% to 40%" → single value)
- [ ] Guarantee claim matches PW.3 wording

## Cross-cutting (main.tex)
- [ ] Ryan 1.1: fix Table I `\ith{i}` escaping (≈line 228)
- [ ] Ryan 1.1: proofread typos — `works works` (162), `an robot` (128, 211), `aerocrafts` (160), `preferablely` (571), `parital` (638), `dentoes` (415), `pikes` (1365), `temporarily→temporally` (212, 356)
- [ ] Ryan 1.5: add T-ASE "Note to Practitioners"

## Verification
- [ ] No GP/GPR outside methodology; headline number identical in §1, §12–15, §16
- [ ] All Ryan Category-1.1 / 1.5 resolved; Category-1.3 results-config (RRT, covariance) resolved
- [ ] Terminology (Θ_i, "heuristic" vs "optimal", guarantee wording) matches PW.3 throughout
