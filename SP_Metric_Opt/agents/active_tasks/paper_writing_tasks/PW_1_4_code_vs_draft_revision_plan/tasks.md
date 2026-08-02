# PW.1.4 — Tasks (working checklist)

> See `goal.md` for scope. Hard-blocked on PW.1.1 + PW.1.2 + PW.1.3 sketch
> slices. Produces `revision_plan.md` (the row-by-row edit plan PW.3/PW.4
> execute).

## Prereqs
- [ ] PW.1.1 `sketch_foundations.md` exists (items 1–4 + Θ_i finding settled)
- [ ] PW.1.2 `sketch_optimization.md` exists (PA modes + TL + env-task reframing + `\agent` recommendation)
- [ ] PW.1.3 `sketch_fallback.md` exists (important-task guarantee + offline convergence loop)
- [ ] Read Ryan review `paper_sections/review_from_ryan_from_pdf_files.md` (Category-1 items)
- [ ] Skim PW.2 `reorg_plan.md` (if ready) to reference structural moves rather than re-decide them

## Walk methodology sections (6, 7, 8) — primary
- [ ] §6 each subsection: draft-claim → code-reality → action
  - [ ] GP example + `eq: gpr_predict1` → sliding-window + Gaussian-fit predictor (PW.1.1)
  - [ ] Environment framing → dynamic/continuous (PW.1.1)
  - [ ] Optimization framework → match sketch (PW.1.2 + PW.1.1)
  - [ ] Add important-task safety guarantee subsection (PW.1.3)
  - [ ] Ryan 1.2: safety def & Θ_i convention (Option A/B per PW.1.1)
  - [ ] Ryan 1.2: SP-metric equation (paren + `Σ_i w_i`→`Σ_j w_j`, ≈line 478)
  - [ ] Ryan 1.2: Example 2 arithmetic (≈lines 500–503)
  - [ ] Ryan 1.2: SP interpretability claim (≈line 528)
  - [ ] Ryan 1.2: `Normalize()` definition
  - [ ] Ryan 1.2: RTA defs (`hp(i)` strictly higher priority; init includes `C_i`, ≈lines 405/413)
- [ ] §7 each subsection: PA update (PW.1.2); Algorithm 1 pseudocode fixes (≈lines 634–657); relabel heuristic not "optimal" (PW.1.2); soften guarantee + assumptions/regimes
- [ ] §8 each subsection: `\sen` reframing (PW.1.2); coordinate-descent steps (PW.1.2); resolve `\agent` theorem note (PW.1.2 recommendation); keep `eq: incremental_configuration`

## Walk sections 1–5, 9–16 — secondary
- [ ] §1: GP contribution (≈line 59); headline number; "first work" softening
- [ ] §2: "first work" (≈line 26); reframe "adaptive SP-aware scheduling framework"
- [ ] §3: light terminology pass
- [ ] §4: framework figure/description vs sketch (PW.1.1)
- [ ] §5: keep Gaussian-distribution assumption (≈line 54); align env-dependent/TL def (PW.1.2)
- [ ] §9: DM priority building (was RM); fallback path (PW.1.3); `OptimizeIncre_w_TL_UntilConvergence` (PW.1.3); Ryan 1.3 RRT config (period 10ms vs ET 1–3s)
- [ ] §10: complexity vs code (BF exp; modified Audsley O(m·N²); incremental O(N·O_RTA); RTA-cache speedup) (PW.1.1/PW.1.2)
- [ ] §11: GPR removal (≈line 13); `\relax_smooth_assumption` reframe (PW.1.1); PW.2 relocation; polar footnote (Ryan 1.4)
- [ ] §12–15: PW.2 merge (12+13, 14+15); Ryan 1.3 covariance matrix; headline number; (opt) task-level ground truth
- [ ] §16: headline number; guarantee wording matches PW.3 (PW.1.3)
- [ ] Cross-cutting `main.tex`: Ryan 1.1 Table I `\ith{i}` (≈line 228); typos list; Ryan 1.5 Note to Practitioners

## Map + cross-link
- [ ] Every Ryan Category-1 item mapped to ≥1 subsection row
- [ ] Structural actions reference PW.2 `reorg_plan.md` rows (no duplication)
- [ ] Θ_i convention + headline-number open decisions flagged at their subsections

## Verification
- [ ] `revision_plan.md` has a row per affected subsection `{section, subsection, draft-claim, code-reality, action, Ryan-item?, PW.2-ref?}`
- [ ] The four content changes each appear as concrete subsection edits
- [ ] PW.3/PW.4 could execute edits row-by-row using only this plan + the three sketch slices (no C++ re-read)
