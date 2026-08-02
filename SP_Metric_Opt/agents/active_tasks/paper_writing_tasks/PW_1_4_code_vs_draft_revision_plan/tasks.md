# PW.1.4 — Tasks (working checklist)

> See `goal.md` for scope. Hard-blocked on PW.1.1 + PW.1.2 + PW.1.3 sketch
> slices. Produces a **per-section file set** (one `section_{n}_{desc}.md` per
> included section + `overall_revision_plan.md` master) — the row-by-row edit
> plan PW.3/PW.4 execute. §1/§2/§16 skipped per user (Cat-1 items still tracked
> in the master's coverage map).

## Prereqs
- [x] PW.1.1 `sketch_foundations.md` exists (items 1–4 + Θ_i finding settled → Option A)
- [x] PW.1.2 `sketch_optimization.md` exists (PA modes + TL + env-task reframing + `\agent` recommendation → stated property)
- [x] PW.1.3 `sketch_fallback.md` exists (important-task guarantee + offline convergence loop)
- [x] Read Ryan review `paper_sections/review_from_ryan_from_pdf_files.md` (Category-1 items)
- [x] Read `main.tex` (now in `full_paper_sections/`): thin `\input` shell; abstract still has GP + headline 15–50%
- [ ] Skim PW.2 `reorg_plan.md` (if ready) to reference structural moves rather than re-decide them

## Walk methodology sections (6, 7, 8) — primary ✅ DONE (in `section_6/7/8_*.md`)
- [x] §6 each subsection: draft-claim → code-reality → action (rows in `section_6_sp_opt_problem.md`)
  - [x] GP example + `eq: gpr_predict1` → sliding-window + Gaussian-fit predictor (§6.1 REPLACE)
  - [x] Environment framing → dynamic/continuous (§6.1, §6.9)
  - [x] Optimization framework → match sketch + important-task guarantee forward-pointer (§6.7 ADD)
  - [x] Ryan 1.2: safety def & Θ_i convention — §6.3 already Option A (VERIFY); Θ-table drift is in §12, not §6
  - [x] Ryan 1.2: SP-metric equation — already fixed (Σ_j w_j + balanced parens) (§6.5 VERIFY)
  - [x] Ryan 1.2: Example 2 arithmetic — already fixed (-0.01, P-coeffs matched) (§6.5 VERIFY)
  - [x] Ryan 1.2: SP interpretability claim "0.9 ⇒ both ≥ 0.9" (§6.6 FIX — false for weighted sum)
  - [x] Ryan 1.2: `Normalize()` definition (§6.3 FIX — state domain/min/max/saturation)
  - [x] Ryan 1.2: RTA defs `hp(i)` strict + init includes `C_i` (§6.2 FIX)
- [x] §7 each subsection (§7.1 VERIFY / §7.2 FIX Alg.1 pseudocode + relabel / §7.3 FIX smoothness + ADD DM seed) — `section_7_pa_opt.md`
- [x] §8 each subsection (lede REWRITE `\sen` reframing / `eq: incremental_configuration` REPLACE δ-radius with patience-walk / PROMOTE `\agent` coordinate-descent / RESOLVE 1-task-ET-diff as stated property) — `section_8_task_config_opt.md`

## Walk sections 3–5, 9–15 — secondary (high-level files written; row-by-row deferred)
- [ ] §1: SKIPPED per user (GP contribution + headline + "first work" — tracked in master coverage map)
- [ ] §2: SKIPPED per user ("first work" + "adaptive SP-aware scheduling framework" — tracked in master)
- [x] §3: light terminology pass — `section_3_background.md` (high-level)
- [x] §4: framework figure/description vs sketch (PW.1.1) — `section_4_overview.md` (high-level)
- [x] §5: keep Gaussian-distribution assumption; align env-dependent/TL def (PW.1.2) — `section_5_system_model.md` (high-level)
- [x] §9: DM priority building (was RM); fallback path (PW.1.3); `OptimizeIncre_w_TL_UntilConvergence` (PW.1.3); Ryan 1.3 RRT config (period 10ms vs ET 1–3s) — `section_9_software_impl.md` (high-level)
- [x] §10: complexity vs code (BF exp; modified Audsley O(m·N²); incremental O(N·O_RTA) + cache-invariant dependency; RTA-cache speedup) — `section_10_complexity.md` (high-level)
- [x] §11: GPR removal; `\relax_smooth_assumption` reframe; PW.2 relocation; polar footnote (Ryan 1.4) — `section_11_limitations.md` (high-level)
- [x] §12: Θ-table inversion (Option A, MPC=0.99) + headline + (opt) task-level ground truth — `section_12_real_exp.md` (high-level)
- [x] §13: §13.3 guarantee self-guarantee rewrite + PW.2 merge — `section_13_real_exp_analysis.md` (high-level)
- [x] §14: Ryan 1.3 covariance PSD + negative ET; threshold `1.0` scrub; `\agent` promote — `section_14_simu_exp.md` (high-level)
- [x] §15: `\agent` INCR_WCET note resolve; headline; figure legibility (Ryan 2.1/2.2) — `section_15_simu_exp_analysis.md` (high-level)
- [ ] §16: SKIPPED per user (headline + guarantee wording — tracked in master)
- [ ] Cross-cutting `main.tex`: Ryan 1.1 Table I `\ith{i}`; typos list; Ryan 1.5 Note to Practitioners (in master)

## Map + cross-link
- [x] Every Ryan Category-1 item mapped to ≥1 section row (master coverage map)
- [x] Structural actions reference PW.2 `reorg_plan.md` rows (no duplication)
- [x] Θ_i convention + headline-number open decisions flagged at their sections (§6.3, §12, §13, §15)

## Verification
- [x] Per-section files exist for §3–§15 (§1/§2/§16 skipped per user, tracked in master)
- [x] `overall_revision_plan.md` master: how-to-read + 5 conventions + section index + Ryan coverage map + skipped note
- [x] The four content changes each appear as concrete section edits
- [x] Monolithic `revision_plan.md` deleted (content redistributed)
- [ ] (deferred) row-by-row detailing for §3–§5, 9–15 — high-level change descriptions written; subsection rows to be filled when PW.3/PW.4 pick up each section
