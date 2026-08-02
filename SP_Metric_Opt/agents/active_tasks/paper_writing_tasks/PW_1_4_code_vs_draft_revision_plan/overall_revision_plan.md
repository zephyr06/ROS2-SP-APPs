# PW.1.4 — Overall Revision Plan (master)

> Master index for the section-split plan. One `section_{n}_{desc}.md` per included
> section; this file is the entry point. Consumed by PW.3 (§6–8 methodology) +
> PW.4 (§3–5, 9–15). Source of code truth: `sketch_foundations.md` (PW.1.1),
> `sketch_optimization.md` (PW.1.2), `sketch_fallback.md` (PW.1.3).

## How to read this plan

- **Locators are content/label-based, not line numbers.** Ryan's review cites
  `main.tex:NNN` from a former monolithic file; the draft is now split across
  `section1..16_*.tex` under `paper_sections/full_paper_sections/` plus a thin
  `main.tex` that only `\input`s them. Each row names the section file + the stable
  label/anchor (e.g. `eq:gpr_predict1`, `alg:modified_audsley`) so the writer can
  `grep`. Before editing, **confirm the anchor still resolves** — the draft was
  edited after Ryan's review, so some Cat-1.2 items are already fixed (marked
  **VERIFY**, not FIX).
- **Verbs:** FIX = still broken vs code/Ryan. VERIFY = appears already fixed,
  re-check only. DRIFT = draft describes an older design absent from code.
  ADD = code has it, draft doesn't. Action verbs: delete/replace/rewrite/merge/
  relocate/add. Structural verbs (merge/relocate/condense) cross-link PW.2's
  `reorg_plan.md`, not re-decided here.
- **Detail level varies by section.** §6/§7/§8 (methodology) have full row-by-row
  subsection detail. §3–5, 9–15 are high-level change descriptions; row-by-row
  detailing is deferred (see each section file's status line).

## Cross-cutting conventions (apply everywhere, not per-row)

1. **GP removal (content change 1).** grep `gaussian.process`/`GPR`/`Rasmussen`/
   `kernel regress` = empty in code. Actual ET prediction = sliding-window sampling
   + Gaussian-*distribution* fit (mean/var/min/max → CDF-discretized FiniteDist PMF),
   per `sketch_foundations.md` §predict. **Distinct from the Gaussian-distribution
   assumption for ET (§5, §9) — that STAYS.** Remove GP/GPR prose, `eq:gpr_predict1`,
   and the abstract/contribution GP claim; replace with the sliding-window +
   Gaussian-fit predictor. Symbols: `ReadExtTimeData` (last-N lines,
   `execution_time_estimator.h:49`), `GaussianDist` (`RegularTasks.cpp:75-79`),
   CDF→PMF (`Probability.cpp:18-44`).
2. **Dynamic/continuous environment (content change 2).** Tasks' ET changes
   *across reoptimization intervals* (`SimulateInterval` loop,
   `SimulationOrchestrator.cpp:554`), not the static/GP-smoothed framing. Reframe
   env-dependent task ET as per-interval-varying.
3. **Framework update (content change 3).** Match code: modified Audsley + beam
   search (heuristic, NOT optimal); incremental ±1-priority; TL coordinate descent
   (patience-bounded full grid, NOT δ-radius-capped); DM seed + important-first
   group lock; `OptimizeIncre_w_TL_UntilConvergence`; RTA cache.
4. **Important-task guarantee (content change 4).** Self-guaranteed in code (NOT
   conditional on a user-supplied ET upper bound): `ComputeSafeFallback` certifies
   `{PA,TL}` on a worst-case DAG built from the per-interval ET dists the loop
   already collects, enforced at runtime by 3 triggers + the BF gate. The draft
   §13.3/§11 framing ("guarantee IF ET is upper-bounded and known…") is DRIFT.
5. **Θ_i = Option A** (`Pr(r_i > D_i) ≤ Θ_i`, max tolerable miss prob) — code-verified
   in PW.1.1 via 3 sites: `GetDDL_MissProbability` (`RTA.cpp:154-169`) returns
   `Pr(R>D)`; `ImportantTasksMeetThresholds` (`SP_Metric.cpp:234`) rejects on
   `ddl_miss_chance > threshold`; `SP_Func` (`SP_Metric.h:35`) rewards when
   `threshold >= violate_probability`. **Apply Option A everywhere.** Open decision
   (§12): the experiment table gives the most critical task (MPC) the *largest* Θ
   (0.99) — the reverse of Option A. If inverting the table numbers is required, a
   re-run is needed; PW.4 owns the check.

## Section index

| § | File (section_{n}_*.md) | Draft .tex | Status | Detail level |
|---|---|---|---|---|
| 1 | — (introduction) | `section1_introduction.tex` | **SKIPPED** per user | — |
| 2 | — (related work) | `section2_related_work.tex` | **SKIPPED** per user | — |
| 3 | `section_3_background.md` | `section3_background.tex` | pending | high-level |
| 4 | `section_4_overview.md` | `section4_overview.tex` | pending | high-level |
| 5 | `section_5_system_model.md` | `section5_system_model.tex` | pending | high-level |
| 6 | `section_6_sp_opt_problem.md` | `section6_sp_opt_problem.tex` | **DONE** | row-by-row |
| 7 | `section_7_pa_opt.md` | `section7_pa_opt.tex` | **DONE** | row-by-row |
| 8 | `section_8_task_config_opt.md` | `section8_task_config_opt.tex` | **DONE** | row-by-row |
| 9 | `section_9_software_impl.md` | `section9_software_impl.tex` | pending | high-level |
| 10 | `section_10_complexity.md` | `section10_complexity.tex` | pending | high-level |
| 11 | `section_11_limitations.md` | `section11_limitations.tex` | pending | high-level |
| 12 | `section_12_real_exp.md` | `section12_real_exp.tex` | pending | high-level |
| 13 | `section_13_real_exp_analysis.md` | `section13_real_exp_analysis.tex` | pending | high-level |
| 14 | `section_14_simu_exp.md` | `section14_simu_exp.tex` | pending | high-level |
| 15 | `section_15_simu_exp_analysis.md` | `section15_simu_exp_analysis.tex` | pending | high-level |
| 16 | — (conclusion) | `section16_conclusion.tex` | **SKIPPED** per user | — |

Cross-cutting (not a §): `main.tex` — Table I `\ith{i}` escape bug (Ryan 1.1),
confirmed typos list (Ryan 1.1), Note to Practitioners (Ryan 1.5). Handled inline
at whichever section the writer is in; no dedicated section file.

## Ryan Category-1 coverage map

Every Cat-1 item maps to ≥1 section row (so nothing is dropped):

| Ryan item | Lands in section file |
|---|---|
| 1.1 Table I `\ith{i}` + typos | `main.tex` cross-cutting (this file) |
| 1.2 safety def & Θ convention | §6.3 VERIFY (Option A) + §12 (table inversion) |
| 1.2 SP-metric equation + Example 2 | §6.5 VERIFY |
| 1.2 SP interpretability "0.9⇒both" | §6.6 FIX |
| 1.2 `Normalize()` precise | §6.3 FIX |
| 1.2 RTA defs `hp(i)` strict + `C_i` init | §6.2 FIX |
| 1.2 Algorithm 1 pseudocode | §7.2 FIX |
| 1.3 RRT config period vs ET | §9 |
| 1.3 sim covariance matrix | §14 |
| 1.3 headline number consolidation | §12/§13/§15/§16 (skipped) → note in §12 |
| 1.4 relabel "optimal"→heuristic | §7.2 FIX |
| 1.4 soften "guarantee" + assumptions/regimes | §6.7 + §11/§13.3 |
| 1.4 "first work" softening | §2 (SKIPPED — note here) |
| 1.4 GP overclaim removal | §6.1 + `main.tex` (abstract/contribution) |
| 1.4 polar-coordinate footnote | §11 |
| 1.5 Note to Practitioners | `main.tex` cross-cutting |

## Skipped sections (per user)

§1 introduction, §2 related work, §16 conclusion are skipped for per-section
detail files. Their relevant Cat-1 items (§1 GP contribution + headline number;
§2 "first work" softening; §16 headline number + guarantee wording) are still
captured in the coverage map above so they are not dropped — a writer handles them
inline. If detailed files are later wanted, add `section_1/2/16_*.md`.
