# PW.2.1 — Move ET Modeling & RTA from Section 5 into System Model (Section 4)

**Priority:** P1 (structural execution; user-directed 2026-08-05)
**Status:** done
**Depends on:** — (standalone; complements PW.2 plan-only task)

## Goal

Relocate two modeling subsections out of Section V (Safety-Performance
Optimization Problem) and into Section IV (System Models), so that Section V
contains *only* the SP metric and the optimization problem, while all
system-modeling material (execution-time distribution model, response-time
distribution analysis) lives in the System Models section where it belongs.

This is one concrete move from the PW.2 re-org umbrella, pulled forward and
executed now per user direction (PW.2 itself remains a plan-only task).

**Folded-in follow-up (2026-08-05 cont.):** after the ET/RTA move, the merged
IV-A "Computation Tasks" (task model + ET modeling + the two task-type defs) was
too long, so it was split into three subsections, folded into this same task per
user direction ("do that, fold it into p2.1"):

- **IV-A Computation Tasks** — task abstraction, Period/Deadline/Execution-time defs,
  Gaussian assumption, + `fig_rts_concepts`.
- **IV-B Modelling Execution Time Distribution** — `section_et_model`, `eq: et_predict`
  (repositioned, content unchanged).
- **IV-C Task Classification** (NEW, `\label{section_task_classification}`) — the two
  task-type defs, reordered: distinction → `def_env_task` → motion-planning example →
  `def_task_config` → QoS note.

Resulting §IV order: IV-A → IV-B → IV-C → IV-D Computation Platform → IV-E
Schedulability & RTA. The split turns `def_env_task`'s `(Section~\ref{section_et_model})`
into a back-ref (IV-C→IV-B).

## Concrete moves

1. **ET modeling → new subsection in Section IV, right after Computation Tasks.**
   Move the entirety of §V-A "Modelling Execution Time Distribution"
   (`\label{section_et_model}`, incl. environment vector `\textbf{E}`,
   prediction function `\mathcal{F}_i`, `eq: et_predict`, the GPR discussion,
   and the rolling-average `Example`) from `section5_sp_opt_problem.tex` into
   `section4_system_model.tex` as a new subsection placed immediately after
   "Computation Tasks" (after `fig_rts_concepts`) and before "Computation
   Platform". Resulting Section IV order:
   - IV-A Computation Tasks
   - IV-B Modelling Execution Time Distribution  ← moved
   - IV-C Computation Platform
   - IV-D Schedulability Analysis  ← merged (see below)

2. **RTA distribution analysis → merged into Section IV Schedulability Analysis.**
   Move the entirety of §V-B "Response Time Distribution Analysis"
   (`\label{section_rta}`, experimental method, analytical method with
   `eq: rta_scalar` + `eq_prob_rta`, and "Comparison and our choice") from
   `section5_sp_opt_problem.tex` into Section IV's existing "Schedulability
   Analysis" subsection (`\label{sectino_schedulability_analysis}`), replacing
   the thin pointer "Check Section~\ref{section_rta} for more details." (which
   would otherwise become a self-reference) with the actual content.

## Cross-references (verified; all labels are global so refs stay valid)

Labels move WITH their content; no `\ref`/`\eqref` target is lost. References
that *point* to the moved labels (and therefore now point into Section IV):

| Referrer | Label | Notes |
|---|---|---|
| `section4` def_env_task (L93) | `section_et_model` | becomes within-section fwd ref IV-A→IV-B; fine |
| `section4` Schedulability (L153) | `section_rta` | **REMOVE** — self-ref after merge |
| `section6_pa_opt` (L170) | `section_rta` | now points to IV-D; fine |
| `section7_task_config_opt` (L13) | `section_et_model` | now points to IV-B; fine |
| `section10_complexity` (L13) | `eq: rta_scalar` | fine |
| `section6_pa_opt` (L177, L204) | `eq_prob_rta` | fine |
| `section9_software_impl` (L177) | `eq_prob_rta` | fine |
| `section5` §V-G (L202) | `eq: rta_scalar` | now back-refs IV-D; fine |
| moved ET content (L30) | `section: predict_ET_exp` | stays valid (target in §9) |

## Entry point

- `paper_sections/full_paper_sections/sections/section4_system_model.tex`
- `paper_sections/full_paper_sections/sections/section5_sp_opt_problem.tex`

## Done when

- [ ] Section IV has the new IV-B ET modeling subsection and the merged IV-D
      Schedulability/RTA subsection, with the self-reference removed.
- [ ] Section V no longer contains ET modeling or RTA; it opens with the
      Safety metric, and its intro still reads coherently.
- [ ] All cross-references in the table above resolve (no `??` in build).
- [ ] `pdflatex` (or IDE diagnostics) shows no new undefined references.
- [ ] `dev_log.md` records the move; top-level `agents/dev_log.md` milestone
      appended.

## Out of scope

- Rewriting the moved content (writing quality pass on §V already done
  2026-08-05; the moved text is final).
- Other PW.2 structural moves (results/analysis merge, Limitations relocation,
  §8 reframing) — those stay in PW.2's plan.
- One-sentence-per-line reflow of section4 (flag as separate commit if the
  user wants reviewable whitespace diffs; see dev_log).
