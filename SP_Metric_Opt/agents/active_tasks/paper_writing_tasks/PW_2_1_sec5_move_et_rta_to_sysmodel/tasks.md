# PW.2.1 — Tasks (working checklist)

> See `goal.md` for scope. Executing one concrete PW.2 re-org move.

## Section 4 — insert ET modeling subsection (IV-B)
- [x] Cut §V-A "Modelling Execution Time Distribution" (section5 L6–L34, incl. `section_et_model`, `eq: et_predict`, the GPR paragraph, the `Example`) verbatim.
- [x] Paste as new `\subsection{Modelling Execution Time Distribution}` in section4, after `fig_rts_concepts` (end of Computation Tasks) and before `\subsection{Computation Platform}`.

## Section 4 — merge RTA into Schedulability Analysis (IV-D)
- [x] Cut §V-B "Response Time Distribution Analysis" (section5 L37–L79, incl. `section_rta`, experimental/analytical/Comparison, `eq: rta_scalar`, `eq_prob_rta`).
- [x] In section4 Schedulability Analysis: replace the pointer sentence "Response time distribution can be obtained either experimentally or analytically... Check Section~\ref{section_rta} for more details." (L151–L153) with the cut RTA content.
- [x] Reword the RTA intro "This section discusses two methods..." → "This subsection discusses the two methods; our system exploits the advantages of each." (it is now a subsection of Schedulability, not its own section).

## Section 5 — remove the two moved subsections
- [x] Delete §V-A (L6–L34) and §V-B (L37–L80) from section5_sp_opt_problem.tex.
- [x] Verify section5 now goes: §V intro → `\subsection{Safety metric}` (was §V-C).
- [x] Confirm §V intro (L1–L3) still reads coherently without ET/RTA — added a bridge sentence pointing to Section IV for the modeling.

## Cross-reference / label check
- [x] `section_et_model` label now appears once (in section4 IV-B).
- [x] `section_rta` label now appears once (in section4 IV-D, on Analytical method subsubsection).
- [x] `eq: et_predict`, `eq: rta_scalar`, `eq_prob_rta` labels each appear once (moved to section4).
- [x] No `\ref{section_rta}` self-reference left in section4 Schedulability.
- [x] Section5 §V-G still `\eqref{eq: rta_scalar}` — resolves to section4 (back-ref, fine).

## Verify
- [x] `mcp__ide__getDiagnostics` (or pdflatex if available): no undefined references, no new errors.
- [x] Grep all section*.tex for the moved labels → every `\ref`/`\eqref` resolves.
- [x] Append dev_log entry + top-level `agents/dev_log.md` milestone.

## Folded-in: 3-way split of merged Computation-Tasks+ET block (2026-08-05 cont.)
- [x] Split the merged IV-A (task model + ET modeling + task-type defs) into three subsections:
  IV-A Computation Tasks (task model + `fig_rts_concepts`) / IV-B Modelling Execution Time
  Distribution / IV-C Task Classification (NEW `\label{section_task_classification}`).
- [x] Reorder task-type block inside IV-C: distinction → `def_env_task` → motion-planning
  example → `def_task_config` → QoS note.
- [x] Resulting §IV order: IV-A → IV-B → IV-C → IV-D Computation Platform → IV-E Schedulability.
- [x] `def_env_task`'s `(Section~\ref{section_et_model})` now a back-ref (IV-C→IV-B).
- [x] Preserve user's in-progress IV-E edit ("In case of probabilistic execution time
  distribution… via convolution") verbatim.
- [x] IDE diagnostics clean; all moved/new labels defined once in build; referrers resolve.
- [x] Append dev_log entry + top-level `agents/dev_log.md` milestone; update README + MEMORY.
