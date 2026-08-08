# PW.2.1 — move ET modeling & RTA to System Model — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-08-05

- Task scaffolded (goal.md, tasks.md). User-directed structural move: pull the
  ET-modeling and RTA subsections out of Section V into Section IV (System
  Models) so §V is purely the SP-metric + optimization problem.
- Cross-reference map built (see goal.md table): all moved labels are global,
  so no `\ref`/`\eqref` target is lost. The only edit beyond the move itself is
  removing the now-self-referential "Check Section~\ref{section_rta}" pointer in
  section4 Schedulability, and rewording "This section..." → "This subsection...".
- Note on reflow convention: section4 has not previously been touched in a PW
  writing sub-task. Per the README convention, a one-sentence-per-line reflow of
  section4 should precede the content commit so the whitespace-only diff is
  separable. Deferred to a separate step/commit per user preference — flagged
  here, not silently skipped.

### Executed

- **IV-B inserted:** pasted the ET-modeling subsection into section4 between
  `fig_rts_concepts` (end of Computation Tasks) and `\subsection{Computation
  Platform}`. Labels `section_et_model`, `eq: et_predict` now live here.
- **IV-D merged:** replaced section4 Schedulability's pointer sentence
  ("...Check Section~\ref{section_rta} for more details.") with the full RTA
  content (experimental / analytical / comparison). Reworded the intro to
  "This subsection discusses the two methods; our system exploits the advantages
  of each." Placed `section_rta` label on the Analytical-method subsubsection
  (where `hp(i)` / `eq: rta_scalar` live — the target §6 L170's `\ref{section_rta}`
  actually wants). Labels `section_rta`, `eq: rta_scalar`, `eq_prob_rta` now
  live here.
- **Section V trimmed:** deleted §V-A and §V-B from section5_sp_opt_problem.tex.
  Added a one-line bridge in the §V intro pointing readers to
  Section~\ref{section: system_model_tasks}--\ref{sectino_schedulability_analysis}
  for the modeling. Section V now opens with the Safety metric.
- **Verification:** all five moved labels defined exactly once (all in section4);
  every live `\ref`/`\eqref` (§4 def_env_task, §5 §V-G, §6 L170/L177/L204, §7 L13,
  §9 L177, §10 L13) resolves to section4. IDE diagnostics clean on both files
  (only pre-existing cosmetic over/underfull hbox + a float-specifier warning on
  the pre-existing Table I). Top-level `agents/dev_log.md` milestone appended.
- The orphan `section5_system_model.tex` (NOT in main.tex) also contains refs to
  these labels; left untouched as it is not part of the build.

## 2026-08-05 (cont.) — folded-in 3-way split of merged Computation-Tasks+ET block

- User follow-up: after the ET/RTA move, the merged IV-A "Computation Tasks" (task
  model + ET modeling + the two task-type defs) was too long; split it into three
  subsections. This is folded into PW.2.1 (NOT a new PW.2.2) per user direction
  ("do that, fold it into p2.1").
- New §IV structure:
  - **IV-A Computation Tasks** — task abstraction, periodic model, Period/Deadline/
    Execution-time defs, Gaussian assumption, + `fig_rts_concepts` figure.
  - **IV-B Modelling Execution Time Distribution** — `section_et_model`, env vector
    `\textbf{E}`, `eq: et_predict`, GPR mention, rolling-average `Example`.
  - **IV-C Task Classification** (NEW subsection, `\label{section_task_classification}`) —
    the two task-type defs, reordered for flow: distinction sentence →
    `def_env_task` (τ^E_i) → motion-planning anytime example → `def_task_config`
    (τ^Q_i) → QoS simplification note.
  - **IV-D Computation Platform** — `section_rta_distribution` (unchanged).
  - **IV-E Schedulability and Response Time Analysis** — `sectino_schedulability_analysis`
    (unchanged content; incl. user's in-progress edit adding the "In case of probabilistic
    execution time distribution… via convolution" sentence, preserved verbatim).
- Rationale: dependency-ordered so every cross-ref inside §IV is a back-ref. In particular
  `def_env_task`'s `(Section~\ref{section_et_model})` now points back to IV-B (was a forward
  ref when ET modeling sat inside IV-A). The motion-planning example is placed between the
  two defs to motivate the QoS-configurable definition that follows it.
- Execution note: Bash python-heredoc and a /tmp script-file approach were both unavailable
  (classifier denial / user-rejected), so the reorg was applied via a full-file Write
  rewrite in the new order. Content preserved verbatim; the only structural change is the
  subsection reordering + the new IV-C heading/label. (Some trailing-whitespace lines may
  differ from the pre-edit file; LaTeX-inert.)
- Verification: IDE diagnostics on section4 clean (only pre-existing Table-I float-specifier
  warning + two underfull-hbox info messages; no undefined refs, no errors). Grep across all
  section*.tex: every moved/new label (`section_et_model`, `def_env_task`, `def_task_config`,
  `section_task_classification`, `fig_rts_concepts`, `eq: et_predict`) defined exactly once
  in the build (section4); the only other hits are in the orphan `section5_system_model.tex`
  (not in main.tex → harmless). Live referrers §4 L134, §7 L13/L52, §14 L21 all resolve.
- NOT committed. section4 one-sentence-per-line reflow still deferred.
