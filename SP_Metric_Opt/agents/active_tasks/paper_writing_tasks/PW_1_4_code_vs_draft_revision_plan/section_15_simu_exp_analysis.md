# §15 — Simulation Experiment Result Analysis

> Draft: `section15_simu_exp_analysis.tex` (14 lines). Status: high-level pending row-by-row.
> See `overall_revision_plan.md` for conventions. Label: `section: simulation_exp`.

## High-level change

§15 is a short analysis section (CPU-utilization dynamics + proposed-vs-baseline
comparison under light/heavy overload). The prose is broadly consistent with the
dynamic-env framing (content change 2 — `:7` "tasks' execution time varies … all
tasks higher ET in computationally expensive environments"). Revisions: resolve
the trailing `\agent` note (`:15`, an INCR_WCET-vs-INCR reasoning task —
"pessimistic ET → pessimistic RTA → adopts fast TL → less SP; evaluate whether
true for INCR_WCET"); consolidate the headline number (Ryan 1.3); PW.2 merge
(§14+§15); and (optional) figure legibility + confidence intervals (Ryan 2.1/2.2).

## Action summary

- **RESOLVE** the `\agent` note (`:15`): the INCR_WCET reasoning ("more
  pessimistic ET → more pessimistic RTA → TL optimization adopts faster TL →
  lower SP") — either fold the verified conclusion into body prose or delete if
  the analysis moved elsewhere. VERIFY against the INCR vs INCR_WCET A/B
  ([[p18-incr-wcet-outperforms-incr]]: INCR_WCET *outperforms* INCR, root cause =
  generator ET-feasibility defect — so the naive "pessimistic ET → less SP"
  reasoning may NOT hold; check before stating it).
- **CONSOLIDATE** headline number (Ryan 1.3): one number backed by a table
  (cross-section §12/§13/§15/§16). Currently 15–50% / 20–50% / 20–40%.
- **MERGE** §14+§15 per PW.2's `reorg_plan.md` (structural — reference only) —
  §15 is a one-paragraph-analysis candidate for condensing.
- **VERIFY** the proposed-vs-CFS/RM comparison framing (`:9`,`:11`,`:13`)
  matches the actual baselines run (CFS, RM; Ryan 2.1 wants ≥1 adaptive baseline
  — optional ADD).
- **ADD (optional, Ryan 2.1/2.2)** figure legibility (Figs 4–7 render ≈2.29 in
  wide, ≈3.6 pt labels — enlarge fonts, thin box-plot density) + confidence
  intervals / significance tests on the 50 runs.

## Ryan Cat-1

- 1.3 (headline number — cross-section).
- 2.1 / 2.2 (optional: adaptive baseline, figure legibility, CIs).

## Open notes

Row-by-row detailing deferred. §15 is tiny; the one substantive item is the
`\agent` INCR_WCET reasoning — verify it against [[p18-incr-wcet-outperforms-incr]]
before promoting (the naive reasoning may be inverted by the generator-feasibility
root cause). Flag for PW.4.
