# PW.1.4 — Code-vs-Draft Diff → Section/Subsection Revision Plan

**Priority:** P0 (the bridge from code understanding to writing)
**Status:** not started
**Depends on:** **PW.1.1**, **PW.1.2**, **PW.1.3** (hard — needs all three sketch slices)

## Goal

Produce `revision_plan.md` in this folder: a **section-and-subsection-level
revision plan** keyed to the diff between (a) what the code actually does, per
the three sketch slices (PW.1.1 `sketch_foundations.md`, PW.1.2
`sketch_optimization.md`, PW.1.3 `sketch_fallback.md`), and (b) what the current
draft in `paper_sections/full_paper_sections/*.tex` claims. For every affected
subsection the plan records:

> **draft claim → code reality → revision action** (delete / replace / rewrite /
> merge / relocate) [+ Ryan Category-1 item if any]

This is the artifact PW.3 and PW.4 execute row-by-row. With it, the writing
tasks need not re-read the C++ — they just work the plan. It is also where the
four high-level content changes (GP removal, dynamic/continuous environment,
framework update, important-task guarantee) land as *concrete subsection edits*
rather than vague directives.

PW.1.4 is the **content** layer (what each subsection must say); PW.2 is the
**structural** layer (where sections live / what merges). PW.1.4 cross-links
PW.2's structural moves rather than duplicating them.

## Entry point

Open the three sketch slices alongside the 16 `.tex` sections under
`paper_sections/full_paper_sections/` and Ryan's review
(`paper_sections/review_from_ryan_from_pdf_files.md`). Walk every section and
subsection; for each, record the diff and the required revision action in
`revision_plan.md`.

## What the plan must capture

1. **Methodology sections (6, 7, 8) — primary.** Per subsection: draft claim vs
   code reality vs action. This is where the four content changes concentrate:
   - §6 GP example + GPR equation (`eq: gpr_predict1`) → replace with
     sliding-window + Gaussian-fit predictor (PW.1.1); reframe environment as
     dynamic/continuous (PW.1.1); update framework to match sketch; add
     important-task guarantee (PW.1.3); the Ryan 1.2 math fixes (SP equation,
     Example 2, RTA defs, safety/Θ, `Normalize()`, interpretability).
   - §7 PA update to match code (PW.1.2); Algorithm 1 pseudocode fixes
     (Ryan 1.2/1.4); relabel heuristic not "optimal" (PW.1.2); soften guarantee
     + assumptions/regimes.
   - §8 apply the `\sen` reframing (PW.1.2); update coordinate-descent steps
     (PW.1.2); resolve the `\agent` theorem note (PW.1.2 recommendation).
2. **Sections 1–5, 9–16 — secondary.** Per subsection diff: §1 GP contribution +
   headline number + "first work"; §2 "first work" + related-work framing;
   §5 Gaussian-distribution assumption (KEEP) vs env-dependent/TL def; §9 DM
   building + fallback path + `UntilConvergence` + Ryan 1.3 RRT config; §10
   complexity; §11 GPR removal + relocation + polar footnote; §12–15 merge +
   Ryan 1.3 covariance + headline + task-level ground truth; §16 headline +
   guarantee wording.
3. **Ryan Category-1 mapping.** Every Category-1 item from the review mapped to
   the subsection where its fix lands (so nothing is dropped).
4. **Structural cross-links.** Where a revision action is structural (merge /
   relocate / condense), reference PW.2's `reorg_plan.md` row rather than
   re-deciding it here.
5. **Open decisions flagged in place.** The two open decisions (Θ_i convention
   per PW.1.1's finding; headline number) flagged at the subsections they
   affect, not deferred to a separate list.

## Done when

- `revision_plan.md` has a row per affected subsection with
  `{section, subsection, draft-claim, code-reality, action, Ryan-item?, PW.2-ref?}`.
- Every one of the four content changes appears as a set of concrete subsection
  edits (not just a top-level note).
- Every Ryan Category-1 item is mapped to at least one subsection row.
- A writer of PW.3/PW.4 can execute the edits row-by-row using only this plan +
  the three sketch slices, without re-reading the C++.

## Out of scope

- Writing the actual `.tex` prose (that is PW.3/PW.4).
- Deciding structural merges/moves (that is PW.2 — reference only).
- Re-reading/summarizing the code (that is PW.1.1–PW.1.3 — consume their
  sketches).
- Category-2 items unless they're free to fold into an existing row.
