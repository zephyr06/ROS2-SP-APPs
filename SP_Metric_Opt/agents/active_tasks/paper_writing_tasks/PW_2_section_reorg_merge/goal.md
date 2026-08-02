# PW.2 — Section Re-org & Merge Plan

**Priority:** P1 (lightweight plan; unblocks clean writing in PW.3/PW.4)
**Status:** not started
**Depends on:** PW.1 (lightly — needs the framing, not the detail)

## Goal

Produce a section-level re-organization **plan** (a table, not prose rewrites):
which sections merge, which move to an appendix, which condense, and where the
section-8 conceptual reframing lands. Doing this *before* writing keeps PW.3/PW.4
from drafting into sections that will be collapsed or relocated.

This is Ryan's Category-2.4 "Structural cleanup" plus the `\sen`/`\rkw` author
notes already embedded in the draft that call for structural change.

## Entry point

Read `paper_sections/full_paper_sections/section*.tex` (16 files) and
`paper_sections/review_from_ryan_from_pdf_files.md` (Category 2.4). Produce
`reorg_plan.md` in this folder.

## What the plan must decide

1. **Merge the split results/analysis sections.** Ryan: "merge the split
   results/analysis sections." Concretely: real-experiments `section12` +
   `section13` → one section; simulation `section14` + `section15` → one
   section. State the new section number/order and what subsection structure
   survives.
2. **Relocate Limitations.** `\rkw` note in `section11_limitations.tex`:
   "we can place this section in an appendix." Author counter: many other
   sections cross-reference its subsections (`§relax_smooth_assumption`,
   `§gang`, `§drift`). Decide: appendix vs. keep-in-body-but-condensed; if
   moved, list every cross-reference that must be re-pointed.
3. **Condense one-paragraph subsections.** Ryan: "condense one-paragraph
   subsections." Identify candidates across sections 9–11 (e.g. the small
   `§predict_ET_exp` sub-sub-sections) and propose folds.
4. **Apply the section-8 conceptual reframing.** `section8_task_config_opt.tex`
   opens with a `\sen` note: "we'll change the idea of this section, and treat
   tasks that need to optimize TLs as a special type of env-dependent task, then
   trigger incremental optimization after assuming tasks' ET change." Decide the
   new section title, where this reframed content sits relative to section 7
   (PA) and the env-dependent-task definition in section 5/6, and whether
   section 8 stays standalone or folds into the incremental-optimization
   narrative.
5. **Ordering of methodology.** Confirm 6 (SP problem) → 7 (PA) → 8 (TL) is
   still the right order given the reframing, or propose a swap.
6. **"Note to Practitioners."** Ryan Category 1.5: T-ASE mandates this
   plain-language section alongside the abstract. Decide placement (after
   abstract) and a one-line scope.

## Done when

- `reorg_plan.md` exists with a table: `current section → action (keep / merge
  into X / move to appendix / condense) → new number/title → cross-refs to fix`.
- Every `\sen` / `\rkw` / `\rkwprev` / `\agent` author note in the draft that
  implies structural change is accounted for (list them + disposition).

## Out of scope

- Executing the re-org in the `.tex` (that happens as part of PW.3/PW.4, file by
  file).
- Content rewriting (PW.3/PW.4). This task only plans the skeleton.
