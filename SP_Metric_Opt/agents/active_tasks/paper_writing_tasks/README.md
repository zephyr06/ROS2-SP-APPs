# Paper Writing Tasks

Updating the `paper_sections/` draft to match the current code and to the four
high-level content changes decided for the T-ASE revision. The draft is
significantly out of sync: most of the methodology (sections 6–8) and parts of the
limitations (section 11) describe an older design (notably Gaussian-process
prediction, which is **absent from the code**) and omit the important-task
safety-performance guarantee the code now implements.

Each sub-task follows the repo's 3-file convention (`goal.md` / `tasks.md` /
`dev_log.md`) and `<prefix>_<num>_<snake_desc>` folder notation. The canonical
revision checklist is `paper_sections/review_from_ryan_from_pdf_files.md` (Ryan's
T-ASE review; Category 1 = strictly necessary, Category 2 = ideal).

---

## Sub-tasks

The PW.1 stream (code-grounding) is split into four small slices so each is a
focused read → one sketch file. PW.1.4 (revision plan) consumes the three sketches.
All three PW.1.* sketches are written (2026-08-02); PW.1.4 `revision_plan.md` has the
methodology slice §6–8 done, secondary slice §1–5/9–16 next.

| ID | Status | Folder | Purpose | Depends on |
|----|--------|--------|---------|------------|
| **PW.1.1** | ✅ done | `PW_1_1_foundations_and_framework/` | Code read: SP metric + pRTA + RTA cache + per-interval loop + dynamic env + **Θ_i finding** → `sketch_foundations.md`. | — |
| **PW.1.2** | ✅ done | `PW_1_2_priority_and_config_optimization/` | Code read: PA (BF / modified-Audsley+beam / incremental ±1) + TL coordinate descent + env-task reframing → `sketch_optimization.md`. | — |
| **PW.1.3** | ✅ done | `PW_1_3_important_task_fallback_conv/` | Code read: important-task safe-fallback guarantee (new claim) + offline convergence loop → `sketch_fallback.md`. | — |
| **PW.1.4** | 🔄 in progress | `PW_1_4_code_vs_draft_revision_plan/` | Diff the three sketches vs the draft, section-by-subsection → `revision_plan.md` (draft-claim → code-reality → action) that PW.3/PW.4 execute row-by-row. **§6–8 (methodology) DONE; §1–5,9–16 next.** | **PW.1.1, PW.1.2, PW.1.3** |
| **PW.2** | ⬜ not started | `PW_2_section_reorg_merge/` | **Structural** re-org / merge plan (merge split results+analysis, relocate Limitations, condense one-paragraph subsections, §8 reframing). | PW.1.* (lightly) |
| **PW.2.1** | ✅ done | `PW_2_1_sec5_move_et_rta_to_sysmodel/` | **Executed** PW.2 moves: (1) relocated §V-A ET modeling → new IV-B and merged §V-B RTA → IV-D Schedulability, so §V holds only the SP metric + optimization problem; (2) folded-in 3-way split of the merged Computation-Tasks+ET block → IV-A Computation Tasks / IV-B ET Distribution Modeling / IV-C Task Classification (NEW), so §IV = IV-A→IV-B→IV-C→IV-D Computation Platform→IV-E Schedulability. User-directed 2026-08-05. | — |
| **PW.3** | ⬜ blocked | `PW_3_methodology_rewrite/` | Rewrite methodology §6–8: remove GP, reframe as dynamic/continuous env, update framework to match code, add important-task guarantee; Ryan Cat-1 math fixes. | **PW.1.1–PW.1.4**, PW.2 |
| **PW.4** | ⬜ blocked | `PW_4_non_methodology_sections/` | Update §1–5, 9–16: intro GP removal + headline number, related-work "first work" softening, §11 GPR removal, conclusion number, results ground truth. | PW.1.1–PW.1.4, PW.3 |

Two cross-cutting findings from the reads gate the later writing: Θ_i = **Option A**
(`Pr(r_i>D_i)≤Θ_i`), and the §13.3 guarantee is **self-guaranteed** in code (not
conditional) — see each sketch's "draft drift" section.

---

## The four content changes

1. **Remove Gaussian-process (GP) entirely.** The code does **not** use GP regression
   (grep `gaussian.process`/`GPR`/`Rasmussen`/`kernel regress` = empty). Actual ET
   prediction is sliding-window sampling + Gaussian-dist fit (`§predict_ET_exp`).
   Distinct from the **Gaussian distribution** assumption for ET (§5, §9) — that stays.
2. **Highlight a dynamic, continuous environment** — tasks' ET changes across
   reoptimization intervals, not the static/GP-smooth framing.
3. **Update the optimization framework** to match code: modified Audsley + beam
   search; incremental ±1-priority; TL coordinate descent; DM seed + important-first
   group lock; `OptimizeIncre_w_TL_UntilConvergence`; RTA cache.
4. **Safety-performance guarantee on important tasks** via a safe fallback the
   framework itself finds (`ComputeSafeFallback`, worst-case DAG across intervals,
   RM/DM-fast group-locked plan, the during-walk important-task gate in `UpdateRecords`).

## Sequencing

- **PW.1.1–PW.1.3 first (reading).** The biggest risk is methodology prose that is
  *plausible but wrong* vs the code. The sketches state, in pseudocode + prose
  sourced from the code, exactly what the framework does **and why each component
  exists**. Every later claim is checked against them. Independent slices, parallel.
- **PW.1.4 turns understanding into an edit plan** — walks every section, records per
  subsection: draft claim → code reality → action (delete/replace/rewrite/merge/
  relocate). PW.3/PW.4 execute the plan row-by-row without re-reading the C++.
- **PW.2 is lightweight** — the *structural* layer only (where sections live); PW.1.4
  is the *content* layer (what each says). PW.1.4 cross-links PW.2's moves, not duplicates.
- **PW.3 first real writing (§6–8)** — where the four changes + Ryan's Cat-1 math
  fixes concentrate. Hard-blocked on PW.1.1–PW.1.4; follows PW.2's re-org.
- **PW.4 last** — intro/related-work/conclusion/limitations/results depend on settled
  methodology. Several are pure text-only Cat-1 fixes (typos, `Table I \ith{i}`, GP
  removal from contributions, "first work" softening, polar footnote deletion).

## Two open decisions (resolve when reached)

- **Θ_i convention** (Ryan Cat 1.2): definition says Θ_i = max tolerable miss prob
  (`Pr(r_i>D_i)≤Θ_i`), but the experiment table gives the most critical task the
  *largest* Θ. **PW.1.1 verified Option A** (code reads Θ as max miss prob). PW.1.4/
  PW.3/PW.4 apply one convention everywhere — if Option A inverts table numbers, a
  re-run may be needed (flagged).
- **Headline number** (Ryan Cat 1.3): currently 15–50% / 20–50% / 20–40% in different
  places. PW.4 consolidates to one number from the already-collected prod data (no re-run).

Ryan-review coverage targets Category-1 items; Category-2 items are folded in where
cheap (figure legibility, adaptive baseline, task-level ground truth). Full checklist
in `review_from_ryan_from_pdf_files.md`; each sub-task's `goal.md` owns its items.

---

## Convention

- One folder per sub-task, 3 files: `goal.md` (scope/entry/done-when), `tasks.md`
  (working checklist), `dev_log.md` (chronological log).
- Folder naming `<prefix>_<num>_<snake_desc>` (matches `P0_3_*`, `P1_2_*`, `P3_11_*`).
- On completion, append a one-line milestone to the top-level `agents/dev_log.md`.
- Writing changes go into `paper_sections/full_paper_sections/*.tex` (+ `main.tex`
  for cross-cutting fixes). The PW.1.* sketches and PW.1.4's `revision_plan.md` stay
  in their task folders as durable grounding artifacts.
- **One-sentence-per-line reformat before any content edit.** Before modifying a
  section `.tex`, first reflow it so each sentence occupies one line of code, with a
  blank line separating paragraphs (sentences in the same paragraph stay on
  consecutive lines; no blank line between them). This is a standalone, content-free
  commit (no wording changes) made *before* the content-edit commit, so reviewers can
  diff the reformat (whitespace-only) separately from the substantive changes. Apply
  once per section the first time it is touched in a writing sub-task.
