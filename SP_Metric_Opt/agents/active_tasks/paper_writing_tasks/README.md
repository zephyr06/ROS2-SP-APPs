# Paper Writing Tasks

Updating the `paper_sections/` draft to match the current code in this repo, and
to the four high-level content changes decided for the T-ASE revision. The draft
is significantly out of sync: most of the methodology (sections 6–8) and parts of
the limitations (section 11) describe an older design (notably Gaussian-process
prediction, which is **absent from the code**) and omit the important-task
safety-performance guarantee that the code now implements.

This folder holds the writing sub-tasks. Each sub-task follows the repo's 3-file
convention (`goal.md` / `tasks.md` / `dev_log.md`) and the repo's
`<prefix>_<num>_<snake_desc>` folder notation (same as `P0_3_*`, `P1_2_*`,
`P3_11_*`). The canonical revision checklist the writing must satisfy lives in
`paper_sections/review_from_ryan_from_pdf_files.md` (Ryan's T-ASE review,
Category 1 = strictly necessary, Category 2 = ideal).

---

## Sub-tasks

The PW.1 stream (code-grounding) is split into four small slices so each is a
focused read → one sketch file. PW.1.4 (the revision plan) consumes the three
sketch slices.

| ID | Folder | Purpose | Depends on |
|----|--------|---------|------------|
| **PW.1.1** | `PW_1_1_foundations_and_framework/` | Code read: SP metric + pRTA + RTA cache + per-interval framework loop + dynamic/continuous environment + **Θ_i convention finding** → `sketch_foundations.md`. | — (do first) |
| **PW.1.2** | `PW_1_2_priority_and_config_optimization/` | Code read: PA (BF / modified-Audsley+beam / incremental ±1) + TL coordinate descent + env-task reframing → `sketch_optimization.md`. | — |
| **PW.1.3** | `PW_1_3_important_task_fallback_conv/` | Code read: important-task safe-fallback guarantee (the new claim) + offline convergence loop → `sketch_fallback.md`. | — |
| **PW.1.4** | `PW_1_4_code_vs_draft_revision_plan/` | Diff the three sketch slices against the draft, section-by-subsection → `revision_plan.md` (draft-claim → code-reality → action) that PW.3/PW.4 execute row-by-row. | **PW.1.1, PW.1.2, PW.1.3** |
| **PW.2** | `PW_2_section_reorg_merge/` | Produce the **structural** re-org / merge plan (merge split results+analysis, relocate Limitations, condense one-paragraph subsections, apply the section-8 conceptual reframing). | PW.1.* (lightly) |
| **PW.3** | `PW_3_methodology_rewrite/` | Rewrite methodology sections 6–8: remove GP, reframe as dynamic/continuous environment, update the optimization framework to match code, add the important-task safe-fallback guarantee. Fix the reviewer-flagged math/pseudocode. | **PW.1.1, PW.1.2, PW.1.3, PW.1.4**, PW.2 |
| **PW.4** | `PW_4_non_methodology_sections/` | Update sections 1–5 and 9–16: intro contributions GP removal + headline-number, related-work "first work" softening, limitations §11 GPR removal, conclusion number, results-section task-level ground truth. | PW.1.1–PW.1.4, PW.3 |

The three PW.1.* code-read slices (1.1, 1.2, 1.3) are **independent of each
other** and can be done in parallel; only PW.1.4 waits on all three.

---

## How to proceed (the discussion)

The user's request distilled to four high-level content changes:

1. **Remove Gaussian-process (GP) entirely** from the paper's description. The
   code does **not** use GP regression — grep for `gaussian.process` / `GPR` /
   `Rasmussen` / `kernel regress` returns nothing. Actual ET prediction is
   sampling / sliding-window based (section 9, `§predict_ET_exp`). Note this is
   distinct from the **Gaussian distribution** assumption for ET (section 5
   line 54, section 9) — that distribution model stays; only the GP *predictor*
   goes.
2. **Highlight a dynamic, continuous environment** where tasks' ET changes at
   different times (across reoptimization intervals), instead of the static /
   GP-smooth framing.
3. **Update the overall optimization framework** to match the code (modified
   Audsley + beam search; incremental ±1-priority adjustment; TL coordinate
   descent; DM seed + important-first group lock; `OptimizeIncre_w_TL_UntilConvergence`;
   RTA cache).
4. **Provide a safety-performance guarantee on important tasks** via a safe
   fallback solution found by the framework itself (`ComputeSafeFallback`,
   worst-case DAG across intervals, RM/DM-fast group-locked plan, the
   during-walk important-task gate inside `UpdateRecords`).

### Sequencing rationale

- **PW.1.1–PW.1.3 first, and they are mostly reading tasks.** The single
  biggest risk in this revision is writing methodology prose that is *plausible
  but wrong* relative to the code (the current draft already suffers this —
  e.g. GP). So before touching any `.tex`, the three code-read slices produce
  grounding sketches (`sketch_foundations.md`, `sketch_optimization.md`,
  `sketch_fallback.md`) that state, in pseudocode + prose sourced from the code,
  exactly what the framework does **and why each major component exists** (its
  motivation). Every later writing claim is checked against them. These are
  also where the deep code investigation the user deferred out of planning
  lands — split into three small tasks rather than one monolithic read. The
  three slices are independent and can run in parallel.

- **PW.1.4 turns that understanding into an edit plan.** With the three sketches
  in hand, PW.1.4 walks every section/subsection of the draft and records, per
  subsection: what the draft currently claims, what the code actually does, and
  the revision action (delete / replace / rewrite / merge / relocate). This is
  the bridge from "we understand the code" to "here is exactly what to write."
  PW.3/PW.4 then execute the plan row-by-row without re-reading the C++.

- **PW.2 is lightweight and overlaps the PW.1.* reads.** It is the *structural*
  layer only — a table of "section X merges into Y / moves to appendix /
  condenses." PW.1.4 handles *content* diffs (what each subsection must say);
  PW.2 handles *structure* (where sections live). Both feed PW.3/PW.4; PW.1.4
  cross-links PW.2's moves rather than duplicating them.

- **PW.3 is the first real writing, on sections 6–8.** Methodology is where the
  four content changes concentrate and where the reviewer's Category-1 math
  corrections live (SP-metric equation, Example 2 arithmetic, Algorithm 1
  pseudocode, RTA definitions, safety/Θ convention). It is hard-blocked on
  PW.1.1–PW.1.4 and should follow PW.2's re-org so the rewrite lands in the
  right structural place.

- **PW.4 last.** Intro/related-work/conclusion/limitations/results depend on the
  methodology being settled (consistent terminology, headline framing, the
  guarantee statement). Several PW.4 items are pure text-only Category-1 fixes
  (typos, Table I `\ith{i}`, GP removal from contributions list, "first work"
  softening, polar footnote deletion) and can be picked off quickly once the
  methodology vocabulary is fixed.

### Two open decisions the writing will surface (resolve when reached, not now)

- **Θ_i convention** (Ryan Category 1.2): the definition says Θ_i = maximum
  tolerable miss probability (`Pr(r_i > D_i) ≤ Θ_i`), but the experiment table
  gives the most critical task the *largest* Θ. Option B (redefine Θ_i = min
  required success probability, table values stay, text-only if the code already
  treats Θ as a reliability target) is preferred — **PW.1.1 verifies which way
  the code reads Θ**, then PW.1.4/PW.3/PW.4 apply one convention everywhere.
- **Headline number** (Ryan Category 1.3): currently 15–50% / 20–50% / 20–40% in
  different places. PW.4 consolidates to one number from the already-collected
  prod data (no re-run).

### Ryan-review coverage

These tasks target the Category-1 (strictly necessary) items; Category-2 items
are noted where they're cheap to fold in (e.g. figure legibility, adaptive
baseline, task-level ground truth) but are not the primary scope. The full
checklist is in `review_from_ryan_from_pdf_files.md`; each sub-task's `goal.md`
calls out the specific items it owns.

---

## Convention

- One folder per sub-task, 3 files: `goal.md` (scope / entry / done-when),
  `tasks.md` (working checklist), `dev_log.md` (chronological log).
- Folder naming: `<prefix>_<num>_<snake_desc>` (matches the repo's other active
  tasks, e.g. `P0_3_prod_figure_run`, `P1_2_*`, `P3_11_*`) — numeric
  sub-indices, no alphabetic suffixes.
- On sub-task completion, append a one-line milestone to the top-level
  `agents/dev_log.md`.
- Writing changes go into `paper_sections/full_paper_sections/*.tex` (and
  `main.tex` for cross-cutting fixes). The PW.1.* sketch files
  (`sketch_foundations.md` / `sketch_optimization.md` / `sketch_fallback.md`)
  and PW.1.4's `revision_plan.md` stay in their task folders as durable
  grounding artifacts.
