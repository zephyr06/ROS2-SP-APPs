# PW.1.4 code-vs-draft revision plan — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-08-02

- Task scaffolded. Renamed from the former `PW_1b` (alphabetic suffix dropped per
  the user's "use same notation as other active tasks, e.g. P1_2" rule → numeric
  sub-index). Now PW.1.4, the final slice of the PW.1 stream. Hard-blocked on
  PW.1.1 + PW.1.2 + PW.1.3 (the three sketch slices); consumes them to produce
  `revision_plan.md` (row-per-subsection: draft-claim → code-reality → action).
  Content layer; PW.2 remains the structural layer (cross-linked, not
  duplicated).
- Started `revision_plan.md`. Wrote header + cross-cutting conventions (5: GP removal,
  dynamic/continuous env, framework update, important-task guarantee, Θ_i Option A) +
  the **methodology slice §6–§8** (primary, highest-drift). Row per subsection with
  draft-claim → code-reality → action + Ryan Cat-1 mapping.
- **Structural facts established**: (1) NO monolithic `main.tex` — it's a thin `\input`
  shell (`full_paper_sections/main.tex`); Ryan's `main.tex:NNN` line numbers are stale
  (former monolithic file) → plan uses content/label locators (e.g. `eq:gpr_predict1`,
  `alg:modified_audsley`), not line numbers. (2) Draft was edited AFTER Ryan's review →
  several Cat-1.2 items already fixed → plan marks FIX vs VERIFY (not blind transcription).
- **§6 findings**: §6.1 GP Example + `eq:gpr_predict1` REPLACE (sliding-window + Gaussian-fit);
  §6.2 RTA FIX hp(i)→strict + init+C_i (Ryan 1.2); §6.3 safety already Option A (VERIFY) +
  FIX Normalize() (Ryan 1.2); §6.5 SP-eq + Example 2 already fixed (VERIFY); §6.6 FIX
  "0.9⇒both≥0.9" (false for weighted sum, Ryan 1.2); §6.7 ADD framework + guarantee pointer;
  §6.9 FIX smoothness premise → per-interval-ET-change. Θ-table drift is in §12, NOT §6.
- **§7 findings**: §7.1 BF VERIFY (BF legitimately "optimal"); §7.2 FIX Algorithm 1
  pseudocode (pool-shrink, copy-before-push, SelectTop objective, "Optimal"→"Selected",
  lowest-priority-first convention — Ryan 1.2) + soften "provably optimal" (1.4); §7.3 FIX
  smoothness + ADD DM seed + important-first group lock.
- **§8 findings (MOST STALE)**: lede REWRITE to implement `\sen` env-task reframing (content
  change 3); `eq: incremental_configuration` `‖λ−λ^(k)‖≤δ` + δ=150 Example REPLACE with
  patience-bounded outward coordinate descent over full grid (δ-radius is STALE,
  `RecordCloseTimeLimitOptions` only in tests); PROMOTE `\agent` coordinate-descent note to
  body (matches code: weight-desc/deadline-asc/ID-asc ordering, O(M^N)→O(M·N));
  RESOLVE 1-task-ET-diff as **stated property** backed by RTA-cache single-change invariant
  (`RTA_Cache.cpp:358` throws on |diff|>1), NOT a formal theorem (PW.1.2 recommendation).
- NEXT (after /compact): secondary slice §1–5, 9–16. §1/§2/abstract GP + headline + "first
  work"; §9 DM building + fallback + UntilConvergence + Ryan 1.3 RRT; §11 GPR + polar footnote;
  §12 Θ-table inversion (Option A) + covariance (Ryan 1.3); §16 guarantee wording.
- **2026-08-02 (cont.) — split into per-section files per user request.** Decomposed the
  monolithic `revision_plan.md` into one `section_{n}_{desc}.md` per included section +
  a thin `overall_revision_plan.md` master. Skipped §1/§2/§16 per user (their Cat-1 items
  still captured in the master's coverage map so nothing is dropped). The detailed §6/§7/§8
  row-by-row content was relocated (lightly compressed) into `section_6/7/8_*.md` with a
  High-level-change header per file. §3/§4/§5/§9/§10/§11/§12/§13/§14/§15 got high-level
  change descriptions grounded against the actual `.tex` (Θ-table MPC=0.99 inversion
  confirmed in `section12:29-32`; §13.3 conditional-guarantee drift confirmed at
  `:72-83`; covariance PSD bug confirmed at `section14:30-32,45`; `\agent` INCR_WCET note
  in `section15:15`), each marked "high-level pending row-by-row" with the gating items
  flagged (Θ-table decision → re-run; covariance fix → re-run; §13.3/§11 guarantee
  rewrite). Master holds: how-to-read, 5 cross-cutting conventions, section index table,
  Ryan Cat-1 coverage map, skipped-sections note. Monolithic `revision_plan.md` deleted
  (content fully redistributed).
- **2026-08-02 (cont.) — §7/§8 re-planned + NEW §9 safety fallback per user guidance.**
  User instruction: §7 = PA-only (QoS fixed) → add explicit partial problem statement;
  §8 = collaborative PA+QoS, COMPLETE REWRITE around the unified incremental loop
  (sort by SP weight → per-task dispatch: TL task → trial-and-error, env task →
  incremental PA solver), motivation = RTA-cache reuse; §7 also gets a NEW RTA-cache
  subsection. Rewrote `section_7_pa_opt.md` (§7.0 partial-problem ADD / §7.1 VERIFY /
  §7.2 FIX Alg.1 / §7.3 FIX+DM seed / §7.4 NEW RTA cache) and `section_8_task_config_opt.md`
  (§8.0–§8.5: lede+joint problem / overall flow / serialization / QoS walk / env-task PA
  move / complexity+single-change). Grounded in verified code locators
  (`BuildSerializedTaskQueue` `OptimizeSP_TL_Incre.cpp:436-484`, `WalkSerializedTaskQueue`
  `:486-514`, `WalkOneTaskWithTimeLimitOptions` `:647-694`, `RTACache` `RTA_Cache.h:61`).
  Corrected a prior-plan error: the queue sort is weight-desc stable ONLY (the old
  "deadline asc" was wrong — `TaskSortingHeuristic` uses threshold-asc, and the queue
  doesn't even use that helper). Then **added NEW §9 = safety fallback** per user
  ("why / how / when"): `section_9_safety_fallback.md` grounded in `sketch_fallback.md`
  — §9.0 guarantee statement (self-guarantee, Option A `Pr(r_i>D_i)≤Θ_i`, important-task
  scope) / §9.1 why (SP-vs-sched tension + dynamic-env ET-jump) / §9.2 how
  (`ComputeSafeFallback:1002` on worst-case DAG `BuildDAGForObtainSafeFallBAckAcrossIntervals`
  + offline `OptimizeIncre_w_TL_UntilConvergence:889` + BF gate `AdoptRmFastFallbackIfUnschedulable`)
  / §9.3 when (3 triggers: a=`DetectETJump`, b-i=`UpdateRecords` gate, b-ii=`AdoptFallbackIfUnschedulable`)
  / §9.4 complexity + loud-fail contract. Master section index + Ryan coverage map + tasks.md
  updated for the new section + the 9→10…15→16, concl→§17 renumber cascade (final numbering
  deferred to PW.2 `reorg_plan.md`, not yet started). Ryan 1.4 "soften guarantee" row now
  points to §9 as the methodology home.
