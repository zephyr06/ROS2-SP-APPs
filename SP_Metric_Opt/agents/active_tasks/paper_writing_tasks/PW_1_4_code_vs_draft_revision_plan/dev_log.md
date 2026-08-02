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
- **2026-08-02 (cont.) — added Stage 3 to the revision-stages model per user request.** Was
  two stages (Stage 1 content completeness / Stage 2 accuracy+clarity+conciseness polish);
  now three. Split Stage 2's conciseness lens out into a dedicated **Stage 3 = simplify +
  reduce length** (length pass), so Stage 2 = accuracy + clarity only. Rationale: Stage 2's
  "conciseness" lens was doing double duty — incremental "keep tight as you go" while
  writing/verifying vs whole-paper aggressive cutting are different operations and want to
  run at different times. Stage 2 now keeps prose tight incrementally but defers aggressive
  cutting; Stage 3 runs AFTER Stage 2 so cuts land on stable, already-correct prose (never
  cut text about to be rewritten/re-verified). Stage 3 moves = cut redundancy, merge
  passages, tighten prose, structural condensation (section-level merges still owned by
  PW.2 `reorg_plan.md`; Stage 3 handles the prose-level condensation those merges expose).
  Stage 3 changes no technical claims / introduces no new content — remove, merge, condense
  only. Target length is an open decision (journal limit or user-set page/word count);
  flagged as a TODO in the master. Like Stage 2, Stage 3 owns no per-section plan files —
  it's a rubric applied to the prior stage's output, tracked in the section-index Status
  column when the pass begins.
- **2026-08-02 (cont.) — §6 .tex Stage 1 edits APPLIED (complete modification, not QoS-only).**
  Per user instruction: "complete modification of all related records, rather than only
  QoS change, fix all the issues that you can find." All seven §6 subsection rows landed in
  `section6_sp_opt_problem.tex` (file was clean in git — QoS reframe already committed in a
  prior session; this pass layered the remaining Stage 1 content on top). Code-truth
  re-verified before each edit. (1) §6.1: deleted GPR Example + `eq:gpr_predict1` + both
  `\sen` notes + stale `\rkw`/`\Sen`; rewrote lede as environment-dependent ET varying across
  re-optimization intervals; GPR kept as a one-line "one applicable option" mention (NOT
  highlighted, per user "not the paper's core contributions"); exact method deferred to §9
  `predict_ET_exp`; NEW rolling-average Example; label `section_et_model_gp`→`section_et_model`
  (no external `\ref` users). (2) §6.2: `hp(i)`→strictly-higher (both sites); `R_i^0` init
  includes own `C_i` (verified `RTA.cpp:33,47`). (3) §6.3: Option A VERIFY; `Normalize()`
  precise def added as `eq_normalize` (verified `SP_Metric.h:31-41` interpolate between
  `PenaltyFunc(1,Θ)`→0 and `RewardFunc(0,Θ)`→1, clipped). (4) §6.5: Example 2 Normalize ref →
  `eq_normalize` + branch spelling-out. (5) §6.6: "0.9⇒both≥0.9" restricted to per-task product
  term + system-level caveat (Ryan 1.2). (6) §6.7: ADD forward pointers to §7/§8
  (`section_priority_opt`/`section_config_opt`) + new §9 guarantee (`section_safety_fallback`,
  forward-looking — resolves at PW.3 §9 .tex creation); guarantee as built-in self-constraint.
  (7) §6.9: smoothness premise → per-interval-small-ET-change + incremental warm-start
  (content change 2). Forward-looking `\ref{section_safety_fallback}` flagged (only unresolved
  ref). Pre-existing repo-wide macro breakage (`\extDist`/`\rtDist`/`\hptasks`/`\configs`/
  lowercase `\sen` undefined in `main.tex`) left as-is — preamble fix is PW.2 cross-cutting,
  not §6 content. Plan file `section_6_sp_opt_problem.md` updated with implementation log;
  status → ".tex edits APPLIED, awaiting user review." NEXT: §7 .tex (PA-only re-plan: §7.0
  partial problem + §7.4 RTA cache + §7.2 Alg.1 FIX + §7.3 smoothness+DM seed).
- **2026-08-02 (cont.) — §6.7 important-task concept + explicit constraint ADDED.**
  Per user follow-up ("add the explicit constraints that important tasks must meet
  their specified SP thresholds… first introduce the concept of important tasks…
  in the same subsection"). §6.7 now (a) introduces the important-task subset
  $\mathcal{I}$ = top-50% by SP weight $w_i$ (designer's most safety-critical
  tasks must not miss deadlines), and (b) adds NEW formal constraint
  `eq_important_task_constraint` ($Pr(r_i > D_i) \leq \Theta_i,\ \forall \tau_i \in
  \mathcal{I}$) beside `eq_overall_obj`. Non-important tasks contribute to the
  objective only, NOT individually constrained. Code truth verified:
  `ImportantTasksMeetThresholds` (`SP_Metric.cpp:208-239`) gates each important
  task (`is_important`, top-50% by sp_weight, `SP_Metric.h:131`) on
  `GetDDL_MissProbability(...) <= thresholds_node[i]` — the SAME Θ_i as §6.3 safety
  metric (Option A), NOT a separate SP threshold. Constraint stated as a hard gate
  (enforced by §9 fallback, NOT left to the objective to satisfy softly).
  LaTeX verified: 13 equation begins = 13 ends; no IDE diagnostics. Plan file
  updated. Status unchanged (.tex edits APPLIED, awaiting user review).
- **2026-08-02 (cont.) — important-task symbol adopted: $\boldsymbol{\tau}^{VIP}$.**
  Per user ("we need a notation for important tasks, let's use
  $\boldsymbol{\tau}^{VIP}$, also update the symbol table in section system models").
  Replaced the provisional $\mathcal{I} \subseteq \mathcal{T}$ at all four §6.7 sites
  (eq_important_task_constraint + three prose occurrences) with
  $\boldsymbol{\tau}^{VIP} \subseteq \boldsymbol{\tau}$ — consistent with §5's existing
  task-set symbol $\boldsymbol{\tau}$ (NOT $\mathcal{T}$, which the draft does not use
  as the task set). Added a NEW row to the §5 `notation_table`
  (`section5_system_model.tex:25`): "Important task subset & $\boldsymbol{\tau}^{VIP}$",
  placed right after the "Task's importance & $w_i$" row (mid-rule boundary preserved).
  Plan file `section_6_sp_opt_problem.md` updated to match the adopted symbol.
  Status unchanged (.tex edits APPLIED, awaiting user review).
- **2026-08-02 (cont.) — important-task symbol renamed ^{VIP} → ^{safe}.** Per user
  ("let's call it with ^{safe} rather than ^{vip}"). Replaced $\boldsymbol{\tau}^{VIP}$
  with $\boldsymbol{\tau}^{safe}$ at all §6.7 sites + the §5 `notation_table` row.
  "safe" reads more naturally alongside the safety-threshold semantics (the subset
  whose $\Theta_i$ safety thresholds are honored as hard constraints). Status unchanged.
- **2026-08-02 (cont.) — §7 .tex Stage 1 edits APPLIED (PA-only re-plan).** Per
  the §6→§7 sequence, applied all four §7 subsection rows to
  `section7_pa_opt.tex`. QoS reframe was already committed in a prior session
  (`\configs` gone, `\boldsymbol{\mathcal{Q}}` in use); this pass layered the
  remaining Stage 1 content. Code locators re-verified against current source
  (an earlier Explore agent to verify §7 locators failed with "Model not found";
  verified directly instead). (1) §7.0: lede rewritten as explicit partial
  problem statement scoping $\boldsymbol{\mathcal{Q}}$ fixed throughout §7,
  §8 lifts it; forward-ref `section_config_opt`; fixed redundant "However…However".
  (2) §7.1: VERIFY + added BF-as-offline-optimality-reference line (forward-ref
  `section: simulation`, the real simu-exp label in `section14_simu_exp.tex`).
  (3) §7.2: Algorithm 1 pseudocode rewritten against `OptimizeFromScratch(int K)`
  (`OptimizeSP_Incre.cpp:100`) — pool shrinks per partial path
  (`AssignAndUpdateSP` `.erase` at `:92`; draft's full `\taskpool` was wrong);
  copy-before-push (`new_path = path` at `:118`; draft's `Push;Push;Pop` aliasing
  was wrong); `SelectTop` objective defined = lowest accumulated `sp_lost`
  (`CompPriorityPath` `OptimizeSP_Incre.h:60`, `sp_lost` `:55`); "Optimal"→
  "Selected" (Ryan 1.4); surrounding prose aligned; broken `\taskpool` usages
  replaced with real prose; "provably optimal"→"optimal for schedulability, not
  SP". (4) §7.3: smoothness FIX ("continuously"→per-interval-small-ET-change,
  content change 2) + DM-seed ADD (`DeadlineMonotonicPriorityVec`
  `OptimizeSP_TL_Incre.cpp:920` + `GroupLock::kImportantFirst`
  `PriorityBuilders.h:17` + `BuildPriorityPlan` `:34`). (5) §7.4: ADD new
  `section_rta_cache` — why / key idea (champion + single-change invariant
  $|\mathrm{diff}|\leq1$) / interface (`Initialize`/`Evaluate`/`AdoptChampion`,
  `RTA_Cache.h:70/78/90`) / invariant (`ComputeTaskSetDifference` throws on
  $|\mathrm{diff}|>1$, `RTA_Cache.cpp:358`) / forward-ref §8; micro-architecture
  deferred to `section_implementation`. **OPEN DECISION flagged for user: §7.3
  four-scenario table does NOT match code.** Real `AnalyzePriorityChangeStatus`
  (`OptimizeSP_Incre.cpp:290-308`) is a 2×2 over `{et_increased,
  if_highest_weight_unique}` → search direction `{Increase, Decrease}` (selects
  which half of priority positions to re-search; never "no change"; discriminator
  is the single highest-weight task, not a binary important split). Draft's four
  scenarios (incl. two "no change" cases) have no code counterpart. Left
  unwritten — three options recorded in `section_7_pa_opt.md` (rewrite-to-code /
  simplified-framing-with-caveat / defer-to-Stage-2). Plan file + status updated.
  NEXT: §8 .tex (collaborative PA+QoS, complete rewrite per
  `section_8_task_config_opt.md`).
- **2026-08-02 (cont.) — §7.4 RTA cache EXPANDED per user request.** User:
  "rta cache needs more explanation, explicit conditions from code, single-task-
  change condition, and for a task set with N tasks and only one task's ET and
  priority change, how does other tasks' RTA change, which tasks' RTA can be
  safely reused, etc." Re-read `RTA_Cache.h` (full) + `RTA_Cache.cpp` (full) +
  §6.2 RTA notation for symbol consistency (`\rtDist{i}`, `\extDist{i}`, `hp(i)`).
  Replaced the prior terse "Key idea" paragraph with three grounded blocks:
  (1) **Champion + single-change condition** — candidate may change ≤1 task's ET,
  ≤1 task's priority position on its core, or both on the SAME task/core;
  enumerates the violations (2 ETs / core migration / 2 moves / ET+move on
  different tasks) that trigger `|diff|>1`; `|diff|==0` reuses all verbatim.
  Sourced from `IsSingleTaskChange` (`RTA_Cache.cpp:262-347`) + the throw at
  `:358`. (2) **Which RTs reuse** — other cores verbatim (partitioned scheduling
  ⇒ hp(i) is same-core only); on the changed core, p_min=min(old,new_pos),
  p_max=max(old,new_pos): Rule A (`has_et_diff`, ET changed ±move) `pos≥p_min`→
  recompute, above→verbatim bit-identical; Rule B (pure priority move)
  `[p_min,p_max]`→recompute, above p_min AND below p_max→reuse — the below-p_max
  reuse is a SAFE UPPER BOUND not bit-identical (HP set same members/ETs only
  permuted within the window, convolution commutative, but the lossy Compress
  step makes cached ≥ true). Sourced from `ClassifyReusePerTask` (`:368-426`) +
  its code comment (`:406-418`). Recomputed tasks form a contiguous range on one
  core. (3) Interface (Initialize/Evaluate/AdoptChampion) kept; dropped the
  standalone "The invariant" block (folded into the condition block). Plan file
  `section_7_pa_opt.md` §7.4 row updated to match. Status unchanged (.tex edits
  APPLIED, awaiting user review). §7.3 four-scenario-table OPEN DECISION still
  pending.
- **2026-08-02 (cont.) — §7.4 RTA reuse FORMALIZED into lemmas+proofs per user
  request.** User: "instead of using rules, we'll add lemmas and proof for these
  rules; proof doesn't need to be complicated, mostly based on the FTP scheduling
  property that high-priority tasks' RTA are not impacted by low-priority tasks'
  ET and priority; this section needs more mathematical and rigorous description;
  high-level description: N tasks sorted by priorities, one task tau_c's ET and
  priority may be different → new task set bold tau^c; for each tau_i with old
  RTA r_i, how does r_i change in bold tau^c, denoted r_i^c, described based on
  old r_i if reusable." Replaced the two Rule A/B `\begin{itemize}` items with
  formal statements + proofs in `section7_pa_opt.tex`:
  (1) **Observation (cross-core reuse)** — tau_i not on core c ⇒ r_i^m=r_i;
  proof: partitioned scheduling ⇒ hp(i) same-core only ⇒ tau_m's change never
  enters tau_i's eq_prob_rta. (2) **Lemma (execution-time change) = Rule A** —
  ET change (±priority move of tau_m): pos<p_min → r_i^m=r_i bit-identical;
  pos>=p_min → recompute; proof via fixed-priority property (r_i depends only on
  tau_i's own ET + hp(i) membership/ETs/order, never lower-priority tasks):
  pos<p_min ⇒ tau_m notin hp(i) in both orderings, tau_i!=tau_m, hp(i)
  unchanged ⇒ r_i^m=r_i; pos>=p_min ⇒ tau_m in hp(i) with changed ET (or
  tau_i=tau_m) ⇒ recompute. (3) **Lemma (pure priority move) = Rule B** —
  pos<p_min verbatim; [p_min,p_max] recompute (tau_m crosses, membership
  changes); pos>p_max — HP SET identical, only permuted within window,
  convolution commutative + preempting-job count ceil(r_i/T_j) order-independent
  ⇒ lossless r_i^m=r_i; implementation's lossy Compress steps are stochastically
  conservative (mass moved to later/larger RT values only) ⇒ cached r_i
  stochastically dominates true r_i^m ⇒ safe upper bound, never underestimates
  miss-prob (code comment RTA_Cache.cpp:406-418). Rigor framing adopted user's
  bold tau / bold tau^c notation but renamed the changed-task index c→m to avoid
  clash with core c (candidate task set bold tau^m, candidate RTA r_i^m); p_min/
  p_max bracket kept. Environments: `\newtheorem{lemma}{Lemma}` (main.tex:31) +
  `\newtheorem{observation}{Observation}` (:33) already defined; `proof` env NOT
  available (no amsthm) → manual `\noindent\textit{Proof.}…\hfill$\square$`
  (amsfonts loaded for $\square$). Re-verified code conditions before writing:
  IsSingleTaskChange `:262-347`, throw `:358`, ClassifyReusePerTask `:368-426`
  (Rule A `:397-404`, Rule B `:406-425`, comment `:406-418`), GetRTA_OneTask
  `RTA.cpp:32-44` (Compress placement, Convolve commutativity),
  ResolvePreemptionsAndCompress `RTA.cpp:9-30` (CompressDeadlineMissProbability +
  CompressDistributionWithOnlySize conservative). Plan file §7.4 row updated.
  LaTeX verified: 14/14 begin/end, no IDE diagnostics. Status unchanged (.tex
  edits APPLIED, awaiting user review). §7.3 four-scenario-table OPEN DECISION
  still pending.

2026-08-02 (cont.) — §7.3 code-vs-text double-check + two fixes.

User asked to double-check §7.2 (beam search) and §7.3 (incremental PA) against
the code.

§7.2 (Algorithm 1, `OptimizeFromScratch`) — CLEAN. Every pseudocode line checks
out: pool shrinks (`AssignAndUpdateSP:92` erase), copy-before-extend (`:118`),
`SelectTop` = lowest accumulated `sp_lost` (`CompPriorityPath:34-55`), beam width
K (`:100`), lowest-priority-first (`:95`/`:102`), output `partial_paths[0]`
(`:151`). Two minor omissions (acceptable for methodology, not errors): the
`CompPriorityPath` tie-break when `sp_lost` within 5e-2 (weight, then ET,
`:39-50`) and the `std::reverse` (`:152`) flipping internal lowest-first to the
§5 highest-first convention.

§7.3 — TWO discrepancies found.

(1) "at most one level" (L122) was WRONG. Stage 1 pass had marked it VERIFY;
corrected. Code does NOT move the changed task by ±1 position —
`FindPriorityVec1D_Variations` (`:248-286`) removes the task and re-inserts it at
EVERY position in the chosen half (`[0,old]` Increase, `[old,end]` Decrease,
`:254-261`), adopting the best strictly-better position (`OptimizeIncre_SingleTask`
`:354-362`). The changed task can jump many levels; only OTHER tasks retain
relative order. L122 rewritten: "re-search each such task's priority position
over one half of the priority range, adopting the best position found."

(2) Four-scenario table (L134-140) did NOT match code — OPEN DECISION RESOLVED
(user chose option A "do it"). Real `AnalyzePriorityChangeStatus` (`:290-308`)
is a 2×2 over `{et_increased, if_highest_weight_unique(task_id)}` → direction
`{Increase, Decrease}`, never "no change"; discriminator = single uniquely-
highest-weight task (`ParametersSP.h:34-43`), NOT binary important/not-important.
Table rewritten to the real 2×2 (upward/downward by the two factors); closing
sentence states the remove+re-insert-at-each-position + strict-improvement
adoption. Worded to avoid "important" to prevent conflation with `\boldsymbol{\tau}^{safe}`.

All code locators re-verified against current source before editing. LaTeX
verified 13/13 begin/end after edits. §7.3 plan row updated (OPEN DECISION →
RESOLVED; "≤ one level" → FIXED). Nothing committed.
