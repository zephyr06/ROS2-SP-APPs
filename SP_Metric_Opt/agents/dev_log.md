# Development Log

> **Archived log** (pre-2026-07-25): [`agents/finished_tasks/dev_log_2026-07-25.md`](finished_tasks/dev_log_2026-07-25.md).
> Per-task detail (design, TDD, verification) lives in each task's folder under
> `agents/active_tasks/` or `agents/finished_tasks/`; the status index is `MEMORY.md`.

---

## 2026-08-08

- **P1.29 — in-search important-task gate on the INCR reopt from-scratch beam COMPLETE
  (NOT committed; `git add`-only; 17/17 ctest).** Completes the INCR side P1.27
  (`cbdde63b`) left BF-only. The reopt beam ran DISARMED (`ResetIncumbentBaseline(true)`
  → `rta_cache_active_=false` before the beam → `UpdateRecords` no-op else-branch →
  SP-max leaf committed unchecked → backstop binary-swaps to RM-Fast). FIX (3 phases):
  (1) records; (2) behavior-preserving API refactor — `OptimizeFromScratch`/
  `OptimizeIncre`/`OptimizeIncre_SingleTask` return a new `PriorityOptResult` struct
  (`{priority_vec, sp_opt, schedulable}`, mirrors `ResourceOptResult`); callers unpack
  `.priority_vec`; bit-identical; (3) hard-prune at push time —
  `PriorityPartialPath::UpdateSP` (which ALREADY computes each decided task's FINAL
  RTA — HP set = the still-unassigned same-processor tasks = its true future HP
  interferers) returns `false` when `is_important && GetDDL_MissProbability(rta,ddl) >
  thresholds_node[id]`; `OptimizeFromScratch` skips the `pq.push` on false (plain
  greedy top-K otherwise). **Reuses the `UpdateSP` RTA (NOT the 4-arg
  `ImportantTasksMeetThresholds` recompute — user-directed: avoid double RTA cost +
  order-dependence).** Contract (user-settled 2026-08-08: "this priority optimization
  doesn't need to consider fallback, if unschedulable, return unschedulable with an
  empty priority assignment vector"): schedulable → `{priority_vec=<plan>, sp_opt,
  true}`; unschedulable → `{priority_vec={}, INT_MIN, false}`. The priority optimizer
  does NOT itself fall back — the explicit `CallOptimizerGivenTimeLimits`→
  `SeedBaselineAndArmCache`→`ReOptimizePeriodic` wiring was DECLINED; the reopt layer's
  existing backstop `AdoptFallbackIfUnschedulable`@`:784` handles the unschedulable
  result (an emptied beam leaves `res_opt_` = pre-reopt incumbent; `UpdateRecords`
  won't commit `INT_MIN`; verified the empty-beam flow is safe end-to-end).
  `OptimizeSP_TL_Incre.cpp` UNTOUCHED. Bit-identical w/o `is_important` (prune vacuous).
  TDD: `tests/testOptimizeIncrePA.cpp` — `InSearchGate_AdoptsSchedulableLeafOverUnschedulableSpMax`
  (SP-max 0.8 leaf unsched for the important task → prune adopts the 0.2 sched leaf;
  `EXPECT_NEAR(sp_opt,0.2)`+`schedulable=true`) + `InSearchGate_EmptyBeamReportsUnschedulable`
  (both tasks important, point-mass RTA 15 > both ddls → beam empties →
  `EXPECT_FALSE(schedulable)`+`EXPECT_TRUE(priority_vec.empty())`). Does NOT fix P1.28's
  21/22 incremental collapses (by design — those are ET-drift-at-incremental-intervals,
  reopt plan schedulable at reopt time). Detail: `agents/active_tasks/P1_29_reopt_in_search_important_gate/`.

- **P1.29 follow-up refinements (NOT committed; `git add`-only; 17/17 ctest).**
  (1) `UpdateSP` budget-timeout: `if (BFSharedBudgetCancelled())` returned `true`
  (don't prune) → changed to `return false`. On timeout the RTA + important-task
  gate below it never run, so a half-evaluated candidate would be pushed ungated
  (the P1.29 defect resurfacing) with under-counted `sp_lost`. Returning false
  skips the push; an emptied beam returns the unschedulable contract → backstop.
  LIVE on the reopt path (the from-scratch beam runs inside the `BFDLSharedBudget`
  scope @ `Optimize_w_TL_ScratchOrIncre:766`). TDD:
  `InSearchGate_BudgetTimeoutEmptiesBeam` (`BFDLSharedBudget` + `TIME_LIMIT=0` →
  `schedulable==false` + `priority_vec.empty()`; non-vacuous — fails under the old
  `return true`). (2) `OptimizeFromScratch`: `partial_paths.reserve(K)` →
  `reserve(K * N)` (N = task count). (3) Minimized the P1.29-tagged comment blocks
  in `OptimizeSP_Incre.{h,cpp}` + `testOptimizeIncrePA.cpp` (≤3 lines, tags
  dropped). **BF follow-up DONE 2026-08-08 (SEPARATE commit, NOT P1.29; git
  add-only; 17/17 ctest):** the "same 3 updates" framing was wrong about BF's
  structure — items 1 (timeout-prune) & 3 (`reserve(K*N)`) are N/A (BF has no
  beam/partial-path; `EvaluateSPWithPriorityVec` returns `INT_MIN` on any budget
  interruption @ `OptimizeSP_Base.cpp:181-215` so interrupted PAs lose to the
  incumbent → BF only commits COMPLETE PAs; only `reserve` is `reserve(N)` already
  correct), 2 (comment-min) marginal. REAL BF analogue (user reframed: "add a
  sched check when a candidate outperforms the current best; for BF don't worry
  about efficiency") = in-search per-candidate gate in
  `OptimizePA_BF::IterateAllPAs` (`OptimizeSP_BF.cpp`): `if (sp_eval>opt_sp_)` →
  `if (sp_eval>opt_sp_ && ImportantTasksMeetThresholds(dag_tasks_,sp_,pa))`; new
  3-arg `ImportantTasksMeetThresholds(dag,sp,pa)` overload (`SP_Metric.{h,cpp}`)
  for a TL-pre-baked dag (mirrors `EvaluateSPWithPriorityVec`). Each TL combo now
  surfaces its best SCHEDULABLE PA, not just SP-max. Bit-identical w/o
  `is_important`. `OptimizeSP_TL_BF.cpp` UNTOUCHED (P1.27 TL-level gate kept —
  rejects the unsched seed when a TL combo has no schedulable PA → backstop). TDD
  `InSearchGate_AdoptsSchedulablePAOverUnschedulableSpMax` in `testOptimizePA.cpp`
  (same point-mass fixture as INCR P1.29; RED 0.8 unsched → GREEN 0.2 sched).
- **P1.29 refactor — removed the `tl=-1` trick from the gate overloads (NOT
  committed; `git add`-only; 17/17 ctest; speed test PASS).** User flagged the
  3-arg `ImportantTasksMeetThresholds(dag,sp,pa)` overload as wrong-by-design: it
  delegated to the 4-arg via a synthetic `tl_noop=[-1,...]` vector so the 4-arg's
  `ApplyTimeLimitsToTasksExecutionTime` became a no-op (unintuitive sentinel
  leaking into the call graph). Extracted the gate's per-task check into a shared
  anonymous-namespace core `ImportantTasksBelowThresholds(tasks_prioritized, sp,
  node_rtas)` (no `tl`, no baking); all three public overloads (5-arg contract /
  4-arg / 3-arg) reduce to it. 3-arg now prioritizes the already-baked
  `dag_tasks.tasks` + fresh RTA + core — no `tl` sentinel. Behavior-identical
  (the old `tl=-1` bake was a no-op → `tasks_baked==dag_tasks.tasks` → same ops).
  Speed test: INCR_Reopt_1 0.037s/interval, INCR_Reopt_10 0.018s/interval (both
  PASS, < 0.1s target).

- **P1.28 — root-caused INCR_Reopt_10 interval-SP collapse to 0.527888 (RM-Fast fallback
  swap-down); fix not yet implemented.** `taskset_2` `INCR_Reopt_10` collapses to 0.527888
  at intervals 16/21/38/41/44/52 (aggregate 0.855923) while `INCR_WCET` stays 0.885–0.954
  (0.920009) on the identical taskset. The 0.527888 is the RM-Fast fallback, adopted by the
  P0.7 trigger **(b-ii) post-walk backstop** `AdoptFallbackIfUnschedulable`
  (`OptimizeSP_TL_Incre.cpp:1106`) — confirmed 1:1 against `interval_fallback_log.txt`
  (culprit always task 0, `important`, threshold 0.371116, miss_chance 0.43–0.70 > thr).
  This is the **INCR analogue of P1.27**: the reopt from-scratch beam runs DISARMED
  (`ResetIncumbentBaseline(true)` clears the cache before the `from_scratch=true` beam;
  re-armed only after), so the during-walk gate (b-i, `UpdateRecords:238-258`) is skipped
  → the SP-max plan is committed with NO schedulability check → the post-hoc backstop
  binary-swaps it DOWN to RM-Fast instead of re-searching for the schedulable ≥0.93 plan
  that INCR_WCET reaches. The P1.27 fix `cbdde63b` was **BF-only** and never touched INCR.
  INCR_WCET is immune only because `use_wcet_execution_time=true` yields a stricter
  (scheduling-friendlier) ET surface — not a better search. Fix direction: mirror P1.27
  (in-search gate on the INCR reopt beam; keep the backstop as safety net; bit-identical
  for legacy no-`is_important` tasksets). Detail: `agents/active_tasks/P1_28_incr_reopt_fallback_swap_down/`.

## 2026-08-05

- **PW.2.1 — moved ET modeling & RTA out of Section V into Section IV (System Models).**
  Structural re-org (user-directed; one concrete move pulled forward from PW.2's plan-only
  scope). §V-A "Modelling Execution Time Distribution" → new IV-B subsection right after
  Computation Tasks; §V-B "Response Time Distribution Analysis" → merged into IV-D
  Schedulability Analysis (replacing the thin "Check Section~\ref{section_rta}" self-pointer
  with the full experimental/analytical/comparison content). Section V now opens with the
  Safety metric and contains only the SP-metric + optimization problem. All moved labels
  (`section_et_model`, `section_rta`, `eq: et_predict`, `eq: rta_scalar`, `eq_prob_rta`) are
  global, so every `\ref`/`\eqref` in §6/§7/§9/§10/§V-G still resolves (now back-pointing to
  §IV). IDE diagnostics clean (only pre-existing cosmetic over/underfull hbox warnings).
  Task folder: `agents/active_tasks/paper_writing_tasks/PW_2_1_sec5_move_et_rta_to_sysmodel/`.
  NOTE: section4 one-sentence-per-line reflow deferred to a separate commit per user preference.
- **PW.2.1 (cont.) — split merged Computation-Tasks+ET block into three Section IV
  subsections** (folded into PW.2.1; user-directed). After the ET/RTA move, the merged
  IV-A "Computation Tasks" (task model + ET modeling + task-type defs) was too long, so it
  was split into: **IV-A Computation Tasks** (task abstraction, period/deadline/ET-distribution
  defs, Gaussian assumption, + `fig_rts_concepts`) → **IV-B Modelling Execution Time
  Distribution** (`section_et_model`, `eq: et_predict`) → **IV-C Task Classification** (NEW
  `\label{section_task_classification}`; reordered so the two task-type defs flow as:
  distinction sentence → `def_env_task` → motion-planning anytime example → `def_task_config`
  → QoS simplification note). Resulting §IV order: IV-A → IV-B → IV-C → IV-D Computation
  Platform → IV-E Schedulability & RTA. The split makes `def_env_task`'s
  `(Section~\ref{section_et_model})` a back-ref (IV-C→IV-B) instead of a forward ref. All
  labels still defined exactly once in the build; every live referrer (§4 L134, §7 L13/L52,
  §14 L21) resolves. IDE diagnostics clean (only pre-existing cosmetic warnings). User's
  in-progress edit (the "In case of probabilistic execution time distribution… via
  convolution" sentence in IV-E) preserved. NOT committed.

## 2026-08-02

- **PW.1.4 revision_plan.md — methodology slice §6–8 DONE** (T-ASE revision). Row-per-subsection
  edit plan (draft-claim → code-reality → action + Ryan Cat-1 mapping) for PW.3/PW.4 to execute
  row-by-row. §6–8 (primary, highest-drift) complete; §1–5/9–16 (secondary) next. Key findings:
  no monolithic `main.tex` (thin `\input` shell → plan uses label locators, not Ryan's stale
  line numbers); draft edited post-review so several Cat-1.2 items already fixed (marked
  VERIFY). §6: GP Example+`eq:gpr_predict1`→sliding-window+Gaussian-fit REPLACE; RTA hp(i)→strict
  +init+C_i FIX; Normalize() FIX; "0.9⇒both≥0.9" FIX (false for weighted sum); §6.3 already
  Option A (Θ-table drift is in §12, not §6). §7: Algorithm 1 pseudocode FIX (pool-shrink,
  copy-before-push, SelectTop obj, "Optimal"→"Selected") + soften "provably optimal"; ADD DM
  seed. §8 (most stale): `\sen` env-task reframing REWRITE; `‖λ−λ^(k)‖≤δ`/δ-radius REPLACE with
  patience-bounded full-grid coordinate descent (δ-radius STALE); PROMOTE `\agent` note to body;
  1-task-ET-diff = stated property (RTA-cache |diff|>1 throw), NOT a theorem.
- **PW.1.1–PW.1.3 grounding sketches DONE** (T-ASE revision). Three code-read slices
  → `sketch_foundations` / `sketch_optimization` / `sketch_fallback.md`. No code
  changes; nothing committed; PW.1.4 (revision plan) unblocked. Findings: **Θ_i =
  Option A** (`Pr(r_i>D_i)≤Θ_i`); prediction = sliding-window + Gaussian-dist fit,
  NOT GP (grep-confirmed); guarantee SELF-guaranteed in code (draft §13.3 frames it
  *conditional* → drift); `‖λ−λ^(k)‖≤δ`/3-candidates stale (prod walk = patience-bounded
  full grid).
- **P0.7 + P0.10 CLOSED** → `finished_tasks/`. P0.10 (BF-side) §1+§1b `2f6c7c4c`, §2
  `02f3c8fd`: BF gate `AdoptRmFastFallbackIfUnschedulable` inside
  `OptimizePA_with_TimeLimitsStatus::Optimize()` gates every BF caller; fail → RM-Fast
  group-locked plan, double-fail → throw. P0.7 (INCR-side) triggers (a) `DetectETJump` /
  (b-i) during-walk gate / (b-ii) `AdoptFallbackIfUnschedulable` — `c87ae0d4`…`aefed906`/`f371c543`;
  A/B N=[4,6] ≈1% SP penalty (prod 0.7673/0.9220 vs `INCR_NO_FALLBACK` 0.7742/0.9298);
  N=6 clean post-P2.18; N=8 skipped per user.
- **P0.3 figure triage:** fig_p25_et_vs_period + P2.6 sim-RT SP → DEFER P3; P2.1 fig2
  sweep → CLOSE; **fallback-rejection-ratio figure → KEEP & BUILD** (needs a NEW
  per-interval-log reader).

## 2026-08-01

- **P2.18 fix COMMITTED `f371c543`** (P0.7-gate regression, NOT P2.17). During-walk
  gate sourced RTA from `rta_cache_.Evaluate` in `UpdateRecords` (gated by
  `enable_fallback_use_` not `rta_cache_active_`) → Reopt from-scratch beam ADOPTED a
  champion mid-beam → `ComputeTaskSetDifference` throws `|diff|>1` → SIGABRT. Fix:
  predicate `enable_fallback_use_ && rta_cache_active_ && !BFSharedBudgetCancelled()`.
  INCR_WCET taskset_2 exit 0 (was 0-byte SIGABRT); 80/80 N=4 arms exit 0.
- **Commit split LANDED** — P0.7+P2.17 → 3 modular commits (≤3-src-file rule): C1
  `a8148dc7` (P0.7 step 2), C2 `1ef3c26c` (P0.7 step 3 configs+records), C3
  `aefed906` (P2.17 gate wiring: `compare_optimizers.py` routed through gated
  pipeline + `+20` seed step, default ON).
- **P2.19 LANDED (points 1+3)** — `INCR_Reopt_10` SIGABRT fixed. Worst-case DAG
  over-inflated perf WCET (TL-grid bound) → seed picks max TL → important perf task
  missed → throw. Fix: perf task → point mass at MIN TL option; rename
  `BuildDAGForObtainSafeFallBAckAcrossIntervals`. Point (2) → P2.20 (stub).
- **P0.10 §1/§1b/§2 LANDED (TDD).** `RateMonotonicFastGroupLocked` + shared
  `BuildPriorityPlan`/`PriorityBuilderConfig` in NEW TU `PriorityBuilders.{h,cpp}`;
  gate settled INSIDE `OptimizePA_with_TimeLimitsStatus::Optimize()` (2 user
  redirects). `testIncreOpt_w_TL` 125/125; legacy BF green; 16/17 ctest (pre-existing CFS).
- **P0.8 CLOSED.** Important-task schedulability gate fully landed + committed:
  `run_full_generation_pipeline_with_important_task_gate` (retry 20 → loud raise,
  `seed+attempt`), prod-wired default-ON; P2.17 routed `compare_optimizers.py` through
  it. Step 3 rejection-rate = 0 across N=4/8/16. Folder → `finished_tasks/`.

## 2026-07-31

- **P0.6 FINISHED + COMMITTED `630cda4d`.** Offline safe-fallback artifact: deterministic
  `{PA,TL}` (`safe_fallback_`) computed once before the interval loop, for P0.7 to swap
  in. Seed at P0.8-cert point → TL-only walk w/ hard per-candidate gate → keep
  best-SP-feasible. §8 cross-interval worst-case DAG (`BuildWorstCaseDagAcrossIntervals`,
  NEW `WorstCaseDAG.cpp`); loud-fail re-gate on final stored result. 17/17 ctest.
- **P0.7 step 2 LANDED (uncommitted) + step 3 A/B PARTIAL.** A/B hook
  (`enable_fallback_use` + `INCR_NO_FALLBACK`). N=4 A/B done (≈1% SP penalty); **N=6
  blocked by P0.6 loud-fail (NOT P0.7)** → filed P2.17 (gate-wiring bypass).

## 2026-07-29

- **P0.8 config-tuning COMMITTED `7c8748c0`** (12 files +864/−81). (1) perf WCET →
  `execution_time_mu`; (2) env cap 0.45→0.27 + variance [0.3,0.4]; (3) no-inflation
  cpu_util [0.5,1.0] + drop proportional redistribution. `DEADLINE_MODE=implicit`.
- **P0.8 Step 2 prod-wiring + Step 3 LANDED (git add-only).** `run_generator.py`
  routes through gate BY DEFAULT; `run_sim_experiments.py` widens seed step to 20 when
  gate ON (silent-duplicates fix); e2e test asserts gate certificate genuine. 48 py.

## 2026-07-28

- **P0.9 DM + important-first priority lock LANDED + FULLY COMMITTED**
  (`0b9dae4a`/`6a35080b`/`352f13d5`/`7dd1a7ac`). Seed PA system-wide RM→**Deadline
  Monotonic + important-first group lock** so C++ seed agrees with P0.8's Python RTA
  cert. D1–D7 RESOLVED (D6 = behavior change, NOT bit-identical). Lockstep prereq for
  P0.6 + P0.8. 43 py + 351/353 py + 17/17 ctest.
- **P0.8 Step 2b gate wrapper LANDED + staged (uncommitted).**
  `run_full_generation_pipeline_with_important_task_gate` (`orchestrator.py`):
  generation-time gate certifying every emitted taskset schedulable for important tasks
  under DM+top-priority-lock at the seed point. Seed-advancing retry (budget 20, D4),
  loud `RuntimeError` on exhaustion (never silent).

## 2026-07-25 – 2026-07-26

- **P3.6 INCR_NO_REOPT baseline IMPL DONE (uncommitted).** New arm: pure incremental
  w/ RM-fast bootstrap at interval 0, `OptimizeIncre_w_TL` after, NEVER
  `ReOptimizePeriodic`. Paper-grade baseline, NOT an E3 gate. 17/17 ctest. Awaits A/B.
- **P2.13/P2.14/P2.15.** P2.13 miss-rate-vs-SP filed (analytic RTA vs empirical miss
  count). P2.14 `SP_THRESHOLD_RANGE: [0.001,0.9]` (scrub `1.0`); P2.15
  `SP_WEIGHT_RANGE: [0.1,1.0]`. 372/372 py green.

## 2026-08-07

- **P1.27 BF < INCR_Reopt_10 — FIX LANDED (uncommitted; cross-link P0.2).** Anomaly:
  compare_against_bf N=4 run had BF Mean_SP 0.9086 < INCR_Reopt_10 0.9220, violating
  `INCR ≤ BF`. Gap was ENTIRELY taskset_2: BF run.log showed search FOUND
  `Optimal SP 0.793→0.963` (≥ INCR 0.954) but reported `0.527888` (RM-Fast fallback).
  Root cause: P0.10's important-task gate ran POST-enumeration only, so BF adopted
  the SP-max plan even when it FAILED the gate, then `AdoptRmFastFallbackIfUnschedulable`
  swapped it DOWN to RM-Fast — even though a schedulable 0.954072 plan existed (which
  INCR finds via its DURING-WALK gate). Fix: gate each leaf IN-SEARCH in
  `OptimizePA_with_TimeLimitsStatus::Optimize` recursion (`OptimizeSP_TL_BF.cpp`) —
  `if (sp_opt > res_opt.sp_opt && ImportantTasksMeetThresholds(dag, sp, pa, tl))`.
  Post-hoc gate KEPT as safety net. TDD RED→GREEN (`testBF_w_TL.cpp::BF_NotWorseThan_INCR_Reopt`,
  fixture `test_p127_bf_incr_taskset2_i0.yaml` = taskset_2 interval 0). 17/17 ctest;
  legacy bit-identical (gate vacuous w/o `is_important`). DEFERRED: end-to-end re-run.

---

> Pre-2026-07-25 milestones (pipeline foundation, multi-core/period, incumbent
> redesign, RTA cache evolution) are in the archived log linked above.
