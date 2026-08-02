# Development Log

> **Archived log** (pre-2026-07-25): [`agents/finished_tasks/dev_log_2026-07-25.md`](finished_tasks/dev_log_2026-07-25.md).
> Per-task detail (design, TDD, verification) lives in each task's folder under
> `agents/active_tasks/` or `agents/finished_tasks/`; the status index is `MEMORY.md`.

---

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

---

> Pre-2026-07-25 milestones (pipeline foundation, multi-core/period, incumbent
> redesign, RTA cache evolution) are in the archived log linked above.
