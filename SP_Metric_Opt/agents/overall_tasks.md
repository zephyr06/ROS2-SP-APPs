# SP-Metric Optimization — Overall Task List

> The TODO index. Active tasks point into `active_tasks/<Pn_n_name>/` (each has
> `goal.md`, `dev_log.md`, `tasks.md`); completed work is summarized in
> `finished_tasks/summary.md`; the canonical chronological narrative is
> `dev_log.md`. Priorities: P0 blocks publication / correctness; P1 is active
> investigation; P2 is should-do hygiene; P3 is deferred (let go for future).
> *Smaller value = higher priority.*

**Overall goal:** reliable, accurate figures for paper publication, within a
limited time budget. Two buckets: (1) **code development** — correct
functionality + reasonable algorithm design; bugs unacceptable, fewer features
okay; (2) **figures & analysis** — core figures are must-have (prove algorithms
work + show advantages over baselines), not all analysis is necessary.

---

## Active tasks

### P0 — blocks publication / correctness & clean baseline

| Task | Folder | One-line |
|------|--------|----------|
| ~~**P0.1** Persist adopted TL to YAML~~ | — | **RESOLVED 2026-07-10 (subsumed by P0.5; inspectability write discarded).** Functional TL-init bug fixed by P0.5 by construction (`CommitIncumbent` single-writes `res_opt_`; `BuildChallengerFromIncumbent` rebuilds the adopted-TL DAG → both diff sides carry the adopted TL → the P1.1 false-positive class structurally impossible; runtime-confirmed `ndiff` 5→0). The demoted inspectability remainder (overwrite the taskset YAML with the adopted TL after each commit, for post-run debuggability) was never implemented and is **discarded** — the orchestrator clamps job ET to `res.id2time_limit` (`SimulationOrchestrator.cpp:461-463`), so no scheduler decision honors the stale Gaussian once optimization has run. No code was ever written for P0.1. See `finished_tasks/summary.md`. |
| **P0.2** Focused BF correctness audit | [`active_tasks/P0_2_bf_correctness_audit/`](active_tasks/P0_2_bf_correctness_audit/) | Verify on one small fixed taskset that `OptimizeSP_TL_BF` enumerates the global optimum and `INCR ≤ BF`. Closes the "is BF optimal?" reviewer question. |
| **P0.3** Run prod pipeline + generate core figures | [`active_tasks/P0_3_prod_figure_run/`](active_tasks/P0_3_prod_figure_run/) | THE publication deliverable. Core figures 1a/1c/1f/ab_a/ab_b/2/3 + NEW `fig_p25_et_vs_period`. Depends on P0.5 (trustworthy incumbent) rather than P0.1. |
| **P0.4** Project evaluation suite (north-star integration test) | [`active_tasks/P0_4_project_evaluation_suite/`](active_tasks/P0_4_project_evaluation_suite/) | Slow (~20 min) deterministic integration test on N=4/6/8 — measures avg SP + scheduler ET vs the `project_evaluation_northstar.md` red-flag lines. The single tuning target for any code/algorithm change. Filed only; not started. |
| ~~**P0.5** Redesign optimizer iteration process (incumbent state)~~ | [`finished_tasks/P0_5_optimizer_iteration_redesign/`](finished_tasks/P0_5_optimizer_iteration_redesign/) | **RESOLVED 2026-07-10** (committed `a8dba07f`→`7fa2e9d2`; 46 `testIncreOpt_w_TL` + 16/16 ctest green, DEBUG re-verified at closeout). Architectural redesign of the incumbent STATE — owned once in `res_opt_` (no parallel `prev_optimizer_` cache), `CommitIncumbent`/`BuildChallengerFromIncumbent` helpers, transient challenger gated by `IfInitialized()` (the `has_incumbent_` bool was removed in Phase-5 issue 5d as provably redundant — `CommitIncumbent` is the single writer of `opt_pa_`). `prev_optimizer_` member removed; Phases 1–5 complete. Phase-5 review issues: 5a unified `ResetIncumbentBaseline`, 5c dropped the stale-TL guard, 5e renamed `starting_time_limits`, 5f removed dead `any_eval_ran` fallback, 5g total-budget patience; 5b kept rebuild-from-champion; 5h (efficiency) moved to P3.1. **Subsumes the functional TL-init bug by construction** — confirmed at runtime: the P1.1 INCR_P10 probe went `ndiff` 5 → 0 at call=0 (only ever 0 or 1 across all 302 calls). See `finished_tasks/summary.md`. |

### P1 — active investigation

| Task | Folder | One-line |
|------|--------|----------|
| **P1.1** P25 residual investigation | [`active_tasks/P1_1_p25_residual_investigation/`](active_tasks/P1_1_p25_residual_investigation/) | 3× ET growth eliminated but literal flip not met. Investigate first (reconcile 2-vs-8 changed-task-count; equal-radii A/B; confirm Fix D inert) — do NOT re-frame or implement Fix C yet. Reference: [`investigation/`](investigation/). |
| **P1.3** `ReoptStartFromAdoptedTL` A/B regression | [`active_tasks/P1_3_adopted_tl_regression/`](active_tasks/P1_3_adopted_tl_regression/) | First A/B with `INCR_P<n>_ADOPTED` arms (commit `8cbbbc12`) collapsed far below plain twins. **Root cause FOUND: stale `release/` binary** (built 2026-07-08, predates the 2026-07-11 commit; `run_end_to_end.sh` defaults `BIN_DIR=release` with no rebuild). Under the pre-commit code `_ADOPTED` is an unrecognized mode → dispatch fall-through → empty `ResourceOptResult` → period-independent degenerate schedule. Fingerprint: all four `_ADOPTED` arms produce byte-identical SP traces across P1/P10/P30/P60. Code itself verified sound (49 + 16/16 ctest green on the DEBUG build). **Step 2 DONE 2026-07-11**: `release/` rebuilt (mtime 11:26, after the commit) & verified fresh (`adopted` strings 0→3, mangled symbol linked); functional probe confirms the stale fingerprint is GONE — `INCR_P1_ADOPTED` SP 0.322→0.729, no longer byte-identical across periods at the collapsed SP. **Step 3 DONE 2026-07-11**: user re-ran the A/B on the fresh binary; `_ADOPTED` arms now distinct per-period and within 0.3–0.8 SP pts of plain twins (P1 0.5793→0.5710, P10 0.5597→0.5527, P30 0.5521→0.5491, P60 0.5456→0.5456 byte-identical), gap shrinking with reopt period. Verdict: stale-binary artifact, code sound — but the result seeded P1.4 (the incumbent seed is slightly worse; the user then required it anyway on algorithmic grounds). |
| **P1.4** Reopt seed TL = carried incumbent (permanent, unconditional) | [`active_tasks/P1_4_reopt_seed_from_incumbent/`](active_tasks/P1_4_reopt_seed_from_incumbent/) | User constraint: the reopt seed TL must be **algorithm-derived** (the optimizer's own prior result), not read from YAML or the generator. The old "off" default (`InitializeTimeLimitsFromETConfig`) is YAML-derived (`et_dist_` from `RegularTasks.cpp:61-106`; generator `mu` independent of the option grid). Closest algorithmic option = `ReconstructTimeLimitVecFromResOpt` (the `res_opt_` incumbent). **Implemented choice (b)**: REMOVED the `ReoptStartFromAdoptedTL` knob entirely (not kept as an ablation opt-out) — `ReOptimizePeriodic` now does `IfInitialized() ? ReconstructTimeLimitsFromResOpt() : InitializeTimeLimitsFromETConfig()` unconditionally; the `IfInitialized()` gate still auto-falls-back at interval 0 / INCR_SCRATCH. Also REMOVED the `INCR_P<n>_ADOPTED` A/B arms + their parsing (a stale `_ADOPTED` config now fails LOUDLY in `MaybeOverrideReoptPeriod` rather than silently dispatching to an empty result — the P1.3 trap); the p25 config is 10→6 arms. TDD red→green (48/48 `testIncreOpt_w_TL` + 16/16 ctest). Accepted tradeoff: ~0.3–0.8 SP pts worse at high reopt frequency (→0 at P60). |
| ~~**P1.5** Add INCR_Px_INIT baselines (default seed)~~ | — | **RETIRED 2026-07-11 (invalidated by P1.4 choice (b)).** P1.5's premise was to add `INCR_P<n>_INIT` arms that set `ReoptStartFromAdoptedTL=false` to A/B the incumbent seed (P1.4's default) against the YAML/generator seed — the same A/B P1.3 ran via `_ADOPTED`. Choice (b) removes the flag entirely (the incumbent seed is the only reopt seed), so P1.5 is impossible as specified. The incumbent-vs-YAML-seed A/B is no longer runnable as a config arm; P1.3's measured tradeoff (~0.3–0.8 SP pts, →0 at P60) stands as the final record. Folder `active_tasks/P1_5_new_baseline_default_seed/` deleted. |
| **P1.6** Pure-incremental baseline (RM-fast bootstrap, no reopt) | [`active_tasks/P1_6_incr_only_baseline/`](active_tasks/P1_6_incr_only_baseline/) | **PLANNING ONLY 2026-07-11 (no implementation; plans updated in a follow-up).** A new paper-grade A/B arm: pure incremental — `OptimizeIncre_w_TL` (warm-started) every interval, **never** `ReOptimizePeriodic`, with the **first interval bootstrapped from RM-fast** (RM priorities + smallest-TL = the orchestrator's `RM_FAST`) instead of a from-scratch reopt. The interval-0 RM-fast seed already exists (`ResetIncumbentBaseline` `else` branch, `OptimizeSP_TL_Incre.cpp:437`); the arm = run that seed step, **skip** the `PerformCoordinateDescentForTaskConfigOpt` descent `ReOptimizePeriodic` runs afterwards, then pure incremental. NOT just `ReoptimizationPeriod=∞` (that still descends at interval 0). Purpose: clean incr-vs-scratch A/B — vs `INCR_SCRATCH` isolates the value of carrying the incumbent via warm-start (both bootstrap RM-fast); vs `INCR_P1` isolates whether the from-scratch descent earns its cost (both carry the incumbent). **INCR_P1-vs-INCR_SCRATCH distinction (resolved 2026-07-11):** both run the from-scratch descent every interval — the difference is optimizer lifetime: `INCR_P1` uses the persistent `incr_optimizer_` (carries `res_opt_` → compare-and-keep vs the running best), `INCR_SCRATCH` builds a fresh `scratch_opt` each interval (amnesiac → compare-and-keep vs a synthetic RM baseline); `INCR_P1` weakly dominates, `INCR_SCRATCH` is the no-memory control. The new arm keeps `INCR_P1`'s incumbent-carrying but drops the descent. **D2+D5 settled** (dispatch = a dedicated `scheduler_mode_` branch mirroring `INCR_SCRATCH`'s shape; paper-grade A/B probe, NOT an E3 gate). D1 (mode name), D3 (interval-0 mechanics), D4 (config placement) open. Also de-risks P1.2 (no `ReOptimizePeriodic` ⇒ the memoryless from-scratch search never runs ⇒ incumbent evolves only via the 1-D `OptimizeIncre` walk). Inherits P1.4's incumbent seed unchanged. See `goal.md`. |

### P2 — should do (figure safety + doc hygiene)

| Task | Folder | One-line |
|------|--------|----------|
| **P2.1** Confirm Fig 2 sweep runs in prod | [`active_tasks/P2_1_fig2_sweep_confirmation/`](active_tasks/P2_1_fig2_sweep_confirmation/) | Stale-flags bug resolved in code (verified 2026-07-08: `compare_optimizers.py` accepts both `--on_taskset_config_change` and `--run_root`) but never confirmed on a full prod run. Can run during P0.3. |
| **P2.2** Doc & memory hygiene | [`active_tasks/P2_2_doc_memory_hygiene/`](active_tasks/P2_2_doc_memory_hygiene/) | Mark memory `interval-sweep-stale-flags-bug` RESOLVED; apply `investigation/` §5 corrections; `issues.md` #9 SUPERSEDED, #3/#6 let-go. Parallel anytime. |
| **P2.3** `FiniteDist::approx_equal` dead-code cleanup | [`active_tasks/P2_3_finite_dist_dead_code/`](active_tasks/P2_3_finite_dist_dead_code/) | `FiniteDist::approx_equal` (`Probability.cpp:345`) has zero production callers; `operator!=` hardcodes tolerance=1e-1 inline and doesn't delegate. Delete (default) or wire up. Cleanup left behind by the P1.1 "Fix D inert" finding. Parallel anytime. |
| **P2.4** Optimizer methods & mode-string refactor | [`active_tasks/P2_4_optimizer_methods_refactor/`](active_tasks/P2_4_optimizer_methods_refactor/) | **Slices A+B + 1-arg overload test migration COMPLETE 2026-07-11 (16/16 ctest DEBUG + 291 python green; `git add` staged, no commit).** Naming/clarity refactor (NOT a computation bug). Trigger: `INCR_P1` reads as "incremental" but is the MAX-reopt arm (`ReoptimizationPeriod=1` → reopt every interval, never incremental; the `P<n>` knob runs the wrong way for a reader). Issue catalogue I1–I5: I1 the `INCR_P1` inversion; I2 plain `INCR` vs `INCR_P1` silently coupled to the YAML `ReoptimizationPeriod` knob; I3 stale `prev_optimizer_` comments (P0.5 removed the member); I4 stale `ReoptStartFromAdoptedTL` history comments (P1.4 removed the flag); I5 method-name/role misalignment in `OptimizePA_Incre_with_TimeLimits`. **Shipped this pass:** Slice A (stale-comment rewrite) + Slice B (mode-string rename `INCR_P<n>`→`INCR_Reopt_X`, X∈{1,5,10,30,60}; X=5 NEW; stale `INCR_P<n>` FAILS LOUDLY — the P1.4 `_ADOPTED` pattern, not a silent alias; bare `INCR` stays canonical = `INCR_Reopt_10` via `parameters.yaml`) + the ~11 test/example 1-arg `ReOptimizePeriodic(K)` call sites migrated to the 2-arg `ReOptimizePeriodic(dag_tasks, K)` form. **DEFERRED:** Slice C (I5 method renames — conflicts with P1.6/P1.2 in-flight edits to `OptimizeSP_TL_Incre`). **Step 5 (SEPARATE result-changing sub-task, NOT this pass):** `INCR_SCRATCH` removal + E2/north-star/P1.6 cleanup (the 1-arg overload stays until Step 5 removes it together with the INCR_SCRATCH branch). Renames are behavior-preserving; the user runs the full new-name A/B. |

---

## Deferred / let go for future (P3)

Intentionally dropped from the publication path. *Features or perf, not
correctness* — acceptable under "fewer features okay."

| Task | Source | Why let go |
|------|--------|------------|
| **P1.2** Reopt incumbent degradation (structural corruption) | [`active_tasks/P1_2_reopt_incumbent_degradation/`](active_tasks/P1_2_reopt_incumbent_degradation/) | Spawned by the Gemini/Kimi debate. Out of scope in current stage of development. The possibility of committing an opportunistic but structurally worse/fragile priority assignment is kept as a known theoretical hazard. |
| Trial-and-error for **incremental priority** assignment | (was NEW TASK #1 in old tasks.md) | Algorithm feature; TL trial-and-error already shipped. Not needed to prove correctness or advantages. |
| `PriorityPartialPath` → pointers | [`P3_1_efficiency_optimizations/`](active_tasks/P3_1_efficiency_optimizations/) §A | Pure perf; runtime figures adequate without it. Revisit only if P1.1 finds ET is a paper blocker. |
| Incremental HP-task convolution O(N²)→O(N) | [`P3_1_efficiency_optimizations/`](active_tasks/P3_1_efficiency_optimizations/) §2A | Already ON HOLD for correctness verification; perf, not correctness. |
| Reuse one challenger across `EvaluateTimeLimitConfig_ScratchOrIncre` calls | [`P3_1_efficiency_optimizations/`](active_tasks/P3_1_efficiency_optimizations/) §3 | Moved from P0.5 Phase-5 issue 5h on 2026-07-10. Perf only. The rebuild-from-champion design (P0.5 5b) was chosen over the persistent challenger — drift risk to the diff baseline. Pick up only if profiling shows the rebuild is a blocker. |
| Issue #3 — ignored `SP_THRESHOLD_RANGE` | `issues.md` #3 | Config flexibility; default `SP_THRESHOLDS_SET` works for paper. |
| Issue #6 — per-task-type config separation | `issues.md` #6 | Config flexibility; not needed for the paper's taskset family. |
| Debug figures 1B/1D/1E/3b | `plan_publication_figures.md` | Already classified Debug-only; not core. Keep code, don't polish for paper. |

---

## Completed

See [`finished_tasks/summary.md`](finished_tasks/summary.md) for the full
index (Pre-2026-07-01, 2026-07-01 P5–P9, P24/P25, trial-and-error TL).
Highlights: P24/P25 commits 1–7 + INCR-ET fix bundle committed (partial A/B
pass); trial-and-error TL rewrite done + TDD-green (pending commit in P0.1).

## Suggested execution order

~~P0.5~~ RESOLVED 2026-07-10 (committed `a8dba07f`→`7fa2e9d2`; subsumes the
functional TL-init bug by construction — see `finished_tasks/summary.md`) →
P0.2 (BF audit) → P0.3 (prod figures; depends on P0.5's trustworthy incumbent) →
P2.1 (fig2 confirm, during P0.3) → ~~P1.3~~ CLOSED 2026-07-11 (stale-binary
artifact; `_ADOPTED` A/B re-run on the fresh binary showed the arms within
0.3–0.8 SP pts of plain twins, gap shrinking with reopt period) → **P1.4**
(choice (b): incumbent seed made the unconditional, only reopt seed — the
`ReoptStartFromAdoptedTL` knob + the `_ADOPTED` arms REMOVED; user rebuilds
`release/` + re-runs the 6-arm A/B to confirm on the loaded tasksets) → P1.1
(P25 ET investigation, open research question) → ~~P1.5~~ RETIRED 2026-07-11
(invalidated by P1.4 choice (b); needed the removed flag) → P1.2 (reopt
incumbent-degradation investigation; reads the same P25 A/B data as P1.1 but
asks a different question — structural corruption, not ET growth; parallel
anytime until it needs a code change) → P2.2 (doc hygiene, parallel anytime)
→ P2.3 (`approx_equal` dead-code cleanup, parallel anytime).
**P2.4** (Slices A+B + 1-arg overload test migration COMPLETE 2026-07-11, 16/16
ctest DEBUG + 291 python green, staged not committed — optimizer methods &
mode-string refactor; trigger = the `INCR_P1` inversion: `ReoptimizationPeriod=1`
→ reopt every interval, never incremental, the `P<n>` knob runs the wrong way
for a reader. Shipped: stale-comment rewrite + mode-string rename
`INCR_P<n>`→`INCR_Reopt_X` (X∈{1,5,10,30,60}, X=5 NEW; stale old names FAIL
LOUDLY; bare `INCR`=`INCR_Reopt_10`). DEFERRED Slice C (method renames, conflicts
with P1.6/P1.2 in-flight edits). Step 5 = `INCR_SCRATCH` removal filed as a
SEPARATE result-changing sub-task. The user runs the full new-name A/B).
**P1.6** (planning only — pure-incremental RM-fast-bootstrap paper-grade A/B arm;
D2+D5 settled 2026-07-11 = dedicated `scheduler_mode_` branch mirroring
`INCR_SCRATCH`'s shape + A/B probe not an E3 gate; settle D1/D3/D4 with the
user, then TDD the bootstrap + dispatch; reads the same P25 A/B harness as
P1.3/P1.4 but adds a new arm, asks a different question — incr-vs-scratch +
whether the from-scratch descent earns its cost).
~~P0.1~~ RESOLVED 2026-07-10 (subsumed by P0.5; inspectability write discarded —
see `finished_tasks/summary.md`). Each P0/P1 task is one review-and-commit cycle
per `agent_coding_rules.md`.
