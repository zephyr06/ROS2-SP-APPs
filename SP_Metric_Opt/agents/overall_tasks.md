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

### P2 — should do (figure safety + doc hygiene)

| Task | Folder | One-line |
|------|--------|----------|
| **P2.1** Confirm Fig 2 sweep runs in prod | [`active_tasks/P2_1_fig2_sweep_confirmation/`](active_tasks/P2_1_fig2_sweep_confirmation/) | Stale-flags bug resolved in code (verified 2026-07-08: `compare_optimizers.py` accepts both `--on_taskset_config_change` and `--run_root`) but never confirmed on a full prod run. Can run during P0.3. |
| **P2.2** Doc & memory hygiene | [`active_tasks/P2_2_doc_memory_hygiene/`](active_tasks/P2_2_doc_memory_hygiene/) | Mark memory `interval-sweep-stale-flags-bug` RESOLVED; apply `investigation/` §5 corrections; `issues.md` #9 SUPERSEDED, #3/#6 let-go. Parallel anytime. |
| **P2.3** `FiniteDist::approx_equal` dead-code cleanup | [`active_tasks/P2_3_finite_dist_dead_code/`](active_tasks/P2_3_finite_dist_dead_code/) | `FiniteDist::approx_equal` (`Probability.cpp:345`) has zero production callers; `operator!=` hardcodes tolerance=1e-1 inline and doesn't delegate. Delete (default) or wire up. Cleanup left behind by the P1.1 "Fix D inert" finding. Parallel anytime. |

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
P2.1 (fig2 confirm, during P0.3) → P1.1 (P25 ET investigation, open research
question) → P1.2 (reopt incumbent-degradation investigation; reads the same P25
A/B data as P1.1 but asks a different question — structural corruption, not ET
growth; parallel anytime until it needs a code change) → P2.2 (doc hygiene,
parallel anytime) → P2.3 (`approx_equal` dead-code cleanup, parallel anytime).
~~P0.1~~ RESOLVED 2026-07-10 (subsumed by P0.5; inspectability write discarded —
see `finished_tasks/summary.md`). Each P0/P1 task is one review-and-commit cycle
per `agent_coding_rules.md`.
