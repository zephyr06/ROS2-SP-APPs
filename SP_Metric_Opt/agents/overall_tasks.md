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
| **P0.2** Focused BF correctness audit | [`active_tasks/P0_2_bf_correctness_audit/`](active_tasks/P0_2_bf_correctness_audit/) | Verify on one small fixed taskset that `OptimizeSP_TL_BF` enumerates the global optimum and `INCR ≤ BF`. Closes the "is BF optimal?" reviewer question. |
| **P0.3** Run prod pipeline + generate core figures | [`active_tasks/P0_3_prod_figure_run/`](active_tasks/P0_3_prod_figure_run/) | THE publication deliverable. Core figures 1a/1c/1f/ab_a/ab_b/2/3 + NEW `fig_p25_et_vs_period`. Depends on P0.1. |

### P1 — active investigation

| Task | Folder | One-line |
|------|--------|----------|
| **P1.1** P25 residual investigation | [`active_tasks/P1_1_p25_residual_investigation/`](active_tasks/P1_1_p25_residual_investigation/) | 3× ET growth eliminated but literal flip not met. Investigate first (reconcile 2-vs-8 changed-task-count; equal-radii A/B; confirm Fix D inert) — do NOT re-frame or implement Fix C yet. Reference: [`investigation/`](investigation/). |

### P2 — should do (figure safety + doc hygiene)

| Task | Folder | One-line |
|------|--------|----------|
| **P2.1** Confirm Fig 2 sweep runs in prod | [`active_tasks/P2_1_fig2_sweep_confirmation/`](active_tasks/P2_1_fig2_sweep_confirmation/) | Stale-flags bug resolved in code but never confirmed on a full prod run. Can run during P0.3. |
| **P2.2** Doc & memory hygiene | [`active_tasks/P2_2_doc_memory_hygiene/`](active_tasks/P2_2_doc_memory_hygiene/) | Mark memory `interval-sweep-stale-flags-bug` RESOLVED; apply `investigation/` §5 corrections; `issues.md` #9 SUPERSEDED, #3/#6 let-go. Parallel anytime. |

---

## Deferred / let go for future (P3)

Intentionally dropped from the publication path. *Features or perf, not
correctness* — acceptable under "fewer features okay."

| Task | Source | Why let go |
|------|--------|------------|
| Trial-and-error for **incremental priority** assignment | (was NEW TASK #1 in old tasks.md) | Algorithm feature; TL trial-and-error already shipped. Not needed to prove correctness or advantages. |
| `PriorityPartialPath` → pointers | [`P3_1_efficiency_optimizations/`](active_tasks/P3_1_efficiency_optimizations/) §A | Pure perf; runtime figures adequate without it. Revisit only if P1.1 finds ET is a paper blocker. |
| Incremental HP-task convolution O(N²)→O(N) | [`P3_1_efficiency_optimizations/`](active_tasks/P3_1_efficiency_optimizations/) §2A | Already ON HOLD for correctness verification; perf, not correctness. |
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

P0.1 (commit clean baseline) → P0.2 (BF audit) → P0.3 (prod figures) →
P2.1 (fig2 confirm, during P0.3) → P1.1 (P25 investigation, the open research
question) → P2.2 (doc hygiene, parallel anytime). Each P0/P1 task is one
review-and-commit cycle per `agent_coding_rules.md`.
