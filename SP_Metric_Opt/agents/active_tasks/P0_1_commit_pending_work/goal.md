# P0.1 — Commit Pending Uncommitted Work

**Priority:** P0 (blocks the clean baseline all figure work builds on)
**Status:** not started

## Goal

Commit the uncommitted working-tree changes in modular, functional commits so
the tree is clean and all subsequent figure/audit work builds on a committed
baseline. Per `agents/agent_coding_rules.md`: agent runs `git add` only, then
asks the user for review; the user runs `git commit`. One commit per small,
functional change.

## Scope — modular commit groups

| # | Commit | Files |
|---|--------|-------|
| a | INCR_P<n> mode override | `tests/RunOrchestrator.cpp` (`MaybeOverrideReoptPeriod`), `simulation_experiments/configs/p25_period_ab_config.json` |
| b | env-dependent-ratio refactor follow-ups (if any config drift remains after commit `4b44aabc` landed the source) | configs / fixtures as needed |
| c | trial-and-error + patience TL rewrite (already TDD-verified 43/43) | `sources/parameters.yaml`, `sources/Utils/Parameters.{h,cpp}`, `sources/Optimization/OptimizeSP_TL_Incre.{h,cpp}` |
| d | agent docs (this reorg + the investigation docs) | `agents/**` |

## Out of scope

- The agents-folder reorg itself is a **separate doc-only commit** (already
  staged by Task #7). P0.1 covers source + config + remaining agent-doc edits.

## Done when

- `git status` clean after user commits.
- `ctest` 16/16 + `pytest tests/python` green on the committed tree.
- The trial-and-error TL rewrite (currently uncommitted but TDD-green) is on a
  committed commit.

## Notes

- `trial_and_error_tl_opt_task.md` was moved to `finished_tasks/` by the reorg
  with a "pending commit (P0.1)" note in `finished_tasks/summary.md`; flip that
  note to "committed @ <sha>" once this lands.
