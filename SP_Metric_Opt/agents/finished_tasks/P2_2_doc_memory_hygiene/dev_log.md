# P2 2 doc memory hygiene — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-07

- Task scaffolded during the agents-folder reorg. Not yet started.

## 2026-07-12 — CLOSED as superseded / moot (NOT executed)

User review concluded all three substantive items (a)/(b)/(c) are outdated and
the task should be closed, not worked. Verified against current code + git
history:

- **(a)** The `interval-sweep-stale-flags-bug` memory's premise is stale in the
  *opposite* direction: it claims `compare_optimizers.py` **removed**
  `--on_taskset_config_change` / `--run_root`, but current code **accepts** both
  (`:303` `--run_root`, `:334` `--on_taskset_config_change`); the described
  flag-removal never persisted, the sweep-stage crash doesn't reproduce. So the
  task's "mark RESOLVED-cite-the-fix" edit is the wrong action (the honest note
  would be "premise was wrong / bug no longer reproduces"). **No algorithm-
  performance impact** — even live it would be a sweep-stage CLI plumbing crash
  (exit 2); the algorithm runs in the simulate stage (SP metric / optimizer
  computation unaffected). `run_end_to_end.sh` always runs simulate→sweep→
  aggregate (P22) and the P25 A/B config uses `interval_sweep_seconds_list=[10]`
  (single point → sweep no-op) — e2e doesn't crash. The memory file itself was
  left untouched (out of scope for this task's "update + move"); flagged for a
  separate one-line memory retirement.
- **(b)** §5 of `investigation_problems_encountered.md` predates the P1.1
  resolution. Memory `p25-ndiff-diff-semantics` (2026-07-07) re-derived the
  "2-vs-8 discrepancy" as false-positives (NOT UNRESOLVED) and concluded
  "Fix C & Fix D both wrong levers"; P0.5 (2026-07-10, `a8dba07f`→`7fa2e9d2`)
  resolved the gate (both diff sides carry adopted TL; `ndiff` 5→0). Both §5
  asks moot.
- **(c)** `issues.md` was deleted in commit `66c96c14` (2026-07-06). No file to
  mark #9/#3/#6 on — resolved-by-deletion.
- **(d)** `trial_and_error` already DONE in `finished_tasks/summary.md`
  (committed @ `88af2c54` + `fa0b857f`); P0.1 has since landed. Self-resolved.

Disposition recorded in `goal.md` ("Closure disposition") + `tasks.md`. Folder
`git mv`'d to `finished_tasks/`; top-level `agents/dev_log.md` milestone
appended, `overall_tasks.md` P2.2 row + `finished_tasks/summary.md` entry
updated. **No code, agent doc, or memory file was edited** — the task was
closed, not executed, because every substantive item is superseded or moot.
