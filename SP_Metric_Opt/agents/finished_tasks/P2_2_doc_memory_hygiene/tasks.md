# P2.2 — Tasks (working checklist)

> See `goal.md` for scope. Parallel anytime.

**2026-07-12 — CLOSED as superseded / moot; NOT executed.** All four items were
overtaken by later resolutions before this task was picked up. Verdicts below;
rationale in `goal.md` "Closure disposition." Folder moved to `finished_tasks/`.

## (a) Memory: interval-sweep-stale-flags-bug → CLOSED, NOT edited (premise stale)
- [x] NOT edited — verified current `compare_optimizers.py` STILL accepts
      `--on_taskset_config_change` (`:334`) + `--run_root` (`:303`); the
      described flag-removal never persisted, the crash doesn't reproduce, so
      the task's "mark RESOLVED-cite-the-fix" edit is the wrong action. The
      memory is stale in the opposite direction.
- [x] Confirmed NO algorithm-performance impact: sweep-stage CLI plumbing crash
      only; the algorithm runs in the simulate stage (SP metric / optimizer
      computation unaffected). `--steps simulate aggregate` workaround yields
      correct optimize/aggregate output minus the period-sensitivity data.
- [ ] OUT OF SCOPE HERE: retire the memory file itself (one-line RESOLVED note or
      delete) — left untouched by this task; flag for a separate memory edit.

## (b) P25 doc corrections (§5 of investigation_problems_encountered.md) → OUTDATED, NOT applied
- [x] NOT applied — §5 predates the P1.1 resolution. Memory `p25-ndiff-diff-semantics`
      (2026-07-07) re-derived the "2-vs-8 discrepancy" as false-positives (update
      side using Gaussian-mean TL instead of carried adopted TL), NOT UNRESOLVED;
      concluded "Fix C & Fix D both wrong levers." P0.5 (2026-07-10,
      `a8dba07f`→`7fa2e9d2`) then resolved the gate (both diff sides carry the
      adopted TL; runtime `ndiff` 5→0). §5's "record 2-vs-8 as UNRESOLVED" is moot
      (no longer unresolved); "correct Fix D as inert" is moot (abandoned lever;
      P2.3 behavior-neutrally cleaned the `approx_equal` area).
- [x] NOT applied — memory `p25-incr-et-grows-with-period` superseded by
      `p25-ndiff-diff-semantics` + `p05-subsumes-tl-init-bug`.

## (c) issues.md → OUTDATED, target file deleted
- [x] NOT applied — `issues.md` was deleted in commit `66c96c14` (2026-07-06,
      "remove issues.md"). Resolved-by-deletion; no file to mark #9/#3/#6 on.

## (d) trial_and_error — already DONE (self-resolved)
- [x] Confirmed `finished_tasks/summary.md` records trial_and_error as DONE
      2026-07-05, committed @ `88af2c54` + `fa0b857f`.
- [x] P0.1 has since landed (`a8dba07f`→`7fa2e9d2`) — the "pending commit (P0.1)"
      note is itself stale; the trial_and_error work is committed.

## Done when
- [x] Closure recorded (this file + `goal.md` "Closure disposition" + top-level
      `dev_log.md` milestone + `overall_tasks.md` P2.2 row + `finished_tasks/summary.md`
      entry). No `git diff agents/*.md` or memory-file edits were made — the task
      was closed, not executed, because all items are superseded/moot.
