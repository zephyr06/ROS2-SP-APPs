# P3.1 Efficiency Optimizations — Dev Log
> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-06
- Trimmed `improve_efficiency.md` to reference-style `goal.md` and moved into
  `active_tasks/P3_1_efficiency_optimizations/`. Stripped the obsolete per-TL
  cache item (B) and the inline `Convolve` code block (C — source is the source
  of truth); kept the two live-but-deferred items (PriorityPartialPath→pointers,
  incremental HP-task convolution) with file/line refs. All live items are
  cross-referenced from `overall_tasks.md`'s Deferred/P3 table. Not yet started.

## 2026-07-10
- Moved the challenger-reuse efficiency item from P0.5 Phase-5 issue 5h into
  here. P0.5 5h was "Reuse a single optimizer instance across
  `EvaluateTimeLimitConfig_ScratchOrIncre` calls instead of rebuilding per
  candidate." — a pure efficiency item (perf, not correctness), so it belongs
  in the deferred P3.1 bucket, not the P0.5 redesign. Added as a third
  deferred item in `goal.md` + `tasks.md` with the trade-off: the current
  rebuild-from-champion design (P0.5 5b, decided 2026-07-10) was chosen OVER
  the persistent challenger because the champion tracks the working TL so the
  diff flags only the one task being walked; a persistent challenger would
  drift the diff baseline to non-adopted candidates and flag extras. Pick up
  only if profiling shows the rebuild is a runtime blocker. P0.5 5h marked
  MOVED in P0.5's `tasks.md`.
