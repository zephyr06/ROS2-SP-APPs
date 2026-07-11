# P0.1 — Tasks (working checklist) — RESOLVED 2026-07-10 (none executed)

> **RESOLVED — no checklist item was ever executed; no code was written.**
> Subsumed by P0.5 by construction (functional bug gone) + inspectability write
> discarded by user decision. See `goal.md` banner + `finished_tasks/summary.md`.
> The checklist below is the original plan, kept for the record only.

## P0.1a — Persist adopted TL to YAML (the bug fix)
- [ ] Decide the write site: `UpdateRecords` commit only, or also `SeedStateFromIncumbent`.
      Record decision + rationale in `dev_log.md`.
- [ ] Add a YAML writer helper (generalize `WriteTimeLimitToYamlOSM` at
      `OptimizeSP_Base.cpp:175`, or add a sibling for the taskset file). Must
      round-trip against the read paths in `DAG_Model.cpp` / `RegularTasks.cpp`.
- [ ] Writer semantics: for each TL-optimizable task, set ET to adopted-TL-implied
      ET (`GetUnitExecutionTimeDist(TL)`, matching `UpdateExtDistBasedOnTimeLimit`);
      leave non-TL-optimizable (TL=−1) tasks as their raw Gaussian. Atomic write
      (temp + rename).
- [ ] TDD red: test loads taskset YAML, runs one interval (adopted TL committed),
      reloads YAML, asserts reloaded ET == adopted-TL-implied ET (≠ original
      Gaussian mean). Fails before writer.
- [ ] TDD green: writer lands, test passes.
- [ ] Re-run P1.1 probe (INCR_P10, N=8 taskset_0, interval 0→1); confirm `ndiff`
      → ~2 (only gaussian-only tasks flag). Record in `dev_log.md`.
- [ ] Decide: keep or remove the P1.1 descent-start-TL workaround
      (`OptimizeIncre_w_TL:387` `ReconstructTimeLimitVecFromResOpt()` + guard).
      Re-derive against the corrected file — do not assume.
- [ ] `testIncreOpt_w_TL` + `ctest` green; update any stale start-TL-specific
      expectations to structural invariants (P1.1 precedent).
- [ ] Milestone to top-level `agents/dev_log.md`.

---

> **P0.1b (the data-member refactor) was moved to P0.5** (optimizer iteration
> redesign, full scope = flow + state). P0.1 is now bug-fix-only. See
> [`../P0_5_optimizer_iteration_redesign/`](../P0_5_optimizer_iteration_redesign/).
