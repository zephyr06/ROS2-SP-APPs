# P2.3 — Tasks (working checklist)

> See `goal.md` for scope. Small cleanup; TDD not strictly required (deletion),
> but `make check.SP_OPT -j5` must stay green.

- [x] Re-confirm zero production callers: `grep -rn "approx_equal" sources/`
      (expect only `Probability.h:45` `approx_equal_double` helper,
      `Probability.h:148` decl, `Probability.cpp:14-15,345-355,365-366` impl +
      sibling; no `sources/` callers outside the class itself).
      — DONE 2026-07-12: still zero production callers of `approx_equal`; the
      only non-self reference is `tests/testProbability.cpp`.
- [x] Decide delete vs wire-up (default delete per design rules). Record in
      `dev_log.md`.
      — DECIDED: **wire-up** (goal.md option 2). `operator==` was already
      partially wired by the user to `approx_equal(other, 1e-2)`; corrected to
      `1e-1` to preserve the old inline tolerance exactly (see dev_log). Delete
      was rejected because `approx_equal` is the cleaner single source of truth
      and is exercised by a pinning test (`approx_equal_respects_tolerance`).
- [x] If delete: remove `FiniteDist::approx_equal` decl (`Probability.h:148`) +
      impl (`Probability.cpp:345`) + the `operator!=`-adjacent tolerance helper
      if also dead; remove stale tests `testProbability.cpp:41-70`.
      — N/A (wire-up chosen). No deletion; `approx_equal` retained as the
      delegated-to body of `operator==`.
- [x] `make check.SP_OPT -j5` green.
      — **16/16 ctest pass.** testProbability suite green (incl. `FiniteDist.equal`
      + `approx_equal_respects_tolerance`). ALSO FIXED: `testScheduleSimulate`'s
      `OrchestratorTest.CFS_RunOrchestrator_Binary` was failing because the
      standalone `RunOrchestrator` binary (`tests/CMakeLists.txt:19-20`, outside
      the `gtsamAddTestsGlob` glob) was never built by `check.SP_OPT`. Added
      `add_dependencies(check.SP_OPT RunOrchestrator)` (guarded by
      `if(TARGET check.SP_OPT)`); clean-build verified (deleted binary → rebuilt
      automatically → 16/16 green). Pre-existing, not a P2.3 regression;
      bundled as a build-wiring fix so the suite is green before commit.
- [x] Milestone to top-level `agents/dev_log.md`.
