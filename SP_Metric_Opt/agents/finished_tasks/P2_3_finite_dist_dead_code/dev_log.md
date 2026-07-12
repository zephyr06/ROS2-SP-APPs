# P2.3 — `FiniteDist::approx_equal` dead-code cleanup — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-08

- Task created from the P1.1 investigation's "Fix D inert" finding
  (`investigation_summary.md` §5.5): `FiniteDist::approx_equal`
  (`Probability.cpp:345`) has zero production callers; the live comparison
  `operator!=` hardcodes `tolerance=1e-1` inline and does not delegate to it.
  Fix D was ruled out as the wrong lever for P25, but left this dead code.
- Verified this session: `grep -rn "approx_equal" sources/` returns only the
  class's own internals + `tests/testProbability.cpp` — no production callers.
- Not yet started. Default plan: delete `approx_equal` + stale tests.

## 2026-07-12 — DONE (wire-up path, not delete)

- User began the **wire-up** option (goal.md option 2) themselves: rewrote
  `FiniteDist::operator==` to `return this->approx_equal(other, 1e-2);`,
  replacing the old inline loop. This made `approx_equal` no longer dead code
  (it is now the delegated-to body of `operator==`), so the "delete" default no
  longer applied. But the change broke `testProbability.cpp`.
- **Root cause of the break:** the tolerance. The old inline `operator==`
  hard-coded `tolerance = 1e-1`. The user's `1e-2` tightened it 10×, which flips
  `EXPECT_TRUE(dist1 == dist1_approx)` at `testProbability.cpp:35`
  (`dist1={(3,0.1),(7,0.9)}` vs `dist1_approx={(3.1,0.099),(6.9,0.901)}`:
  value 3 vs 3.1 is a 3.3% relative drift — inside 1e-1, outside 1e-2).
- **Why fix the tolerance, not the test:** `FiniteDist::operator==`/`operator!=`
  is the optimizer's **live changed-task detector** (see P1.1 / P25 memory
  `p25-ndiff-diff-semantics`: "Fix D both wrong levers"). Tuning this tolerance
  is an explicitly-ruled-out behavior change to production reoptimization
  detection. A P2 cleanup must be semantically neutral. `approx_equal(other,
  1e-1)` is behavior-identical to the old inline `operator==` — same
  `approx_equal_double` calls for samples AND min/max, same tolerance — so it is
  a true refactor (parameterize tolerance in one place), not a behavior change.
- **Fix applied:** `Probability.cpp:368-370` `1e-2 → 1e-1`.
  ```cpp
  bool FiniteDist::operator==(const FiniteDist& other) const {
      return this->approx_equal(other, 1e-1);
  }
  ```
- **Build/test (DEBUG build dir `build/`, uppercase per sp-opt-test-build-debug
  memory):** `testProbability` suite green — both `FiniteDist.equal` and
  `FiniteDist.approx_equal_respects_tolerance` pass.
- **Pre-existing unrelated failure — NOW FIXED (2026-07-12):** `testScheduleSimulate`
  failed on `OrchestratorTest.CFS_RunOrchestrator_Binary` because
  `build/tests/RunOrchestrator` binary was never built (`sh: RunOrchestrator: not
  found`, `ret=32512`). Confirmed to fail **identically on stashed-pristine
  HEAD** (P2.3 change removed) → NOT a P2.3 regression. Root cause: the
  `RunOrchestrator` executable (`tests/CMakeLists.txt:19-20`, declared outside
  the DEBUG-gated `gtsamAddTestsGlob` glob) is a standalone helper that
  `check.SP_OPT` never depended on, yet `CFS_RunOrchestrator_Binary`
  (`testScheduleSimulate.cpp:717-751`) shells out to it at runtime. Fix: added
  `add_dependencies(check.SP_OPT RunOrchestrator)` (guarded by
  `if(TARGET check.SP_OPT)`) so the binary builds with the test group.
  Verified clean-build: deleted the binary, re-ran
  `cmake --build . --target check.SP_OPT`, the binary rebuilt automatically and
  **16/16 ctest pass**. This fix is in the working tree alongside the P2.3
  `operator==` change; it is a separate concern (build-wiring, not the dead-code
  cleanup) but bundled so the suite is green before commit.
- **Status: DONE.** Working-tree change only (not committed). Top-level
  milestone appended to `agents/dev_log.md`.
