# P2.3 — `FiniteDist::approx_equal` dead-code cleanup

**Priority:** P2 (doc/cleanup hygiene; not a behavior fix)
**Status:** DONE 2026-07-12 (wire-up path, working tree only — not committed).
`operator==` now delegates to `approx_equal(other, 1e-1)`; tolerance kept at the
old inline `1e-1` to stay behavior-neutral. **16/16 ctest green** (also fixed a
pre-existing build-wiring gap: `RunOrchestrator` binary now built by
`check.SP_OPT` via `add_dependencies`). See `dev_log.md`.

## The issue

`FiniteDist::approx_equal` (`sources/Safety_Performance_Metric/Probability.cpp:345`)
is **dead code in production**: it has zero production callers. The only callers
are `tests/testProbability.cpp` and its own sibling logic. The live comparison
used by the optimizer is `FiniteDist::operator!=` (`Probability.cpp:359-368`),
which hardcodes `tolerance=1e-1` in its own inline loop and does **not** delegate
to `approx_equal`.

This was confirmed during the P1.1 investigation (see
`agents/active_tasks/P1_1_p25_residual_investigation/investigation_summary.md`
§5.5 "Fix D inert"): Fix D (the idea of tuning the `GetAvgValue` band /
`approx_equal` tolerance to widen/narrow the changed-task diff) was ruled out as
the *wrong lever* for the P25 residual — but it left behind the finding that
`approx_equal` is dead code.

## Goal

Resolve the dead code, one of:
1. **Delete** `FiniteDist::approx_equal` (and its `operator!=`-adjacent
   tolerance helper if also dead) — ruthlessly prune unused features per the
   project design rules ("We don't consider backward compatibility and
   ruthlessly prune features that are not used anymore").
2. **Wire it up** — make `operator!=` delegate to `approx_equal` so the
   tolerance is parameterized in one place — *only* if there is a concrete
   reason to want the tolerance tunable (there is none today; Fix D was the
   only candidate and it was ruled out).

Default to (1) delete, unless a caller materializes.

## Approach

1. Re-confirm zero production callers (`grep -rn "approx_equal" sources/`).
2. If still zero: delete `approx_equal` (decl in `Probability.h:148` + impl in
   `Probability.cpp:345`), the `operator!=`-adjacent tolerance helper if dead,
   and the now-stale `testProbability.cpp:41-70` tests that exercise only the
   deleted function.
3. `make check.SP_OPT -j5` green.
4. Record in `dev_log.md`.

## Files

- `sources/Safety_Performance_Metric/Probability.{h,cpp}` — the dead function.
- `tests/testProbability.cpp` — stale tests for the deleted function.

## Done when

- `approx_equal` removed (or wired up with a recorded reason).
- `make check.SP_OPT -j5` green.
- Milestone to top-level `agents/dev_log.md`.

## Out of scope

- Fix D as a *behavior* change — ruled out (wrong lever for P25, P1.1 §5.5).
  This task is cleanup of the dead code Fix D left behind, not a revival of Fix D.
- The TL bug (P0.1) or the iteration redesign (P0.5).
