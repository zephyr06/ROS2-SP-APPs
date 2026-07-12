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
