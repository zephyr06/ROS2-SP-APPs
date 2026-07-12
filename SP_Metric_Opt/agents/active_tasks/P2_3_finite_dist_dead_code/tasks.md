# P2.3 — Tasks (working checklist)

> See `goal.md` for scope. Small cleanup; TDD not strictly required (deletion),
> but `make check.SP_OPT -j5` must stay green.

- [ ] Re-confirm zero production callers: `grep -rn "approx_equal" sources/`
      (expect only `Probability.h:45` `approx_equal_double` helper,
      `Probability.h:148` decl, `Probability.cpp:14-15,345-355,365-366` impl +
      sibling; no `sources/` callers outside the class itself).
- [ ] Decide delete vs wire-up (default delete per design rules). Record in
      `dev_log.md`.
- [ ] If delete: remove `FiniteDist::approx_equal` decl (`Probability.h:148`) +
      impl (`Probability.cpp:345`) + the `operator!=`-adjacent tolerance helper
      if also dead; remove stale tests `testProbability.cpp:41-70`.
- [ ] `make check.SP_OPT -j5` green.
- [ ] Milestone to top-level `agents/dev_log.md`.
