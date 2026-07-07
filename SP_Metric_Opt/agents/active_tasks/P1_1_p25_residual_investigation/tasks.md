# P1.1 — Tasks (working checklist)

> See `goal.md` for scope. **Investigation, not implementation** — do not
> re-frame or implement Fix C until the gate is resolved.

## Gate: reconcile 2-vs-8 changed-task-count discrepancy
- [ ] Rebuild with `debugMode:1`
- [ ] Run N=10 `taskset_0` INCR_P10
- [ ] Grep `[INCR-ET-DBG]` / `[INCR-NDIFF-DBG]` from the log
- [ ] Compare instrumented count (8) vs ground-truth YAML diff (2: task ids 4, 9)
- [ ] Document the reconciliation in `dev_log.md`

## Equal-radii A/B (sibling "NEW TASK")
- [ ] Set `ReoptimizationTimeLimitSearchRadius == IncrementalTimeLimitSearchRadius`
- [ ] Run from-scratch vs incremental A/B with equal radii
- [ ] Record: is the residual structural (REOPT genuinely cheaper) or an artifact?

## Confirm Fix D inert
- [ ] Re-verify `FiniteDist::approx_equal` (`Probability.cpp:345-364`) ignores tolerance
- [ ] Record "GetAvgValue band is inert today" as a confirmed finding

## Decision (only after the above)
- [ ] Choose: re-frame to "flat, bounded ET" / implement Fix C / GetAvgValue band
- [ ] Record the decision + rationale in `dev_log.md`
- [ ] Milestone appended to top-level `dev_log.md`
