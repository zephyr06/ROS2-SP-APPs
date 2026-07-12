# P0.2 — Tasks (working checklist)

> See `goal.md` for scope. TDD: write the test first, watch it fail, then make
> it pass.

## Setup
- [ ] Pick the small fixed taskset (N=4, the INCR-ties-BF case)
- [ ] Confirm `OptimizeSP_TL_BF` runs on it and produces a single SP value

## Hand-enumerated reference
- [ ] Brute-force the priority × time-limit space in the test (or a reference
      Python script) to get the true optimum SP, independent of `OptimizeSP_TL_BF`
- [ ] Record the reference SP value + the config that achieves it

## Audit tests (in `tests/testBF_w_TL.cpp`)
- [ ] Test: `BF_SP ≥ INCR_SP` on the shared DAG
- [ ] Test: `BF_SP ≥ hand_enumerated_reference_SP`
- [ ] `make check.SP_OPT -j5` green

## Verdict
- [ ] If BF ≥ reference → BF confirmed correct; record finding in `dev_log.md`
- [ ] If BF < reference → real bug; open a sub-investigation before closing
- [ ] Milestone appended to top-level `agents/dev_log.md`
