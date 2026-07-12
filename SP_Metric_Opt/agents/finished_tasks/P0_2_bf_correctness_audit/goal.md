# P0.2 — Focused BF Correctness Audit

**Priority:** P0 (closes the reviewer question "how do you know BF is optimal?")
**Status:** not started

## Goal

Verify on **one small fixed taskset** that `OptimizeSP_TL_BF` truly enumerates
the global optimum (exhaustive priority × time-limit) and that `INCR ≤ BF` on
identical DAGs. This closes the founding `agents/prompts.md` concern ("does INCR
beat BF?") that was mostly a normalization artifact (P12 fix: divide by
`ideal_SP`, not BF) but was never closed with an explicit BF-correctness check.

## Why this matters

- The P12 normalization rework (`dev_log.md:274+`) explains why INCR only
  *ties* BF at N=4 (both 3.7972) rather than beating it: the metric now divides
  by `ideal_SP` (= Σ sp_weight × perf_coeff, a constant), so the absolute ceiling
  is the same for both schedulers.
- That rework assumed BF finds the true optimum. If BF is *not* exhaustive, the
  "ceiling" it provides is unreliable and the normalization is suspect.
- This audit confirms BF itself is correct, independent of INCR.

## Approach (TDD)

1. Pick one small fixed taskset (e.g. N=4, the same one where INCR ties BF).
2. Hand-enumerate the reference optimum: for the priority × time-limit space,
   compute the SP of the best configuration by brute force in the test itself
   (or a reference Python script), independent of `OptimizeSP_TL_BF`.
3. Add a test in `tests/testBF_w_TL.cpp` asserting:
   - `BF_SP ≥ INCR_SP` on the shared DAG (BF must not be worse than INCR).
   - `BF_SP ≥ hand_enumerated_reference_SP` (BF reaches the true optimum).
4. If BF < reference → real bug, investigate. If BF ≥ reference → BF confirmed
   correct, write the finding to `dev_log.md` and close.

## Files

- `sources/Optimization/OptimizeSP_TL_BF.{h,cpp}` — the algorithm under audit.
- `tests/testBF_w_TL.cpp` — new audit test.

## Done when

- New `testBF_w_TL` test asserts `BF_SP ≥ INCR_SP` and `BF_SP ≥ reference_SP`.
- `make check.SP_OPT -j5` green.
- Finding recorded in top-level `dev_log.md` (milestone).

## Out of scope

- Improving BF's performance (it's a correctness ceiling, not a shipped
  scheduler — runtime doesn't matter for the paper).
- Re-running the P25 A/B (that's P1.1).
