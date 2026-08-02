# P2.20 — Tasks (stub; filled in when P2.19 lands)

## 0. Prereq
- [ ] P2.19 (points 1 + 3) LANDED + green regression (crash gone, worst-case DAG
      uses min TL for perf tasks).

## 1. Design + TDD red
- [ ] Confirm `ApproxEqualSP` (or equivalent SP tolerance helper) exists + sig.
- [ ] Decide convergence wrapper shape: loop `OptimizeIncre_w_TL`'s serialized
      pass, or a new `OptimizeIncre_w_TL_UntilConvergence` wrapper.
- [ ] Decide max-iteration cap value (guaranteed termination).
- [ ] Confirm `BFDLSharedBudget` (`ComputeSafeFallback:974`) bounds the looped
      runtime across multiple passes.
- [ ] TDD red: a test where single-pass leaves SP on the table and a second pass
      improves it (assert the converged SP > single-pass SP, within tolerance).

## 2. Implement
- [ ] Add convergence loop in `ComputeSafeFallback` (or wrapper in
      `OptimizeSP_TL_Incre.cpp`/`.h`).
- [ ] Keep the in-walk gate (P0.7) rejecting each pass; keep loud-fail re-gate
      (`:1052-1060`) as the final backstop.
- [ ] TDD green.

## 3. Regression + records
- [ ] `testIncreOpt_w_TL` 125→N/125; legacy BF 10/5/2; ctest 16/17 (sole
      pre-existing CFS).
- [ ] Repro: P2.19 repro still exits 0 (no regression to the crash fix).
- [ ] `dev_log.md` + memory + MEMORY.md.
- [ ] `git add` staged; user reviews (no commit).
