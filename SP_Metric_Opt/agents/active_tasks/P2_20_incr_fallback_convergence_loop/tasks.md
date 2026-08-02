# P2.20 — Tasks

## 0. Prereq
- [x] P2.19 (points 1 + 3) LANDED + green regression — COMMITTED `58264b20`
      (crash gone, worst-case DAG uses min TL for perf tasks).

## 1. Design + TDD red
- [x] Confirm `ApproxEqualSP` (or equivalent SP tolerance helper) exists + sig —
      `OptimizeSP_TL_Incre.h:11` (`rel_tol=1e-9`).
- [x] Decide convergence wrapper shape: new `virtual OptimizeIncre_w_TL_UntilConvergence`
      wraps `OptimizeIncre_w_TL` (chosen: virtual-wrapper + stub TDD; `OptimizeIncre_w_TL`
      made virtual as the stub seam). Loop lives in the wrapper; `ComputeSafeFallback:1000`
      calls the wrapper instead of the raw pass.
- [x] Decide iteration cap — **NO cap (user, 2026-08-01).** Termination is guaranteed
      by monotonicity over a finite config space (finite tasks × finite TL grid × finite
      PA permutations → finite distinct SP values → finitely many strict improvements).
      `N+1` was an unproven heuristic and could even prematurely cut a case needing >N
      sweeps. Runtime guard = `BFDLSharedBudget`, not a cap. Stop = 1e-3 `ApproxEqualSP`
      + explicit `opt_sp_ <= sp_before` (break unless strictly higher).
- [x] Confirm `BFDLSharedBudget` (`ComputeSafeFallback:974`) bounds the looped runtime
      across multiple passes — wraps whole compute; `BFSharedBudgetCancelled()` polled
      in-walk → a cancelled mid-pass commits nothing → `sp_after==sp_before` → break.
- [x] TDD red: `OptimizeIncre_w_TL_UntilConvergence_StopsOnNonImprovingPass` — stub
      overrides `OptimizeIncre_w_TL` to inject `{0.5,0.8,0.8}`; asserts 3 passes run,
      stops on non-improving 3rd, carries best SP. RED with single-pass stub (1 pass).

## 2. Implement
- [x] Add convergence loop in `OptimizeIncre_w_TL_UntilConvergence` (wrapper in
      `OptimizeSP_TL_Incre.cpp`/`.h`); `ComputeSafeFallback` routed through it.
- [x] Keep the in-walk gate (P0.7) rejecting each pass; keep loud-fail re-gate
      (`:1025-1033`) as the final backstop — unchanged.
- [x] TDD green.

## 3. Regression + records
- [x] `testIncreOpt_w_TL` 126→127/127; ctest 16/17 (sole pre-existing CFS).
- [ ] Repro: P2.19 repro still exits 0 (no regression to the crash fix) — TODO.
- [x] `dev_log.md` + memory + MEMORY.md.
- [ ] `git add` staged; user reviews (no commit) — TODO.
