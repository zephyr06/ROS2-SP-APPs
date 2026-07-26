# P2.9 — Speed Up Re-Optimization (Simple Methods)

## The Goal

At N=16, a re-optimization activation costs ~0.8s on average (verified against
prod `INCR_Reopt_1` data: per-sim totals 29–65s over 60 reopt intervals →
~0.5–1.1s/activation, mean ≈ 0.8s). That is too long for the cold path; the
incremental path it sits next to is ~5–8× cheaper (same data, `INCR_Reopt_10`:
4.5–22.6s/sim). Reduce the per-activation reopt cost with **simple** methods —
config knobs or small, localized algorithm/implementation changes — NOT a
redesign of the optimizer. The user wants to conclude the main experiment
config today, so prefer the lowest-effort, lowest-risk levers first.

## Why it matters now

`ReoptimizationPeriod: 10` is the only periodic drift-correction mechanism. The
period-10 arm (`INCR_Reopt_10`) spends ~6 activations on reopt per 600s sim —
tolerable. But the period-1 arm (`INCR_Reopt_1`), which is the cleanest
experimental contrast for "what does reopt buy," spends all 60 activations on
reopt and is the one hitting the 1s `TIME_LIMIT` budget (cancellations at
N=16). If reopt stays expensive, either the contrast arm is unreliable
(cancelled mid-beam) or the period must be raised (weakening the contrast).
Cheaper reopt keeps the experimental design honest.

## The cost model (grounded, not guessed)

One reopt activation = `PerformCoordinateDescentForTaskConfigOpt(from_scratch=true)`
(`OptimizeSP_TL_Incre.cpp:488`). It does:

1. **One baseline beam** — `EvaluateTimeLimitConfig_ScratchOrIncre(..., from_scratch=true)`
   at `:503` → `OptimizeFromScratch(K=2)`. K-wide beam over N priority levels;
   each node scored via `PriorityPartialPath::UpdateSP` → `GetRTA_OneTask`
   DIRECTLY (no cache; `OptimizeSP_Incre.cpp:80`). ≈ O(K·N²/2) RTA calls.
2. **The TL coordinate walk** — per TL-flexible task, a backward (`step=-1`)
   + forward (`step=+1`) pass. **Each trial TL re-runs the full beam** via the
   eval lambda bound in `OptimizeSingleTaskTimeLimit` (`:432-436` →
   `EvaluateTimeLimitConfig_ScratchOrIncre(..., from_scratch=true)`). With
   `ReoptimizationTimeLimitSearchPatience=1` and ~N TL-flexible tasks × ~3
   steps/direction, that is **~O(N) full beam searches** stacked on the 1
   baseline beam.

So the beam runs ~O(N) times, not once. **The TL walk is the dominant cost,
because it re-runs a global priority re-search on every single-task TL nudge.**

Key structural fact: the incremental path already avoids this exact waste. The
comment at `OptimizeSP_TL_Incre.cpp:424-427` states the serialized incremental
step calls `OptimizeSingleTaskTimeLimit_Impl` *directly* with a sub-incremental
eval ("same walk, cheaper per-candidate score") — i.e. it re-scores the carried
PA + does a 1D single-task re-search (`|diff|==1`, cache-routed), NOT a full
beam. Reopt uses the legacy wrapper instead and pays the full beam per step.

## Guardrails (what this task is NOT)

- NOT a redesign of `OptimizeFromScratch` or the beam, NOT a new priority
  search algorithm, NOT touching the cache contract (`|diff|≤1`).
- NOT result-changing by default. Any lever that changes SP must be flagged as
  such and gated behind a flag / run as an A/B, not silently flipped.
- Simple = small diff, local to the reopt path, reusing existing tested
  machinery. Each lever should be landable + verifiable in one sitting.
