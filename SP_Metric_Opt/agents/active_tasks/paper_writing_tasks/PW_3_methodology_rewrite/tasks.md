# PW.3 — Tasks (working checklist)

> See `goal.md` for scope. Hard-blocked on PW.1; follows PW.2's re-org.
> First real `.tex` writing.

## Section 6 (SP optimization problem) — primary
- [ ] Delete GP Example + GPR equation (`eq: gpr_predict1`, ≈lines 27–38); replace with sliding-window sampling + Gaussian-fit predictor (x-ref §predict_ET_exp)
- [ ] Reframe environment as dynamic/continuous (ET changes across reoptimization intervals)
- [ ] Update optimization framework to match code (modified Audsley + beam search = heuristic; incremental ±1; TL coordinate descent; DM seed + important-first group lock; `OptimizeIncre_w_TL_UntilConvergence`; RTA cache)
- [ ] Add important-task safety guarantee (worst-case DAG, RM/DM-fast seed, during-walk gate, `AdoptFallbackIfUnschedulable`) — grounded in PW.1
- [ ] Ryan 1.2: repair safety definition & Θ_i convention (Option A/B per PW.1)
- [ ] Ryan 1.2: fix SP-metric equation (unmatched paren; `Σ_i w_i` → `Σ_j w_j`, ≈line 478)
- [ ] Ryan 1.2: correct Example 2 arithmetic (swapped perf coeffs; `−0.1` vs `−0.01`, ≈lines 500–503)
- [ ] Ryan 1.2: fix SP interpretability claim (≈line 528) — state correctly or remove
- [ ] Ryan 1.2: define `Normalize()` precisely (domain, min/max, saturation)
- [ ] Ryan 1.2: RTA defs — `hp(i)` strictly higher priority; init includes `C_i` (≈lines 405, 413)

## Section 7 (PA optimization)
- [ ] Update BF / modified-Audsley+beam / incremental-PA to match PW.1 (DM seed, important-first group lock, four ±1 scenarios)
- [ ] Ryan 1.2/1.4: fix Algorithm 1 pseudocode (≈lines 634–657) — shrink pool as assigned; copy partial vectors before push; define `SelectTop` objective; "Optimal"→"Selected"; reconcile priority ordering
- [ ] Ryan 1.4: relabel as heuristic, not "optimal"; soften "guarantee" except for the real important-task fallback; add assumptions/regimes statement

## Section 8 (TL optimization)
- [ ] Apply PW.2 reframing (TL-optimizable = special env-dependent task → incremental after assumed ET change; `FindEnvTaskWithDifferentEt`)
- [ ] Update coordinate-descent 4 steps to match code; confirm O(M^N)→O(M·N); resolve the `\agent` theorem note (proof vs. stated property)
- [ ] Keep `eq: incremental_configuration`; confirm δ→3 candidates

## Verification
- [ ] No GP/GPR in sections 6–8; no "optimal"-as-exhaustive claim
- [ ] Environment framed dynamic/continuous; framework matches the PW.1 sketch slices
- [ ] Important-task safe-fallback guarantee stated and grounded
- [ ] All Ryan Category-1.2 items for 6–8 resolved
- [ ] Diff read vs the PW.1 sketch slices: no unsupported claim
