# PW.4 — Tasks (working checklist)

> See `goal.md` for scope. Depends on PW.1 + PW.3 (terminology + guarantee
> wording must be settled first).

## Section 1 (Introduction)
- [ ] Remove GP contribution (≈line 59)
- [ ] Consolidate headline number (currently "20–50%"; single value from prod data)
- [ ] Soften "to the best of our knowledge, first work…"

## Section 2 (Related work)
- [ ] Soften "first work…" (≈line 26); reframe as "an adaptive SP-aware scheduling framework" (shares sentence with smARTflight cite)
- [ ] (opt, Cat 2.5) related-work comparison table

## Sections 3–5 (Background / Overview / System model)
- [ ] §3 light terminology pass
- [ ] §4 update framework figure/description to match PW.1 (per-interval collect→predict→optimize→update; DM seed; fallback path)
- [ ] §5 keep Gaussian-distribution assumption (≈line 54); align env-dependent/TL-optimizable def with PW.2/PW.3

## Section 9 (Software & implementation)
- [ ] Light updates: DM priority building (was RM), important-task fallback in scheduler path, `OptimizeIncre_w_TL_UntilConvergence`
- [ ] Ryan 1.3: fix RRT config (period 10 ms vs ET 1–3 s impossible)

## Section 10 (Complexity)
- [ ] Update to match code (BF exponential; modified Audsley O(m·N²); incremental O(N·O_RTA); RTA-cache speedup)

## Section 11 (Limitations)
- [ ] Remove GPR mention (≈line 13); reframe `§relax_smooth_assumption` around sampling/sliding-window predictor
- [ ] Apply PW.2 relocation decision (appendix vs. condensed-in-body); fix cross-refs
- [ ] Ryan 1.4: remove/correct polar-coordinate footnote (one-line deletion)

## Sections 12–15 (Results — real + simulation)
- [ ] Apply PW.2 merge (12+13 → one; 14+15 → one)
- [ ] Ryan 1.3: fix simulation covariance matrix (constrain ρ; nonnegative distribution)
- [ ] Consolidate headline number (single value from §1)
- [ ] (opt, Cat 2.1) report task-level ground truth (per-task miss rates, RT CDFs, SLAM error, path length, MPC tracking error, scheduler overhead)

## Section 16 (Conclusion)
- [ ] Consolidate headline number ("20% to 40%" → single value)
- [ ] Guarantee claim matches PW.3 wording

## Cross-cutting (main.tex)
- [ ] Ryan 1.1: fix Table I `\ith{i}` escaping (≈line 228)
- [ ] Ryan 1.1: proofread typos — `works works` (162), `an robot` (128, 211), `aerocrafts` (160), `preferablely` (571), `parital` (638), `dentoes` (415), `pikes` (1365), `temporarily→temporally` (212, 356)
- [ ] Ryan 1.5: add T-ASE "Note to Practitioners"

## Verification
- [ ] No GP/GPR outside methodology; headline number identical in §1, §12–15, §16
- [ ] All Ryan Category-1.1 / 1.5 resolved; Category-1.3 results-config (RRT, covariance) resolved
- [ ] Terminology (Θ_i, "heuristic" vs "optimal", guarantee wording) matches PW.3 throughout
