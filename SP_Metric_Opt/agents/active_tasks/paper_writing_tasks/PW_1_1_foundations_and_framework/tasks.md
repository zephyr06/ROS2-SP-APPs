# PW.1.1 — Tasks (working checklist)

> See `goal.md` for scope. First slice of the code read: metric + prediction
> foundations + per-interval framework loop. Mostly a reading task.

## Read the foundation sources
- [ ] `sources/Safety_Performance_Metric/Probability.h` — `GaussianDist` (distribution, NOT GP regression)
- [ ] SP metric source(s) — SP definition, `Normalize()`, safety/performance terms
- [ ] pRTA fixed-point convolution source
- [ ] `RTA_Cache.{h,cpp}` — `RTACache`, `RTACacheOpt`
- [ ] Outer scheduler loop + ET prediction (sliding-window sampling + Gaussian fit; `§predict_ET_exp`)
- [ ] `sources/Optimization/OptimizeSP_Base.{h,cpp}` — base classes, `UpdateTaskSetPriorities`
- [ ] Grep-confirm NO GP/GPR/kernel-regression symbol exists anywhere in code

## Verify the Θ_i convention (gates PW.3/PW.4 Option A vs B)
- [ ] Find where the code compares response time / miss probability against Θ_i
- [ ] State one sentence: (A) max tolerable miss prob `Pr(r_i>D_i)≤Θ_i`, or (B) min required success prob `Pr(r_i≤D_i)≥Θ_i`?

## Write `sketch_foundations.md` (items 1–4 from goal.md)
> For each item state BOTH the **algorithm** (what) AND the **motivation** (why it exists / what tension it resolves), tied to the concrete symbol/file.

- [ ] 1. Outer loop / scheduler + dynamic/continuous environment; sliding-window + Gaussian-fit prediction (NOT GP). *Why: ET non-stationary → re-derive per interval.*
- [ ] 2. pRTA fixed-point convolution. *Why: probabilistic ET needs probabilistic schedulability.*
- [ ] 3. RTA cache reuse (≈36% faster, bit-identical). *Why: incremental moves share the HP prefix; avoid re-convolving it.*
- [ ] 4. SP metric + safety definition + `Normalize()`. *Why: SP fuses safety+performance into one optimizable scalar.*
- [ ] Each item tied to the concrete symbol/file it came from
- [ ] Θ_i convention stated explicitly (Option A or B named)

## Verification
- [ ] Each of 1–4 states both algorithm AND motivation
- [ ] No GP/GPR content; sliding-window + Gaussian-fit predictor is the stated method
- [ ] Θ_i convention named (A or B)
- [ ] A reader could rewrite the relevant methodology foundations using only this sketch + existing `.tex`, without re-reading C++
