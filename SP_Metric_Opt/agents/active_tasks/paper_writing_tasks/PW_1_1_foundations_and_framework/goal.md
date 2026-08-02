# PW.1.1 — Foundations & Framework (SP / pRTA / RTA cache / dynamic environment)

**Priority:** P0 (grounding — first slice of the code read)
**Status:** done — `sketch_foundations.md` written; Θ_i convention resolved → Option A
**Depends on:** —

## Goal

Produce `sketch_foundations.md` in this folder: the first slice of the code-side
truth the writing tasks are checked against. Covers the **metric and prediction
foundations** and the **per-interval optimization framework loop**, each with
both the **algorithm** (what) and the **motivation** (why it exists / what
design tension it resolves), tied to the concrete symbol/file.

This is where the deferred deep code investigation lands — one small, focused
reading task rather than a single monolithic sketch. It is a reading task, not a
writing-of-the-paper task.

## Entry point

Read these sources (verified present) and write `sketch_foundations.md`:

- `sources/Safety_Performance_Metric/Probability.h` — `GaussianDist` (an ET
  distribution type — NOT Gaussian-process regression; confirm by grep that no
  GP/GPR/kernel-regression symbol exists).
- The SP metric source(s) under `sources/Safety_Performance_Metric/` — the SP
  definition, `Normalize()`, safety `Pr(r_i > D_i) ≤ Θ_i`, the performance term.
- The pRTA fixed-point convolution source (response-time analysis).
- `RTA_Cache.{h,cpp}` — `RTACache` class, `RTACacheOpt`.
- The outer scheduler loop (per-interval collect→predict→optimize→update) and
  the ET prediction method (sliding-window sampling + Gaussian-distribution fit;
  cite `§predict_ET_exp`, section 9).
- `sources/Optimization/OptimizeSP_Base.{h,cpp}` — base classes
  `BFDLSharedBudget`, `OptimimizePA_Base`; `UpdateTaskSetPriorities`.

## What this slice must capture

For each: state the **algorithm** (what) **and the motivation** (why), each tied
to the concrete symbol/file.

1. **Outer loop / scheduler + dynamic environment.** Periodic scheduler task
   every `T`; per interval: collect recent ET samples → predict next-interval ET
   distribution → optimize PA + TL → update Linux priority values. State the
   prediction method precisely: **sliding-window sampling + Gaussian-distribution
   fit (mean, variance, min, max)**, NOT GP. Cite `§predict_ET_exp`. Reframe the
   environment as **dynamic and continuous**: tasks' ET changes across
   reoptimization intervals, so the solution is re-derived per interval.
   *Why:* ET is non-stationary in a dynamic environment, so the PA/TL solution
   must be re-derived per interval rather than computed once offline — this is
   the "dynamic/continuous environment" content change.
2. **pRTA.** Fixed-point response-time convolution of ET distributions.
   *Why:* probabilistic ET demands probabilistic schedulability (not a hard
   WCET bound).
3. **RTA cache.** `RTACache`/`RTACacheOpt` reuses pRTA results across incremental
   moves (≈36% faster, bit-identical SP).
   *Why:* incremental moves share almost all of the pRTA higher-priority set;
   recomputing the full convolution every move is wasted work, so the cache
   reuses the unchanged prefix.
4. **SP metric + safety definition.** SP = safety × performance, normalized;
   safety `Pr(r_i > D_i) ≤ Θ_i`; `Normalize()` (domain, min/max, saturation).
   *Why:* SP fuses safety + performance into one scalar the optimizer can
   maximize.

## Verify the Θ_i convention (gates PW.3/PW.4 Option A vs B)

- [ ] Find where the code compares response time / miss probability against Θ_i.
- [ ] State one sentence: does the code treat Θ_i as (A) max tolerable miss prob
      `Pr(r_i>D_i)≤Θ_i`, or (B) min required success prob `Pr(r_i≤D_i)≥Θ_i`?
      This resolves Ryan's Category-1.2 convention question and gates PW.3/PW.4.

This finding lives in `sketch_foundations.md` and is consumed by PW.1.4.

## Done when

- `sketch_foundations.md` exists and captures items 1–4 above, each with both the
  **algorithm** and the **motivation**, tied to the concrete symbol/file.
- The Θ_i convention the code uses is stated explicitly (one sentence, Option A
  or B named).
- No GP/GPR content appears; the sliding-window + Gaussian-fit predictor is the
  stated prediction method.

## Out of scope

- Priority assignment / TL optimization (PW.1.2).
- Important-task fallback / convergence loop (PW.1.3).
- The code-vs-draft revision plan (PW.1.4 — consumes this slice).
- Editing any `.tex` (PW.3/PW.4).
