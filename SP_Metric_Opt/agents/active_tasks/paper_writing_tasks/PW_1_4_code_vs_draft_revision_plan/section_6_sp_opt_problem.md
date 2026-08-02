# §6 — Safety-Performance Optimization Problem

> Draft: `section6_sp_opt_problem.tex`. Highest-drift methodology section.
> Status: row-by-row DONE. See `overall_revision_plan.md` for conventions.

## High-level change

§6 is the densest revision target: GP example + RTA defs + Normalize + benefits
claim all need work, AND framework/important-task material must be added. Four of
the five cross-cutting conventions converge here: GP removal (6.1), dynamic env
(6.1, 6.9), framework update + important-task guarantee (6.7), Θ_i Option A
(6.3). Most Ryan 1.2 math fixes land here, though several are already fixed
(VERIFY). Θ-table drift is in §12, NOT §6.

## Subsection rows

### §6.1 Modelling Execution Time Distribution — DRIFT + ADD
- **label `section_et_model_gp`** (line 7) — name itself encodes GP; the `\rkw`/
  `\Sen` (9-10) and both `\sen` (40-41) are unresolved author discussion.
- **draft claim:** ET predicted via GPR `eq:gpr_predict1` (28-38) under a "smooth
  environment"; `\sen` notes concede GP is bad and that "the sampling+constant
  method (what we use in the paper)" is what's actually used.
- **code reality:** sliding-window ET sampling (`ReadExtTimeData`, last-N lines,
  `execution_time_estimator.h:49`) + Gaussian-*distribution* fit (mean/var/min/max,
  `GaussianDist` `RegularTasks.cpp:75-79`) + CDF discretization to FiniteDist PMF
  (`Probability.cpp:18-44`). NO GP/GPR. Env is dynamic/continuous: ET changes
  *across reoptimization intervals* (`SimulateInterval` loop). See
  `sketch_foundations.md`.
- **action:** **replace** the GPR Example (27-38) + `eq:gpr_predict1` with the
  sliding-window + Gaussian-fit predictor; **delete** both `\sen` (40-41) and
  stale `\rkw`/`\Sen` (9-10); **rewrite** §6.1 prose to frame the environment as
  dynamic/continuous (ET varies per-interval), dropping "smooth environment" GP
  framing. Keep the environment-vector Definition (17-21) — still used. Relabel
  `section_et_model_gp`→`section_et_model` (update `\ref` users).
- **Ryan:** 1.4 (GP removal). Cross-ref §9 (`§predict_ET_exp`) which already
  describes the real method.

### §6.2 Response Time Distribution Analysis — FIX (Ryan 1.2)
- **label `section_rta`**; `eq:rta_scalar` (70), `eq_prob_rta` (85).
- **draft claim:** `hp(i)` = "tasks with higher **or equal** priority than τ_i"
  (72, 82); `R_i^0 = ⊗_{j∈hp(i)} C_j` (80) initializes RT from hp(i) conv only.
- **code reality:** `ProbabilisticRTA_TaskSet` (`RTA.cpp:129`) computes RT
  convolution; hp(i) must be *strictly* higher priority (a task is not in its own
  hp set), and the RTA init includes the task's own `C_i`.
- **action:** **FIX** `hp(i)`→"strictly higher priority" at 72 + 82; **FIX**
  `R_i^0` init to include τ_i's own `C_i` (80). Confirm fixed-point form matches
  `RTA.cpp` exactly. (Analytical method is a "rough high-level review" deferring
  to Maxim — keep framing, correct only the two symbols.) Also touches §6.7
  `eq_prob_rta_in_opt` `\hptasks{i}` (228: "priority higher than τ_i" — already
  strict, VERIFY consistent).
- **Ryan:** 1.2 (RTA defs).

### §6.3 Safety metric — VERIFY (Option A) + FIX (Normalize) + ADD (Θ-table note)
- **label `eq_safety`** (102), `eq_safety_function` (112).
- **draft claim:** Definition states `Pr(r_i > D_i) ≤ Θ_i` (102) with Θ_i =
  "probability the system can tolerate τ_i to miss deadline" (105) — **Option A,
  already correct**. Safety function `S(Pr(r_i>D_i;A) − Θ_i)` (111) and `\hat S`
  penalty (119-124, `-0.01·e^{10|x|}` for x>0) match code's
  reward-when-below-threshold logic.
- **code reality:** `GetDDL_MissProbability` (`RTA.cpp:154-169`) returns `Pr(R>D)`;
  `ImportantTasksMeetThresholds` (`SP_Metric.cpp:234`) rejects on
  `ddl_miss_chance > threshold`; `SP_Func` (`SP_Metric.h:35`) rewards when
  `threshold >= violate_probability`. Option A ✓.
- **action:** **VERIFY** §6.3 Definition/Θ text is Option A (it is — no §6 Θ
  change). **FIX** `Normalize()`: defined only as "transforms output into [0,1]"
  (125) and "scales linearly to [0,1]" (Example 2, 173) — Ryan demands precise
  definition (domain, min/max, saturation). State the exact Normalize() the code
  uses (linear interpolation between observed min/max, clamped to [0,1]) per
  `sketch_foundations.md`.
- **Ryan:** 1.2 (Define Normalize()). **OPEN DECISION (Θ table):** §12–15 gives
  MPC the *largest* Θ (0.99) — reverse of Option A. §6.3 itself is fine; the fix
  lands in §12. Flag here so PW.3 keeps §6.3's Option A wording consistent with
  whatever §12 convention is adopted.

### §6.4 Performance metric — VERIFY (minor)
- **label `def_perf_metric`** (135).
- **draft claim:** `P(τ_i, E_k)→[0,1]` normalized, depends on task config + env.
- **code reality:** per-task, normalized to [0,1] (path length, localization
  error). Matches.
- **action:** **VERIFY** — no drift. Optional minor: tie to perf-WCET =
  execution_time_mu decision (P0.8) if performance/ET coupling is mentioned. Low.

### §6.5 SP-Metric — VERIFY (Ryan 1.2 already fixed) + FIX (Example 2 ref)
- **label `eq_sp_def`** (150), `fig_sp_threshold` (181).
- **draft claim:** `SP = Σ_{τ_i} (w_i/Σ_j w_j)·P(τ_i;E_k)·S(Pr(r_i>D_i;A)−Θ_i)`
  (147-150) — index `Σ_j w_j` (correct, no clash), parens balanced. Example 2
  (159-172): τ_0 P=0.5/Pr=0.3 → `0.5·Normalize(log(1.2))`; τ_1 P=1.0/Pr=0.8 →
  `1.0·Normalize(-0.01·e^{10·0.3})`.
- **code reality:** `SP_Func` (`SP_Metric.h:31-41`) + `ObtainSP_DAG`
  (`SP_Metric.cpp:95`) compute the weighted product sum; weights normalized.
  Matches the equation.
- **action:** **VERIFY** eq_sp_def (Ryan 1.2 SP-equation items already fixed —
  index `Σ_j w_j`, balanced parens). **VERIFY** Example 2 arithmetic: P-coeffs
  now appear correctly matched (τ_0→0.5, τ_1→1.0), safety coeff `-0.01` (matches
  `\hat S` 122). Re-confirm the `-0.01·e^{10·0.3}` exponent argument is `|x|`
  where `x = Pr−Θ = 0.8−0.5 = 0.3` → `e^{10·0.3}` ✓. If still correct, mark done;
  if Ryan's "swapped" remark now applies to a different residual, fix that.
  **FIX** the `Normalize` reference inside Example 2 to point to the precise
  Normalize() definition added in §6.3.
- **Ryan:** 1.2 (SP equation; Example 2 arithmetic).

### §6.6 Benefits of the new SP metric — FIX (Ryan 1.2)
- **draft claim** (197): "if an SP-metric threshold of 0.9 is required, both the
  safety and performance metrics must independently achieve at least 0.9." Also
  Normalize() interpretability claim (200-202).
- **code reality:** SP is a *weighted sum over tasks* of `P_i·S_i` products. A
  single task's `P_i·S_i ≥ 0.9` does imply both ≥ 0.9 for that task, but the
  *system-level* weighted sum ≥ 0.9 does NOT imply every task's terms are ≥ 0.9 —
  a strong task can mask a weak one.
- **action:** **FIX** the "threshold 0.9 ⇒ both ≥ 0.9" claim: either restrict the
  statement to a single product term `P_i·S_i ≥ 0.9`, or remove it. Keep the
  additive-vs-multiplicative benefit framing (194-196) — correct, stays.
- **Ryan:** 1.2 (SP interpretability — "true for a single product term but false
  for the weighted sum").

### §6.7 SP Metric Optimization Problem — ADD (framework) + minor
- **label `eq_overall_obj`** (218), `eq_prob_rta_in_opt` (222).
- **draft claim:** `max_{A,λ} SP(A;E_k)` (217); A = priority assignments; λ = task
  configs (running time limits); `R_i = R(C_i^D(E_k;λ_i), hp(i))` (221). No
  mention of important-task guarantee or actual solver structure.
- **code reality:** objective matches; but the code's optimization has structure
  the draft omits — DM seed + important-first group lock, modified Audsley + beam,
  incremental ±1, TL coordinate descent, RTA cache, AND the important-task
  safe-fallback guarantee that constrains the search (`ComputeSafeFallback`
  certifies a feasible `{PA,TL}` before the walk). See `sketch_optimization.md` +
  `sketch_fallback.md`.
- **action:** **ADD** a forward pointer (1-2 sentences) to §7/§8 for the solver,
  and a forward pointer to the new important-task-guarantee subsection (PW.3 adds
  it — likely §6.8 or a new §6.x) stating the optimization is *constrained* to
  maintain a certified safe fallback for important tasks. **FIX**
  `eq_prob_rta_in_opt` wording consistency with the §6.2 hp(i) fix.
- **Ryan:** 1.4 (soften "guarantee" + add assumptions/regimes — the
  important-task guarantee is *self-guaranteed*, so §6.7 framing should state it
  as a built-in constraint, not a conditional claim). Links to new guarantee
  subsection.

### §6.8 Challenges of SP-metric optimization — VERIFY (minor ADD)
- **draft claim** (237-244): mixed combinatorial+continuous vars; complicated
  non-convex objective; limited online compute → need lightweight (preferably
  linear) online algo.
- **code reality:** matches — the incremental ±1 + TL coordinate descent exist
  precisely because SP is non-monotone and online cost must be bounded.
- **action:** **VERIFY** — keep as motivation. Optional minor ADD: note the
  incremental design exploits that typically only one task's ET changes per
  interval (`FindTaskWithDifferentEt` premise) — but this lands more naturally in
  §7.3, so leave §6.8 as-is unless PW.3 sees a gap.

### §6.9 Motivation for solution algorithms — FIX (smooth-assumption drift)
- **draft claim** (252): "environment often changes smoothly … execution time
  distributions vary smoothly. Thus, optimal solutions are likely to evolve
  continuously." This is the *premise for incremental optimization*.
- **code reality:** the code does NOT assume smoothness — it *detects* ET
  change/jump (`DetectETJump`, ratio ≥ 1.5) and falls back to safe plan when the
  change is large. The incremental ±1 re-search is justified by "only one task's
  ET typically changed per interval," NOT by smoothness.
- **action:** **FIX** the smoothness premise (252): reframe as "because only a
  small number of tasks' ETs typically change between consecutive intervals, the
  optimal priority/config usually differs from the previous solution by a small
  perturbation — motivating incremental search." Drop/soften the "smooth" claim
  to match the dynamic/continuous framing (content change 2). Tie to
  `FindTaskWithDifferentEt` (§7.3).
