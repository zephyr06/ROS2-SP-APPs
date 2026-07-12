# P1.2 — Reopt Incumbent Degradation (Structural Corruption) Investigation

**Priority:** P3 (deferred; out of scope in the current stage of development; kept as a known theoretical hazard)
**Status:** deferred / let go
**Reference docs:** `agents/agent_communication/gemini.md` (R1–R5),
`agents/agent_communication/kimi.md` (R1–R4)

## The user's invariant (the thing being checked)

> "not changing priority assignments if the found solution is worse than
> initial solution from last intervals"

i.e. a re-search must never leave the system on a *worse* priority assignment
than the carried incumbent. The compare-and-keep guard is supposed to enforce
this. The question is whether it does so **only at the SP-value level** (which
it does, soundly) while leaving the system exposed at the **structural
(permutation) level** — committing a permutation that is SP-equal-or-better but
structurally worse, then being chained to it.

## Context — what the debate converged on

A multi-round review (Gemini R1–R5, Kimi R1–R4, in `agents/agent_communication/`)
examined Gemini's "Incumbent Degradation (Corruption)" claim against the code.
The converged diagnosis:

1. **The SP-value guard is sound.** `UpdateRecords`
   (`sources/Optimization/OptimizeSP_TL_Incre.cpp:105-139`) commits a candidate
   only when `(candidate SP strictly > incumbent SP, not ApproxEqualSP)` OR
   `(ApproxEqualSP AND sum_new_tl < sum_old_tl)`. SP is monotonic non-decreasing
   within an interval. So at the SP-value level the user's invariant holds.

2. **But the guard is structurally blind.** The reopt path's from-scratch beam
   search `OptimizeFromScratch(K)` (`sources/Optimization/OptimizeSP_Incre.cpp:74`)
   is an Audsley-constructive search over N! permutations and is **memoryless**
   w.r.t. the carried permutation π_incumbent. At the shared beam width
   `K = Layer_Node_During_Incremental_Optimization = 2` (`sources/parameters.yaml:7`,
   passed identically to every arm), it is myopic and can return a permutation
   with equal-or-marginally-better SP but a structurally worse global task
   ordering. If that candidate squeaks past the SP guard (via the
   `ApproxEqualSP` tie-break, or a marginally-better SP on a degenerate
   landscape), it is committed.

3. **The incremental path cannot recover it.** `OptimizeIncre` warm-starts 1D
   priority variations (`FindPriorityVec1D_Variations`) around whatever
   permutation was committed — it perturbs one task's priority at a time and
   cannot restructure a poor global ordering. So a structurally-degraded
   permutation committed at a reopt bootstrap chains forward until a later
   reopt happens to find something better.

4. **Reopt is the source, not the escape.** Re-search every interval (INCR_P1)
   mitigates *catastrophic forgetting* but is also the *source* of structural
   corruption — each reopt is a fresh memoryless draw. Infrequent reopt (P60)
   draws less often but drifts longer on each bad draw.

**Crux:** the guard protects SP-value monotonicity, **not** permutation
structural quality. Whether this hazard actually fires in practice — degrading
safety-performance on the real tasksets — is **unverified**. Both agents agreed:
**empirical check first, no code change until the hazard is shown to fire.**

## The synthesis (the fix, IF the hazard fires)

If the empirical check shows the hazard fires, the agreed design is
**Option B + Option C**:

- **Option B (Warm-Start Seed the Bootstrap):** in `OptimizeFromScratch(K)`,
  seed the bootstrap beam with π_incumbent (pre-populate `pa_vec_lower_pri` to
  match the incumbent's suffix) so the re-search can only build on or improve
  the carried relative ordering — never forget it. Resolves the structural
  memory risk.
- **Option C (Chained Walk):** in
  `PerformCoordinateDescentForTaskConfigOpt` (`OptimizeSP_TL_Incre.cpp:241`),
  call the bootstrap search once (from_scratch=true) for the starting TL
  config, then chain all subsequent TL-walk steps via `OptimizeIncre`
  warm-started from the previous step (from_scratch=false). Pays the
  from-scratch cost once per reopt instead of per TL step (current code re-runs
  `OptimizeFromScratch` at every TL step via `EvaluateTimeLimitConfig_ScratchOrIncre`
  at `:152`). Resolves the ET cost.

B+C together get structural memory **and** the ET cut. Option C alone does NOT
solve the corruption — it relocates the memoryless draw into one bootstrap step
(arguably worse: one bad draw chains the whole walk). Option B alone preserves
memory but keeps the per-step from-scratch ET cost.

## Investigation steps (empirical check FIRST)

1. **Mine the existing P25 A/B per-interval SP traces.** The data already
   exists — no new run needed for the first pass. For each INCR_P{n} arm, read
   `interval_sp_metrics.txt` (format: `interval,sp`) and look for a **post-reopt
   SP dip that does not recover to the pre-reopt trajectory** at reopt intervals
   (every n-th interval). That signature — a sharp drop at a reopt interval
   followed by sub-pre-reopt SP — is the structural-corruption fingerprint.
   Contrast against INCR_P1 (reopt every interval = upper bound, memory + most
   frequent recovery) and INCR_SCRATCH (amnesiac = lower bound, no carried
   incumbent).

   Data roots (test-mode A/B, 600s @ 10s interval ≈ 60 intervals/run):
   - `simulation_experiments/optimizer_comparison/runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6/sim/tasks{4,6}_dur600_interval10_seed1000/taskset_<t>/<ARM>/<ARM>/interval_sp_metrics.txt`
   - Aggregate per-arm mean SP + ET:
     `.../sim/tasks{4,6}_dur600_interval10_seed1000/comparison_summary.csv`
     (columns: `Scheduler,Mean_SP_Metric,...,Mean_Scheduler_Execution_Time_s,...`).

   Already-visible aggregate signal (tasks6): `BF 0.522 > INCR_P1 0.5049 >
   P10 0.5024 > P30 0.5000 > P60 0.4989 > INCR_SCRATCH 0.4968`. Consistent with
   "memory helps, infrequent re-search costs a little" — but **not** by itself
   evidence of corruption (could be benign drift). The per-interval trace is
   what distinguishes corruption (non-recovering dip) from benign drift (smooth
   decay).

2. **If the per-interval trace is ambiguous**, instrument the reopt path to log
   the committed permutation's structural distance from π_incumbent at each
   reopt (e.g. Kendall-tau of `res_opt_.priority_vec` before vs after). A
   reopt that commits a high-Kendall-tau-different permutation for a
   near-zero SP gain is the smoking gun. (Rebuild with `debugMode:1`; the
   P0.5 `SeedStateFromIncumbent` dump at `OptimizeSP_TL_Incre.cpp:~430` is a
   template.)

3. **Only after (1) [and (2) if needed] converges:** decide among
   - **Hazard does not fire** → close the task; the SP-value guard is
     sufficient in practice; record the negative result.
   - **Hazard fires** → implement B+C (the synthesis above), TDD-first: a
     failing test where `OptimizeFromScratch` returns a structurally-worse
     permutation that squeaks past the SP guard, asserting π_incumbent is
     preserved after `ReOptimizePeriodic`.

## Files

- `sources/Optimization/OptimizeSP_TL_Incre.cpp` — `UpdateRecords` (`:105-139`,
  the guard), `EvaluateTimeLimitConfig_ScratchOrIncre` (`:142`, from_scratch
  branch `:149-154`), `PerformCoordinateDescentForTaskConfigOpt` (`:241`, the
  TL walk), `ReOptimizePeriodic` (`:453`), `BuildChallengerFromIncumbent`
  (`:402`), `ReconstructTimeLimitVecFromResOpt` (`:345`).
- `sources/Optimization/OptimizeSP_Incre.cpp` — `OptimizeFromScratch` (`:74`,
  the memoryless Audsley beam), `OptimizeIncre` (`:~145`, 1D variations).
- `sources/Optimization/OptimizeSP_TL_Incre.h:11` — `ApproxEqualSP` (1e-9 rel tol).
- `sources/parameters.yaml:7` — `Layer_Node_During_Incremental_Optimization: 2`
  (the single shared beam width; there is no separate reopt beam width today).
- `simulation_experiments/optimizer_comparison/runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6/`
  — the existing A/B data (step 1 input).

## Done when

- Per-interval SP traces mined for INCR_P1/P10/P30/P60/SCRATCH; a recorded
  verdict on whether the post-reopt non-recovering-dip signature appears.
- If ambiguous: instrumentation run; structural-distance verdict recorded.
- An explicit decision recorded: hazard-does-not-fire (close) OR
  hazard-fires → B+C implementation (with rationale tied to the evidence).
- Milestone to top-level `agents/dev_log.md`.

## Out of scope

- Investigating or fixing the structural robustness vs. opportunism trade-off (e.g., where a from-scratch search commits a structurally worse/fragile priority assignment that demotes critical/tight-deadline tasks for a short-term SP tie/gain). While mathematically possible and relevant to real-time scheduling theory, this is classified as a theoretical hazard and is out of scope for the current stage of development.
- Implementing B+C (or any source change) before the empirical check converges.
  Both agents agreed: check the data first.
- Re-running the full A/B suite myself — the user runs the actual A/B
  (`MODE=test/prod CONFIG_JSON=.../p25_period_ab_config.json
  ./scripts/run_end_to_end.sh`). Step 1 reads already-existing output; a fresh
  run is the user's call.
- `git commit` — user's standing constraint (`git add` only).
- Re-litigating the SP-value guard's soundness — settled (it is sound; the
  question is structural, not SP-value).
