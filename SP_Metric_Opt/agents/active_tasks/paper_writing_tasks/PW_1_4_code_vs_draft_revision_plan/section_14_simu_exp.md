# §14 — Simulation Experiment Setup

> Draft: `section14_simu_exp.tex` (126 lines). Status: high-level pending row-by-row.
> See `overall_revision_plan.md` for conventions.

## High-level change

§14 is the simulation setup (GMM ET model, covariance matrix, task-set
generation). The dominant revision is the **covariance-matrix fix** (Ryan 1.3):
with `ρ_{x,c}` and `ρ_{y,c}` each drawn independently from [−1,1] (`:45`), the
3×3 covariance matrix (`:30-32`) is positive-semidefinite only if
`ρ_{x,c}² + ρ_{y,c}² ≤ 1` (~21% of sampled matrices are currently invalid), and
Gaussian sampling can yield **negative execution times**. Secondary: promote the
`\agent` task-generation note (`:47`) to body prose (it's the real generator
spec); align the threshold set `[0.2,0.4,0.6,0.8,1.0]` (`:47`) with Option A
(note the `1.0` — Option A treats Θ=1.0 as "tolerate always missing", likely
wrong for a critical task); PW.2 merge (§14+§15).

## Action summary

- **FIX the covariance matrix (Ryan 1.3):** constrain the correlations so
  `ρ_{x,c}² + ρ_{y,c}² ≤ 1` (e.g. sample on the unit disk, or bound the pair),
  and use a **nonnegative ET distribution** (truncated normal / lognormal) so
  Gaussian sampling can't produce negative execution times. Verify the GMM model
  (`eq: pdf_gmm`, `:30-36`) stays valid post-fix. This may require a sim re-run.
- **PROMOTE** the `\agent` task-generation note (`:47`) into body prose (delete
  the `\agent` wrapper, keep content): three task types — environment-dependent
  (full GMM + spatial `ρ`), normal (Gaussian, zero spatial `ρ`), performance
  (deterministic ET, time-limit options span `[0.05,0.9]·period`). VERIFY it still
  matches the generator after the covariance fix.
- **FIX threshold set vs Option A:** `[0.2,0.4,0.6,0.8,1.0]` (`:47`) includes
  `1.0` — under Option A, Θ=1.0 means "tolerate missing 100% of the time" (the
  *least* critical). If a critical task is assigned 1.0 that's the inversion
  again (mirrors §12). Scrub `1.0`; align with the §12 convention adopted.
  Cross-link [[p213-important-task-ddl-vs-sp-metric]] (config
  `SP_THRESHOLD_RANGE→[0.001,0.9]` + scrub `1.0`).
- **MERGE** §14+§15 per PW.2's `reorg_plan.md` (structural — reference only).
- **VERIFY** the env-dependent/normal/performance task typing (`:47`) is
  consistent with §8's env-task reframing (content change 3) — a TL-optimizable
  (performance) task = special env-dependent task.

## Ryan Cat-1

- 1.3 (covariance matrix PSD violation + negative ET — constrain correlations,
  nonnegative distribution).
- (related) 1.2 (threshold `1.0` vs Option A — mirrors §12 Θ-table decision).

## Open notes

Row-by-row detailing deferred. The covariance fix is the gating item (may force a
sim re-run, paired with the §12 Θ-table re-run if Option A is adopted). The
`\agent` note promotion + threshold-`1.0` scrub are content-change 3 + Option A
touchpoints. Flag for PW.4.
