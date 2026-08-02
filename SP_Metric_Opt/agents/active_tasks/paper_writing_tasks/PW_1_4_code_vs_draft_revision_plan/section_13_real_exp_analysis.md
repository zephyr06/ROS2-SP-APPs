# §13 — Real Experiment Result Analysis

> Draft: `section13_real_exp_analysis.tex` (84 lines). Status: high-level pending row-by-row.
> See `overall_revision_plan.md` for conventions.

## High-level change

§13 is the real-experiment analysis. The dominant revision is **§13.3 "Can we
provide a safety-performance guarantee?"** (`:72-83`): it frames the guarantee as
*conditional* on a user-supplied ET upper bound ("if the execution time
distribution … can be upper-bounded and the estimation is known in advance … in
conflict with our original proposal about working in an unknown environment",
`:77-78`) — this is **DRIFT** (content change 4): the code **self-guarantees** a
safe fallback (`ComputeSafeFallback` certifies `{PA,TL}` on a worst-case DAG
built from the per-interval ET dists the loop already collects, enforced by 3
triggers + the BF gate; no user upper bound needed; no-safe-solution → throw →
regenerate). §13.3 must be rewritten to state the self-guarantee; the "in
conflict with unknown environment" caveat is obsolete. Secondary: headline-number
consolidation; PW.2 merge (§12+§13).

## Action summary

- **REWRITE §13.3** (`:72-83`) to state the **self-guarantee** (not conditional):
  the framework self-discovers a gate-feasible fallback for important tasks —
  `ComputeSafeFallback` certifies `{PA,TL}` on the cross-interval worst-case DAG
  (stochastically dominates every interval's ET → interval-independent), enforced
  at runtime by 3 triggers (a `DetectETJump` / b-i during-walk gate / b-ii
  `AdoptFallbackIfUnschedulable` backstop) + the BF gate; guarantee =
  `Pr(r_i>D_i)≤Θ_i` for every important task, every shipped interval; no-safe-
  solution → fails loud (throw → regenerate). **DELETE** the "if ET can be
  upper-bounded and known in advance" conditional (`:77`) and the "in conflict
  with our unknown-environment proposal" caveat (`:78`) — obsolete. Keep `:81`'s
  "stronger safety-performance guarantee" but make it concrete (worst-case-DAG
  fallback + gate, not just "higher SP"). Keep `:83`'s fine-grained per-task
  constraint flexibility (matches the important-task scoping). See
  `sketch_fallback.md` §3.
- **SOFTEN** "guarantee" + add regime boundary (Ryan 1.4): the self-guarantee
  holds on the worst-case DAG the framework builds from collected per-interval ET
  dists; beyond that regime (truly adversarial ET outside the observed support)
  claims are empirical/adaptive. State the regime explicitly.
- **CONSOLIDATE** headline number (Ryan 1.3): one number, backed by a table
  (cross-section with §12/§15/§16).
- **MERGE** §12+§13 per PW.2's `reorg_plan.md` (structural — reference only).
- **VERIFY** the §13.1/§13.2 ET-distribution analysis (`:12+`) is consistent with
  the dynamic-env framing (content change 2) — likely fine, light pass.

## Ryan Cat-1

- 1.4 (soften "guarantee" + assumptions/regimes — but per code the guarantee is
  self-guaranteed, so the §13.3 *rewrite* supersedes the draft's conditional
  framing rather than just softening it).
- 1.3 (headline number — cross-section).

## Open notes

Row-by-row detailing deferred. §13.3 is the experiment-side counterpart of §11's
guarantee drift — both must be reconciled (guarantee = contribution, stated in
methodology; §13.3 = the *discussion* of it). Depends on the §12 Θ-table decision
(Option A) for the precise guarantee wording. Flag for PW.3 + PW.4.
