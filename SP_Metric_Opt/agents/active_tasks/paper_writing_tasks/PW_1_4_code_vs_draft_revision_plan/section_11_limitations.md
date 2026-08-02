# §11 — Limitations

> Draft: `section11_limitations.tex` (48 lines). Status: high-level pending row-by-row.
> See `overall_revision_plan.md` for conventions.

## High-level change

§11 is the highest-drift *limitations* section: it opens with an unanswered `\sen`
note ("how to provide hard safety / schedulability guarantee", `:1`) that is now
**implemented** (content change 4), and its `§relax_smooth_assumption` subsection
(`:10-21`) is built on **GPR + smooth-environment** framing that content change 1
removes. Revision: relocate the guarantee *statement* out of limitations (it is a
contribution now — likely into methodology §6/§7 per PW.2's re-org), keeping §11
as honest limitations only; rewrite the GPR/smooth subsection to the
sliding-window + dynamic-env reality; and delete the polar-coordinate footnote
claim ("the math basically remains the same" — false under a nonlinear polar
transform, Ryan 1.4).

## Action summary

- **RELOCATE** the guarantee: the `\sen` note (`:1-2`) "how to provide hard safety
  / schedulability guarantee / first propose hard schedulability guarantee PA +
  TL / limit the optimization process's search space into that region" is now
  **ANSWERED** — the during-walk gate IS "limit the search space into the
  gate-feasible region," the backstop + BF gate guarantee the final result, and
  `ComputeSafeFallback` certifies the fallback. Move the guarantee *statement*
  into methodology (PW.2 relocation; forward-link from §6.7); keep §11's residual
  (empirical-only outside the worst-case-DAG regime) as a genuine limitation.
- **REWRITE** `§relax_smooth_assumption` (`:10-21`): delete GPR (`:13`,`:16`) and
  the "environment changes smoothly" premise; replace with the sliding-window +
  dynamic/continuous framing (content changes 1+2). The residual limitation =
  per-interval ET prediction accuracy in highly non-stationary environments (the
  code detects jumps via `DetectETJump` but cannot predict them).
- **DELETE/ CORRECT** the polar-coordinate footnote (Ryan 1.4): "the math
  basically remains the same" is false — a Gaussian does not stay Gaussian under
  a nonlinear polar transform. One-line deletion.
- **VERIFY** the incremental-optimization "drift" limitation (`:41`) — keep; it's
  a real limitation (the convergence loop is offline-only; online walk is
  single-pass). Cross-link §8's stated-property framing.
- **SOFTEN** "guarantee" language generally (Ryan 1.4): the guarantee is
  *self-guaranteed* on the worst-case DAG, empirical/adaptive beyond it — state
  the regime boundary.

## Ryan Cat-1

- 1.4 (soften "guarantee" + assumptions/regimes statement — the
  unknown-environment setting yields empirical/adaptive claims only; the paper
  already concedes this).
- 1.4 (polar-coordinate footnote — delete).

## Open notes

Row-by-row detailing deferred. §11 is the limitations-side counterpart of §13.3's
guarantee drift (see `section_13_real_exp_analysis.md`). Both must be reconciled:
guarantee = contribution (state in methodology), NOT a conditional limitation.
Flag for PW.2 (relocation) + PW.3 (rewrite).
