# §3 — Background

> Draft: `section3_background.tex` (16 lines). Status: high-level pending row-by-row.
> See `overall_revision_plan.md` for conventions.

## High-level change

§3 is a short, low-drift background section (CFS, RTS, SLAM/motion-planning timing).
No code-reality drift and no GP content here. Revision is a **light terminology
pass**: keep it stable, but ensure the terms used here agree with the updated §6
(e.g. "soft schedulability" / deadline-miss-probability framing consistent with
Option A's `Pr(r_i > D_i) ≤ Θ_i`, not a "meets deadline 99%" reading) and that any
forward references still resolve after §6.1's GP removal and the §5 notation
updates. Lowest priority of the included sections.

## Action summary

- **VERIFY** terminology consistency with §5/§6 (soft schedulability, miss prob).
- **VERIFY** no GP/GPR/smoothness framing creeps in (it doesn't here).
- **light pass** on any `\ref` to anchors that §6.1 relabels
  (`section_et_model_gp`→`section_et_model`).

## Ryan Cat-1

(none directly land in §3.)

## Open notes

Row-by-row detailing deferred — §3 is essentially background prose; if PW.3 finds
a concrete claim that contradicts code, add a subsection row here then.
