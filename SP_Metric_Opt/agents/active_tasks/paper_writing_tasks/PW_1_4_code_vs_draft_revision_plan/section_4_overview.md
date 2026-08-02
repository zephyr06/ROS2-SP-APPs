# §4 — Overview

> Draft: `section4_overview.tex` (14 lines). Status: high-level pending row-by-row.
> See `overall_revision_plan.md` for conventions.

## High-level change

§4 is a one-paragraph overview + the framework figure (`fig_framework`,
`pictures/framework.pdf`). The prose is mostly fine ("online resource
optimization … unknown environments … online scheduling and configuration
optimization"). The main revision is **VERIFY the framework figure still matches
the code's actual loop** (collect→predict→optimize→update→simulate→score,
`RunSimulation`/`SimulateInterval` per `sketch_foundations.md` §1), since the
framework has gained: the offline safe-fallback pre-compute
(`ComputeSafeFallback`), the three runtime triggers, and the RTA cache. If the
figure predates these, it is **DRIFT** and the figure should be updated (or the
caption qualified) so it does not imply a single-pass predict→optimize loop with
no safety fallback. Caption text is otherwise consistent with the dynamic-env
framing (content change 2) — keep "execution time distribution may vary spatially
and temporally."

## Action summary

- **VERIFY** `fig_framework` matches the code's collect→predict→optimize→update→
  simulate→score loop + offline safe-fallback pre-compute + 3 triggers; **UPDATE**
  the figure/caption if it shows a single-pass loop with no fallback (DRIFT).
- **VERIFY** caption + prose align with content change 2 (dynamic/continuous env)
  — they already do.
- **light pass** ensure no GP framing in the figure (the overview prose has none).

## Ryan Cat-1

(none directly — but the framework figure is what reviewers scan first; an
inaccurate figure undermines 1.4 honest-claim calibration.)

## Open notes

Row-by-row detailing deferred; this is a figure+paragraph section. The figure
match-check is the one concrete action — flag for PW.3 to do alongside §6.7's
framework/guarantee forward-pointer.
