# §9 — Software Implementation

> Draft: `section9_software_impl.tex` (164 lines). Status: high-level pending row-by-row.
> See `overall_revision_plan.md` for conventions. (Includes the `§predict_ET_exp`
> block cited by §6.1 as already describing the real predictor.)

## High-level change

§9 mixes (a) the real ET-prediction method (sliding-window + Gaussian-dist fit,
`:26-35`) — which §6.1 must be made consistent WITH — and (b) the
smooth-environment assumption (`:26`, `:35`) that content change 2 removes/reframes.
It also states the RRT execution-time figure (`:64`: "1 to 3 seconds") that
clashes with a 10 ms period elsewhere (Ryan 1.3). Revision: keep the
sliding-window + Gaussian-fit description (it's the code truth, mirror §6.1's
replacement); reframe the smooth-assumption caveats to the dynamic/continuous
framing; fix the RRT config contradiction; and add the code's actual building blocks
the draft omits — DM priority building (was RM), the safe-fallback path, and the
offline `OptimizeIncre_w_TL_UntilConvergence` loop.

## Action summary

- **KEEP** sliding-window ET sampling + Gaussian-dist fit + linear-complexity
  argument (`:26-35`) — this is the real method (PW.1.1 §predict); make §6.1
  match it. **REFRAME** the "environment changes smoothly" premise (`:26`) and
  the `section: relax_smooth_assumption` caveat (`:35`) to the dynamic/continuous
  framing (content change 2): the window captures per-interval ET change; no
  global smoothness assumed (code *detects* jumps via `DetectETJump`).
- **FIX** the RRT config contradiction (Ryan 1.3): `:64` "RRT execution time
  varies from 1 to 3 seconds" vs a 10 ms period stated elsewhere (`main.tex:808`
  per Ryan / `:1025`). A single job cannot meet its deadline. Sanity-check ALL
  periods exceed WCET; correct the period or the ET figure.
- **ADD** DM priority building: the seed/scheduler priority construction is
  **Deadline Monotonic + important-first group lock**
  (`DeadlineMonotonicPriorityVec`, `BuildPriorityPlan` `GroupLock::kImportantFirst`),
  not RM — verify §9's priority-value mapping section (`:73`,`:88`) doesn't assert
  RM as the building rule. (RM survives only as the RM-Fast fallback floor,
  `RateMonotonicFastGroupLocked`.)
- **ADD** the safe-fallback path + offline convergence loop: the framework
  pre-computes a certified `{PA,TL}` (`ComputeSafeFallback` on a worst-case DAG)
  before the sim loop, enforced at runtime by 3 triggers; the offline
  `OptimizeIncre_w_TL_UntilConvergence` loop (offline ONLY) refines the seed.
  Forward-link to the new important-task-guarantee subsection (PW.3). See
  `sketch_fallback.md`.
- **QoS reframe (content change 6):** the TSP task description
  (`section_generate_tsp_perf`, `:76-78`, `:120-153`, Table `table_tsp_perf`)
  frames the running-time limit as a configurable optimization variable. Reframe
  it as the TSP anytime algorithm's **QoS budget** (longer budget ⇒ shorter path
  ⇒ higher quality, per the anytime property). This is the concrete QoS instance
  the abstract/§8/§6.7 all reference. Symbol rename `\lambda`→`\mathcal{Q}` if
  any λ appears in this block (check `:130-153`); most of §9 uses "running time
  limit" prose, so the reframe is mostly terminological.

## Ryan Cat-1

- 1.3 (RRT config: period 10 ms vs ET 1–3 s — impossible; sanity-check all
  periods > WCET).

## Open notes

Row-by-row detailing deferred. Three substantive touchpoints: smooth→dynamic
reframe (content change 2), RRT-config fix (Ryan 1.3), and the DM/fallback/
convergence-loop ADDs (content changes 3+4). The `§predict_ET_exp` block here is
the canonical predictor description — §6.1 should defer to / duplicate it
consistently.
