# §5 — System Model

> Draft: `section5_system_model.tex` (118 lines). Status: high-level pending row-by-row.
> See `overall_revision_plan.md` for conventions.

## High-level change

§5 defines the task model, configs (λ_i / running time limits), execution-time
distribution, deadline/period, schedulability, priority assignments. Two
targeted edits: (1) **KEEP the Gaussian-distribution assumption** for ET (this is
distinct from GP removal — content change 1 removes GPR, NOT the
Gaussian-distribution modeling); ensure §5's ET-dist definition stays consistent
with §6.1's new sliding-window + Gaussian-fit predictor (the prediction function
`F_i(E)→extDist{i}`, `eq_notation_task` `:46`). (2) **ALIGN the env-dependent /
TL-config definition** with §8's env-task reframing (content change 3): a
TL-optimizable task is a special kind of env-dependent task — §5's
`def_task_config` (`:70`) should not contradict this. Also fix the `\ith{i}` Table
I escape bug (`:12`, Ryan 1.1) if the notation table lives here.

## Action summary

- **KEEP** Gaussian-distribution ET modeling (`:53` Maxim2013 citation) — do NOT
  remove; only GP/GPR is removed.
- **VERIFY** `F_i(E)→extDist{i}` prediction-function notation (`:25`,`:46`)
  matches §6.1's sliding-window + Gaussian-fit predictor (no `eq:gpr_predict1`).
- **ALIGN** `def_task_config` λ_i (`:70-75`) with §8's env-task reframing
  (TL-optimizable = special env-dependent task). State the running-time-limit
  config influences both ET dist and output quality (`:75` already does — keep).
- **FIX** `\ith{i}` in the notation table (`:12`, Ryan 1.1) — renders literal
  `\ith{i}` instead of "i-th".
- **VERIFY** soft-schedulability definition (`:104-105`) is consistent with
  Option A (`Pr(r_i>D_i)≤Θ_i`), not a "meets deadline %" reading.

## Ryan Cat-1

- 1.1 (`\ith{i}` Table I escape — if the notation table is in §5).

## Open notes

Row-by-row detailing deferred. The two substantive items (Gaussian-KEEP vs
GP-remove boundary; env-task/TL def alignment) are content-change 1 + 3 touchpoints
— flag for PW.3 to do alongside §6.1 and §8.
