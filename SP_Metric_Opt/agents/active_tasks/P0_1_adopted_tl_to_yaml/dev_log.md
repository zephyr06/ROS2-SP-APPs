# P0.1 — Persist adopted TL to YAML — Dev Log

> Detailed working log for this task. Append chronological entries below.
> On task completion, append a one-line milestone to the **top-level**
> `agents/dev_log.md` (the canonical narrative).

## 2026-07-08

- Task scaffolded from the P1.1 investigation conclusion. The P1.1 residual was
  root-caused to "the runtime treats the YAML Gaussian as the prior ET, but a
  TL-optimizable task's prior ET is its adopted TL." P1.1 applied a descent-start
  workaround (`OptimizeIncre_w_TL:387` starts from `ReconstructTimeLimitVecFromResOpt()`
  + option-set guard); this task is the **root-cause fix** — persist the adopted
  TL (and its implied ET) back to the taskset YAML so the file is the source of
  truth and the cold-start readers see the carried-forward state directly.
- Sibling refactor (P0.1b: collapse the duplicated incumbent state across
  `prev_optimizer_` / `res_opt_` / `opt_pa_` / `opt_sp_`) split into its own
  checklist item so the behavior fix and the structural cleanup land separately.
- Not yet started.
