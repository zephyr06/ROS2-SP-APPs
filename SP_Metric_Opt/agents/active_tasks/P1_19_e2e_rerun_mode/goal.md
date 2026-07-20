# P1.19 — End-to-end pipeline `--rerun_mode` (clear-all / clear-results / resume)

## The Goal
Add a single `--rerun_mode` knob to the end-to-end experiment pipeline that
decides how to treat prior run artifacts before the stages run. Three intended
modes (user request 2026-07-20):

1. **`clear_all`** — remove ALL existing generated task sets AND simulation
   results, then re-run everything from scratch.
2. **`clear_results`** — keep the generated task sets, remove only the existing
   simulation results, then re-run results (re-simulate against the same
   tasksets). *(deferred)*
3. **`resume`** — don't re-run analysis if something is already done; skip
   stages / arms whose outputs already exist. *(deferred)*

A fourth, `reuse` (default), is the current behavior: keep everything and let
each stage's own reuse/resume guards decide.

This is a **Python harness** task (the e2e orchestrator
`simulation_experiments/run_end_to_end_experiments.py` + the shell wrapper
`scripts/run_end_to_end.sh`). NOT a correctness task — it changes WHEN prior
artifacts are cleared, not HOW the SP metric is computed.

## Scope boundary
- **In scope this task (per user 2026-07-20: "we can start with adding modes
  about whether to clear all simulated task sets and re-run first"):**
  - The `--rerun_mode` CLI arg + `RERUN_MODE` env var plumbing.
  - The `clear_all` mode (wipes `<run_root>/sim/`).
  - The `reuse` default (true no-op, bare invocation unchanged).
- **Deferred to later increments of THIS task (not separate tasks):**
  - `clear_results` (keep tasksets, wipe only sim outputs).
  - `resume` (skip stages whose outputs already exist).
- **Out of scope:** the C++ binary, the SP metric, the per-stage reuse guards
  inside `compare_optimizers.py` / `interval_sweep.py` /
  `aggregate_across_tasks.py` (the rerun policy sits ABOVE them and either
  preempts them with a wipe or leaves them alone — it does not rewrite their
  internals). P1.15 owns the harness's loud-failure behavior; P1.19 does not
  touch crash handling.

## Why now
The user re-runs the pipeline frequently (P1.14 Phase 3, P1.15 Phase 3, P1.17
3b all need fresh A/B runs). Today, forcing a clean re-run means manually
`rm -rf`-ing the sim tree or fighting each stage's own reuse/resume guards
(`--skip_generation_if_exists` + the `[Y/n]` config-drift prompt +
`--resume` skipping completed arms). A single up-front knob that wipes the
right subtree before any stage runs is cheaper and less error-prone than
negotiating three per-stage guards.

## Design
- The policy lives in the **Python orchestrator** (not the shell) because it
  owns `run_root` and can respect `--dry_run` + `--verbose`. The shell just
  forwards an env var, matching the existing `CONFIG_JSON` / `DRY_RUN` pattern.
- `clear_all` wipes `<run_root>/sim/` — the whole sim tree (generated tasksets
  + per-scheduler results + sweep variants) — which exactly matches "remove
  existing generated task sets to re-run." It runs BEFORE any stage, preempting
  the per-stage reuse/resume guards rather than fighting them.
- `--rerun_mode` is an extensible enum so the deferred `clear_results` / `resume`
  modes slot in without re-plumbing.
- `reuse` (default) is a true no-op so the bare `./run_end_to_end.sh` invocation
  is byte-for-byte unchanged.
