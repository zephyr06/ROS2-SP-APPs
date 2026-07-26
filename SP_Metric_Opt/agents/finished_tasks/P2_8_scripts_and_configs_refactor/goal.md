# P2.8 — Scripts & Configs Refactor (entry-point + config consolidation)

## The Goal

Consolidate the `scripts/` entry points and `simulation_experiments/configs/`
JSON files so the repo has a small, clearly-divided set of run entry points and
one config per distinct purpose. Today there are 4 shell scripts and 4 configs
with overlapping responsibilities, stale references, and misleading names. The
target is:

- **(a)** one script that generates **all** figures needed by the paper (from
  existing experiment data — no re-simulation);
- **(b)** one script that runs the full simulation+plot pipeline and can be
  pointed at any per-purpose config (one `.json` per test purpose, e.g.
  `incr_et_profiling.json`);
- the north-star gate evaluation merged into / callable from the pipeline
  script (reduce the duplicated build+delegate code between the two current
  top-level scripts);
- two configs kept: `paper_simulation_config.json` (renamed from
  `experiment_config.json`) + `incr_et_profiling.json`; the other two removed.

This is a **refactor for clarity** (naming, consolidation, stale-reference
cleanup) — NOT a correctness change. Gate = the pipeline still produces
bit-identical figures + the north-star suite still emits the same verdict for an
unchanged config.

---

## Current state (inventory + findings)

### `scripts/` (4 shell scripts + 1 python one-off + `lib/`)

| File | Lines | Role | Called by? |
|------|-------|------|-----------|
| `run_end_to_end.sh` | 105 | Full pipeline: build → simulate → sweep → aggregate. Default config `experiment_config.json` (via `CONFIG_JSON`). THE main entry point. | `run_evaluation_suite.sh` (delegates the pipeline stage) |
| `run_evaluation_suite.sh` | 107 | build → call `run_end_to_end.sh` with the eval config → run `evaluation_suite.py` (north-star gates). ~90% overlaps `run_end_to_end` (build + delegate); the "one extra evaluation" = `evaluation_suite.py`. | nothing (top-level) |
| `run_interval_sweep.sh` | 26 | Standalone: run ONLY the `interval_sweep` stage (produces Fig 2 data+plot). | **nothing** — manual single-stage entry point |
| `run_paper_figures.sh` | 40 | Standalone: run ONLY `aggregate_across_tasks` (Figs 1A–1F, 3) from existing data. **Closest existing thing to goal (a)** — but it does NOT produce Fig 2. | **nothing** — manual figures-from-existing-data entry point |
| `generate_taskset_6.py` | — | One-off 6-task taskset generator (uses `Gen_Taskset`). Unrelated to the pipeline. | nothing — OUT OF SCOPE |
| `lib/common.sh` | — | Sourced helpers: `print_header`, `print_footer`, `require_binary`, `register_signal_trap`. | all 4 shell scripts source it |

**Key finding on "how the other 2 scripts are used":** `run_interval_sweep.sh`
and `run_paper_figures.sh` are **NOT sourced by / assisting** `run_end_to_end.sh`
or `run_evaluation_suite.sh`. They are **peer standalone entry points** for
running ONE pipeline stage by hand. The main pipeline (`run_end_to_end.sh` →
`run_end_to_end_experiments.py`) calls the stage modules
(`compare_optimizers` / `interval_sweep` / `aggregate_across_tasks`) directly —
it does not shell out to these two thin scripts. So "move them to `lib/`" is a
**category mismatch**: `lib/` holds *sourced* shell helpers (`common.sh`), not
standalone entry-point scripts. Their real disposition is keep-as-standalone vs
delete (see D3).

### `simulation_experiments/configs/` (4 JSON files)

| File | Loader default? | `eval_*` keys? | Scheduler set | Verdict per user plan |
|------|-----------------|----------------|---------------|----------------------|
| `experiment_config.json` | ✅ `DEFAULT_CONFIG_PATH` (`experiment_config_loader.py:24`); default for `run_end_to_end.sh`, `interval_sweep.py`, `aggregate_across_tasks.py`, `run_end_to_end_experiments.py` | ❌ none | paper figure set (`test_mode`: INCR_Reopt_10, BF, RM, CFS) | **KEEP + RENAME** → `paper_simulation_config.json` |
| `evaluation_suite_config.json` | ✅ eval-suite default (`evaluation_suite.py:611`, `run_evaluation_suite.sh:39`) | ✅ `eval_quality_task_counts`/`eval_overhead_task_count`/`eval_period_arms` (test+prod) | the 10-scheduler gate set (5 INCR_Reopt_X + BF/RM/CFS/INCR_NO_TL/INCR_WCET) | **user says REMOVE** — but see D1 (carries the gate scheduler set + `eval_*` keys the suite needs) |
| `INCR_ET_Profiling.json` | ❌ none | ✅ (copy of eval suite's) | `INCR_Reopt_10` only; N=[4,6,8,10,12,14,16] | **KEEP as-is** (the per-purpose INCR-ET profiling config) |
| `simulation_only_config.json` | ❌ none | ❌ none | single-N compare set | **user says REMOVE** — and its documented invocation is already BROKEN (see D2) |

---

## Open design questions (resolve with user before implementing)

### D1 — The eval-suite config coupling (CRUX; blocks "remove evaluation_suite_config.json")

`evaluation_suite_config.json` is NOT a redundant near-clone — it carries two
things `paper_simulation_config.json` (the renamed `experiment_config.json`)
does NOT have:

1. the **gate-specific scheduler set** — 10 schedulers (5 `INCR_Reopt_X` +
   BF/RM/CFS/INCR_NO_TL/INCR_WCET) — which differs from the paper figure
   scheduler set;
2. the **`eval_*` keys** (`eval_quality_task_counts`, `eval_overhead_task_count`,
   `eval_period_arms`) that `evaluation_suite.py` reads (with fallbacks:
   `eval_quality_task_counts`→`num_tasks_for_cross_task_comparison` at
   `evaluation_suite.py:529-530`; `eval_period_arms`→`DEFAULT_PERIOD_ARMS` at
   `:664`; `eval_overhead_task_count` has NO clean fallback).

`evaluation_suite.py:611` + `run_evaluation_suite.sh:39` default to this file.
Removing it breaks the eval-suite default. So "remove evaluation_suite_config"
requires a decision:

- **(α)** Fold the gate scheduler set + `eval_*` keys INTO
  `paper_simulation_config.json` (one config does both paper figures AND gate
  eval). Cost: paper figure runs would also run the 10-scheduler gate set
  (heavier) unless gate eval is opt-in via a flag.
- **(β)** Keep a dedicated eval-purpose config (contradicts "remove 2 configs",
  but preserves the clean scheduler-set separation). Rename it for clarity.
- **(γ)** Make the north-star gate eval **opt-in via a flag** on the pipeline
  script (e.g. `--eval_ns`), reading `eval_*` from whatever config is active
  (fallbacks otherwise); drop the dedicated eval config. Run gate eval by
  passing `--eval_ns` + pointing at a config that has the gate scheduler set.

**RESOLVED (2026-07-24): RE-RESOLVED α** (was β earlier this date). User picked
"paper set; drop gate_eval" — fold the `eval_*` keys into
`paper_simulation_config.json` and DELETE `gate_eval_config.json`. The eval
suite does NOT simulate (it reads whatever the pipeline simulated), so merging
to one config means the gate eval runs against the PAPER scheduler set, not the
former 10-scheduler gate set. **Accepted consequence:** E3 (INCR_Reopt_X
period-monotonicity) needs ≥2 period arms; test_mode simulates only
`INCR_Reopt_10` → E3 reports MISSING at every N (non-fatal; Q1/Q2/Q3/E1 still
evaluate). The gate scheduler set itself is NOT folded (the paper set wins).
The `eval_*` keys are folded verbatim so the gate→N mapping is unchanged.
Pinned by `TestPaperConfigCarriesEvalKeys` + stale-name-fails-loudly.

### D2 — `--steps` is stale (already removed)

`run_end_to_end_experiments.py` has **no `--steps` flag** (P22 removed step
selection; the pipeline always runs simulate→sweep→aggregate). Yet
`simulation_only_config.json`'s `_comment` documents `--steps simulate`, and
`run_paper_figures.sh` / `run_interval_sweep.sh` reference the deleted
`run_simulation.sh` + step selection in their comments. These references are
**stale/broken**. Removing `simulation_only_config.json` moots its case; the two
thin scripts' comments need cleanup regardless.

### D3 — Disposition of `run_interval_sweep.sh` + `run_paper_figures.sh`

Per the finding above, they are standalone single-stage entry points, not lib
helpers. Options:

- **(a)** KEEP both as top-level standalone scripts (rename for clarity):
  `run_paper_figures.sh` IS goal (a) (figures-only) — must stay; extend it to
  also render Fig 2 (it currently misses it) so it truly produces "all figures."
  `run_interval_sweep.sh` is subsumed by the pipeline's sweep stage — keep as a
  convenience or delete.
- **(b)** DELETE `run_interval_sweep.sh` (subsumed by the pipeline); KEEP +
  rename `run_paper_figures.sh` as the "all paper figures" script (goal a),
  extending it to render Fig 2 too.

**Recommendation:** (b). Confirm the user still wants a standalone "re-run sweep
only" entry point.

### D4 — Renames (user-specified)

- `run_end_to_end.sh` → `run_simulation_and_plot_figures.sh`
- `run_evaluation_suite.sh` → `run_simulation_plot_eval_ns.sh`

Note: both renamed names imply simulation runs, but each has a skip mode
(`run_evaluation_suite.sh` has `SKIP_PIPELINE=1` for eval-only; the figures-only
script doesn't simulate). Minor naming tension — accept per user's choice.

### D5 — `experiment_config.json` rename ripple

Renaming to `paper_simulation_config.json` requires updating:
`experiment_config_loader.py:24` (`DEFAULT_CONFIG_PATH`) + the docstring/help
references in `interval_sweep.py:369-370`, `aggregate_across_tasks.py:935-936`,
`run_end_to_end_experiments.py:413-414`, `run_end_to_end.sh:4,27`, and
`P0_3/tasks.md:14`. Per the "no backward compat / stale names fail loudly"
convention (cf. P1.4, P2.4), a stale `experiment_config.json` reference should
FAIL LOUDLY, not silently alias.

### D6 — Merge shape for the two top-level scripts

`run_evaluation_suite.sh` is ~90% duplicated with `run_end_to_end.sh` (build +
delegate). Options:

- **(i)** MERGE: one script with a flag (`--eval_ns`) that appends the
  `evaluation_suite.py` stage after the pipeline.
- **(ii)** KEEP two scripts but factor the shared build+header+delegate block
  into `lib/common.sh` (a `run_pipeline` helper) so both source it.

**Recommendation:** (i) — a single script with `--eval_ns` matches D1(γ) and
eliminates the duplication. The user asked to "merge OR make evaluation suite
call some code in run end to end" — (ii) is the lighter touch. NEEDS USER
DECISION (or take recommendation (i)).

---

## High-level steps (after D1/D3/D6 resolved)

1. **Configs:** rename `experiment_config.json`→`paper_simulation_config.json`
   (update loader default + all refs, stale name fails loudly); resolve D1 for
   `evaluation_suite_config.json` (fold `eval_*` keys / flag-gate gate eval);
   remove `simulation_only_config.json`; keep `INCR_ET_Profiling.json`.
2. **Scripts:** rename the two top-level scripts (D4); resolve D6 (merge vs
   factor into `lib/common.sh`); resolve D3 for the two thin scripts; extend the
   figures-only script to render all paper figures incl. Fig 2; clean stale
   `--steps` / `run_simulation.sh` comments (D2).
3. **Verify:** pipeline produces bit-identical figures for an unchanged config;
   north-star suite emits the same verdict; `--dry_run` paths still print
   correctly; TDD pin for the rename (stale config name fails loudly) + the
   merge (flag-gated eval stage).

See `tasks.md` for the working checklist. Gate = bit-identical figures + same
gate verdict (refactor, not correctness).
