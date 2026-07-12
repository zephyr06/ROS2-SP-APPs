# P2.2 — Doc & Memory Hygiene

**Priority:** P2 (parallel anytime)
**Status:** CLOSED 2026-07-12 as superseded / moot — NOT executed. Every substantive
item was overtaken by later resolutions before this task was picked up; the
task-as-specified is obsolete. Closure disposition per item is recorded below
and in `tasks.md`. Folder moved to `finished_tasks/`. (Original "not started"
scaffold preserved as the historical body below.)

## Closure disposition (2026-07-12)

User review concluded all three substantive items are outdated; the task should
be closed, not worked. Verified against current code + git history:

- **(a) Memory `interval-sweep-stale-flags-bug` → NOT edited; premise stale.**
  The memory asserts `compare_optimizers.py` **removed** `--on_taskset_config_change` /
  `--run_root`, crashing the sweep stage. Verified current code: both flags are
  **accepted** (`compare_optimizers.py:303` `--run_root`, `:334`
  `--on_taskset_config_change`; `--skip_generation_if_exists` also still present
  at `:327`), and `interval_sweep.py` still emits them. The described flag-removal
  either was reverted or never persisted past a working-tree edit — the crash
  doesn't reproduce. The memory is therefore wrong in the *opposite* direction
  from what the task anticipated (it expected to mark RESOLVED-cite-the-fix; the
  honest note would be "the described bug no longer reproduces / premise was
  wrong"). **Algorithm-performance impact: NONE** — even on the memory's own terms
  this was a sweep-stage CLI-arg plumbing mismatch (exit 2 /
  `subprocess.CalledProcessError`), not an optimizer or SP-metric defect. The
  algorithm runs in the **simulate** stage; the sweep only re-runs the scheduler
  across `interval_sweep_seconds_list` and aggregates. The memory's own workaround
  (`--steps simulate aggregate`) yields correct optimize/aggregate output, just
  without the period-sensitivity comparison data. A live sweep-stage crash costs
  measurement *coverage*, never correctness or optimizer behavior. Per the P22
  refactor `run_end_to_end.sh` always runs simulate→sweep→aggregate (no `--steps`
  skip) and the P25 A/B config sets `interval_sweep_seconds_list=[10]` (single
  point → sweep no-op for that experiment) — so the e2e path doesn't crash. The
  memory file itself was **left untouched** (out of this task's scoped action);
  it remains stale and should be retired in a separate one-line memory edit.
- **(b) §5 corrections → OUTDATED.** §5 of `investigation_problems_encountered.md`
  predates the P1.1 resolution. Per memory `p25-ndiff-diff-semantics` (2026-07-07)
  the "2-vs-8 discrepancy" was re-derived — not UNRESOLVED: false-positives from
  the update side using Gaussian-mean TL instead of the carried adopted TL on
  TL-optimizable tasks; that memory also concludes "Fix C & Fix D both wrong
  levers." P0.5 (committed 2026-07-10, `a8dba07f`→`7fa2e9d2`) then resolved the
  gate by making both diff sides carry the adopted TL (`ndiff` 5→0 at runtime).
  So both §5 asks are moot: "record 2-vs-8 as UNRESOLVED" — it's no longer
  unresolved; "correct Fix D as inert" — true observation, but Fix D is an
  abandoned lever (and P2.3 behavior-neutrally cleaned the `approx_equal` area
  anyway). The investigation moved past §5.
- **(c) `issues.md` #9/#3/#6 → OUTDATED.** `issues.md` was **deleted** in commit
  `66c96c14` (2026-07-06, "remove issues.md"). There is no file to mark
  SUPERSEDED / let-go on — resolved-by-deletion.
- **(d) `trial_and_error` → already DONE.** `finished_tasks/summary.md` records
  it DONE 2026-07-05, committed @ `88af2c54` + `fa0b857f`; P0.1 has since landed
  (`a8dba07f`→`7fa2e9d2`). Self-resolved.

## Goal

Bring the agent docs and memory notes into agreement with the committed code
state. Several docs/memories describe behavior that has since changed; they
read as stale and will mislead the next agent (or reviewer).

## Items

### (a) Memory `interval-sweep-stale-flags-bug` → RESOLVED

Verified: `compare_optimizers.py:303,333` accepts the flags;
`test_interval_sweep.py` covers forwarding. Mark RESOLVED (the prod
confirmation is P2.1, but the code fix is done).

### (b) Apply `investigation/investigation_problems_encountered.md` §5 corrections

§5 lists outstanding corrections. Apply them to:
- `investigation/debug_runtime0704_incr.md` §10/§11 — drop the unsupported
  "modal ndiff=1" claim; record the **2-vs-8** changed-task-count discrepancy
  as UNRESOLVED (it's the P1.1 gate).
- Memory `p25-incr-et-grows-with-period` — same corrections.

### (c) `issues.md` updates

- **#9** (exporter vs randomized `N_ENV_DEPENDENT_TASKS`) → mark **SUPERSEDED**
  by the env-dependent-ratio refactor (commit `4b44aabc` removed the key
  entirely). The issues.md re-verification predates that commit and is stale
  on this point.
- **#3** (ignored `SP_THRESHOLD_RANGE`) and **#6** (per-task-type config
  separation) → mark as **accepted-for-paper let-go** (see the Deferred table
  in `overall_tasks.md`). They're config-flexibility, not correctness.

### (d) `trial_and_error_tl_opt_task.md` — already DONE + moved

The reorg moved it to `finished_tasks/`. Confirm `finished_tasks/summary.md`
records it as DONE. The "pending commit (P0.1)" note flips to "committed @
<sha>" once P0.1 lands.

## Files

- Memory: `interval-sweep-stale-flags-bug.md`, `p25-incr-et-grows-with-period.md`
- `agents/issues.md`
- `agents/investigation/debug_runtime0704_incr.md`
- `agents/investigation/investigation_problems_encountered.md`
- `agents/finished_tasks/summary.md` (cross-reference)

## Done when

- `git diff` on `agents/*.md` + memory files shows the corrections.
- `issues.md` #9 marked SUPERSEDED; #3/#6 marked let-go.
- Memory `interval-sweep-stale-flags-bug` marked RESOLVED.

## Out of scope

- The P1.1 investigation itself (this task only fixes the *docs* describing it).
- The prod sweep confirmation (P2.1).
