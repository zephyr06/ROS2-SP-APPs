# P2.2 — Doc & Memory Hygiene

**Priority:** P2 (parallel anytime)
**Status:** not started

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
