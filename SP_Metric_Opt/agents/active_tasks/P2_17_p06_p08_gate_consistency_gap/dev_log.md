# P2.17 P0.6-vs-P0.8 Gate Consistency Gap — Dev Log

> Chronological working log. On task completion, append a one-line milestone to the
> top-level `agents/dev_log.md` (the canonical narrative).

## 2026-08-01 — filed (surfaced by P0.7 Step 3)

Filed from the P0.7 Step-3 SP-penalty A/B run (`measure_p07_penalty.json`). The
measurement aborted at N=6 taskset_3 with SIGABRT in `ComputeSafeFallback`
(`OptimizeSP_TL_Incre.cpp:1030`) — the loud-fail throw "No safe fallback exists
for this task set — regenerate a new task set."

- **NOT a P0.7 bug:** both arms (`INCR_Reopt_10` prod, `INCR_NO_FALLBACK`
  measurement) crash with the IDENTICAL throw, because P0.6's safe-fallback
  COMPUTE is unconditional and P0.7 only gates the USE. Crash is independent of
  the P0.7 flag.
- **Root cause:** a taskset that passes P0.8's generator gate (important-task
  schedulability checked at `et_mean`) but fails P0.6's worst-case-DAG gate
  (same property checked on the per-task point mass at `max(execution_time_max)`
  across intervals). tasks6 taskset_3 task_0: `et_mean=2.61` (util 0.079, passes
  P0.8) but `execution_time_max=29.7` vs `period=33` (worst-case util 0.90 for
  one important task, fails P0.6). The high variance (`sigma=1.0`, max 18× the
  mean) drives the gap.
- **P0.7 unaffected:** N=4 completed cleanly for both arms; P0.7's mechanism
  demonstrably fired there (≈1% SP penalty). P0.7 reports from N=4; the full
  N=[4,6,8] re-run waits on this task.

D1–D3 open (see `goal.md`). No code yet.

## 2026-08-01 — ROOT CAUSE OVERTURNS the original framing (NOT a gate gap — a GATE BYPASS)

Investigated the compare_against_bf N=4 taskset_1 SIGABRT (same signature as the
N=6 crash). **The original "P0.8 (mean-ET) vs P0.6 (worst-case-ET) consistency
gap" framing is WRONG.** Verified facts:

1. **P0.8's gate is NOT mean-ET.** `important_task_rta.py:_wcets_from_loaded_tasks`
   (lines 399-414): perf task WCET = `execution_time_mu`; **non-perf task WCET =
   `execution_time_max`** of the worst-case-ET representative interval
   (`_load_emitted_tasks_by_gid` picks the interval with the largest
   `execution_time_max` per gid). It is a DETERMINISTIC WCET fixed-point RTA with
   a HARD `R <= deadline` check (`_rta_one_task`). So P0.8 is the HARDER gate,
   not the softer one.
2. **`ComputeSafeFallback`'s loud-fail is ALSO worst-case-deterministic**, not
   probabilistic: it re-gates on the worst-case DAG (per-task point mass at
   `max(execution_time_max)` across intervals). On point masses the probabilistic
   RTA degenerates to deterministic → `ddl_miss_chance` ∈ {0,1} → same hardness
   as P0.8. The online P0.7 triggers (b-i/b-ii) ARE probabilistic/softer, but
   the crash is NOT there — it's in the offline worst-case compute.
3. **Both gates read the SAME WCETs** for taskset_1 (verified by running the
   Python gate directly on the emitted taskset_1 interval YAMLs):
   task_1 max_et=12.91 (period=dl=33, proc 1), task_3 max_et=11.24 (period=dl=20,
   proc 1), both important. DM: task_3 > task_1. Python gate verdict =
   `schedulable=False`, culprit task_1, `R=35.39 > 33` — the IDENTICAL result
   `ComputeSafeFallback` computed. So P0.8's gate **would have rejected** this
   taskset. The two gates AGREE.
4. **THE BUG = `compare_optimizers.py` bypasses P0.8's gate.** It calls the
   UNGATED `run_full_generation_pipeline` (lines 591/604), NEVER
   `run_full_generation_pipeline_with_important_task_gate`. It has NO
   `--important_tasks_schedulability_check` flag at all (only `run_sim_experiments.py`
   wires the gate). Both crashed runs — compare_against_bf N=4 taskset_1 AND the
   original measure_p07_penalty N=6 taskset_3 — were launched via
   `compare_optimizers.py` (artifacts live under `optimizer_comparison/runs/`),
   so both generated tasksets with NO schedulability certification. The saved
   `generator_config.json` seeds confirm: base+idx (+1 legacy step, e.g.
   taskset_1=1001) instead of the gate-widened +20 step (would be 1020) —
   proving the gate was OFF at generation.

**Conclusion:** This is NOT a P0.8-vs-P0.6 gate-consistency defect. P0.8's gate
is sound (it correctly rejects the crashed taskset). P0.6's loud-fail is sound
(it catches what slipped through). The gap is that `compare_optimizers.py`
(the harness both P0.7 measurement runs used) does not route generation through
P0.8's gate, so unschedulable-at-worst-case tasksets reach the sim and trip
`ComputeSafeFallback`'s loud-fail. Fix = route `compare_optimizers.py` generation
through the gated pipeline (mirror `run_sim_experiments.py`'s gate wiring + the
gate-widened seed step). This likely UNBLOCKS the full P0.7 A/B re-run WITHOUT
any P0.8/P0.6 gate-logic change. Re-framed D1–D3 below (in goal.md) to match.

## 2026-08-01 — D1 LANDED (gate wiring; git add-only, NOT committed)

D1 settled with user: gate defaults **ON** in `compare_optimizers.py` (parity
with `run_sim_experiments.py`). Implemented the mirror wiring in
`simulation_experiments/compare_optimizers.py`:

- Imported `run_full_generation_pipeline_with_important_task_gate` +
  `IMPORTANT_TASK_GATE_MAX_ATTEMPTS` from `Gen_Taskset.lib.orchestrator`.
- New `--important_tasks_schedulability_check` BooleanOptionalAction, default
  `True` (so `--no-...` is the diagnostic opt-out) — identical shape to
  `run_sim_experiments.py:357-365`.
- Per-taskset seed changed from `args.base_seed + idx` to
  `args.base_seed + idx * seed_step` where
  `seed_step = IMPORTANT_TASK_GATE_MAX_ATTEMPTS (20)` when the gate is ON else
  `1` (legacy +1, bit-identical to pre-P2.17 when OFF). This prevents the gate's
  internal +0..19 retry window for taskset k from colliding with taskset k+1's
  starting draw.
- Extracted `_generate_taskset(args, cfg_file, dir_path)` — routes to the gated
  pipeline (surfacing `report['attempts_used']`) when ON, else the ungated
  pipeline — replacing the two duplicated `run_full_generation_pipeline` call
  sites (the verbose `>=2` branch and the quiet devnull-redirect branch). This
  is the modular single-entry-point shape `run_sim_experiments.py` already uses.

**TDD:** `tests/python/test_compare_optimizers_gate.py` (4 tests, RED→GREEN):
- `test_gate_on_by_default_routes_through_gated_pipeline` — no flag ⇒ gated
  pipeline called for all tasksets, ungated never called.
- `test_gate_on_uses_widened_seed_step` — ON ⇒ seeds `[1000,1020,1040]`.
- `test_gate_off_routes_through_ungated_pipeline` — `--no-...` ⇒ ungated.
- `test_gate_off_uses_legacy_plus_one_seed_step` — OFF ⇒ seeds `[1000,1001,1002]`.

Tests mock both generation entry points as module attributes (so the production
code MUST import them by name — the seam the bug lived in) and
`run_single_simulation` (no C++ binary). The recorder reads the stamped
`RANDOM_SEED` back from the temp config file `main()` writes, so it sees the
exact per-taskset seed.

**Verification:** 28/28 compare tests pass (4 new + 24 existing
`test_compare_optimizers` + `test_compare_optimizers_crash`). No C++ change.
3 unrelated config-JSON test failures
(`test_experiment_config_loader::test_all_configs_carry_time_limit_seconds`,
`test_generation_config_parser::test_synthesized_config_per_core_util_and_cores`,
`test_patch_time_limit::test_all_shipped_configs_readable_in_both_modes`)
confirmed PRE-EXISTING — they fail identically with my change stashed; they
assert on `time_limit_seconds:10` / `per_core_util` in the uncommitted P0.7
config edits (`compare_against_bf.json`, `paper_simulation_config.json`), out of
P2.17 scope.

**NEXT:** user reviews + commits (1 source file + 1 test file — within the
≤3-source-file commit guideline). Then re-run `measure_p07_penalty.json` at
N=[4,6,8] (Step 3) to report the full SP-penalty delta back to P0.7.
