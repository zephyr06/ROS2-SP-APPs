# P2.17 — Tasks (working checklist)

> RE-FRAMED 2026-08-01 (see `goal.md`): NOT a P0.8-vs-P0.6 gate-consistency gap.
> `compare_optimizers.py` bypasses P0.8's gate entirely → unschedulable tasksets
> reach the sim → `ComputeSafeFallback` loud-fails. Blocked P0.7's Step-3 A/B.

## 0. Design decisions — SETTLED 2026-08-01
- [x] D1: route `compare_optimizers.py` generation through the gated pipeline
      (mirror `run_sim_experiments.py`'s `--important_tasks_schedulability_check`
      + the +20 gate-widened seed step). **DEFAULT ON** (parity with
      `run_sim_experiments.py`; user-confirmed 2026-08-01). BooleanOptionalAction
      so `--no-important_tasks_schedulability_check` is the diagnostic opt-out.
- [ ] D2: only if a TRUE gate-consistency case appears post-D1 (none yet).

## 1. Root cause — VERIFIED 2026-08-01
- [x] P0.8's gate is deterministic worst-case-WCET (non-perf=`execution_time_max`),
      hard `R<=deadline` — NOT mean-ET. The original framing was wrong.
- [x] `ComputeSafeFallback`'s loud-fail is ALSO worst-case-deterministic (point
      masses → probabilistic RTA degenerates); same hardness as P0.8.
- [x] Both gates read the SAME WCETs for compare_against_bf taskset_1 (task_1
      max_et=12.91, task_3 max_et=11.24). Python gate on the emitted taskset =
      `schedulable=False`, `R=35.39 > 33` = IDENTICAL to the C++ throw. Gates AGREE.
- [x] THE BUG = `compare_optimizers.py:591/604` calls the UNGATED
      `run_full_generation_pipeline`; no `--important_tasks_schedulability_check`
      flag. Both crashed runs (compare_against_bf N=4 taskset_1, measure_p07 N=6
      taskset_3) launched via it → legacy +1 seeds (1000,1001,...) not +20.
- [x] NOT a P0.7 bug: both arms crash identically in the unconditional P0.6 compute.

## 2. Fix (D1 LANDED — git add-only, NOT committed 2026-08-01)
- [x] Wire `compare_optimizers.py` generation through
      `run_full_generation_pipeline_with_important_task_gate` + the +20 seed step
      (mirror `run_sim_experiments.py:485-531`). Net change:
      - import `run_full_generation_pipeline_with_important_task_gate` +
        `IMPORTANT_TASK_GATE_MAX_ATTEMPTS` from the orchestrator;
      - new `--important_tasks_schedulability_check` BooleanOptionalAction
        (default ON), parity with `run_sim_experiments.py`;
      - per-taskset seed = `base_seed + idx * (20 if gate_on else 1)` (was `+ idx`);
      - extracted `_generate_taskset(args, cfg_file, dir_path)` helper that routes
        to the gated pipeline (surfaces the gate's `attempts_used` report) or the
        ungated one, replacing the two duplicated `run_full_generation_pipeline`
        call sites (verbose + quiet).
- [x] TDD: `tests/python/test_compare_optimizers_gate.py` (4 tests) — pins BOTH
      the routing (gated-when-on, ungated-when-off) AND the seed step
      (`[1000,1020,1040]` ON vs `[1000,1001,1002]` OFF). RED before the wiring,
      GREEN after. Mocks both generation entry points + `run_single_simulation`
      (no C++ binary). 28/28 compare tests pass (4 new + 24 existing).
- [x] No C++ change (expected). 3 pre-existing unrelated config-JSON test failures
      (`test_experiment_config_loader` / `test_generation_config_parser` /
      `test_patch_time_limit`) confirmed present WITHOUT my change — they assert
      on `time_limit_seconds:10` / `per_core_util` in the uncommitted P0.7 config
      edits, out of P2.17 scope.

## 3. Unblock + re-run P0.7 Step 3 (pending user A/B re-run)
- [ ] Re-run `measure_p07_penalty.json` at N=[4,6,8] once `compare_optimizers.py`
      is gated (now gated by default — needs the commit first).
- [ ] Report the full SP-penalty delta (prod ON vs meas OFF) back to P0.7.
