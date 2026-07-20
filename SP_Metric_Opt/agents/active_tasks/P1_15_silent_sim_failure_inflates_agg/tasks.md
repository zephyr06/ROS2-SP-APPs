# P1.15 — Task Breakdown

> Investigation (Phase 0) is DONE — root cause confirmed, see `goal.md`.
> Decisions RESOLVED 2026-07-19 (user): (A) harness first, then (B) crash
> investigation; harness must **stop on crash + show it loudly**.
>
> **Requirement SIMPLIFIED 2026-07-19 (user, verbatim):** "as long as there is
> one crash, then no results are useful anymore, no need to report any numbers,
> fail it loudly. For one task number such as N=4/6/8, if any method has a crash
> at any task set's any interval, then results are not useful anymore. fail
> loudly, we'll restart all exp after fixing it." This OVERRIDES the earlier
> per-taskset-mean-then-average / NaN-row / unequal-N design (A4): on any crash,
> write NO `comparison_summary.csv`, NO plots — only the diagnostic artifacts
> (`crash_report.txt` + `taskset_arm_status.csv`) + a non-zero exit. The
> aggregate is never written over a partial run, so its internal averaging
> semantics no longer matter.

## Phase 0 — Investigation (DONE 2026-07-19)

- [x] Confirm BF's apparent loss is a harness artifact, not an optimizer
      result. → Confirmed: BF and INCR_Reopt_1 are byte-identical on
      taskset_0; the aggregate differs only because Reopt_5+ silently
      dropped 5 of 10 tasksets.
- [x] Map per-taskset × per-arm output presence. → 10×6 table in `goal.md`;
      Reopt_5/10/30/60 missing on tasksets {0,3,6,7,8}.
- [x] Locate the silent-failure path in the harness. →
      `compare_optimizers.py:519-527` (`concurrent.futures.wait` with no
      `.result()`), `:534-572` (`try/except Exception` swallow),
      `run_sim_experiments.py:80` (missing file → empty, not raise),
      `utils.py:206-218` (`np.mean` over unequal N).
- [x] Confirm the binary actually crashes on the dropped arms. → Reproduced
      `taskset_3 INCR_Reopt_5/10/30/60` all exit 134 (SIGABRT, core dumped)
      on the current release binary (rebuilt 2026-07-19 14:49).
- [x] Capture the crash stack + throw site. → `RTA_Cache.cpp:357-364`
      (`ComputeTaskSetDifference` throws `std::runtime_error` when
      `TryComputeSingleChange` returns false → candidate differs from
      champion by >1 task, "violates P1.10 single-change invariant"). Stack
      `Evaluate → ClassifyReusePerTask → ComputeTaskSetDifference` thrown
      inside `EvaluateTimeLimitConfig_SubIncremental` (`OptimizeSP_TL_Incre.cpp:279`,
      the P1.12 read-side swap) within the serialized walk where
      `rta_cache_active_=true` (`:439`). See `dev_log.md`.
- [x] Check binary freshness vs source. → Binary is FRESH (rebuilt
      2026-07-19 14:49 from HEAD). `git status` shows NO uncommitted
      changes to `sources/` or `tests/` — the optimizer is fully committed
      at HEAD `bfbec7e5` (P1.14 guard = `ecf0c597`; P1.12 cache series =
      `8e18c39b`/`5a172973`/`bfbec7e5`). Crash is in committed current code.

## Phase 1 — Harness: stop-on-crash + show-loudly (layer A) — DONE 2026-07-19

> User's requirement (verbatim): "as long as there is one crash, then no
> results are useful anymore, no need to report any numbers, fail it loudly."
> TDD-first: RED tests, then GREEN.

- [x] **A0. (TDD, RED)** Added `tests/python/test_compare_optimizers_crash.py`
      (9 tests: stop-on-crash, crash_report + status map, run.log captures
      throw text, analyze raises on missing/empty output, summary emits NaN
      for empty arm, unequal-N fails loudly). All RED on the pre-fix code.
- [x] **A1. Propagate worker exceptions + stop-on-crash.**
      `compare_optimizers.py` `main()` now uses `ThreadPoolExecutor` +
      `fut.result()` (was `ProcessPoolExecutor` + `concurrent.futures.wait`
      fire-and-forget). The first `CalledProcessError` aborts the run,
      remaining arms marked NOT_RUN.
- [x] **A2. Capture per-arm stdout/stderr.** `run_single_simulation`
      (`run_sim_experiments.py`) ALWAYS captures binary stdout+stderr to
      `<sched_dir>/<scheduler>/run.log` (was `subprocess.DEVNULL` unless
      `verbose>=2`); tees to console at `verbose>=2`; raises
      `CalledProcessError` on non-zero exit. The C++ throw text
      (`ComputeTaskSetDifference` / `single-change invariant`) survives the
      crash and is surfaced in `crash_report.txt`.
- [x] **A3. `analyze_single_instance` fails loudly on missing output.**
      `run_sim_experiments.py`: raises `FileNotFoundError` if
      `interval_sp_metrics.txt` is absent; raises `ValueError` if no parseable
      SP data rows. The silent-zero (which let a crashed arm read as "perfect
      0.0 miss rate") is gone.
- [x] **A4. On crash, NO summary/plots are written.** SIMPLIFIED from the
      earlier per-taskset-mean-then-average design. The crash branch in
      `compare_optimizers.py main()` no longer calls `write_summary_and_plots`
      — on any crash it writes ONLY `crash_report.txt` +
      `taskset_arm_status.csv` and exits non-zero. No
      `comparison_summary.csv`, no plots: a partial aggregate can never be
      mistaken for a complete one. (`write_summary_and_plots` in `utils.py`
      retains its NaN-for-empty-arm + unequal-N guard as defensive depth on
      the clean path, but the crash path never reaches it.)
- [x] **A5. Guard `--resume`.** `_has_complete_metrics`
      (`compare_optimizers.py`) requires `interval_sp_metrics.txt` to exist
      AND have ≥1 parseable data row; a 0-byte / header-only file from a
      half-written abort is re-attempted, not silently skipped.
- [x] **A6. Loud end-of-run crash report.** `_write_crash_report` +
      `_write_status_map` write `crash_report.txt` (names every crashed
      (taskset, arm) + run.log tail with the C++ throw text) and
      `taskset_arm_status.csv` (per-(taskset, arm) ✅/❌/NOT_RUN/RESUMED map).
      `main()` prints `[CRASH] ...` to stderr and `sys.exit(msg)` (non-zero).
- [x] **A7. (TDD, GREEN)** 9/9 `test_compare_optimizers_crash.py` GREEN;
      300/300 full `tests/python/` suite GREEN (no regression). E2E validated
      against a stub crashing binary: stop-on-crash fires, loud `[CRASH]`
      stderr, crash_report + status_map written, NO `comparison_summary.csv`
      / plots, exit 1.

## Phase 2 — Optimizer: find & fix the abort (layer B) — DONE (by P1.16)

> The C++ crash that *caused* the silent-failure bug is fixed by **P1.16**
> (`agents/active_tasks/P1_16_rta_cache_desync_fix/`): `UpdateRecords` returns
> `bool`; `EvaluateTimeLimitConfig_SubIncremental` backs up `rta_cache_` at
> entry and reverts it when the candidate is not adopted, so speculative
> champion updates no longer desynchronize on rejected walks. Reproduced
> `taskset_3 INCR_Reopt_5/10/30/60` — all now exit 0 (was 134). Layer B is
> resolved; Phase 1 (layer A) was the harness half and is also DONE. The
> B1–B4 items below are kept for the record; B1/B2/B3 are satisfied by P1.16.

- [ ] **B1.** Reproduce under gdb on the CURRENT release binary:
      `gdb --args ./release/tests/RunOrchestrator <taskset_3> <out>
      INCR_Reopt_5 10000 1`; `run`; `bt` from the abort. Confirm the throw
      site (`RTA_Cache.cpp:357-364`) + which of the 4 `TryComputeSingleChange`
      false-return paths fires.
- [ ] **B2.** Classify the crash + identify the champion drift. The throw
      says the candidate at `OptimizeSP_TL_Incre.cpp:279` differs from the
      adopted champion by >1 task. Open question: WHY does the champion drift
      to >1 diff only at Reopt_X>1 (Reopt_1 realigns each interval; Reopt_X>1
      carries the champion across multiple env moves;
      `ResetIncumbentBaseline:829` resets the cache each interval). Hypothesis
      to check: does P1.14's `BFDLSharedBudget` cooperative-cancel
      (`EvaluateSPWithPriorityVec` returns `INT_MIN`) corrupt the champion
      state such that a later `Evaluate` sees >1 diff? Cross-check memory
      [[p112-rta-cache-evaluate-pa-move-indexing-bug]] + [[p114-bf-time-limit-violation]].
- [ ] **B3.** Fix the root cause. Two viable shapes (decide after B2):
      (i) make the throw impossible — prove/enforce the candidate at `:279`
      always satisfies |diff|<=1 (fix the invariant violation at source);
      (ii) make `Evaluate` degrade gracefully on >1 (full RTA recompute =
      cache miss, never a crash) as the immediate unblock, then (i) as a
      follow-up if B2 reveals a real logic bug. Add a focused TDD repro
      (RED on current, GREEN after fix) at the smallest fixture (likely a
      small N=4 taskset + Reopt_X>1).
- [ ] **B4.** Confirm no other (taskset, arm) pairs still abort: re-run the
      full 10×6 matrix via the fixed harness from Phase 1 and verify all 60
      cells produce output (the per-taskset ✅/❌ map is all ✅).

## Phase 3 — Re-run + verify (unblocks P1.14 Phase 3)

- [ ] **C1.** Rebuild release binary with the Phase 2 fix.
- [ ] **C2.** Re-run the P25 period A/B (`p25_period_ab_config`, N=4×6) on
      the fixed binary + fixed harness.
- [ ] **C3.** Verify the correctness gate in `goal.md`: every scheduler has
      N=10 tasksets × 60 intervals, no `0.000000 ± 0.000000` rows unless
      genuinely zero, the per-taskset ✅/❌ table is all ✅.
- [ ] **C4.** Record the true BF-vs-INCR_Reopt_X verdict; unblock P1.14
      Phase 3 (BF ET re-run); update `overall_tasks.md` + memory.
