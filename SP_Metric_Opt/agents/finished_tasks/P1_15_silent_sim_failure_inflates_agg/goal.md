# P1.15 — Silent Sim Failure Inflates the Aggregated A/B (BF appears to lose)

> Filed 2026-07-19. **Investigation complete; root cause CONFIRMED.
> Phase 1 (harness loud-failure fix) IN PROGRESS 2026-07-19 per the user's
> chosen ordering (harness first, then crash investigation).** The user's
> hypothesis is correct: this is NOT an optimizer-correctness bug in the SP
> sense; it is a **silent-failure + unequal-N aggregation** bug in the
> experiment harness. The optimizer does crash on some (taskset, arm) pairs,
> but the crash is swallowed and the aggregate then compares schedulers over
> **different taskset sets**, manufacturing BF's apparent loss.

---

## The symptom (grounded in the run output)

Run:
`simulation_experiments/optimizer_comparison/runs/p25periodAB_run_test_dur600_interval10_seed1000_tasks4x6/sim/tasks4_dur600_interval10_seed1000/`

`comparison_summary.csv`:

| Scheduler          | Mean_SP_Metric | Mean_Miss_Rate | Important_Miss_Rate | Mean_Sched_Time_s |
|--------------------|---------------:|---------------:|--------------------:|-------------------:|
| BF                 | 0.612907       | 0.021834       | 0.168333            | 0.127734           |
| INCR_Reopt_1       | 0.591043       | 0.194476       | 0.430000            | 0.085644           |
| INCR_Reopt_5       | 0.621938       | 0.000000       | 0.000000            | 0.059619           |
| INCR_Reopt_10      | 0.621989       | 0.000000       | 0.000000            | 0.052819           |
| INCR_Reopt_30      | 0.622112       | 0.000000       | 0.000000            | 0.061546           |
| INCR_Reopt_60      | 0.622112       | 0.000000       | 0.000000            | 0.049011           |

BF loses to INCR_Reopt_5/10/30/60 (~0.622 vs 0.613), and BF is the *only* arm
with a non-zero Important_Miss_Rate (0.168) while the Reopt_5+ arms report
**exactly 0.000000** miss rate and 0.000000 std — an impossible "perfect"
record that is the fingerprint of missing data, not a real scheduler result.

## What is actually on disk (the smoking gun)

Per-taskset `interval_sp_metrics.txt` presence (10 tasksets × 6 arms):

| taskset | BF | Reopt_1 | Reopt_5 | Reopt_10 | Reopt_30 | Reopt_60 |
|---------|:--:|:-------:|:-------:|:--------:|:--------:|:--------:|
| 0       | ✅ | ✅      | ❌      | ❌       | ❌       | ❌       |
| 1       | ✅ | ✅      | ✅      | ✅       | ✅       | ✅       |
| 2       | ✅ | ✅      | ✅      | ✅       | ✅       | ✅       |
| 3       | ✅ | ✅      | ❌      | ❌       | ❌       | ❌       |
| 4       | ✅ | ✅      | ✅      | ✅       | ✅       | ✅       |
| 5       | ✅ | ✅      | ✅      | ✅       | ✅       | ✅       |
| 6       | ✅ | ✅      | ❌      | ❌       | ❌       | ❌       |
| 7       | ✅ | ✅      | ❌      | ❌       | ❌       | ❌       |
| 8       | ✅ | ✅      | ❌      | ❌       | ❌       | ❌       |
| 9       | ✅ | ✅      | ✅      | ✅       | ✅       | ✅       |

- BF and INCR_Reopt_1 produced output on **all 10** tasksets.
- INCR_Reopt_5/10/30/60 produced output on only **5 of 10** tasksets
  (0, 3, 6, 7, 8 are empty — the dir exists but contains no nested
  `<sched>/<sched>/interval_sp_metrics.txt`).
- On the 5 tasksets where Reopt_5+ are missing, BF **beats** Reopt_1
  (e.g. taskset_3: BF 0.5433 vs Reopt_1 0.5143; taskset_6: BF 0.5922 vs
  0.5139; taskset_8: BF 0.6231 vs 0.5486). These are exactly the "hard"
  tasksets that drag BF's mean down — and they are exactly the tasksets
  Reopt_5+ silently dropped.
- On taskset_0, BF and INCR_Reopt_1 are **byte-identical**
  (`interval_sp_metrics.txt` 60/60 lines match, SP summary 0.672911 for
  both, same 4300/39900 misses). So the optimizer is *not* producing
  different SP for the same inputs across arms on the tasksets that ran.

## Why the aggregate is wrong (the bug)

Two compounding harness defects, both in the Python layer:

1. **The simulation crash is swallowed silently.**
   `compare_optimizers.py:519-527` submits each arm via
   `executor.submit(run_single_simulation, ...)` then
   `concurrent.futures.wait(sim_futures)` — exceptions in the futures are
   **never retrieved** (`wait` does not raise; no `.result()` call).
   `run_single_simulation` (`run_sim_experiments.py:55`) uses
   `subprocess.run(sim_cmd, check=True, ...)`, so a binary abort raises
   `CalledProcessError` inside the worker — which `wait` discards. The
   per-arm analyze loop (`compare_optimizers.py:534-572`) then wraps
   `analyze_single_instance` in `try/except Exception` that only `print`s
   and continues. Net: a crashing arm leaves an empty output dir and
   contributes **nothing** to that scheduler's results, with no non-zero
   exit, no log, no warning at the run level.

2. **The aggregate averages over unequal, per-scheduler taskset sets.**
   `analyze_single_instance` (`run_sim_experiments.py:80`) treats a missing
   `interval_sp_metrics.txt` as "no SP values this run" (returns empty
   `sp_values_run`, `miss_rate=0` — note the silent zero, not NaN).
   `write_summary_and_plots` (`utils.py:206-218`) then does
   `np.mean(sp_arr) if len(sp_arr) > 0 else 0.0` over **whatever SP values
   survived** — BF contributes 10×60=600 values; Reopt_5+ contribute
   5×60=300 values drawn only from the *easy* tasksets. The means are
   computed over different taskset populations, so the comparison is
   apples-to-oranges. BF's 0.613 includes the 5 hard tasksets; Reopt_5+'s
   0.622 excludes them. This manufactures BF's apparent loss.

The "perfect 0.000000 miss rate + 0.000000 std" on Reopt_5+ in the CSV is
the tell-tale: those arms' miss-rate lists are empty for the dropped
tasksets, and `np.mean([]) → 0.0`, `np.std([]) → 0.0`.

## Confirmed: the binary aborts (exit 134 / SIGABRT) on the dropped arms

Reproduced directly on `taskset_3` with the **current** release binary
(rebuilt 2026-07-19 14:49 from HEAD, which is fully committed — NOT stale,
NOT a working-tree-only build):

```
taskset_3 BF:          exit=0   HAS_OUTPUT
taskset_3 INCR_Reopt_1: exit=0   HAS_OUTPUT
taskset_3 INCR_Reopt_5: exit=134 NO_OUTPUT   ← Aborted (core dumped)
taskset_3 INCR_Reopt_10:exit=134 NO_OUTPUT
taskset_3 INCR_Reopt_30:exit=134 NO_OUTPUT
taskset_3 INCR_Reopt_60:exit=134 NO_OUTPUT
```

The abort happens **after** interval 0 completes and prints
`opt_sp_ = 0.514286` (matching the recorded Reopt_1 value), so interval 0
runs fine; the crash is downstream — most likely at the **first
compare-and-keep / reopt boundary** specific to Reopt_X>1 (interval 5 for
Reopt_5, etc.), or in the P1.12 RTA-cache path now engaged in `OptimizeIncre`
(see memory `p112-rta-cache-evaluate-pa-move-indexing-bug` — the `:285`
cache flip landed in this working tree). No C++ assert/exception text is
captured in stdout/stderr before the core dump — the abort is silent at the
text level (likely a segfault or an untrolled `abort()`), so the only signal
is the process exit code, which the harness ignores.

> NOTE on recency / git state (verified 2026-07-19): the optimizer sources
> at HEAD are FULLY COMMITTED — `git status` shows no uncommitted changes to
> `sources/` or `tests/`. P1.14's `BFDLSharedBudget` guard landed in commit
> `ecf0c597`; P1.12's `:285` cache flip + `rta_cache_` integration landed in
> the `8e18c39b` / `5a172973` / `bfbec7e5` series. (Memory entries
> `p112-rta-cache-evaluate-pa-move-indexing-bug` and `p114-bf-time-limit-violation`
> still describe these as "working tree, NOT committed" — that is STALE
> relative to git; they have since been committed.) So the crash is in
> **committed, current** code, not in-flight uncommitted work. The release
> binary was rebuilt 2026-07-19 14:49 from these committed sources → it
> reflects HEAD. The P1.12 cache path and the P1.14 guard remain candidate
> crash sources because they are recent + live in the INCR/Reopt path; BF
> itself does NOT crash, consistent with the crash being in the INCR/Reopt
> path (P1.12) rather than the BF path (P1.14).

## Scope of the fix

This task covers **both** layers. They are independent and should be fixed
together, because fixing only one still leaves the result untrustworthy:

- **(A) Harness: make silent failure impossible.** This is the core of the
  user's report and the part that "decides whether task sets can be reused."
  At minimum: (1) retrieve future exceptions (call `.result()` or switch to
  `as_completed` with error propagation) so a crashing arm **fails the run**
  instead of being skipped; (2) in `analyze_single_instance`, treat a missing
  `interval_sp_metrics.txt` as a hard error (raise), not an empty/zero
  contribution; (3) in `write_summary_and_plots`, either fail loudly when
  schedulers have unequal N, or — better — aggregate **per-taskset then
  average over tasksets** (so each scheduler is scored on the same taskset
  set, and a missing arm is visible as a gap, not a silent 0). The
  `--resume` path (`compare_optimizers.py:513-518`) also needs a guard: a
  partial dir with no `interval_sp_metrics.txt` must not be mistaken for a
  completed run.
- **(B) Optimizer: find and fix the abort.** Reproduce under a debugger
  (gdb/`catch throw` + `bt`) on `taskset_3 INCR_Reopt_5`, get the stack,
  identify whether it is the P1.12 cache path, the P1.14 shared-budget
  guard, or an unrelated reopt-boundary defect, and fix it. Even after (A)
  makes the failure loud, the optimizer must not crash.

## Decisions (resolved 2026-07-19 by the user)

1. **Fix ordering.** **(A) harness first, then (B) optimizer.** Land the
   harness loud-failure fix so the next run surfaces the crash loudly
   instead of manufacturing a fake aggregate, *then* investigate the crash
   with the debugger. (A) is small, it is the user's stated concern, and it
   unblocks a clean re-run that will reveal the real BF-vs-INCR verdict.
2. **Harness crash policy (the user's two explicit requirements).**
   (i) **Stop the experiment when one optimizer crashes** — do NOT continue
   running the remaining arms of that taskset (or, by default, the remaining
   tasksets) as if nothing happened; a crashing arm is a run-level failure.
   (ii) **Show the crash loudly** — never treat a crashed arm's missing
   output as a silent `0.000000` row. Capture the C++ stdout/stderr (so the
   `std::runtime_error` text from `RTA_Cache.cpp:357` survives for the
   crash investigation), print a loud crash report at the end, write a
   `crash_report.txt` + the per-taskset ✅/❌ map, and exit non-zero.
3. **Aggregate semantics.** Per-taskset-mean-then-average (each scheduler
   scored on the same taskset set) **and** fail-loudly on unequal N — both.
   A missing arm surfaces as an explicit gap (NaN in the row, not 0), and
   the run exits non-zero so no downstream consumer mistakes a partial
   aggregate for a complete one.
4. **`--resume` policy.** A partial/empty arm dir (no
   `interval_sp_metrics.txt`) is treated as **not complete → re-attempted**,
   not silently skipped. The existing file-existence check is kept but
   tightened: the file must exist *and* be non-empty (a 0-byte or
   header-only file from a half-written abort is not a complete run).

## Correctness gate

- A re-run of this A/B on the fixed binary + fixed harness must produce a
  `comparison_summary.csv` where **every scheduler has the same N** (10
  tasksets × 60 intervals) and no arm reports a suspicious `0.000000 ±
  0.000000` miss rate unless it is genuinely zero across all tasksets.
- The per-taskset table (✅/❌ above) must be all ✅.
- A TDD test that injects a crashing arm (e.g. a stub `RunOrchestrator`
  that exits 134) must fail the harness run loudly, not silently drop the
  arm.

## Related

- Memory: [[p112-rta-cache-evaluate-pa-move-indexing-bug]] (the `:285`
  cache flip — a candidate crash source; note that memory still says
  "working tree, NOT committed" but git shows it IS committed at HEAD),
  [[p114-bf-time-limit-violation]] (the `BFDLSharedBudget` guard — another
  candidate; same staleness caveat), [[p19-cache-redesign-single-champion]]
  (P1.12 integration state).
- Sibling: P1.14 (BF time-limit) — the same A/B run, different symptom
  (BF ET, not BF SP). P1.14's fix touched the BF path; this task's crash is
  in the INCR/Reopt path, so they are likely independent. Both fixes are
  committed at HEAD (not uncommitted, despite the memory framing).
